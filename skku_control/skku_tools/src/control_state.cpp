#include <stdexcept>
#include <type_traits>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <skku_tools/control_state.h>
#include <cerrno>
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace SKKU {

    namespace {
        constexpr canid_t SENSOR_ID_1 = 0x01;
        constexpr canid_t SENSOR_ID_2 = 0x02;
        constexpr canid_t INDEX_ID = 0x102;
        constexpr uint8_t SENSOR_ID = 0x01;
        constexpr const char* CAN_INTERFACE = "can0";

        float decodeForce(uint8_t high, uint8_t low) {
            return (static_cast<int>(high) * 256 + static_cast<int>(low)) / 100.0f - 300.0f;
        }

        float decodeTorque(uint8_t high, uint8_t low) {
            return (static_cast<int>(high) * 256 + static_cast<int>(low)) / 500.0f - 50.0f;
        }
    }

    // // Sensor_data 클래스의 생성자
    // Sensor_data::Sensor_data() {
    //     // ROS 노드 핸들 초기화 및 콜백 함수 연결
    //     sensor_sub_ = nh_.subscribe("/sensor_data", 10, &Sensor_data::sensorDataCallback, this);
    // }

    // // 센서 데이터를 반환하는 함수
    // std::array<float, 6> Sensor_data::getAFTWrench() const {
    //     return AFT_wrench_;
    // }

    // // ROS 콜백 함수: 센서 데이터 업데이트
    // void Sensor_data::sensorDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    //     if (msg->data.size() >= 6) {
    //         for (int i = 0; i < 6; ++i) {
    //             AFT_wrench_[i] = msg->data[i];
    //         }
    //     }
    // }

    Sensor_data::Sensor_data() {
        if (!openCanSocket()) {
            ROS_ERROR("Sensor_data: failed to open %s. AFT wrench will remain zero.", CAN_INTERFACE);
            return;
        }

        if (!initializeSensor()) {
            ROS_ERROR("Sensor_data: failed to initialize AFT sensor.");
            close(can_socket_);
            can_socket_ = -1;
            return;
        }

        usleep(1000000);
        transmitMode();

        can_running_ = true;
        can_thread_ = std::thread(&Sensor_data::canReadLoop, this);
    }

    Sensor_data::~Sensor_data() {
        can_running_ = false;

        if (can_socket_ >= 0) {
            close(can_socket_);
            can_socket_ = -1;
        }

        if (can_thread_.joinable()) {
            can_thread_.join();
        }
    }

    // raw sensor wrench 반환
    std::array<float, 6> Sensor_data::getAFTWrench() const {
        std::lock_guard<std::mutex> lock(wrench_mutex_);
        return AFT_wrench_;
    }

    // matched sensor wrench 반환
    std::array<float, 6> Sensor_data::getMatchedAFTWrench() const {
        std::lock_guard<std::mutex> lock(wrench_mutex_);
        return AFT_wrench_matched;
    }

    // raw -> matched frame conversion
    std::array<float, 6> Sensor_data::matchAFTWrench(const Eigen::Matrix3f& rotationMatrix) {
        // ------------------------------------------------------------
        // Step 1) raw sensor frame -> controller frame sign matching
        // 기존에 네가 쓰던 부호 규칙 그대로 반영
        // ------------------------------------------------------------
        std::array<float, 6> matched = {0, 0, 0, 0, 0, 0};
        std::array<float, 6> raw = {0, 0, 0, 0, 0, 0};

        {
            std::lock_guard<std::mutex> lock(wrench_mutex_);
            raw = AFT_wrench_;
        }

        matched[0] = -raw[0];
        matched[1] = -raw[1];
        matched[2] =  raw[2];
        matched[3] = -raw[3];
        matched[4] = -raw[4];
        matched[5] =  raw[5];

        // ------------------------------------------------------------
        // Step 2) rotate force / torque
        // force, torque 각각 3x3 rotation 적용
        // ------------------------------------------------------------
        Eigen::Vector3f forceVector(matched[0], matched[1], matched[2]);
        Eigen::Vector3f torqueVector(matched[3], matched[4], matched[5]);

        Eigen::Vector3f rotatedForce  = rotationMatrix * forceVector;
        Eigen::Vector3f rotatedTorque = rotationMatrix * torqueVector;

        matched[0] = rotatedForce(0);
        matched[1] = rotatedForce(1);
        matched[2] = rotatedForce(2);

        matched[3] = rotatedTorque(0);
        matched[4] = rotatedTorque(1);
        matched[5] = rotatedTorque(2);

        // ------------------------------------------------------------
        // Step 3) 내부 저장
        // ------------------------------------------------------------
        {
            std::lock_guard<std::mutex> lock(wrench_mutex_);
            AFT_wrench_matched = matched;
        }
        return matched;
    }

    // ROS 콜백 함수: raw sensor data 업데이트
    void Sensor_data::sensorDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() >= 6) {
            std::lock_guard<std::mutex> lock(wrench_mutex_);
            for (int i = 0; i < 6; ++i) {
                AFT_wrench_[i] = msg->data[i];
            }
        }
    }

    bool Sensor_data::openCanSocket() {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            ROS_ERROR("Sensor_data: socket open failed: %s", std::strerror(errno));
            return false;
        }

        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        struct ifreq ifr;
        std::memset(&ifr, 0, sizeof(ifr));
        std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);

        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            ROS_ERROR("Sensor_data: ioctl SIOCGIFINDEX failed for %s: %s",
                      CAN_INTERFACE, std::strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            ROS_ERROR("Sensor_data: bind failed for %s: %s",
                      CAN_INTERFACE, std::strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        return true;
    }

    bool Sensor_data::initializeSensor() {
        if (can_socket_ < 0) {
            return false;
        }

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = INDEX_ID;
        frame.can_dlc = 8;
        frame.data[0] = SENSOR_ID;
        frame.data[1] = 0x02;
        frame.data[2] = 0x01;

        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            ROS_ERROR("Sensor_data: initialize command failed: %s", std::strerror(errno));
            return false;
        }

        ROS_INFO("Sensor_data: initialize command sent successfully.");
        return true;
    }

    void Sensor_data::transmitMode() {
        if (can_socket_ < 0) {
            return;
        }

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        frame.can_id = INDEX_ID;
        frame.can_dlc = 8;
        frame.data[0] = SENSOR_ID;
        frame.data[1] = 0x03;
        frame.data[2] = 0x01;

        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            ROS_ERROR("Sensor_data: transmit mode command failed: %s", std::strerror(errno));
        } else {
            ROS_INFO("Sensor_data: transmit mode command sent successfully.");
        }
    }

    void Sensor_data::canReadLoop() {
        struct can_frame frame;

        while (can_running_) {
            ssize_t nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

            if (!can_running_) {
                break;
            }

            if (nbytes < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                    continue;
                }

                ROS_WARN_THROTTLE(1.0, "Sensor_data: CAN read failed: %s", std::strerror(errno));
                continue;
            }

            if (nbytes != sizeof(struct can_frame)) {
                continue;
            }

            std::lock_guard<std::mutex> lock(wrench_mutex_);

            if (frame.can_id == SENSOR_ID_1) {
                AFT_wrench_[0] = decodeForce(frame.data[0], frame.data[1]);
                AFT_wrench_[1] = decodeForce(frame.data[2], frame.data[3]);
                AFT_wrench_[2] = decodeForce(frame.data[4], frame.data[5]);
            } else if (frame.can_id == SENSOR_ID_2) {
                AFT_wrench_[3] = decodeTorque(frame.data[0], frame.data[1]);
                AFT_wrench_[4] = decodeTorque(frame.data[2], frame.data[3]);
                AFT_wrench_[5] = decodeTorque(frame.data[4], frame.data[5]);
            }
        }
    }

    Torques::Torques(const float torques[6]) {
        for (int i = 0; i < 6; i++) {
            tau_d[i] = torques[i];
        }
    }

    Forces::Forces(Eigen::Matrix<float, 6, 1> forces_ext, Eigen::Matrix<float, 6, 1> forces_imp) {
        for (int i = 0; i < 6; i++) {
            Fext[i] = forces_ext(i);
            Fimp[i] = forces_imp(i);
        }
    }

    Errors::Errors(const std::array<float, 6>& errors, const std::array<float, 6>& derrors, const std::array<float, 6>& errors_int) noexcept
        : e(errors), de(derrors), e_integral(errors_int) {}

    Errors::Errors(std::initializer_list<float> errors, std::initializer_list<float> derrors, std::initializer_list<float> errors_int) {
        if (errors.size() != e.size()) {
            throw std::invalid_argument("Invalid number of elements in position error.");
        }
        std::copy(errors.begin(), errors.end(), e.begin());

        if (derrors.size() != de.size()) {
            throw std::invalid_argument("Invalid number of elements in error derivative.");
        }
        std::copy(derrors.begin(), derrors.end(), de.begin());

        if (errors_int.size() != e_integral.size()) {
            throw std::invalid_argument("Invalid number of elements in error integral.");
        }
        std::copy(errors_int.begin(), errors_int.end(), e_integral.begin());
    }

    // Trajectory::Trajectory(const std::array<float, 6>& pos, const std::array<float, 6>& vel, const std::array<float, 6>& acc) noexcept
    //     : pos_d(pos), vel_d(vel), acc_d(acc) {}

    Trajectory::Trajectory(const std::array<float, 7>& pos, const std::array<float, 7>& vel, const std::array<float, 7>& acc) noexcept
    {
        pos_d = pos;
        vel_d = vel;
        acc_d = acc;
    }
        
    Trajectory::Trajectory(std::initializer_list<float> pos, std::initializer_list<float> vel, std::initializer_list<float> acc) {
        if (pos.size() != pos_d.size()) {
            throw std::invalid_argument("Invalid number of elements in position trajectory.");
        }
        std::copy(pos.begin(), pos.end(), pos_d.begin());

        if (vel.size() != vel_d.size()) {
            throw std::invalid_argument("Invalid number of elements in velocity trajectory.");
        }
        std::copy(vel.begin(), vel.end(), vel_d.begin());

        if (acc.size() != acc_d.size()) {
            throw std::invalid_argument("Invalid number of elements in acceleration trajectory.");
        }
        std::copy(acc.begin(), acc.end(), acc_d.begin());
    }

    Prev::Prev(const std::array<float, 6>& x, const std::array<float, 6>& v, 
               const std::array<float, 6>& F_ext, const std::array<float, 6>& derr) noexcept
               : xPrev(x), vPrev(v), F_extPrev(F_ext), derrPrev(derr) {}

    Prev::Prev(std::initializer_list<float> x, std::initializer_list<float> v,
               std::initializer_list<float> F_ext) {
        if (x.size() != xPrev.size()) {
            throw std::invalid_argument("Invalid number of elements in cartesian pose.");
        }
        std::copy(x.begin(), x.end(), xPrev.begin());

        if (v.size() != vPrev.size()) {
            throw std::invalid_argument("Invalid number of elements in cartesian velocity.");
        }
        std::copy(v.begin(), v.end(), vPrev.begin());

        if (F_ext.size() != F_extPrev.size()) {
            throw std::invalid_argument("Invalid number of elements in external force.");
        }
        // 수정된 부분: F_ext의 데이터를 F_extPrev에 복사
        std::copy(F_ext.begin(), F_ext.end(), F_extPrev.begin());
    }
    
    Desired::Desired(const std::array<float, 6>& pos) noexcept
        : q_d(pos) {}

    Desired::Desired(std::initializer_list<float> pos) {
        if (pos.size() != q_d.size()) {
            throw std::invalid_argument("Invalid number of elements in joint velocity.");
        }
        std::copy(pos.begin(), pos.end(), q_d.begin());
    }

    // ROS 노드 초기화 함수
    void initializeRosNode(int argc, char** argv) {
        ros::init(argc, argv, "control_state_node"); // ROS 노드 초기화
        Sensor_data sensor_data; // Sensor_data 객체 생성

        // ROS 이벤트 루프 실행
        ros::spin();
    }

} // namespace SKKU
