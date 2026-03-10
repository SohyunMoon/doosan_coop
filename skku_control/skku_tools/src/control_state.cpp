#include <stdexcept>
#include <type_traits>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <skku_tools/control_state.h>

namespace SKKU {

    // Sensor_data 클래스의 생성자
    Sensor_data::Sensor_data() {
        // ROS 노드 핸들 초기화 및 콜백 함수 연결
        sensor_sub_ = nh_.subscribe("/sensor_data", 10, &Sensor_data::sensorDataCallback, this);
    }

    // 센서 데이터를 반환하는 함수
    std::array<float, 6> Sensor_data::getAFTWrench() const {
        return AFT_wrench_;
    }

    // ROS 콜백 함수: 센서 데이터 업데이트
    void Sensor_data::sensorDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        if (msg->data.size() >= 6) {
            for (int i = 0; i < 6; ++i) {
                AFT_wrench_[i] = msg->data[i];
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

    Trajectory::Trajectory(const std::array<float, 6>& pos, const std::array<float, 6>& vel, const std::array<float, 6>& acc) noexcept
        : pos_d(pos), vel_d(vel), acc_d(acc) {}
    
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
