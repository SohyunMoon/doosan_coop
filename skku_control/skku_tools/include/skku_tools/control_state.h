/*#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <vector>
#include <array>
#include <cmath>
#include <initializer_list>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

enum class RealtimeConfig { kEnforce, kIgnore };

namespace SKKU {

struct finishable {
    bool motion_finished = false;
};

// 기존 클래스 정의
class Torques : public finishable {
public:
    Torques() {};
    Torques(const float torques[6]);
    float tau_d[6] = {0};
};

class Forces : public finishable {
public:
    Forces() {};
    Forces(Eigen::Matrix<float, 6, 1> forces_ext, Eigen::Matrix<float, 6, 1> forces_imp);
    float Fext[6] = {0};
    float Fimp[6] = {0};
    float F_DBIC[6] = {0};
    float F_PBIC[6] = {0};
    float F_rest[6] = {0};
    float F_coriolis[6] = {0};
};

class Errors {
public:
    Errors() {};
    Errors(const std::array<float, 6>& errors, const std::array<float, 6>& derrors, const std::array<float, 6>& errors_int) noexcept;
    Errors(std::initializer_list<float> errors, std::initializer_list<float> derrors, std::initializer_list<float> errors_int);
    std::array<float, 6> e = {0.0};
    std::array<float, 6> de = {0.0};
    std::array<float, 6> e_integral = {0.0};
};

// class Trajectory {
// public:
//     Trajectory() {};
//     Trajectory(const std::array<float, 6>& pos, const std::array<float, 6>& vel, const std::array<float, 6>& acc) noexcept;
//     Trajectory(std::initializer_list<float> pos, std::initializer_list<float> vel, std::initializer_list<float> acc);
//     std::array<float, 6> pos_d = {0.0};
//     std::array<float, 6> vel_d = {0.0};
//     std::array<float, 6> acc_d = {0.0};
// };

// [수정 후] -> 전부 7로 변경!
class Trajectory {
public:
    Trajectory() {};
    Trajectory(const std::array<float, 7>& pos, const std::array<float, 7>& vel, const std::array<float, 7>& acc) noexcept;
    Trajectory(std::initializer_list<float> pos, std::initializer_list<float> vel, std::initializer_list<float> acc);
    std::array<float, 7> pos_d = {0.0};
    std::array<float, 7> vel_d = {0.0};
    std::array<float, 7> acc_d = {0.0};
};

class Total_trajectory {
public:
    Total_trajectory() {};
    Total_trajectory(const std::vector<float>& pos, const std::vector<float>& vel, const std::vector<float>& acc) noexcept;
    Total_trajectory(std::initializer_list<float> pos, std::initializer_list<float> vel, std::initializer_list<float> acc);
    std::vector<std::vector<float>> pos_d = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0));
    std::vector<std::vector<float>> vel_d = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0));
    std::vector<std::vector<float>> acc_d = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0));
};

class Prev {
public:
    Prev() {};
    Prev(const std::array<float, 6>& x, const std::array<float, 6>& v, 
         const std::array<float, 6>& F_ext, const std::array<float, 6>& derr) noexcept;
    Prev(const std::array<float, 6>& x, const std::array<float, 6>& v,
         const std::array<float, 6>& F_ext) noexcept;
    Prev(std::initializer_list<float> x, std::initializer_list<float> v,
         std::initializer_list<float> F_ext);
    std::array<float, 6> qPrev = {0.0};
    std::array<float, 6> xPrev = {0.0};
    std::array<float, 6> vPrev = {0.0};
    std::array<float, 6> F_extPrev = {0.0};
    std::array<float, 6> derrPrev = {0.0};
};

class Desired : public finishable {
public:
    Desired() {};
    Desired(const std::array<float, 6>& pos) noexcept;
    Desired(std::initializer_list<float> pos);
    std::array<float, 6> q_d = {0.0};
};

// Impedance 구조체 정의
typedef struct _Impedance {
    Eigen::Matrix<float, 6, 1> pos_m = Eigen::Matrix<float, 6, 1>::Constant(0.0);
    Eigen::Matrix<float, 6, 1> vel_m = Eigen::Matrix<float, 6, 1>::Constant(0.0);
    Eigen::Matrix<float, 6, 1> acc_m = Eigen::Matrix<float, 6, 1>::Constant(0.0);
} Impedance, *LPImpedance;

// Sensor_data 클래스를 추가하여 AFT_wrench를 관리하고 ROS 메시지 구독
class Sensor_data {
public:
    Sensor_data();

    // AFT_wrench 데이터를 반환하는 함수
    std::array<float, 6> getAFTWrench() const;
    std::array<float, 6> AFT_wrench_ = {0, 0, 0, 0, 0, 0};
    std::array<float, 6> AFT_wrench_matched = {0, 0, 0, 0, 0, 0};

private:
    // AFT_wrench 데이터를 저장하는 멤버 변수

    // ROS 관련 멤버 변수
    ros::NodeHandle nh_;
    ros::Subscriber sensor_sub_;

    // ROS 콜백 함수: 센서 데이터 업데이트
    void sensorDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
};

} // namespace SKKU

#endif // CONTROL_STATE_H
*/ 
//new0317
#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include <vector>
#include <array>
#include <cmath>
#include <initializer_list>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

enum class RealtimeConfig { kEnforce, kIgnore };

namespace SKKU {

struct finishable {
    bool motion_finished = false;
};

// 기존 클래스 정의
class Torques : public finishable {
public:
    Torques() {};
    Torques(const float torques[6]);
    float tau_d[6] = {0};
};

class Forces : public finishable {
public:
    Forces() {};
    Forces(Eigen::Matrix<float, 6, 1> forces_ext, Eigen::Matrix<float, 6, 1> forces_imp);
    float Fext[6] = {0};
    float Fimp[6] = {0};
    float F_DBIC[6] = {0};
    float F_PBIC[6] = {0};
    float F_rest[6] = {0};
    float F_coriolis[6] = {0};
    float F_task[6] = {0};
    float error[6] = {0};
    float error_dot[6] = {0};
};

class Errors {
public:
    Errors() {};
    Errors(const std::array<float, 6>& errors, const std::array<float, 6>& derrors, const std::array<float, 6>& errors_int) noexcept;
    Errors(std::initializer_list<float> errors, std::initializer_list<float> derrors, std::initializer_list<float> errors_int);
    std::array<float, 6> e = {0.0};
    std::array<float, 6> de = {0.0};
    std::array<float, 6> e_integral = {0.0};
};

// class Trajectory {
// public:
//     Trajectory() {};
//     Trajectory(const std::array<float, 6>& pos, const std::array<float, 6>& vel, const std::array<float, 6>& acc) noexcept;
//     Trajectory(std::initializer_list<float> pos, std::initializer_list<float> vel, std::initializer_list<float> acc);
//     std::array<float, 6> pos_d = {0.0};
//     std::array<float, 6> vel_d = {0.0};
//     std::array<float, 6> acc_d = {0.0};
// };
//new0412
// [수정 후] -> 전부 7로 변경!
// class Trajectory {
// public:
//     Trajectory() {};
//     Trajectory(const std::array<float, 7>& pos, const std::array<float, 7>& vel, const std::array<float, 7>& acc) noexcept;
//     Trajectory(std::initializer_list<float> pos, std::initializer_list<float> vel, std::initializer_list<float> acc);
//     std::array<float, 7> pos_d = {0.0};
//     std::array<float, 7> vel_d = {0.0};
//     std::array<float, 7> acc_d = {0.0};
// };
class Trajectory {
public:
    Trajectory() {};
    Trajectory(const std::array<float, 7>& pos,
               const std::array<float, 7>& vel,
               const std::array<float, 7>& acc) noexcept;
    Trajectory(std::initializer_list<float> pos,
               std::initializer_list<float> vel,
               std::initializer_list<float> acc);

    // pose = [x(mm), y(mm), z(mm), qx, qy, qz, qw]
    std::array<float, 7> pos_d = {0.0f};

    // legacy translation derivatives
    std::array<float, 7> vel_d = {0.0f};
    std::array<float, 7> acc_d = {0.0f};

    // quaternion-based rotational motion
    std::array<float, 3> w_d = {0.0f, 0.0f, 0.0f};        // [rad/s]
    std::array<float, 3> alpha_d = {0.0f, 0.0f, 0.0f};    // [rad/s^2]
};

class Total_trajectory {
public:
    Total_trajectory() {};
    Total_trajectory(const std::vector<float>& pos, const std::vector<float>& vel, const std::vector<float>& acc) noexcept;
    Total_trajectory(std::initializer_list<float> pos, std::initializer_list<float> vel, std::initializer_list<float> acc);
    std::vector<std::vector<float>> pos_d = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0));
    std::vector<std::vector<float>> vel_d = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0));
    std::vector<std::vector<float>> acc_d = std::vector<std::vector<float>>(1, std::vector<float>(1, 0.0));
};

class Prev {
public:
    Prev() {};
    Prev(const std::array<float, 6>& x, const std::array<float, 6>& v, 
         const std::array<float, 6>& F_ext, const std::array<float, 6>& derr) noexcept;
    Prev(const std::array<float, 6>& x, const std::array<float, 6>& v,
         const std::array<float, 6>& F_ext) noexcept;
    Prev(std::initializer_list<float> x, std::initializer_list<float> v,
         std::initializer_list<float> F_ext);
    std::array<float, 6> qPrev = {0.0};
    std::array<float, 6> xPrev = {0.0};
    std::array<float, 6> vPrev = {0.0};
    std::array<float, 6> F_extPrev = {0.0};
    std::array<float, 6> derrPrev = {0.0};
};

class Desired : public finishable {
public:
    Desired() {};
    Desired(const std::array<float, 6>& pos) noexcept;
    Desired(std::initializer_list<float> pos);
    std::array<float, 6> q_d = {0.0};
};
enum class TaskPointMode {
    kTCP,
    kFlange
};

enum class ImpedanceImplMode {
    kPBIC_TDC,
    kDBIC
};

struct TaskRef {
    Eigen::Vector3f p_d = Eigen::Vector3f::Zero();        // [m]
    Eigen::Vector3f v_d = Eigen::Vector3f::Zero();        // [m/s]
    Eigen::Vector3f a_d = Eigen::Vector3f::Zero();        // [m/s^2]

    Eigen::Quaternionf q_d = Eigen::Quaternionf::Identity();
    Eigen::Vector3f w_d = Eigen::Vector3f::Zero();        // [rad/s]
    Eigen::Vector3f alpha_d = Eigen::Vector3f::Zero();    // [rad/s^2]

    bool motion_finished = false;
};

struct TaskState {
    Eigen::Vector3f p = Eigen::Vector3f::Zero();          // [m]
    Eigen::Vector3f v = Eigen::Vector3f::Zero();          // [m/s]

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f w = Eigen::Vector3f::Zero();          // [rad/s]

    Eigen::Matrix<float, 6, 6> J = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, 6, 1> F_env_on_robot = Eigen::Matrix<float, 6, 1>::Zero(); // [N; Nm]
};
// Impedance 구조체 정의
//new0412
// typedef struct _Impedance {
//     Eigen::Matrix<float, 6, 1> pos_m = Eigen::Matrix<float, 6, 1>::Constant(0.0);
//     Eigen::Matrix<float, 6, 1> vel_m = Eigen::Matrix<float, 6, 1>::Constant(0.0);
//     Eigen::Matrix<float, 6, 1> acc_m = Eigen::Matrix<float, 6, 1>::Constant(0.0);
// } Impedance, *LPImpedance;
typedef struct _Impedance {
    // translational impedance state [mm]
    Eigen::Vector3f p_m = Eigen::Vector3f::Zero();
    Eigen::Vector3f v_m = Eigen::Vector3f::Zero();
    Eigen::Vector3f a_m = Eigen::Vector3f::Zero();

    // rotational impedance state
    Eigen::Quaternionf q_m = Eigen::Quaternionf::Identity();
    Eigen::Vector3f w_m = Eigen::Vector3f::Zero();        // [rad/s]
    Eigen::Vector3f alpha_m = Eigen::Vector3f::Zero();    // [rad/s^2]

    // legacy mirror for logging / compatibility
    // pos_m = [x, y, z, z1, y, z2] in [mm, deg]
    // vel_m / acc_m tail = angular components in [deg/s], [deg/s^2] for logging only
    Eigen::Matrix<float, 6, 1> pos_m = Eigen::Matrix<float, 6, 1>::Constant(0.0f);
    Eigen::Matrix<float, 6, 1> vel_m = Eigen::Matrix<float, 6, 1>::Constant(0.0f);
    Eigen::Matrix<float, 6, 1> acc_m = Eigen::Matrix<float, 6, 1>::Constant(0.0f);
} Impedance, *LPImpedance;
//
// Sensor_data 클래스를 추가하여 AFT_wrench를 관리하고 ROS 메시지 구독
// class Sensor_data {
// public:
//     Sensor_data();

//     // AFT_wrench 데이터를 반환하는 함수
//     std::array<float, 6> getAFTWrench() const;
//     std::array<float, 6> AFT_wrench_ = {0, 0, 0, 0, 0, 0};
//     std::array<float, 6> AFT_wrench_matched = {0, 0, 0, 0, 0, 0};

// private:
//     // AFT_wrench 데이터를 저장하는 멤버 변수

//     // ROS 관련 멤버 변수
//     ros::NodeHandle nh_;
//     ros::Subscriber sensor_sub_;

//     // ROS 콜백 함수: 센서 데이터 업데이트
//     void sensorDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
// };

class Sensor_data {
public:
    Sensor_data();

    // raw sensor wrench 반환
    std::array<float, 6> getAFTWrench() const;

    // frame matching 완료된 wrench 반환
    std::array<float, 6> getMatchedAFTWrench() const;

    // raw -> matched 변환 수행
    std::array<float, 6> matchAFTWrench(const Eigen::Matrix3f& rotationMatrix);

    std::array<float, 6> AFT_wrench_ = {0, 0, 0, 0, 0, 0};
    std::array<float, 6> AFT_wrench_matched = {0, 0, 0, 0, 0, 0};

private:
    ros::NodeHandle nh_;
    ros::Subscriber sensor_sub_;

    // ROS 콜백 함수: raw sensor data 업데이트
    void sensorDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
};

} // namespace SKKU

#endif // CONTROL_STATE_H