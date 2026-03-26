#include <skku_tools/impedance_controller.h>
#include <skku_tools/control_loop.h>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <cmath>
#include <boost/filesystem.hpp>
#include <cppflow/cppflow.h>
#include <cppflow/ops.h>
#include <cppflow/model.h>
#include <iostream>
#include <fstream>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <Eigen/Geometry>

cppflow::model MLP_model("/home/rbl/catkin_ws/src/skku-robot/model_RISE_250828_tf");

// add
namespace {
    constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;

    inline Eigen::Quaternionf normalizeQuat(Eigen::Quaternionf q) {
        if (q.norm() < 1e-6f) {
            return Eigen::Quaternionf::Identity();
        }
        q.normalize();
        return q;
    }

    inline Eigen::Quaternionf quatFromEulerDeg(float roll_deg, float pitch_deg, float yaw_deg) {
        Eigen::AngleAxisf rollAngle (roll_deg  * DEG2RAD, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch_deg * DEG2RAD, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle  (yaw_deg   * DEG2RAD, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        return normalizeQuat(q);
    }

    inline void alignQuatHemisphere(Eigen::Quaternionf& q, const Eigen::Quaternionf& ref) {
        if (q.coeffs().dot(ref.coeffs()) < 0.0f) {
            q.coeffs() *= -1.0f;
        }
    }
}
//new0317
namespace {
    constexpr bool kRawJacobianIsFlange = true;  // DSR 문서 확인 후 false로 바꿀 수 있음

    inline Eigen::Matrix<float, 6, 6> mapMat6(const float src[NUMBER_OF_JOINT][NUMBER_OF_JOINT]) {
        Eigen::Matrix<float, 6, 6> out;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                out(i, j) = src[i][j];
            }
        }
        return out;
    }

    inline Eigen::Matrix<float, 6, 1> mapVec6(const float* src) {
        Eigen::Matrix<float, 6, 1> out;
        for (int i = 0; i < 6; ++i) out(i) = src[i];
        return out;
    }

    inline Eigen::Matrix<float, 6, 1> mapJointVelDegToRad(const float* src_deg_s) {
        Eigen::Matrix<float, 6, 1> out;
        for (int i = 0; i < 6; ++i) {
            out(i) = src_deg_s[i] * DEG2RAD;
        }
        return out;
    }

    inline Eigen::Matrix3f skew(const Eigen::Vector3f& r) {
        Eigen::Matrix3f S;
        S <<     0.f, -r.z(),  r.y(),
              r.z(),     0.f, -r.x(),
             -r.y(),  r.x(),    0.f;
        return S;
    }

    inline Eigen::Matrix<float, 6, 6> twistShiftMatrix(const Eigen::Vector3f& r) {
        Eigen::Matrix<float, 6, 6> X = Eigen::Matrix<float, 6, 6>::Identity();
        X.block<3, 3>(0, 3) = -skew(r);
        return X;
    }

    inline Eigen::Quaternionf quatFromRotm(const float (*R)[3]) {
        Eigen::Matrix3f m;
        m << R[0][0], R[0][1], R[0][2],
             R[1][0], R[1][1], R[1][2],
             R[2][0], R[2][1], R[2][2];
        Eigen::Quaternionf q(m);
        q.normalize();
        return q;
    }

    inline Eigen::Vector3f quatLog(const Eigen::Quaternionf& q_in) {
        Eigen::Quaternionf q = q_in.normalized();
        if (q.w() < 0.0f) q.coeffs() *= -1.0f;

        const float vnorm = q.vec().norm();
        if (vnorm < 1e-8f) return Eigen::Vector3f::Zero();

        const float angle = 2.0f * std::atan2(vnorm, q.w());
        return (angle / vnorm) * q.vec();
    }

    inline Eigen::Vector3f quatLogError(const Eigen::Quaternionf& q_d,
                                        const Eigen::Quaternionf& q) {
        return quatLog(q_d * q.conjugate());
    }

    inline Eigen::Matrix<float, 6, 6> dampedPseudoInverse(
        const Eigen::Matrix<float, 6, 6>& J,
        float lambda) {
        const Eigen::Matrix<float, 6, 6> I =
            Eigen::Matrix<float, 6, 6>::Identity();
        return J.transpose() * (J * J.transpose() + lambda * lambda * I).inverse();
    }
}//


namespace SKKU
{
    namespace fs = boost::filesystem;
    bool PBIC::isFileInitialized_1 = false;
    bool PBIC::isFileInitialized_2 = false;


    
    Eigen::Matrix<float, 6, 1> PBIC::F_estimate(const Eigen::Matrix<float, 1, 6>& input_data_1, const Eigen::Matrix<float, 1, 6>& input_data_2, const Eigen::Matrix<float, 1, 6>& input_data_3)
    {
        Eigen::Matrix<float, 1, 18> input_data;

        input_data << input_data_1, input_data_2, input_data_3;

        //Whole Scaler 정보 - 학습 데이터가 바뀌면 그에 맞게 수정 필요
        Eigen::Matrix<float, 1, 18> input_scaler_min;
        input_scaler_min << -180, -180, -180, -180, -180, -180,
                            -400, -400, -400, -10000000, -10000000, -10000000,
                            -2.53435, 13.5215, 39.189, 0.139093, -0.125222, 1.70364; 
      
        Eigen::Matrix<float, 1, 18> input_scaler_max;
        input_scaler_max << 180, 180, 180, 180, 180, 180,
                            1000, 1000, 1000,10000000, 10000000, 10000000,
                            3.84662, 55.8215, 46.2397, 0.445514, 1.2106, 3.10663; 

        Eigen::Matrix<float, 1, 6> output_scaler_min;
        output_scaler_min << -82.5755, -35.2342, -102.986, -3.80781, -10.2144, -3.09124;
       
        Eigen::Matrix<float, 1, 6> output_scaler_max;
        output_scaler_max << -7.75589, 37.6976, -65.7997, 4.94948, 0.494817, -1.72246;

        ///////////////////////////////////////////////////////////////////
     
        std::vector<float> input_v(18, 0.0);
        for (int i = 0; i < 18; ++i) {
            input_data(0, i) = (input_data(0, i) - input_scaler_min(0, i)) / (input_scaler_max(0, i) - input_scaler_min(0, i));
            input_v[i] = input_data(0,i);
            
        }
    
        std::vector<int64_t> shape = {1, 18};
        cppflow::tensor input_tensor(input_v, shape);

        auto output_tensor = MLP_model(input_tensor);
        std::vector<float> output_vector = output_tensor.get_data<float>();

        Eigen::Matrix<float, 1, 6> output_data;
        for (int i = 0; i < 6; ++i) {
            output_data(0, i) = output_vector[i];
            output_data(0, i) = output_data(0, i) * (output_scaler_max(0, i) - output_scaler_min(0, i)) + output_scaler_min(0, i);
        }

        Eigen::Matrix<float, 6, 1> F_estimate = output_data.transpose();

        return F_estimate;
    }

    //new0317
    //void PBIC::appendMatrixToFile_1(const Eigen::Matrix<float, 6, 1>& matrix, const string& filename) {
    void PBIC::appendMatrixToFile_1(const Eigen::Matrix<float, 6, 1>& matrix, const std::string& filename){
        std::ofstream file;
        if (!isFileInitialized_1) {
            file.open(filename); // 파일을 처음 실행될 때만 초기화
            isFileInitialized_1 = true;
        } else {
            file.open(filename, std::ios::app); // 파일이 이미 초기화되었으면 추가 모드로 열기
        }

        if (file.is_open()) {
            file << matrix.transpose() << std::endl; // 행렬을 한 줄로 쓰기
            file.close();
        } else {
            std::cerr << "Unable to open file " << filename << std::endl;
        }
    }
    //new0317
    //void PBIC::appendMatrixToFile_2(const Eigen::Matrix<float, 6, 1>& matrix, const string& filename) {
    void PBIC::appendMatrixToFile_2(const Eigen::Matrix<float, 6, 1>& matrix, const std::string& filename){
        std::ofstream file;
        if (!isFileInitialized_2) {
            file.open(filename); // 파일을 처음 실행될 때만 초기화
            isFileInitialized_2 = true;
        } else {
            file.open(filename, std::ios::app); // 파일이 이미 초기화되었으면 추가 모드로 열기
        }

        if (file.is_open()) {
            file << matrix.transpose() << std::endl; // 행렬을 한 줄로 쓰기
            file.close();
        } else {
            std::cerr << "Unable to open file " << filename << std::endl;
        }
    }


    Eigen::Matrix<float, 6, 1> PBIC::f(Eigen::Matrix<float, 6, 1> x, Eigen::Matrix<float, 6, 1> v, Eigen::Matrix<float, 6, 1> imp_C)
    {
        Eigen::Matrix<float, 6, 1> a;
        a = M_inv * (-1 * B * v - K * x) + imp_C;
        return a;
    }

    void PBIC::rungeKutta(float t0, Eigen::Matrix<float, 6, 1> &x0, Eigen::Matrix<float, 6, 1> &v0, Eigen::Matrix<float, 6, 1> imp_C)
    {
        (void)t0;  // 현재 사용하지 않음

        Eigen::Matrix<float, 6, 1> x = x0;
        Eigen::Matrix<float, 6, 1> v = v0;

        const float h = dt / static_cast<float>(n);

        for (int i = 0; i < n; ++i)
        {
            Eigen::Matrix<float, 6, 1> k1x = v;
            Eigen::Matrix<float, 6, 1> k1v = f(x, v, imp_C);

            Eigen::Matrix<float, 6, 1> k2x = v + 0.5f * h * k1v;
            Eigen::Matrix<float, 6, 1> k2v = f(x + 0.5f * h * k1x, v + 0.5f * h * k1v, imp_C);

            Eigen::Matrix<float, 6, 1> k3x = v + 0.5f * h * k2v;
            Eigen::Matrix<float, 6, 1> k3v = f(x + 0.5f * h * k2x, v + 0.5f * h * k2v, imp_C);

            Eigen::Matrix<float, 6, 1> k4x = v + h * k3v;
            Eigen::Matrix<float, 6, 1> k4v = f(x + h * k3x, v + h * k3v, imp_C);

            x += (h / 6.0f) * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
            v += (h / 6.0f) * (k1v + 2.0f * k2v + 2.0f * k3v + k4v);
        }

        x0 = x;
        v0 = v;
    }

    void PBIC::loadConfig()
    {
        // Get the path of the current source file
        std::string currentFilePath = __FILE__;

        // Extract the directory portion of the path to get the base directory
        std::string baseDir = fs::path(currentFilePath).parent_path().string();

        std::cout << baseDir <<std::endl;
        // // Construct the relative path to the config file
        std::string yamlFilePath = baseDir + "/../config/control_gains_utf8.yaml";

        YAML::Node config = YAML::LoadFile(yamlFilePath);

        std::cout << config << std::endl;

        // controller
        K1 = config["K1"].as<std::array<float, 6>>();
        K2 = config["K2"].as<std::array<float, 6>>();
        M_hat = config["M_hat"].as<std::array<float, 6>>();
        F_offset_gain = config["F_offset_gain"].as<std::array<float, 6>>();

        // Impedance model gains (calculated based on imp_m, imp_k, imp_b)
        imp_m = config["imp_m"].as<float>();
        imp_k = config["imp_k"].as<float>();

        M_gains = {imp_m / 1000, imp_m / 1000, imp_m / 1000, imp_m / 1000, imp_m / 1000, imp_m / 1000};
        K_gains = {3*imp_k, 3*imp_k, imp_k, 0.1f*imp_k, 0.1f*imp_k, 0.1f*imp_k};
        for (int i = 0; i < 6; ++i)
        {
            // B_gains[i] = 2 * sqrt(K_gains[i] * M_gains[i]); // 2 critical dmaped
            B_gains[i] = 8 * sqrt(K_gains[i] * M_gains[i]); // 4 Overdmaped
            // B_gains[i] = 0.5 * sqrt(K_gains[i] * M_gains[i]); // 2 Underdmaped
        }
    }

    PBIC::PBIC(u_int64_t loop_time, DRAFramework::CDRFLEx &Drfl) : Drfl_(std::move(Drfl))
    {
        loadConfig();

        for (int i = 0; i < 6; i++)
        {
            M(i, i) = M_gains[i];
            B(i, i) = B_gains[i];
            K(i, i) = K_gains[i];
            F_offset(i) = F_offset_gain[i];
            K1_inv[i] = 1 / K1[i];
            K2_inv[i] = 1 / K2[i];
            M_hat_inv[i] = 1 / M_hat[i];
        }
        /*new0317

        dt = static_cast<float>(loop_time) / 1000;*/
        M_inv = M.inverse();


        // ------------------------------------------------------------
        // DBIC tuning for current z-lift free-space test
        // z translation authority를 키우고, rotation coupling은 약하게 둔다.
        // ------------------------------------------------------------
        Md_.setZero();
        Bd_.setZero();
        Kd_.setZero();


        // 행렬에 들어가는 6개의 숫자들은 차례대로 [X, Y, Z, Roll, Pitch, Yaw] 축을 의미
        Md_.diagonal() << 5.0f, 5.0f, 2.0f, 0.20f, 0.20f, 0.20f;
        Kd_.diagonal() << 400.0f, 400.0f, 400.0f, 200.0f, 200.0f, 200.0f;
        //new0326
        Bd_.diagonal() << 10.0f, 10.0f, 10.0f, 5.0f, 5.0f, 5.0f;

        //new0326
        // for (int i = 0; i < 6; ++i) {
        //     Bd_(i, i) = 2.0f * std::sqrt(Md_(i, i) * Kd_(i, i));
        // }

        // Md_inv_ = Md_.inverse();

    // // ------------------------------------------------------------
    // // DB-IC desired impedance
    // // Kang 2009의 DB-IC/제안법 비교 실험은
    // // translational desired impedance를 직접 (Md, Bd, Kd)로 설계한다.
    // // 즉 Bd를 critical damping으로 자동 계산하지 말고 직접 지정한다.
    // // ------------------------------------------------------------
    // Md_.setZero();
    // Bd_.setZero();
    // Kd_.setZero();

    // // translational axes: paper-style desired impedance
    // // Kang 2009 experiment: Md=20, Bd=900, Kd=400 (2-DOF translational)
    // // 현재 6-DOF에서는 XYZ에 같은 철학을 적용
    // Md_(0,0) = 20.0f;  Md_(1,1) = 20.0f;  Md_(2,2) = 20.0f;
    // Bd_(0,0) = 900.0f; Bd_(1,1) = 900.0f; Bd_(2,2) = 900.0f;
    // Kd_(0,0) = 400.0f; Kd_(1,1) = 400.0f; Kd_(2,2) = 400.0f;

    // // rotational axes: 현재 논문에 직접 값이 없으므로 보수적으로 둔다.
    // Md_(3,3) = 0.20f;  Md_(4,4) = 0.20f;  Md_(5,5) = 0.20f;
    // Bd_(3,3) = 10.0f;  Bd_(4,4) = 10.0f;  Bd_(5,5) = 10.0f;
    // Kd_(3,3) = 5.0f;   Kd_(4,4) = 5.0f;   Kd_(5,5) = 5.0f;

    // Md_inv_ = Md_.inverse();

        dt = static_cast<float>(loop_time) / 1000.0f;
    }

    void PBIC::start_Motion(LPRT_OUTPUT_DATA_LIST &robot_state, Prev &prev, Impedance &imp)
    {
        robot_state = Drfl_.read_data_rt();

        // Previous value setting
        std::copy(robot_state->actual_flange_position, robot_state->actual_flange_position + 6, begin(prev.xPrev));

        for (int i = 0; i < 6; i++)
        {
            imp.pos_m(i) = robot_state->actual_flange_position[i];

            imp.vel_m(i) = 0;
            imp.acc_m(i) = 0;
        }
    }

    // void PBIC::resetDBICControllerState()
    // {
    //     has_prev_J_dbic_ = false;
    //     J_prev_dbic_.setZero();
    //     Jdot_qdot_prev_.setZero();
    //     Fe_filt_dbic_.setZero();
    //     // Fe_filt.setZero(); // <--- Fe_filt 로 수정!
    // }

    void PBIC::resetDBICControllerState()
    {
        has_prev_J_dbic_ = false;
        J_prev_dbic_.setZero();
        Jdot_qdot_prev_.setZero();

        Fe_filt_dbic_.setZero();
        Fe_bias_dbic_.setZero();
        Fe_bias_accum_dbic_.setZero();
        Fe_bias_count_dbic_ = 0;
        fe_bias_ready_dbic_ = false;

        tau_prev_dbic_.setZero();
    }

    //new0317
    //new0322
    // TaskState PBIC::getTaskState(const LPRT_OUTPUT_DATA_LIST robot_state,
    //                          TaskPointMode task_point_mode,
    //                          const Eigen::Isometry3f& T_flange_tcp) {
    // TaskState s;

    // Eigen::Vector3f pF;
    // pF << robot_state->actual_flange_position[0] * 1e-3f,
    //       robot_state->actual_flange_position[1] * 1e-3f,
    //       robot_state->actual_flange_position[2] * 1e-3f;

    // float (*rotm_ptr)[3] = Drfl_.get_current_rotm();
    // Eigen::Quaternionf qF = quatFromRotm(rotm_ptr);

    // Eigen::Isometry3f T_B_F = Eigen::Isometry3f::Identity();
    // T_B_F.linear() = qF.toRotationMatrix();
    // T_B_F.translation() = pF;

    // Eigen::Isometry3f T_B_TCP = T_B_F * T_flange_tcp;
    // Eigen::Vector3f r_F_to_TCP = T_B_TCP.translation() - T_B_F.translation();

    // Eigen::Vector3f vF;
    // vF << robot_state->actual_flange_velocity[0] * 1e-3f,
    //       robot_state->actual_flange_velocity[1] * 1e-3f,
    //       robot_state->actual_flange_velocity[2] * 1e-3f;

    // Eigen::Vector3f wF;
    // wF << robot_state->actual_flange_velocity[3] * DEG2RAD,
    //       robot_state->actual_flange_velocity[4] * DEG2RAD,
    //       robot_state->actual_flange_velocity[5] * DEG2RAD;

    // if (task_point_mode == TaskPointMode::kTCP) {
    //     s.p = T_B_TCP.translation();
    //     s.q = Eigen::Quaternionf(T_B_TCP.linear());
    //     s.q.normalize();
    //     s.v = vF + wF.cross(r_F_to_TCP);
    //     s.w = wF;
    // } else {
    //     s.p = T_B_F.translation();
    //     s.q = qF;
    //     s.q.normalize();
    //     s.v = vF;
    //     s.w = wF;
    // }

    // Eigen::Matrix<float, 6, 6> J_raw = mapMat6(robot_state->jacobian_matrix);

    // if (kRawJacobianIsFlange) {
    //     if (task_point_mode == TaskPointMode::kTCP) {
    //         s.J = twistShiftMatrix(r_F_to_TCP) * J_raw;
    //     } else {
    //         s.J = J_raw;
    //     }
    // } else {
    //     if (task_point_mode == TaskPointMode::kTCP) {
    //         s.J = J_raw;
    //     } else {
    //         s.J = twistShiftMatrix(-r_F_to_TCP) * J_raw;
    //     }
    // }
    TaskState PBIC::getTaskState(const LPRT_OUTPUT_DATA_LIST robot_state,
                             TaskPointMode task_point_mode,
                             const Eigen::Isometry3f& T_flange_tcp)
{
    TaskState s;

    // ------------------------------------------------------------
    // Flange pose from robot state (position + RPY)
    // 현재는 flange 기준 실험이므로 actual_flange_* 를 일관되게 사용
    // ------------------------------------------------------------
    Eigen::Vector3f pF;
    pF << robot_state->actual_flange_position[0] * 1e-3f,
          robot_state->actual_flange_position[1] * 1e-3f,
          robot_state->actual_flange_position[2] * 1e-3f;

    Eigen::Quaternionf qF =
        quatFromEulerDeg(robot_state->actual_flange_position[3],
                         robot_state->actual_flange_position[4],
                         robot_state->actual_flange_position[5]);

    Eigen::Vector3f vF;
    vF << robot_state->actual_flange_velocity[0] * 1e-3f,
          robot_state->actual_flange_velocity[1] * 1e-3f,
          robot_state->actual_flange_velocity[2] * 1e-3f;

    Eigen::Vector3f wF;
    wF << robot_state->actual_flange_velocity[3] * DEG2RAD,
          robot_state->actual_flange_velocity[4] * DEG2RAD,
          robot_state->actual_flange_velocity[5] * DEG2RAD;

    Eigen::Matrix<float, 6, 6> J_raw = mapMat6(robot_state->jacobian_matrix);

    // 현재는 TCP가 없으므로 controller의 external_tcp_force를 flange wrench로 간주
    Eigen::Matrix<float, 6, 1> wrench_flange = Eigen::Matrix<float, 6, 1>::Zero();
    for (int i = 0; i < 6; ++i) {
        wrench_flange(i) = robot_state->external_tcp_force[i];
    }

    // ------------------------------------------------------------
    // flange mode: actual_flange_* 기준으로 끝까지 일관되게 사용
    // ------------------------------------------------------------
    if (task_point_mode == TaskPointMode::kFlange) {
        s.p = pF;
        s.q = qF;
        s.q.normalize();

        s.v = vF;
        s.w = wF;

        s.J = J_raw;
        s.F_env_on_robot = wrench_flange;

        return s;
    }

    // ------------------------------------------------------------
    // TCP mode (future use)
    // flange pose + code-side TCP offset으로 TCP state 구성
    // ------------------------------------------------------------
    Eigen::Isometry3f T_B_F = Eigen::Isometry3f::Identity();
    T_B_F.linear() = qF.toRotationMatrix();
    T_B_F.translation() = pF;

    Eigen::Isometry3f T_B_TCP = T_B_F * T_flange_tcp;
    Eigen::Vector3f r_F_to_TCP = T_B_TCP.translation() - T_B_F.translation();

    s.p = T_B_TCP.translation();
    s.q = Eigen::Quaternionf(T_B_TCP.linear());
    s.q.normalize();

    if (kRawJacobianIsFlange) {
        s.J = twistShiftMatrix(r_F_to_TCP) * J_raw;
    } else {
        s.J = J_raw;
    }

    Eigen::Matrix<float, 6, 1> qdot_task =
        mapJointVelDegToRad(robot_state->actual_joint_velocity);

    Eigen::Matrix<float, 6, 1> twist_task = s.J * qdot_task;
    s.v = twist_task.head<3>();
    s.w = twist_task.tail<3>();

    // flange wrench -> tcp wrench shift
    s.F_env_on_robot.head<3>() = wrench_flange.head<3>();
    s.F_env_on_robot.tail<3>() =
        wrench_flange.tail<3>() - r_F_to_TCP.cross(wrench_flange.head<3>());

    return s;
}

    // // ------------------------------------------------------------
    // // DBIC는 SI 단위(m, rad, m/s, rad/s)로 계산한다.
    // // actual_joint_velocity는 raw joint unit이므로 rad/s로 변환해서 사용
    // // ------------------------------------------------------------
    //     Eigen::Matrix<float, 6, 1> qdot_task =
    //     mapJointVelDegToRad(robot_state->actual_joint_velocity);

    // Eigen::Matrix<float, 6, 1> twist_task = s.J * qdot_task;

    // s.v = twist_task.head<3>();   // [m/s]
    // s.w = twist_task.tail<3>();   // [rad/s]

    // // controller에서 tool offset이 비활성이라면 external_tcp_force는 사실상 flange wrench로 본다.
    // Eigen::Matrix<float, 6, 1> wrench_flange = Eigen::Matrix<float, 6, 1>::Zero();

    // for (int i = 0; i < 6; ++i) {
    //     wrench_flange(i) = robot_state->external_tcp_force[i];
    // }

    // if (task_point_mode == TaskPointMode::kTCP) {
    //     // wrench at TCP = wrench at flange shifted to TCP
    //     s.F_env_on_robot.head<3>() = wrench_flange.head<3>();
    //     s.F_env_on_robot.tail<3>() =
    //         wrench_flange.tail<3>() - r_F_to_TCP.cross(wrench_flange.head<3>());
    // } else {
    //     s.F_env_on_robot = wrench_flange;
    // }

    // return s;
    // }
    
    Torques PBIC::ControlGeneratorDBIC(const TaskRef& ref,
                                    const LPRT_OUTPUT_DATA_LIST robot_state,
                                    TaskPointMode task_point_mode,
                                    const Eigen::Isometry3f& T_flange_tcp)
    {
        Torques torque = Torques();

        TaskState s = getTaskState(robot_state, task_point_mode, T_flange_tcp);

        Eigen::Matrix<float, 6, 6> Hhat = mapMat6(robot_state->mass_matrix);
        Eigen::Matrix<float, 6, 6> Cmat = mapMat6(robot_state->coriolis_matrix);
        Eigen::Matrix<float, 6, 1> g    = mapVec6(robot_state->gravity_torque);
        Eigen::Matrix<float, 6, 1> qdot =
            mapJointVelDegToRad(robot_state->actual_joint_velocity);
        //new0326
        Eigen::Matrix<float, 6, 1> F_int = mapVec6(robot_state->external_tcp_force);

        // ------------------------------------------------------------
        // Force measurement conditioning
        // paper에서는 calibrated force sensor를 사용하지만,
        // 현재 시스템은 raw external_tcp_force를 바로 쓰므로
        // bias 제거 + LPF를 적용해서 free-space jitter를 줄인다.
        // ------------------------------------------------------------
        Eigen::Matrix<float, 6, 1> Fe_meas = -s.F_env_on_robot;
        Eigen::Matrix<float, 6, 1> Fe_paper = Eigen::Matrix<float, 6, 1>::Zero();

        if (!fe_bias_ready_dbic_) {
            Fe_bias_accum_dbic_ += Fe_meas;
            Fe_bias_count_dbic_++;

            if (Fe_bias_count_dbic_ >= 50) {   // 약 0.2초 @ 250Hz
                Fe_bias_dbic_ = Fe_bias_accum_dbic_ / static_cast<float>(Fe_bias_count_dbic_);
                fe_bias_ready_dbic_ = true;
                Fe_filt_dbic_.setZero();
            }
        } else {
            Fe_meas -= Fe_bias_dbic_;
            Fe_filt_dbic_ = 0.05f * Fe_meas + 0.95f * Fe_filt_dbic_;
            Fe_paper = Fe_filt_dbic_;

            // 외력 = 0 experiment setting
            // Fe_paper = Eigen::Matrix<float, 6, 1>::Zero();
        }

        Eigen::Matrix<float, 6, 1> e    = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> edot = Eigen::Matrix<float, 6, 1>::Zero();

        e.head<3>()    = ref.p_d - s.p;
        edot.head<3>() = ref.v_d - s.v;

        e.tail<3>()    = quatLogError(ref.q_d, s.q);
        edot.tail<3>() = ref.w_d - s.w;
        //new0326
        // Eigen::Matrix<float, 6, 1> xdd_d = Eigen::Matrix<float, 6, 1>::Zero();
        // xdd_d.head<3>() = ref.a_d;
        // xdd_d.tail<3>() = ref.alpha_d;
        // DB-IC core
        // Eigen::Matrix<float, 6, 1> u_d =
        //     xdd_d + Md_inv_ * (Bd_ * edot + Kd_ * e - Fe_paper);
        Eigen::Matrix<float, 6, 1> F_task;
        F_task = (Bd_ * edot + Kd_ * e);
        // F_task = F_task + F_int;
        F_task = F_task;

        // ------------------------------------------------------------
        // Jdot*qdot는 raw finite difference를 그대로 쓰면 매우 noisy하므로
        // 저역통과 형태로 한 번 smoothing 한다.
        // ------------------------------------------------------------
        //new0326
        // Eigen::Matrix<float, 6, 1> Jdot_qdot = Eigen::Matrix<float, 6, 1>::Zero();
        // if (has_prev_J_dbic_) {
        //     Eigen::Matrix<float, 6, 6> Jdot = (s.J - J_prev_dbic_) / dt;
        //     Eigen::Matrix<float, 6, 1> Jdot_qdot_raw = Jdot * qdot;
        //     Jdot_qdot = 0.1f * Jdot_qdot_raw + 0.9f * Jdot_qdot_prev_;
        //     Jdot_qdot_prev_ = Jdot_qdot;
        // } else {
        //     Jdot_qdot_prev_.setZero();
        // }

        // J_prev_dbic_ = s.J;
        // has_prev_J_dbic_ = true;

        // ------------------------------------------------------------
        // exact inverse / damped inverse를 분기하면 joint-space 해가 튄다.
        // 항상 같은 형태의 damped pseudo inverse를 써서 joint coordination을 부드럽게 만든다.
        // ------------------------------------------------------------
        Eigen::Matrix<float, 6, 6> J_inv = dampedPseudoInverse(s.J, 5e-3f);

        Eigen::Matrix<float, 6, 1> Nhat = Cmat * qdot + g;
            //new0326
        // Eigen::Matrix<float, 6, 1> tau =
        //     Hhat * J_inv * (u_d - Jdot_qdot)
        //     + Nhat
        //     + s.J.transpose() * Fe_paper;
        Eigen::Matrix<float, 6, 1> tau = s.J.transpose() * F_task + Nhat;
        Eigen::Matrix<float, 6, 1> Fspring = Kd_ * e;
        Eigen::Matrix<float, 6, 1> Fdamp   = Bd_ * edot;
        Eigen::Matrix<float, 6, 1> Fdbic   = Fspring + Fdamp - Fe_paper;

        auto clampf = [](float v, float lo, float hi) {
            return (v < lo) ? lo : ((v > hi) ? hi : v);
        };

        // safety wrapper
        const std::array<float, 6> tau_rate_limit_per_sec = {
            1500.0f, 1500.0f, 1200.0f, 250.0f, 250.0f, 250.0f
        };

        for (int i = 0; i < 6; ++i) {
            F.F_DBIC[i] = Fdbic(i);
            F.F_rest[i] = Fspring(i);
            F.F_coriolis[i] = Fdamp(i);
            F.Fext[i] = s.F_env_on_robot(i);
            F.Fimp[i] = 0.0f;

            const float max_delta = tau_rate_limit_per_sec[i] * dt;
            tau(i) = clampf(tau(i),
                            tau_prev_dbic_(i) - max_delta,
                            tau_prev_dbic_(i) + max_delta);

            tau(i) = clampf(tau(i), -torque_limit[i], torque_limit[i]);

            torque.tau_d[i] = tau(i);
            tau_prev_dbic_(i) = tau(i);
        }

        return torque;
    }    
    
    Torques PBIC::ControlGenerator(Trajectory &trajectory,
                                const Desired desired,
                                const LPRT_OUTPUT_DATA_LIST robot_state,
                                Errors &error,
                                int count)
    {
        // 이 함수는 PBIC-TDC inner loop 전용이다.
        // trajectory의 Cartesian 정보는 outer loop(MotionGenerator)에서 이미 joint target으로 바뀌었으므로
        // 여기서는 직접 사용하지 않는다.
        (void)trajectory;

        std::array<float, 6> err = {0, };
        std::array<float, 6> derr = {0, };
        std::array<float, 6> err_integral = {0, };

        Eigen::Map<Eigen::Matrix<float, 6, 1>> derrPrev(prev.derrPrev.data());

        Torques torque = Torques();

        float joint[6] = {0,};
        float trq_gravity[6] = {0,};

        memcpy(joint, robot_state->actual_joint_position, sizeof(float) * 6);
        memcpy(trq_gravity, robot_state->gravity_torque, sizeof(float) * 6);

        const std::array<float, 6> torque_limits = {519.0f, 519.0f, 244.5f, 75.0f, 75.0f, 75.0f};

        for (int i = 0; i < 6; ++i)
        {
            err[i] = desired.q_d[i] - joint[i];

            // joint6 초기 branch jump 완화용 기존 로직 유지
            if (i == 5 && count <= 500) {
                float scaling_factor = static_cast<float>(count) / 500.0f;
                err[5] *= scaling_factor;
            }

            // angle wrap
            if (err[i] >= 350.0f) {
                err[i] -= 360.0f;
            } else if (err[i] <= -350.0f) {
                err[i] += 360.0f;
            }

            // derivative LPF
            derr[i] = 0.1f * ((err[i] - error.e[i]) / dt) + 0.9f * derrPrev(i);

            // integral
            err_integral[i] = error.e_integral[i] + err[i] * dt;

            // ------------------------------------------------------------------
            // PBIC-TDC inner loop only
            // DBIC torque generation은 ControlGeneratorDBIC()에서만 수행한다.
            // 여기서는 PBIC outer loop(MotionGenerator)가 만든 desired.q_d를
            // joint-space TDC/PID 로 추종한다.
            // ------------------------------------------------------------------
            torque.tau_d[i] =
                M_hat_inv[i] * K1[i] / dt *
                (err[i] + K1_inv[i] * derr[i] + K1[i] * K2_inv[i] * err_integral[i])
                + trq_gravity[i];

            // torque saturation
            if (torque.tau_d[i] > torque_limits[i]) {
                torque.tau_d[i] = torque_limits[i];
            } else if (torque.tau_d[i] < -torque_limits[i]) {
                torque.tau_d[i] = -torque_limits[i];
            }

            // 다음 스텝 derivative filter용 저장
            derrPrev(i) = derr[i];
        }

        error.e = err;
        error.de = derr;
        error.e_integral = err_integral;

        return torque;
    }

    // NOTE:
    // 이 함수는 PBIC outer loop용 IK target generator이다.
    // 현재 DBIC goal/path 실험에서는 ControlGeneratorDBIC() 경로를 사용하므로
    // 여기의 Euler dummy orientation은 현재 DBIC photo issue의 직접 원인이 아니다.
    std::pair<std::array<float, 6>, bool> PBIC::MotionGenerator(Trajectory &trajectory, const LPRT_OUTPUT_DATA_LIST robot_state, Prev &prev, Impedance &imp, int sol_space, bool correction_flag,int operator_call_count_)
    {
        static int singularity_counter = 0;
        bool is_singular = false;
        std::array<float, 6> des = {0, };
        LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> q(robot_state->actual_joint_position);
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> x(robot_state->actual_flange_position);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> xPrev(prev.xPrev.data());
        Eigen::Map<Eigen::Matrix<float, 6, 1>> vPrev(prev.vPrev.data());
        Eigen::Map<Eigen::Matrix<float, 6, 1>> qPrev(prev.qPrev.data());

        Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_raw(robot_state->actual_joint_torque);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> trq_ext(robot_state->external_joint_torque);
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_g(robot_state->gravity_torque);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> F_extPrev(prev.F_extPrev.data());
        // Eigen::Map<Eigen::Matrix<float, 6, 1>> pos(trajectory.pos_d.data());
        // Eigen::Map<Eigen::Matrix<float, 6, 1>> vel(trajectory.vel_d.data());
        // Eigen::Map<Eigen::Matrix<float, 6, 1>> acc(trajectory.acc_d.data());

        // // ✅ 물리적 크기인 7차원으로 우선 매핑
        // Eigen::Map<Eigen::Matrix<float, 7, 1>> pos(trajectory.pos_d.data());
        // Eigen::Map<Eigen::Matrix<float, 7, 1>> vel(trajectory.vel_d.data());
        // Eigen::Map<Eigen::Matrix<float, 7, 1>> acc(trajectory.acc_d.data());

        // 여기서 trajectory는 fillEulerDummyForIK()를 거친 Euler dummy trajectory입니다.
        // 따라서 앞의 6개만 [x, y, z, roll, pitch, yaw]로 사용해야 합니다.
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> pos_euler(trajectory.pos_d.data());
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> vel_euler(trajectory.vel_d.data());
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> acc_euler(trajectory.acc_d.data());

        Eigen::Matrix<float, 6, 6> JPrev;
        float qd_sing[NUMBER_OF_JOINT] = {0,};
        Eigen::Matrix<float, 6, 1> v = 0.05 * (x - xPrev) / dt + 0.95 * vPrev;
        float q_input_array[NUMBER_OF_JOINT]  = {0,};
        float trq_ext_input_array[NUMBER_OF_JOINT] = {0,};
        float task_p_input_array[NUMBER_OF_JOINT] = {0,};
        float jacobianMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
        float F_box[NUMBER_OF_JOINT] = {0,};


        memcpy(q_input_array, robot_state->actual_joint_position, sizeof(float) * 6);
        memcpy(trq_ext_input_array, robot_state->external_joint_torque, sizeof(float) * 6);
        memcpy(task_p_input_array, robot_state->target_tcp_position, sizeof(float) * 6);
        memcpy(jacobianMatrix, robot_data->jacobian_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        memcpy(F_box, robot_data->external_tcp_force, NUMBER_OF_JOINT * sizeof(float));

        Eigen::Matrix<float, 6, 6> J;

        for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
            for (int j = 0; j < NUMBER_OF_JOINT; ++j) {
                J(i,j) = jacobianMatrix[i][j];
            }
            // F_ext(i) = F_box[i];
            // 노이즈로 인한 토크 발산을 막기 위해 로우패스 필터(LPF) 적용 복구
            F_ext(i) = 0.1f * F_box[i] + 0.9f * F_extPrev(i);
        }

        Eigen::Matrix<float, 1, 6> q_input_matrix;
        Eigen::Matrix<float, 1, 6> task_p_input_matrix;
        Eigen::Matrix<float, 1, 6> trq_ext_input_matrix;

        for (int i = 0; i < 6; ++i) {
            q_input_matrix(0, i) = q_input_array[i];
            task_p_input_matrix(0, i) = task_p_input_array[i];
            trq_ext_input_matrix(0, i) = trq_ext_input_array[i];
        }

        // F_estim = F_estimate(q_input_matrix, task_p_input_matrix, trq_ext_input_matrix);
        
        // external force estimation 
        // F_ext = 0.1 * (J.transpose().inverse() * (trq_ext) - F_offset) + 0.9 * F_extPrev; // w/ Gripper 
        // F_ext = 0.1 * (J.transpose().inverse() * (trq_ext) - F_estim) + 0.9 * F_extPrev; // w/ Gripper learning
        // F_ext = 0.1 * (J.transpose().inverse() * (trq_ext)) + 0.9 * F_extPrev; // w/o Gripper 

        // F_ext = (J.transpose().inverse() * (trq_ext)) - F_offset;``````

        // Remove moment
        // F_ext[3] = 0;
        // F_ext[4] = 0;
        // F_ext[5] = 0;
        // 

        // 2. utilize force sensor
        for (int i = 0 ; i < 6 ; i++) {
            F_sensor[i] = sensor_data.AFT_wrench_[i];
        }
        
        // consider adhere frame
        float(*result)[3] = Drfl_.get_current_rotm();

        F_sensor_matched[0] = -F_sensor[0];
        F_sensor_matched[1] = -F_sensor[1];
        F_sensor_matched[2] = F_sensor[2];
        F_sensor_matched[3] = -F_sensor[3];
        F_sensor_matched[4] = -F_sensor[4];
        F_sensor_matched[5] = F_sensor[5];

        Eigen::Matrix3f rotationMatrix;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotationMatrix(i, j) = result[i][j];
            }
        }

        Eigen::Vector3f forceVector(F_sensor_matched[0], F_sensor_matched[1], F_sensor_matched[2]);
        Eigen::Vector3f torqueVector(F_sensor_matched[3], F_sensor_matched[4], F_sensor_matched[5]);

        // Step 3: Multiply the vectors by the rotation matrix
        Eigen::Vector3f rotatedForce = rotationMatrix * forceVector;
        Eigen::Vector3f rotatedTorque = rotationMatrix * torqueVector;

        F_sensor_matched[0] = rotatedForce(0);  // Rotated x-force
        F_sensor_matched[1] = rotatedForce(1);  // Rotated y-force
        F_sensor_matched[2] = rotatedForce(2);  // Rotated z-force

        F_sensor_matched[3] = rotatedTorque(0);  // Rotated x-torque
        F_sensor_matched[4] = rotatedTorque(1);  // Rotated y-torque
        F_sensor_matched[5] = rotatedTorque(2);  // Rotated z-torque
        
        // F_ext = 0.2 * F_sensor + 0.8 * F_extPrev - F_offset; // Sensor value -> External force 

        for (int i = 0 ; i < 6 ; i++) {
            sensor_data.AFT_wrench_matched[i] = F_sensor_matched[i]; 
        }
        
        Eigen::Matrix<float, 6, 1> imp_C = Eigen::Matrix<float, 6, 1>::Zero();

        // imp_C = M_inv * (M * acc + B * vel + K * pos + F_ext); // considering external force
        // // imp_C = M_inv * (M * acc + B * vel + K * pos); // Not considering external force 

        // rungeKutta(t_start, imp.pos_m, imp.vel_m, imp_C);

        // imp.acc_m = M_inv * (-1 * B * imp.vel_m - K * imp.pos_m) + imp_C;
        // F_imp = M * (acc - imp.acc_m) + B * (vel - imp.vel_m) + K * (pos - imp.pos_m);

        // ✅ .head(6)을 사용해 앞의 6칸(X,Y,Z,Roll,Pitch,Yaw)만 추출하여 연산
        // imp_C = M_inv * (M * acc.head(6) + B * vel.head(6) + K * pos.head(6) + F_ext); 

        // rungeKutta(t_start, imp.pos_m, imp.vel_m, imp_C);

        // imp.acc_m = M_inv * (-1 * B * imp.vel_m - K * imp.pos_m) + imp_C;
        
        // // ✅ 여기도 .head(6) 적용
        // F_imp = M * (acc.head(6) - imp.acc_m) + B * (vel.head(6) - imp.vel_m) + K * (pos.head(6) - imp.pos_m);

        // imp_C = M_inv * (M * acc_euler + B * vel_euler + K * pos_euler + F_ext);

        // PBIC outer impedance model
        // xdd_m = xdd_d + M^{-1}[ B(xd_dot - x_m_dot) + K(xd - x_m) - F_int ]
        // F_ext는 environment-on-robot 기준으로 사용
        imp_C = M_inv * (M * acc_euler + B * vel_euler + K * pos_euler - F_ext);

        rungeKutta(t_start, imp.pos_m, imp.vel_m, imp_C);

        imp.acc_m = M_inv * (-1 * B * imp.vel_m - K * imp.pos_m) + imp_C;

        F_imp = M * (acc_euler - imp.acc_m)
            + B * (vel_euler - imp.vel_m)
            + K * (pos_euler - imp.pos_m);

        for (int i = 0; i < 6; i++)
        {
            F.Fext[i] = F_ext(i);
            F.Fimp[i] = F_imp(i); 
        }

        // transform Eigen::Matrix to float[6]
        
        float x_d[6] = {0,};
        float x_d2[6] = {0,};
        
        Eigen::VectorXf::Map(&x_d[0], 6) = imp.pos_m; // Impedance mode
    

        // trajectory는 이미 fillEulerDummyForIK()를 거쳐
        // quaternion -> continuous Euler dummy 로 변환된 상태다.
        // 따라서 여기의 pos_euler(3..5)는 raw quaternion이 아니라
        // [roll, pitch, yaw] [deg] 값이다.
        x_d[3] = pos_euler(3);
        x_d[4] = pos_euler(4);
        x_d[5] = pos_euler(5);

        float current_joint[NUMBER_OF_JOINT] = {0,};
        memcpy(current_joint, robot_state->actual_joint_position, sizeof(float) * 6);

        // 새 motion 시작 시 branch continuity 기준을 현재 joint로 맞춤
        if (count_motion == 0) {
            memcpy(previous_joint_command,
                   robot_state->actual_joint_position,
                   sizeof(float) * 6);
        }

        // ------------------------------------------------------------------
        // IK를 하나의 solution space로만 풀지 말고,
        // 이전 command와 가장 가까운 해를 선택해서 branch jump를 줄인다.
        // ------------------------------------------------------------------
        float best_des[NUMBER_OF_JOINT] = {0,};
        bool found_solution = false;
        float best_cost = 1.0e30f;
        int best_sol_space = sol_space;

        for (int cand_sol = 0; cand_sol < 8; ++cand_sol) {
            LPINVERSE_KINEMATIC_RESPONSE cand =
                Drfl_.ikin(x_d, cand_sol, COORDINATE_SYSTEM_WORLD, 1);

            if (cand == nullptr) {
                continue;
            }

            float cost = 0.0f;
            for (int i = 0; i < 6; ++i) {
                float delta = cand->_fTargetPos[i] - previous_joint_command[i];
                while (delta > 180.0f) delta -= 360.0f;
                while (delta < -180.0f) delta += 360.0f;
                cost += delta * delta;
            }

            if (cost < best_cost) {
                best_cost = cost;
                best_sol_space = cand_sol;
                for (int i = 0; i < 6; ++i) {
                    best_des[i] = cand->_fTargetPos[i];
                }
                found_solution = true;
            }
        }

        if (!found_solution) {
            ROS_WARN("MotionGenerator: IK failed for all solution spaces. Holding previous joint command.");

            for (int i = 0; i < 6; ++i) {
                des[i] = previous_joint_command[i];
            }

            singularity_counter++;
            if (singularity_counter >= 10) {
                is_singular = true;
            }

            return {des, is_singular};
        }

        sol_space = best_sol_space;
        for (int i = 0; i < 6; ++i) {
            des[i] = best_des[i];
        }

        // ------------------------------------------------------------------
        // 여기서 보는 것은 "실제 singularity"가 아니라
        // IK branch jump(갑작스러운 해 점프)다.
        // current_joint가 아니라 previous_joint_command와 비교해야 한다.
        // ------------------------------------------------------------------
        bool branch_jump = false;
        int jump_joint = -1;
        float jump_delta = 0.0f;

        for (int i = 0; i < 6; ++i) {
            float delta = des[i] - previous_joint_command[i];
            while (delta > 180.0f) delta -= 360.0f;
            while (delta < -180.0f) delta += 360.0f;

            // 기존 20 deg는 너무 예민해서 false positive가 잘 난다.
            if (std::abs(delta) > 35.0f) {
                branch_jump = true;
                jump_joint = i;
                jump_delta = delta;
                singularity_counter++;
                break;
            }
        }

        if (branch_jump) {
            std::cout << "IK branch jump at joint " << jump_joint
                      << ", prev_cmd : " << previous_joint_command[jump_joint]
                      << ", ik_cmd : " << des[jump_joint]
                      << ", delta : " << jump_delta
                      << ", sol_space : " << best_sol_space << std::endl;
            ROS_WARN("IK branch jump detected");

            // 갑자기 다른 branch로 튀는 해는 쓰지 않고 이전 command를 유지
            for (int i = 0; i < 6; ++i) {
                des[i] = previous_joint_command[i];
            }
        } else {
            singularity_counter = 0;
        }

        if (singularity_counter >= 10) {
            ROS_WARN("IK branch jump persisted for 10 frames. Exiting motion.");
            is_singular = true;
        }

        std::copy(robot_state->actual_flange_position,
                  robot_state->actual_flange_position + 6,
                  begin(prev.xPrev));

        Eigen::VectorXf::Map(&prev.vPrev[0], 6) = imp.vel_m;
        Eigen::VectorXf::Map(&prev.F_extPrev[0], 6) = F_ext;

        for (int i = 0; i < 6; ++i) {
            previous_joint_command[i] = des[i];
        }

        count_motion++;

        return {des, is_singular};
    }

    void PBIC::printMatrixWithTabs(const Eigen::Matrix<float, 6, 6>& matrix, const std::string& name) {
    std::cout << name << ":\n";
    for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
        for (int j = 0; j < NUMBER_OF_JOINT; ++j) {
            std::cout << matrix(i, j);
            if (j < NUMBER_OF_JOINT - 1) {
                std::cout << " ";
            }
        }
        std::cout << "\n";
    }
    std::cout << std::endl;
    }


    
}