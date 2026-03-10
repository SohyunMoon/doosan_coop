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

        // Eigen::Matrix<float, 1, 6> output_data;
        // for (int i = 0; i < 6; ++i) {F
        //     output_data(0, i) = output_vector[i];
        //     output_data(0, i) = output_data(0, i) * (output_scaler_max(0, i) - output_scaler_min(0, i)) + output_scaler_min(0, i);
        // }

        return F_estimate;
    }

    void PBIC::appendMatrixToFile_1(const Eigen::Matrix<float, 6, 1>& matrix, const string& filename) {
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

    void PBIC::appendMatrixToFile_2(const Eigen::Matrix<float, 6, 1>& matrix, const string& filename) {
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
        Eigen::Matrix<float, 6, 1> x(x0);
        Eigen::Matrix<float, 6, 1> v(v0);

        float t = t0;
        float h = dt / n;

        for (int i = 1; i <= n; i++)
        {
            Eigen::Matrix<float, 6, 1> k1v = h * v0;
            Eigen::Matrix<float, 6, 1> k1a = h * f(x0, v0, imp_C);
            Eigen::Matrix<float, 6, 1> k2v = h * (v0 + 0.5 * k1a);
            Eigen::Matrix<float, 6, 1> k2a = h * f(x0 + 0.5 * k1v, v + 0.5 * k1a, imp_C);
            Eigen::Matrix<float, 6, 1> k3v = h * (v0 + 0.5 * k2a);
            Eigen::Matrix<float, 6, 1> k3a = h * f(x0 + 0.5 * k2v, v + 0.5 * k2a, imp_C);
            Eigen::Matrix<float, 6, 1> k4v = h * (v0 + k3a);
            Eigen::Matrix<float, 6, 1> k4a = h * f(x0 + k3v, v + k3a, imp_C);

            x += (k1v + 2.0 * k2v + 2.0 * k3v + k4v) / 6.0;
            v += (k1a + 2.0 * k2a + 2.0 * k3a + k4a) / 6.0;
            t += h;
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
        M_inv = M.inverse();

        dt = static_cast<float>(loop_time) / 1000;
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

    Torques PBIC::ControlGenerator(Trajectory &trajectory, const Desired desired, const LPRT_OUTPUT_DATA_LIST robot_state, Errors &error, int count)
    {   
        std::array<float, 6> err = {0, };
        std::array<float, 6> derr = {0, };
        std::array<float, 6> err_integral = {0, };
        Eigen::Matrix<float, 6, 1> trq_DBIC, F_DBIC, trq_PBIC, F_rest, F_coriolis;
        Eigen::Matrix<float, 6, 1> qdot_prev, q2dot;
        Eigen::Map<Eigen::Matrix<float, 6, 1>> derrPrev(prev.derrPrev.data());
        Torques torque = Torques();
        float joint[6] = {0,};
        float trq_gravity[6] = {0, };
        float joint_velocity[6] = {0, };
        float joint_acceleration[6] = {0, };
        float filtered_joint_acceleration[6] = {0, };
        float massMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
        float coriolisMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
        float jacobianMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
        float jacobianDotMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
        float alpha = 0.1;
    
        Eigen::Map<Eigen::Matrix<float, 6, 1>> trq_ext(robot_state->external_joint_torque);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> trq_g(robot_state->gravity_torque);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> x(robot_state->actual_flange_position);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> xdot(robot_state->actual_flange_velocity);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> q(robot_state->actual_joint_position);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> qdot(robot_state->actual_joint_velocity);
        Eigen::Map<Eigen::Matrix<float, 7, 1>> x0(trajectory.pos_d.data());
        Eigen::Map<Eigen::Matrix<float, 6, 1>> x0Dot(trajectory.vel_d.data());
        Eigen::Map<Eigen::Matrix<float, 6, 1>> x02Dot(trajectory.acc_d.data());

        memcpy(joint, robot_state->actual_joint_position, sizeof(float) * 6);
        memcpy(joint_velocity, robot_state->actual_joint_velocity, sizeof(float) * 6);
        memcpy(trq_gravity, robot_state->gravity_torque, sizeof(float) * 6);
        memcpy(jacobianMatrix, robot_state->jacobian_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        memcpy(massMatrix, robot_state->mass_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        memcpy(coriolisMatrix, robot_state->coriolis_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        std::array<float, 6> torque_limits = {519.0, 519.0, 244.5, 75.0, 75.0, 75.0}; 
        Eigen::Matrix<float, 6, 6> J, JPrev, JDot, JDot_f, C, Mass,MassPrev, P_PB, D_PB,MassxJinverse;
        Eigen::Matrix<float, 6, 1> qdDot, qdPrev, qd, trq_record, trq_record2,Coriolis,Jxqdot, Stiffness, Damping, MassTerm, qerr,qerrDot,qerrPrev, trq_stat;

        for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
            for (int j = 0; j < NUMBER_OF_JOINT; ++j) {
                J(i,j) = jacobianMatrix[i][j];
                C(i,j) = coriolisMatrix[i][j];
                Mass(i,j) = massMatrix[i][j];
            }
        }


        trq_gg = J.transpose()*F_estim; // Gripper compensation term

        // 1. P_DBIC (Cartesian Stiffness: N/m, Nm/rad)
        // 위치(X, Y, Z)는 약 400~600, 자세(R, P, Y)는 20~40 정도로 시작하는 것이 안전합니다.
        Eigen::Matrix<float, 6, 1> P_DBIC;
        // P_DBIC << 40.0f,40.0f, 40.0f,  // 위치 강성 (X, Y, Z)
        //         1.0f,  1.0f,  0.0f;   // 자세 강성 (R, P, Y)

        
        P_DBIC << 10.0f,10.0f, 10.0f,  // 위치 강성 (X, Y, Z)
                5.0f,  5.0f,  5.0f;   // 자세 강성 (R, P, Y)

        // 2. D_DBIC (Cartesian Damping: Ns/m, Nms/rad)s
        // 댐핑은 임계 댐핑(Critical Damping) 조건인 D = 2 * sqrt(K * M)을 고려해야 합니다.
        // M1013의 유효 질량을 고려했을 때 아래 값이 적절합니다.
        Eigen::Matrix<float, 6, 1> D_DBIC;
        // D_DBIC << 5.0f,  5.0f,  5.0f,   // 위치 댐핑
        //         0.1f,   0.1f,   0.0f;    // 자세 댐핑

        D_DBIC << 1.0f,  1.0f,  1.0f,   // 위치 댐핑
                0.0f,   0.0f,   0.0f;    // 자세 댐핑
                
        qerr = qd-q;
        Coriolis = C*qdot; 


        // 1. 현재 로봇의 자세를 쿼터니언으로 변환 (실제 로봇 피드백 x는 여전히 RPY/deg 기준일 때)
        Eigen::AngleAxisf rollAngle(x(3) * M_PI / 180.0f, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(x(4) * M_PI / 180.0f, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(x(5) * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf q_actual = yawAngle * pitchAngle * rollAngle;

        // 2. [수정] 목표 자세를 x0(3~6)에서 직접 쿼터니언으로 생성
        // Eigen::Quaternionf constructor 순서는 (w, x, y, z)입니다.
        // x0 mapping: 3=x, 4=y, 5=z, 6=w (TrajectoryGen::init에서 보낸 순서)
        Eigen::Quaternionf q_desired(x0(6), x0(3), x0(4), x0(5));
        q_desired.normalize(); // 수치적 안정성을 위해 정규화 수행

        // 3. 쿼터니언 오차 계산
        Eigen::Matrix<float, 6, 1> error_x;
        error_x.head(3) = x0.head(3) - x.head(3); // 위치 오차 (X, Y, Z)

        // [중요] Antipodal 보정 (최단 경로 선택)
        if (q_desired.coeffs().dot(q_actual.coeffs()) < 0.0f) {
            q_actual.coeffs() *= -1.0f; 
        }

        // 쿼터니언 기반 오차 벡터 추출 (Vector-based Orientation Error)
        // q_error = q_actual^-1 * q_desired
        Eigen::Quaternionf q_error(q_actual.inverse() * q_desired);
        
        // Franka Control 등에서 사용하는 표준 오차 벡터 방식 적용
        // 이 방식이 Roll/Pitch/Yaw보다 훨씬 안정적인 제어를 보장합니다.
        error_x.tail(3) << 2.0f * (q_actual * q_error.vec());


        // 4. 오차 미분 및 필터링 (LPF 적용)
        Eigen::Matrix<float, 6, 1> derr_x;
        static Eigen::Matrix<float, 6, 1> error_x_prev = error_x;
        static Eigen::Matrix<float, 6, 1> derr_x_filtered = Eigen::Matrix<float, 6, 1>::Zero();

        // 실험 시작 시 초기화 (Folder 1416 튀는 문제 해결)
        if (count == 0) {
            error_x_prev = error_x;
            derr_x_filtered.setZero();
        }

        derr_x = (error_x - error_x_prev) / dt;
        float alpha_filter = 0.15f; 
        derr_x_filtered = alpha_filter * derr_x + (1.0f - alpha_filter) * derr_x_filtered;
        error_x_prev = error_x;

        // 5. Task Force 및 최종 토크 계산 (그리퍼 보상 포함)
        Eigen::Matrix<float, 6, 1> F_task;
        for(int i = 0; i < 6; i++) {
            F_task(i) = P_DBIC(i) * error_x(i) + D_DBIC(i) * derr_x_filtered(i);
        }

        F_task += F_ext; // 외력 보상
        Eigen::Matrix<float, 6, 1> tau_task = J.transpose() * F_task;

        // 7. Nullspace 제어 (로봇의 자세 유지 - Franka 코드의 필수 요소)
        // 6-DOF라도 특이점 근처나 관절 한계 근처에서 안정성을 위해 사용
        Eigen::Matrix<float, 6, 1> tau_nullspace;
        Eigen::Matrix<float, 6, 6> I = Eigen::Matrix<float, 6, 6>::Identity();
        Eigen::Matrix<float, 6, 6> J_inv = J.inverse(); // Pseudo-inverse 권장
        
        // 관절 강성(k_null)을 아주 작게 주어 현재 자세를 유지하려 함
        float k_null = 0.5; 
        tau_nullspace = (I - J.transpose() * J_inv.transpose()) * (k_null * (q - q)); // q_d 대신 현재 q 유지

        // for(int i=3; i<6; i++) error_x(i) = x_d(i) - x(i);

        trq_DBIC = tau_task + Coriolis + trq_g;
        // trq_DBIC = trq_g;
        
        
        for (int i = 0; i < 6; i++)
        {   
            err[i] = desired.q_d[i] - joint[i];

            if (i == 5 && count <= 500) {
                float scaling_factor = static_cast<float>(count) / 500.0f;
                // std::cout << "Scaling factor: " << scaling_factor << std::endl;
                err[5] *= scaling_factor;
            }
        
            if (err[i] >= 350.0) {
                err[i] -= 360.0;
            } else if (err[i] <= -350.0) {
                err[i] += 360.0;
            }
     
            derr[i] = 0.1 * ((err[i] - error.e[i]) / dt) + 0.9 * derrPrev[i]; 
            err_integral[i] = error.e_integral[i] + err[i] * dt;
    
            //PBIC w/ TDC-based PID controller
            // torque.tau_d[i] = M_hat_inv[i] * K1[i] / dt * (err[i] + K1_inv[i] * derr[i] + K1[i] * K2_inv[i] * err_integral[i])+trq_gravity[i]-trq_gg[i]; // w/ Gripper
            // torque.tau_d[i] = M_hat_inv[i] * K1[i] / dt * (err[i] + K1_inv[i] * derr[i] + K1[i] * K2_inv[i] * err_integral[i]) + trq_gravity[i]; // w/o Gripper
            
            //DBIC 
            torque.tau_d[i] = trq_DBIC[i];

            //PBIC
            // torque.tau_d[i] = trq_PBIC[i];

            // Static
            // torque.tau_d[i] = trq_stat[i];
asdsdasdasdas

            // torque saturation 
            if (torque.tau_d[i] > torque_limits[i]) {
                torque.tau_d[i] = torque_limits[i];
            } else if (torque.tau_d[i] < -torque_limits[i]) {
                torque.tau_d[i] = -torque_limits[i];
            }
    
        }

        error.e = err;
        error.de = derr;
        error.e_integral = err_integral;
        JPrev = J;
        MassPrev = Mass;
        qdot_prev = qdot;
        qdPrev = qd;
        qerrPrev = qerr;
        // Control Input

        return torque;
    }

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
        Eigen::Map<Eigen::Matrix<float, 6, 1>> pos(trajectory.pos_d.data());
        Eigen::Map<Eigen::Matrix<float, 6, 1>> vel(trajectory.vel_d.data());
        Eigen::Map<Eigen::Matrix<float, 6, 1>> acc(trajectory.acc_d.data());
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
            F_ext(i) = F_box[i];
        }

        Eigen::Matrix<float, 1, 6> q_input_matrix;
        Eigen::Matrix<float, 1, 6> task_p_input_matrix;
        Eigen::Matrix<float, 1, 6> trq_ext_input_matrix;

        for (int i = 0; i < 6; ++i) {
            q_input_matrix(0, i) = q_input_array[i];
            task_p_input_matrix(0, i) = task_p_input_array[i];
            trq_ext_input_matrix(0, i) = trq_ext_input_array[i];
        }

        F_estim = F_estimate(q_input_matrix, task_p_input_matrix, trq_ext_input_matrix);
        
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

        imp_C = M_inv * (M * acc + B * vel + K * pos + F_ext); // considering external force
        // imp_C = M_inv * (M * acc + B * vel + K * pos); // Not considering external force 

        rungeKutta(t_start, imp.pos_m, imp.vel_m, imp_C);

        imp.acc_m = M_inv * (-1 * B * imp.vel_m - K * imp.pos_m) + imp_C;
        F_imp = M * (acc - imp.acc_m) + B * (vel - imp.vel_m) + K * (pos - imp.pos_m);

        for (int i = 0; i < 6; i++)
        {
            F.Fext[i] = F_ext(i);
            F.Fimp[i] = F_imp(i); 
        }

        // transform Eigen::Matrix to float[6]
        
        float x_d[6] = {0,};
        float x_d2[6] = {0,};
        
        Eigen::VectorXf::Map(&x_d[0], 6) = imp.pos_m; // Impedance mode
        
        // to check the SAFE CASES
        x_d[3] = trajectory.pos_d[3];
        x_d[4] = trajectory.pos_d[4];

        x_d[5] = trajectory.pos_d[5];

        float current_joint[NUMBER_OF_JOINT] = {0,};
        memcpy(current_joint, robot_state->actual_joint_position, sizeof(float) * 6);

        LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(x_d, 2, COORDINATE_SYSTEM_WORLD, 1);


        std::copy(res->_fTargetPos, res->_fTargetPos + 6, begin(des));


        // // deal with 6 joint ambiguity 
        // float delta_angle = des[5] - current_joint[5];
        // if (delta_angle > 100.0) {
        //     std::cout << ": Adjusting by -180 degrees. "
        //         << "Original des: " << des[5] << ", Current joint: " << current_joint[5]
        //         << ", Delta angle: " << delta_angle << std::endl;
        //     des[5] -= 180.0;
        // } else if (delta_angle < -100.0) {
        //     std::cout << ": Adjusting by +180 degrees. "
        //         << "Original des: " << des[5] << ", Current joint: " << current_joint[5]
        //         << ", Delta angle: " << delta_angle << std::endl;
        //     des[5] += 180.0;
        // }

        
        // qd calculation complete 
        bool singularity = false;
        bool reversed = false;


        if (count_motion == 0) {
            memcpy(previous_joint_command, robot_state->actual_joint_position, sizeof(float) * 6);
        }

        
        for (int i = 0; i < 6; i++) {

            float delta = des[i] - current_joint[i];

            // // check singularity
            // if (std::abs(delta) > 20) {
            //     singularity = true;
            //     std::cout<< "Singularity occured at "<<i<<"th joint, previous joint command : "<<current_joint[i]<<", desired joint command : "<<des[i]<< std::endl; 
            //     ROS_INFO("SINGULARITY OCCURED");
            //     singularity_counter++;
            //     break;
            // }

        }

        if (singularity_counter >= 10) {
            ROS_WARN("SINGULARITY PERSISTED FOR 10 FRAMES, EXITING FUNCTION.");
            is_singular = true;
        }

        // if (!singularity) {
        //     singularity_counter = 0;
        // }


        // Adjust for singularity or first motion
        if (singularity && !reversed) {
            for (int i = 0; i < 6; i++)
            {
                des[i] = current_joint[i];
            } 
        }
        
        std::copy(robot_state->actual_flange_position, robot_state->actual_flange_position + 6, begin(prev.xPrev));

        Eigen::VectorXf::Map(&prev.vPrev[0], 6) = imp.vel_m;
        Eigen::VectorXf::Map(&prev.F_extPrev[0], 6) = F_ext;
        
 
        for (int i = 0; i < 6; i++)
        {
            previous_joint_command[i] = des[i];
        } 

        count_motion ++;
    
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