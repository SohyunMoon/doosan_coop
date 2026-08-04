/*
#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <DRFLEx.h>
#include <yaml-cpp/yaml.h>
#include <skku_tools/control_state.h>
#include <cppflow/cppflow.h>
#include <cppflow/ops.h>
#include <cppflow/model.h>


namespace SKKU {

    class PBIC{
    public:
        PBIC(u_int64_t loop_time, DRAFramework::CDRFLEx& Drfl);
        virtual ~PBIC() {
        
        
        }
        //void startMotion(LPRT_OUTPUT_DATA_LIST& robot_state, Desired& desired, Prev& prev, LPImpedance& impedance, Torques& Torque, Errors& error, Trajectory& trajectory);
        std::pair<std::array<float, 6>, bool> MotionGenerator(Trajectory& trajectory, const LPRT_OUTPUT_DATA_LIST robot_state, Prev& prev, Impedance& impedance, int sol_space, bool correction_flag, int operator_call_count_);
        
        Torques ControlGenerator(Trajectory& trajectory,const Desired desired, const LPRT_OUTPUT_DATA_LIST robot_state, Errors& error, int count);
        
        void rungeKutta(float t_start, Eigen::Matrix<float, 6, 1>& x, Eigen::Matrix<float, 6, 1>& v, Eigen::Matrix<float, 6, 1> imp_C);
        
        Eigen::Matrix<float, 6, 1> f(Eigen::Matrix<float, 6, 1> x, Eigen::Matrix<float, 6, 1> v, Eigen::Matrix<float, 6, 1> imp_C);
        
        void start_Motion(LPRT_OUTPUT_DATA_LIST& robot_state, Prev& prev, Impedance& impedance);
        
        Forces F;
    
        Eigen::Matrix<float, 6, 1> F_estimate(const Eigen::Matrix<float, 1, 6>& input_data_1, const Eigen::Matrix<float, 1, 6>& input_data_2, const Eigen::Matrix<float, 1, 6>& input_data_3);
        void appendMatrixToFile_1(const Eigen::Matrix<float, 6, 1>& matrix, const string& filename);
        void appendMatrixToFile_2(const Eigen::Matrix<float, 6, 1>& matrix, const string& filename);
    
        Eigen::Matrix<float, 6, 1> trq_gg = Eigen::Matrix<float, 6, 1>::Constant(0.0);
        DRAFramework::CDRFLEx Drfl_;
        void printMatrixWithTabs(const Eigen::Matrix<float, 6, 6>& matrix, const std::string& name);
        float previous_joint_command[NUMBER_OF_JOINT] = {0,};
        float previous_joint_velocity[NUMBER_OF_JOINT] = {0,};
        int count_motion = 0;
        bool reversed = false;
        float correctAngleWraparound(float current_x_d[NUMBER_OF_JOINT], float previous_x_d[NUMBER_OF_JOINT], int size);
        Sensor_data sensor_data;
        int count = 0;

    protected:
        float dt;
  
        Prev prev;
        Errors errors;
        Impedance imp;
        Eigen::Matrix<float, 6, 1> F_ext;
        Eigen::Matrix<float, 6, 1> F_sensor;
        Eigen::Matrix<float, 6, 1> F_imp;
        Eigen::Matrix<float, 6, 1> F_ext_2;


    private:
        void loadConfig();
        
        // Gains for PID controller
        // std::array<float, 6> K1 = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0};
        // std::array<float, 6> K2 = {90.0, 90.0, 90.0, 5.0, 5.0, 5.0};
        // std::array<float, 6> M_hat = {450.0, 450.0, 450.0, 1500.0, 1500.0, 2000.0};
        std::array<float, 6> K1;
        std::array<float, 6> K2;
        std::array<float, 6> M_hat;
   
        std::array<float, 6> M_hat_inv;
        std::array<float, 6> K1_inv;
        std::array<float, 6> K2_inv;

        // Gains for Impedance control
        std::array<float, 6> M_gains;
        std::array<float, 6> B_gains;
        std::array<float, 6> K_gains;
        std::array<float, 6> F_offset_gain;

        Eigen::Matrix<float, 6, 6> M = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 6> B = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 6> K = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 6> M_inv;

        Eigen::Matrix<float, 6, 1> F_tool;
        Eigen::Matrix<float, 6, 1> F_estim;
        Eigen::Matrix<float, 6, 1> F_offset;
        Eigen::Matrix<float, 6, 1> F_sensor_matched;


        //

        float imp_m;
        float imp_k;
        float imp_b; 
                
        int n = 10;
        float t_start = 0.0;
        
        //Prev* prev;
        //Errors* error;
        //Desired* desired;
        //Torques* torque;
        //LPImpedance imp = new Impedance;
        
        int Frequency;

        std::array<float, 6> torque_limit = {519.0, 519.0, 244.5, 75.0, 75.0, 75.0};

        // string MATLAB_filepath = "/home/rbl/catkin_ws/data/F_est_by_MATLAB.txt";
        // string Tensorflow_filepath = "/home/rbl/catkin_ws/data/F_est_by_Tensorflow.txt";
        static bool isFileInitialized_1;
        static bool isFileInitialized_2;

    
    };

}

*/
//new0317
#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <array>
#include <string>
#include <utility>
#include <eigen3/Eigen/Geometry>

#include <DRFLEx.h>
#include <yaml-cpp/yaml.h>
#include <skku_tools/control_state.h>
#include <cppflow/cppflow.h>
#include <cppflow/ops.h>
#include <cppflow/model.h>


namespace SKKU {

    class PBIC{
    public:
        PBIC(u_int64_t loop_time, DRAFramework::CDRFLEx& Drfl);
        virtual ~PBIC() {
        
        
        }
        //new0410
        // ---------------- PBIC / legacy branch ----------------
        // std::pair<std::array<float, 6>, bool> MotionGenerator(
        //     Trajectory& trajectory,
        //     const LPRT_OUTPUT_DATA_LIST robot_state,
        //     Prev& prev,
        //     Impedance& impedance,
        //     int sol_space,
        //     bool correction_flag,
        //     int operator_call_count_);
        std::pair<std::array<float, 6>, bool> MotionGenerator(
            Trajectory& trajectory,
            const LPRT_OUTPUT_DATA_LIST robot_state,
            Prev& prev,
            Impedance& impedance,
            int& sol_space,
            bool correction_flag,
            int operator_call_count_,
            TaskPointMode task_point_mode,
            const Eigen::Isometry3f& T_flange_tcp);
            //
        Torques ControlGenerator(
            Trajectory& trajectory,
            const Desired desired,
            const LPRT_OUTPUT_DATA_LIST robot_state,
            Errors& error,
            int count);

        // ---------------- DBIC branch ----------------
        TaskState getTaskState(
            const LPRT_OUTPUT_DATA_LIST robot_state,
            TaskPointMode task_point_mode,
            const Eigen::Isometry3f& T_flange_tcp);

        Torques ControlGeneratorDBIC(
            const TaskRef& ref,
            const LPRT_OUTPUT_DATA_LIST robot_state,
            TaskPointMode task_point_mode,
            const Eigen::Isometry3f& T_flange_tcp);

        void rungeKutta(float t_start,
                        Eigen::Matrix<float, 6, 1>& x,
                        Eigen::Matrix<float, 6, 1>& v,
                        Eigen::Matrix<float, 6, 1> imp_C);
        //0609
        bool preselectPBICInitialSolution(
            const LPRT_OUTPUT_DATA_LIST robot_state,
            int& sol_space);
        //
        Eigen::Matrix<float, 6, 1> f(Eigen::Matrix<float, 6, 1> x, Eigen::Matrix<float, 6, 1> v, Eigen::Matrix<float, 6, 1> imp_C);
        
        void start_Motion(LPRT_OUTPUT_DATA_LIST& robot_state, Prev& prev, Impedance& impedance);
        void resetDBICControllerState();


        Forces F;
        float pbic_ik_jump_log = 0.0f;
        // 0804 Fz adaptive z-reference shaping log
        // [dz(mm), dz_dot(mm/s), |Fz|_filt(N), force error(N),
        //  nominal z ref(mm), adapted z ref(mm)]
        float pbic_fz_adapt_log_[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    
        Eigen::Matrix<float, 6, 1> F_estimate(const Eigen::Matrix<float, 1, 6>& input_data_1, const Eigen::Matrix<float, 1, 6>& input_data_2, const Eigen::Matrix<float, 1, 6>& input_data_3);
        void appendMatrixToFile_1(const Eigen::Matrix<float, 6, 1>& matrix, const std::string& filename);
        void appendMatrixToFile_2(const Eigen::Matrix<float, 6, 1>& matrix, const std::string& filename);
    
        Eigen::Matrix<float, 6, 1> trq_gg = Eigen::Matrix<float, 6, 1>::Constant(0.0);
        DRAFramework::CDRFLEx Drfl_;
        void printMatrixWithTabs(const Eigen::Matrix<float, 6, 6>& matrix, const std::string& name);
        float previous_joint_command[NUMBER_OF_JOINT] = {0,};
        float previous_joint_velocity[NUMBER_OF_JOINT] = {0,};
        int count_motion = 0;
        bool reversed = false;
        float correctAngleWraparound(float current_x_d[NUMBER_OF_JOINT], float previous_x_d[NUMBER_OF_JOINT], int size);
        Sensor_data sensor_data;
        int count = 0;
        void getGainLogValues(float M_log[6],
                            float B_log[6],
                            float K_log[6],
                            float K1_log[6],
                            float K2_log[6],
                            float M_hat_log[6]) const;

    protected:
        float dt;
  
        Prev prev;
        Errors errors;
        Impedance imp;
        Eigen::Matrix<float, 6, 1> F_ext;
        Eigen::Matrix<float, 6, 1> F_sensor;
        Eigen::Matrix<float, 6, 1> F_imp;
        Eigen::Matrix<float, 6, 1> F_ext_2;


    private:
        void loadConfig();
        
        // Gains for PID controller
        // std::array<float, 6> K1 = {10.0, 10.0, 10.0, 1.0, 1.0, 1.0};
        // std::array<float, 6> K2 = {90.0, 90.0, 90.0, 5.0, 5.0, 5.0};
        // std::array<float, 6> M_hat = {450.0, 450.0, 450.0, 1500.0, 1500.0, 2000.0};
        std::array<float, 6> K1;
        std::array<float, 6> K2;
        std::array<float, 6> M_hat;
   
        std::array<float, 6> M_hat_inv;
        std::array<float, 6> K1_inv;
        std::array<float, 6> K2_inv;

        // Gains for Impedance control
        std::array<float, 6> M_gains;
        std::array<float, 6> B_gains;
        std::array<float, 6> K_gains;
        std::array<float, 6> F_offset_gain;

        Eigen::Matrix<float, 6, 6> M = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 6> B = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 6> K = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 6> M_inv;

        Eigen::Matrix<float, 6, 1> F_tool;
        Eigen::Matrix<float, 6, 1> F_estim;
        Eigen::Matrix<float, 6, 1> F_offset;
        Eigen::Matrix<float, 6, 1> F_sensor_matched;

        // ---------------- DBIC desired impedance ----------------
        Eigen::Matrix<float, 6, 6> Md_ = Eigen::Matrix<float, 6, 6>::Identity();
        Eigen::Matrix<float, 6, 6> Bd_ = Eigen::Matrix<float, 6, 6>::Identity();
        Eigen::Matrix<float, 6, 6> Kd_ = Eigen::Matrix<float, 6, 6>::Identity();
        Eigen::Matrix<float, 6, 6> Md_inv_ = Eigen::Matrix<float, 6, 6>::Identity();

        // Eigen::Matrix<float, 6, 6> J_prev_dbic_ = Eigen::Matrix<float, 6, 6>::Zero();
        // Eigen::Matrix<float, 6, 1> Jdot_qdot_prev_ = Eigen::Matrix<float, 6, 1>::Zero();
        // Eigen::Matrix<float, 6, 1> Fe_filt_dbic_ = Eigen::Matrix<float, 6, 1>::Zero();
        // bool has_prev_J_dbic_ = false;
        // Eigen::Matrix<float, 6, 1> tau_prev_dbic_ = Eigen::Matrix<float, 6, 1>::Zero();
        // // Eigen::Matrix<float, 6, 1> Fe_filt = Eigen::Matrix<float, 6, 1>::Zero(); // <--- 이 줄을 반드시 추가하세요!

        Eigen::Matrix<float, 6, 6> J_prev_dbic_ = Eigen::Matrix<float, 6, 6>::Zero();
        Eigen::Matrix<float, 6, 1> Jdot_qdot_prev_ = Eigen::Matrix<float, 6, 1>::Zero();

        // DBIC force conditioning
        Eigen::Matrix<float, 6, 1> Fe_filt_dbic_ = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> Fe_bias_dbic_ = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> Fe_bias_accum_dbic_ = Eigen::Matrix<float, 6, 1>::Zero();
        int Fe_bias_count_dbic_ = 0;
        bool fe_bias_ready_dbic_ = false;

        bool has_prev_J_dbic_ = false;
        Eigen::Matrix<float, 6, 1> tau_prev_dbic_ = Eigen::Matrix<float, 6, 1>::Zero();
        bool has_prev_task_vel_dbic_ = false;
        Eigen::Vector3f prev_task_v_dbic_ = Eigen::Vector3f::Zero();
        Eigen::Vector3f prev_task_w_dbic_ = Eigen::Vector3f::Zero();
        bool has_prev_edot_dbic_ = false;
        Eigen::Matrix<float, 6, 1> prev_edot_dbic_ =
        Eigen::Matrix<float, 6, 1>::Zero();
        bool has_xdd_actual_lpf_dbic_ = false;
        Eigen::Matrix<float, 6, 1> xdd_actual_filt_dbic_ =
            Eigen::Matrix<float, 6, 1>::Zero();
        bool has_prev_qdot_dbic_ = false;
        Eigen::Matrix<float, 6, 1> prev_qdot_dbic_ =
            Eigen::Matrix<float, 6, 1>::Zero();

        Eigen::Matrix<float, 6, 1> qddot_filt_dbic_ =
            Eigen::Matrix<float, 6, 1>::Zero();
        //0609
        bool pbic_initial_ik_preselected_ = false;
        //
        // -------------------------------------------------------

        float imp_m;
        float imp_k;
        float imp_b;

        // 0804 Fz adaptive z-reference shaping — 힘 오차에 대한 PID.
        // 출력이 z reference offset [mm] 자체다.
        //
        //   e  = |Fz| - fz_target_
        //   dz = fz_kp_*e + fz_ki_*(적분 e) + fz_kd_*(e의 변화율)
        //
        // fz_kp_ = fz_kd_ = 0 이면 순수 적분(기존 동작)과 동일.
        // dz 크기 제한 없음, 접촉 게이팅 없음, deadband 없음.
        // 값은 control_gains_utf8.yaml에서 덮어쓴다 (키가 없으면 아래 기본값 유지).
        bool  fz_adapt_enable_ = true;
        float fz_target_ = 5.0f;              // [N]     유지할 접촉력
        float fz_kp_ = 0.0f;                  // [mm/N]      즉각 순응 (유효 강성 완화)
        float fz_ki_ = 10.0f;                 // [mm/(s*N)]  정상상태 오차 제거
        float fz_kd_ = 0.0f;                  // [mm*s/N]    오버슛 완충
        float fz_adapt_rate_ = 100.0f;        // [mm/s]  dz slew limit (안전용)
        float fz_adapt_cutoff_hz_ = 2.0f;     // [Hz]    |Fz| LPF
        float fz_d_cutoff_hz_ = 5.0f;         // [Hz]    D항 미분 LPF
        float fz_print_hz_ = 5.0f;            // [Hz]    터미널 출력 주기 (0이면 끔)
        // 로봇이 실제로 안 따라오면(보호정지/서보오프) 루프가 열려서 적분기가
        // 무한정 감긴다. imp_z와 실제 TCP z의 차이로 그 상태를 잡는다.
        float fz_stall_limit_ = 50.0f;        // [mm]    0이면 검사 끔

        int n = 10;
        float t_start = 0.0;
        
        //Prev* prev;
        //Errors* error;
        //Desired* desired;
        //Torques* torque;
        //LPImpedance imp = new Impedance;
        
        int Frequency;

        // std::array<float, 6> torque_limit = {519.0, 519.0, 244.5, 75.0, 75.0, 75.0};
        // 실기 안전용 보수적 제한값
        // 로그상 J2 안전 한계는 346Nm, J3 안전 한계는 163Nm 근처로 보였으므로
        // 그보다 충분히 낮게 시작한다.
        std::array<float, 6> torque_limit = {250.0f, 300.0f, 120.0f, 40.0f, 40.0f, 40.0f};

        // string MATLAB_filepath = "/home/rbl/catkin_ws/data/F_est_by_MATLAB.txt";
        // string Tensorflow_filepath = "/home/rbl/catkin_ws/data/F_est_by_Tensorflow.txt";
        static bool isFileInitialized_1;
        static bool isFileInitialized_2;

    
    };

}
