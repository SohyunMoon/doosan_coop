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

    // ==================================================================================
    // external joint torque 의 과거 샘플을 들고 있는 링버퍼.
    // MLP2 는 현재 토크와 함께 1 / 3 / 5 샘플 전 토크도 입력으로 받는다
    // (학습 시 external_torque_lag{1,3,5}.txt = external_torque.txt 를 그만큼 민 것).
    //
    // "샘플" 은 이 버퍼에 push 하는 주기 = 그 스레드의 제어/로깅 주기다.
    // 학습 데이터가 기록된 주기와 push 주기가 다르면 lag 이 뜻하는 시간이 달라진다.
    //
    // 스레드마다 각자 인스턴스를 가져야 한다 (공유하면 push 순서가 섞인다).
    // ==================================================================================
    class TorqueLagBuffer {
    public:
        static const int kDepth = 6;   // lag5 까지 쓰므로 최소 6칸

        void reset() {
            count_ = 0;
            head_  = 0;
        }

        void push(const Eigen::Matrix<float, 1, 6>& v) {
            buf_[head_] = v;
            head_ = (head_ + 1) % kDepth;
            if (count_ < kDepth) {
                ++count_;
            }
        }

        // lag = 0 이면 가장 최근에 push 한 값.
        // 아직 그만큼 쌓이지 않았으면 가장 오래된 값을 돌려준다 (기동 직후 몇 샘플만 해당).
        Eigen::Matrix<float, 1, 6> get(int lag) const {
            if (count_ == 0) {
                return Eigen::Matrix<float, 1, 6>::Zero();
            }
            if (lag > count_ - 1) {
                lag = count_ - 1;
            }
            const int idx = ((head_ - 1 - lag) % kDepth + kDepth) % kDepth;
            return buf_[idx];
        }

    private:
        Eigen::Matrix<float, 1, 6> buf_[kDepth];
        int head_  = 0;
        int count_ = 0;
    };

    // ==================================================================================
    // 임피던스 제어에 쓸 외력(F_ext)을 어디서 가져올지 고르는 스위치.
    // 아래 IMPEDANCE_FORCE_SOURCE 값만 바꾸고 다시 빌드하면 된다.
    //
    //   SENSOR : AFT 힘/토크 센서의 matched wrench. 기존 거동이고 기본값이다.
    //   MLP1   : F_estimate()  가 추정한 값 (모델 1)
    //   MLP2   : F_estimate2() 가 추정한 값 (모델 2)
    //
    // 어느 모드든 F_mlp.txt / F_mlp2.txt 로그는 항상 기록되므로, 제어에 넣기 전에
    // sensor_FT_matched.txt 와 겹쳐 그려서 모델 정확도를 먼저 확인할 수 있다.
    //
    // 주의 1) SENSOR 가 아닌 모드에서는 실시간 제어 스레드 안에서 TF 추론이 돌아간다.
    //         실측 지연이 median 54us / p99 145us 인데 드물게 2ms 까지 튄다.
    //         제어 주기를 넘길 수 있으니 loop_times.txt 를 반드시 같이 확인할 것.
    // 주의 2) 모델은 sensor_FT_matched(센서가 볼 값) 자체를 예측하도록 학습돼 있다.
    //         즉 "센서 대체" 용도이지 "그리퍼 보상량"이 아니다.
    // ==================================================================================
    enum class ImpedanceForceSource {
        SENSOR = 0,
        MLP1   = 1,
        MLP2   = 2,
    };

    // 260805: MLP1 으로 전환. 되돌리려면 SENSOR 로 바꾸면 된다.
    //
    // 주의: 260805/1937 데이터 기준으로 두 모델 다 Fz 에 큰 상수 offset 이 있다.
    //   MLP1  Fz  MAE 8.56 N,  bias -8.02 N   (오차의 대부분이 offset)
    //   MLP2  Fz  MAE 10.29 N, bias -9.95 N
    // fz_target 이 5 N 인데 offset 이 그보다 크므로, 접촉 판정과 Fz 보정이
    // 그대로는 맞지 않는다. 실기 투입 전에 영점을 확인할 것.
    // (analyze_mlp_models.py 로 F_mlp.txt 와 sensor_FT_matched.txt 를 비교)
    //
    // MLP1 을 고른 이유: 전체 평균은 MLP2 가 근소하게 낫지만(33.2% vs 34.1%),
    // 접촉 제어에서 가장 중요한 Fz 는 MLP1 이 16.8% 낫다.
    constexpr ImpedanceForceSource IMPEDANCE_FORCE_SOURCE = ImpedanceForceSource::MLP1;

    // ==================================================================================
    // IK 목표를 어디서 가져올지 고르는 스위치. 여기 한 곳에서만 바꾼다.
    //
    //   IMPEDANCE  : 외력을 받아 임피던스 모델(M/B/K)을 rungeKutta 로 적분한 결과를
    //                IK 목표로 쓴다. 기존 동작이고, 접촉하면 순응한다.
    //   TRAJECTORY : 임피던스 모델을 건너뛰고 명령 궤적을 그대로 IK 한다.
    //                순수 위치제어. 외력에 순응하지 않는다.
    //
    // TRAJECTORY 에서는 rungeKuttaPoseQuaternion() 을 호출하지 않으므로
    // M/B/K 게인과 외력이 목표 위치에 전혀 영향을 주지 않는다.
    // (외력은 여전히 읽고 로그에도 남지만 목표를 바꾸지 않는다)
    //
    // 주의: 위치제어는 접촉해도 물러나지 않는다. 표면에 닿은 상태에서 명령이
    //       더 내려가면 힘이 그대로 올라간다. 그리기 실험에 쓸 때는
    //       fz_adapt_enable 로 z 보정을 켜 두거나, z 를 넉넉히 띄워서 시작할 것.
    //       (Fz 적응 보정은 p_d 단계에 걸리므로 이 모드에서도 살아 있다)
    // ==================================================================================
    enum class MotionReferenceSource {
        IMPEDANCE = 0,
        TRAJECTORY = 1,
    };

    constexpr MotionReferenceSource MOTION_REFERENCE_SOURCE =
        MotionReferenceSource::IMPEDANCE;
        // MotionReferenceSource::TRAJECTORY;

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
    
        // 모델 1 : q [deg] 6개 + external joint torque [Nm] 6개 = 12 입력 -> Fx Fy Fz Mx My Mz
        Eigen::Matrix<float, 6, 1> F_estimate(const Eigen::Matrix<float, 1, 6>& q_deg, const Eigen::Matrix<float, 1, 6>& trq_ext);
        // 모델 2 (model_SKKU_260720_lag135_tf) : 36 입력
        //   q [deg] 6 + task position [mm,deg] 6 + external joint torque [Nm] 6
        //   + 같은 토크의 1 / 3 / 5 샘플 전 값 각 6
        // 과거 토크는 호출하는 쪽이 TorqueLagBuffer 로 들고 있다가 넘긴다.
        // 함수를 순수하게 유지해야 로깅 스레드와 RT 스레드가 각자의 이력으로 안전하게 쓸 수 있다.
        Eigen::Matrix<float, 6, 1> F_estimate2(const Eigen::Matrix<float, 1, 6>& q_deg,
                                               const Eigen::Matrix<float, 1, 6>& task_p,
                                               const Eigen::Matrix<float, 1, 6>& trq_ext,
                                               const Eigen::Matrix<float, 1, 6>& trq_ext_lag1,
                                               const Eigen::Matrix<float, 1, 6>& trq_ext_lag3,
                                               const Eigen::Matrix<float, 1, 6>& trq_ext_lag5);
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
