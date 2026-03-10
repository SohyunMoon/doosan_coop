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
        /*
        Prev* prev;
        Errors* error;
        Desired* desired;
        Torques* torque;
        LPImpedance imp = new Impedance;
        */
        int Frequency;

        std::array<float, 6> torque_limit = {519.0, 519.0, 244.5, 75.0, 75.0, 75.0};

        // string MATLAB_filepath = "/home/rbl/catkin_ws/data/F_est_by_MATLAB.txt";
        // string Tensorflow_filepath = "/home/rbl/catkin_ws/data/F_est_by_Tensorflow.txt";
        static bool isFileInitialized_1;
        static bool isFileInitialized_2;

    
    };

}