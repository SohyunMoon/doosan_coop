/*#pragma once

#include <cmath>
#include <functional>
#include <stdint.h>
#include <thread>
#include <skku_tools/duration.h>
#include <skku_tools/impedance_controller.h>
#include <skku_tools/control_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/CartesianTrajectory.h>
#include <../../include/skku_control/dsr_hw_interface.h>


extern bool controlState; 
namespace SKKU {
extern int fail;
extern int operator_call_count_; 
extern bool isDirectoryCreated;
extern std::string dataDirectory;

class TrajectoryGen{
    public:

        int getFrameIndex(const std::string& tracked_frame, size_t prev_msg_size) {
            static const std::unordered_map<std::string, int> frame_map = {
                {"position_goal", 0},
                {"impedance_goal", 0},
                {"position_path", static_cast<int>(prev_msg_size) - 1},
                {"impedance_path", static_cast<int>(prev_msg_size) - 1}
            };

            auto it = frame_map.find(tracked_frame);
            if (it != frame_map.end()) {
                return it->second;  // 해당 프레임이 맵에 있으면 인덱스 반환
            } else {
                return -1;  // 없는 경우 -1 반환
            }
        }

        float adjustAngle(float angle) {
            // 각도를 [-180, 180] 범위로 보정하는 함수
            while (angle < -180.0) {
                angle += 360.0;
            }
            while (angle > 180.0) {
                angle -= 360.0;
            }
            return angle;
        }
        void setLoopTime(uint64_t loop_time) {
            loop_time_ = loop_time;
        }
    
        struct PlanParam {
            float time;

            float ps[7];
            float vs[7];
            float as[7];
            float pf[7];
            float vf[7];
            float af[7];

            float A0[7];
            float A1[7];
            float A2[7];
            float A3[7];
            float A4[7];
            float A5[7];
        };

        struct TraParam {
            float time;

            float pos[7] = {0.0,};
            float vel[7] = {0.0,};
            float acc[7] = {0.0,};
        };

        void TrajectoryPlan(PlanParam* plan);
        void TrajectoryGenerator(PlanParam* plan, TraParam* tra);
        void init(moveit_msgs::CartesianTrajectory msg, moveit_msgs::CartesianTrajectory prev_msg,float current_position[NUMBER_OF_JOINT],int operator_call_count_);

        // std::vector<std::array<double, 6>> quadraticInterpolation(const std::vector<std::array<double, 6>>& points, int newPointsNum);
        // std::vector<std::array<double, 6>> upsampleTrajectory(const moveit_msgs::CartesianTrajectory& msg, int newPointsNum);

        // std::vector<std::array<double, 6>> CurvePoints_;

        // [수정 후] -> 7차원 궤적(쿼터니언)을 담을 수 있도록 전부 7로 변경!
        std::vector<std::array<double, 7>> quadraticInterpolation(const std::vector<std::array<double, 7>>& points, int newPointsNum);
        std::vector<std::array<double, 7>> upsampleTrajectory(const moveit_msgs::CartesianTrajectory& msg, int newPointsNum);

        std::vector<std::array<double, 7>> CurvePoints_;

        DRAFramework::CDRFLEx Drfl_;

    protected:
        u_int64_t loop_time_;

    private:
   
};

class ControlLoop : protected PBIC{
    
    struct SchedSetting {
        int policy;
        sched_param params;
    };

    public:
        static moveit_msgs::CartesianTrajectory previous_msg;
        ControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl);
        virtual ~ControlLoop();

        bool setScheduling(const SchedSetting& setting) {
            if (pthread_setschedparam(pthread_self(), setting.policy, &setting.params) != 0) {
                perror("Failed to set scheduling settings");
                return false;
            }
            return true;
        }

        bool getCurrentScheduling(SchedSetting& setting) {
            if (pthread_getschedparam(pthread_self(), &setting.policy, &setting.params) != 0) {
                perror("Failed to get current scheduling settings");
                return false;
            }
            return true;
        }

        virtual void operator()(const moveit_msgs::CartesianTrajectory& msg) = 0; // Make it a pure virtual function
        virtual void operator_path(const moveit_msgs::CartesianTrajectory& msg) = 0; // Make it a pure virtual function
        // virtual void operator_jpath(const moveit_msgs::CartesianTrajectory& msg) = 0; // Make it a pure virtual function

        void resetToInitialPosition();
        void StateCheckingThread(ControlLoop* controlLoop);
        void convertToArray(const std::array<float, 6>& stdArray, float floatArray[6]);
        void convertToArray(const std::array<float, 7>& stdArray, float floatArray[7]); // <-- 이 줄 추가!
        void logData(const std::string& fileName, const float* data, int dataSize); 
        void logMatrixData(const std::string& fileName, const float matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT], int rows, int cols);
        void logMatrixData3x3(const std::string& fileName, const float matrix[3][3], int rows, int cols);
        void dataSaving();
        void startDataSaving();
        void stopDataSaving();
        void gaindataSavingThread();
        void returnToHome();
        void init(); 
        void GainMove();
        void createNewDataDirectory();
        void alignPitchOrientation(float current_joint_position[NUMBER_OF_JOINT], const moveit_msgs::CartesianTrajectory& msg);
        void waitForMotionCompletion(float target_joint[NUMBER_OF_JOINT], float tolerance = 1); 
        void waitForMotionCompletionWithRetry(float target_joint[NUMBER_OF_JOINT], float tolerance, int timeout_ms, int max_retries);
        void saveLoopTimesToFile(const std::string& filePath);
    protected:
        bool spinMotion(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Desired& desire,int sol_space);
        bool spinMotion_path(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Desired& desired, int sol_space); 
        bool spinControl(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Torques& control_command, Desired& desired, int sol_space);
        int finishMotion();
        int cancelMotion();
        std::string control_mode_;
        uint32_t motion_id_ = 0;
        Total_trajectory* total_trajectory_ = new Total_trajectory;
        RealtimeConfig realtimeconfig_;
        SchedSetting originalSetting_;
        u_int64_t loop_time_;
        int count = 0;
        pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
        Desired desired;
        Trajectory trajectory;
        float initial_joint_position[NUMBER_OF_JOINT];
        bool exitLoop = false;
        bool gaincheckloop = false;
        Torques control_command;
        bool truncate = false;
        TrajectoryGen trajectory_gen_;
        std::array<float, 6> torque_limit = {519.0, 519.0, 244.5, 75.0, 75.0, 75.0};
        Impedance imp;
        float previous_velocityj[NUMBER_OF_JOINT] = {0,};
        float old_p[NUMBER_OF_JOINT] = {0,};
        float old_v[NUMBER_OF_JOINT] = {0,};

        std::thread data_saving_thread_;
        std::atomic<bool> data_saving_running_{false}; // flag for data saving
        // moveit_msgs::CartesianTrajectory previous_msg;
        int sol_space = 0; 
        int saving_count = 0;
      

    private:

};

class ImpedanceControlLoop : public ControlLoop {
public:
    ImpedanceControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl);
    ~ImpedanceControlLoop();
    void operator()(const moveit_msgs::CartesianTrajectory& msg) override;
    void operator_path(const moveit_msgs::CartesianTrajectory& msg) override;
    void adjustSolutionSpace();
    
};

class PositionControlLoop : public ControlLoop {
public:
    PositionControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl);
    ~PositionControlLoop();
    void operator()(const moveit_msgs::CartesianTrajectory& msg) override;
    void operator_path(const moveit_msgs::CartesianTrajectory& msg) override;
    void operator_jpath(const moveit_msgs::CartesianTrajectory& msg);
};

}*/

//new0317
#pragma once

#include <cmath>
#include <functional>
#include <stdint.h>
#include <thread>
#include <array>
#include <string>
#include <vector>
#include <unordered_map>

#include <eigen3/Eigen/Geometry>

#include <skku_tools/duration.h>
#include <skku_tools/impedance_controller.h>
#include <skku_tools/control_tools.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/CartesianTrajectory.h>
#include <../../include/skku_control/dsr_hw_interface.h>


extern bool controlState; 
namespace SKKU {
extern int fail;
extern int operator_call_count_; 
extern bool isDirectoryCreated;
extern std::string dataDirectory;

class TrajectoryGen{
    public:

        int getFrameIndex(const std::string& tracked_frame, size_t prev_msg_size) {
            static const std::unordered_map<std::string, int> frame_map = {
                {"position_goal", 0},
                {"impedance_goal", 0},
                {"position_path", static_cast<int>(prev_msg_size) - 1},
                {"impedance_path", static_cast<int>(prev_msg_size) - 1}
            };

            auto it = frame_map.find(tracked_frame);
            if (it != frame_map.end()) {
                return it->second;  // 해당 프레임이 맵에 있으면 인덱스 반환
            } else {
                return -1;  // 없는 경우 -1 반환
            }
        }

        float adjustAngle(float angle) {
            // 각도를 [-180, 180] 범위로 보정하는 함수
            while (angle < -180.0) {
                angle += 360.0;
            }
            while (angle > 180.0) {
                angle -= 360.0;
            }
            return angle;
        }
        void setLoopTime(uint64_t loop_time) {
            loop_time_ = loop_time;
        }
    
        struct PlanParam {
            float time;

            float ps[7];
            float vs[7];
            float as[7];
            float pf[7];
            float vf[7];
            float af[7];

            float A0[7];
            float A1[7];
            float A2[7];
            float A3[7];
            float A4[7];
            float A5[7];
        };

        struct TraParam {
            float time;

            float pos[7] = {0.0,};
            float vel[7] = {0.0,};
            float acc[7] = {0.0,};
        };
        // ---------------- DBIC 전용 trajectory sampler ----------------
        enum class DBICMode {
            kNone,
            kGoal,
            kPath
        };

        void initDBICGoal(const Eigen::Vector3f& p0_m,
                          const Eigen::Quaternionf& q0,
                          const moveit_msgs::CartesianTrajectory& msg);

        void initDBICPath(const Eigen::Vector3f& p0_m,
                          const Eigen::Quaternionf& q0,
                          const moveit_msgs::CartesianTrajectory& msg,
                          double dt_sec);

        TaskRef sampleDBICGoal(double t_sec, double dt_sec) const;
        TaskRef sampleDBICPath(size_t idx) const;

        DBICMode dbic_mode_ = DBICMode::kNone;
        std::vector<TaskRef> dbic_path_samples_;        

        void TrajectoryPlan(PlanParam* plan);
        void TrajectoryGenerator(PlanParam* plan, TraParam* tra);
        void init(moveit_msgs::CartesianTrajectory msg, moveit_msgs::CartesianTrajectory prev_msg,float current_position[NUMBER_OF_JOINT],int operator_call_count_);

        // std::vector<std::array<double, 6>> quadraticInterpolation(const std::vector<std::array<double, 6>>& points, int newPointsNum);
        // std::vector<std::array<double, 6>> upsampleTrajectory(const moveit_msgs::CartesianTrajectory& msg, int newPointsNum);

        // std::vector<std::array<double, 6>> CurvePoints_;

        // [수정 후] -> 7차원 궤적(쿼터니언)을 담을 수 있도록 전부 7로 변경!
        std::vector<std::array<double, 7>> quadraticInterpolation(const std::vector<std::array<double, 7>>& points, int newPointsNum);
        std::vector<std::array<double, 7>> upsampleTrajectory(const moveit_msgs::CartesianTrajectory& msg, int newPointsNum);

        std::vector<std::array<double, 7>> CurvePoints_;

        DRAFramework::CDRFLEx Drfl_;

    protected:
        u_int64_t loop_time_;

    private:
        Eigen::Vector3f dbic_p0_ = Eigen::Vector3f::Zero();
        Eigen::Vector3f dbic_pf_ = Eigen::Vector3f::Zero();
        Eigen::Quaternionf dbic_q0_ = Eigen::Quaternionf::Identity();
        Eigen::Quaternionf dbic_qf_ = Eigen::Quaternionf::Identity();
        double dbic_T_ = 0.0;    
   
};

class ControlLoop : protected PBIC{
    
    struct SchedSetting {
        int policy;
        sched_param params;
    };

    public:
        static moveit_msgs::CartesianTrajectory previous_msg;
        ControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl);
        virtual ~ControlLoop();

        bool setScheduling(const SchedSetting& setting) {
            if (pthread_setschedparam(pthread_self(), setting.policy, &setting.params) != 0) {
                perror("Failed to set scheduling settings");
                return false;
            }
            return true;
        }

        bool getCurrentScheduling(SchedSetting& setting) {
            if (pthread_getschedparam(pthread_self(), &setting.policy, &setting.params) != 0) {
                perror("Failed to get current scheduling settings");
                return false;
            }
            return true;
        }

        virtual void operator()(const moveit_msgs::CartesianTrajectory& msg) = 0;
        virtual void operator_path(const moveit_msgs::CartesianTrajectory& msg) = 0;

        void setImpedanceImplMode(ImpedanceImplMode mode) { impedance_impl_mode_ = mode; }
        void setTaskPointMode(TaskPointMode mode) { task_point_mode_ = mode; }
        void setToolTransform(const Eigen::Isometry3f& T_flange_tcp) { T_flange_tcp_ = T_flange_tcp; }
        // virtual void operator_jpath(const moveit_msgs::CartesianTrajectory& msg) = 0; // Make it a pure virtual function

        void resetToInitialPosition();
        void StateCheckingThread(ControlLoop* controlLoop);
        void convertToArray(const std::array<float, 6>& stdArray, float floatArray[6]);
        void convertToArray(const std::array<float, 7>& stdArray, float floatArray[7]); // <-- 이 줄 추가!
        void logData(const std::string& fileName, const float* data, int dataSize); 
        void logMatrixData(const std::string& fileName, const float matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT], int rows, int cols);
        void logMatrixData3x3(const std::string& fileName, const float matrix[3][3], int rows, int cols);
        void dataSaving();
        void startDataSaving();
        void stopDataSaving();
        void gaindataSavingThread();
        void returnToHome();
        void init(); 
        void GainMove();
        void createNewDataDirectory();
        void alignPitchOrientation(float current_joint_position[NUMBER_OF_JOINT], const moveit_msgs::CartesianTrajectory& msg);
        void waitForMotionCompletion(float target_joint[NUMBER_OF_JOINT], float tolerance = 1); 
        void waitForMotionCompletionWithRetry(float target_joint[NUMBER_OF_JOINT], float tolerance, int timeout_ms, int max_retries);
        void saveLoopTimesToFile(const std::string& filePath);
    protected:
        // ---------------- PBIC / legacy ----------------
        bool spinMotion(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Desired& desire, int sol_space);
        bool spinMotion_path(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Desired& desired, int sol_space);
        bool spinControl(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Torques& control_command, Desired& desired, int sol_space);

        // ---------------- DBIC 전용 ----------------
        bool spinMotionDBIC(SKKU::Duration time_step,
                            TaskRef& ref_tcp,
                            TaskRef& ref_task);

        bool spinControlDBIC(const LPRT_OUTPUT_DATA_LIST& robot_state,
                             SKKU::Duration time_step,
                             Torques& control_command,
                             const TaskRef& ref_task);

        TaskRef convertRefToTaskPoint(const TaskRef& tcp_ref) const;
        int finishMotion();
        int cancelMotion();
        std::string control_mode_;
        uint32_t motion_id_ = 0;
        Total_trajectory* total_trajectory_ = new Total_trajectory;
        RealtimeConfig realtimeconfig_;
        SchedSetting originalSetting_;
        u_int64_t loop_time_;
        int count = 0;
        pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
        Desired desired;
        Trajectory trajectory;
        float initial_joint_position[NUMBER_OF_JOINT];
        bool exitLoop = false;
        bool gaincheckloop = false;
        Torques control_command;
        bool truncate = false;
        TrajectoryGen trajectory_gen_;
        std::array<float, 6> torque_limit = {519.0, 519.0, 244.5, 75.0, 75.0, 75.0};
        Impedance imp;
        float previous_velocityj[NUMBER_OF_JOINT] = {0,};
        float old_p[NUMBER_OF_JOINT] = {0,};
        float old_v[NUMBER_OF_JOINT] = {0,};

        std::thread data_saving_thread_;
        std::atomic<bool> data_saving_running_{false}; // flag for data saving
        // moveit_msgs::CartesianTrajectory previous_msg;
        int sol_space = 0;
        int saving_count = 0;

        // // ImpedanceImplMode impedance_impl_mode_ = ImpedanceImplMode::kPBIC_TDC;
        // ImpedanceImplMode impedance_impl_mode_ = ImpedanceImplMode::kDBIC;
        // TaskPointMode task_point_mode_ = TaskPointMode::kTCP;
        // Eigen::Isometry3f T_flange_tcp_ = Eigen::Isometry3f::Identity();

        // DBIC 유지
        ImpedanceImplMode impedance_impl_mode_ = ImpedanceImplMode::kDBIC;

        // 기존 Python goal을 PBIC 때와 같은 의미로 먼저 맞추기 위해
        // 우선 flange 기준으로 해석한다.
        TaskPointMode task_point_mode_ = TaskPointMode::kFlange;
        // TaskPointMode task_point_mode_ = TaskPointMode::kTCP;

        // flange 기준 실험에서는 identity 유지
        Eigen::Isometry3f T_flange_tcp_ = Eigen::Isometry3f::Identity();
      

    private:
    

};

class ImpedanceControlLoop : public ControlLoop {
public:
    ImpedanceControlLoop(moveit_msgs::CartesianTrajectory msg,
                         u_int64_t loop_time,
                         RealtimeConfig realtimeconfig,
                         DRAFramework::CDRFLEx& Drfl);
    ~ImpedanceControlLoop();

    void operator()(const moveit_msgs::CartesianTrajectory& msg) override;
    void operator_path(const moveit_msgs::CartesianTrajectory& msg) override;
    void adjustSolutionSpace();

private:
    void runPBICGoal(const moveit_msgs::CartesianTrajectory& msg);
    void runPBICPath(const moveit_msgs::CartesianTrajectory& msg);

    void runDBICGoal(const moveit_msgs::CartesianTrajectory& msg);
    void runDBICPath(const moveit_msgs::CartesianTrajectory& msg);
};

class PositionControlLoop : public ControlLoop {
public:
    PositionControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl);
    ~PositionControlLoop();
    void operator()(const moveit_msgs::CartesianTrajectory& msg) override;
    void operator_path(const moveit_msgs::CartesianTrajectory& msg) override;
    void operator_jpath(const moveit_msgs::CartesianTrajectory& msg);
};

}
