// #include <algorithm>
// #include <cerrno>
// #include <cstring>
// #include <exception>
// #include <fstream>
// #include <thread>
// #include <chrono>
// #include <thread>
// #include <atomic>
// #include <vector>
// extern bool g_nKill_dsr_control;
// #include <skku_tools/control_loop.h>
// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <boost/filesystem.hpp>
// #include <condition_variable>
// #include <../../include/skku_control/dsr_hw_interface.h>
// #include <unordered_map>
// #include <future>


// std::mutex mtx;
// std::condition_variable cv;
// std::thread stateThread;
// std::thread savingThread;
// std::atomic<bool> stopFlag(false);
// float home_abs[NUMBER_OF_JOINT] = {0.00, 0.00, 90.00, 0.0, 90.0, 0.00};
// float current_position[NUMBER_OF_JOINT] = {0, };
// #define DESIRED_TIME 10

// double working_mode;
// // `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// // have to use `using namespace ...`.
// // See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
// using namespace std::string_literals;  // NOLINT(google-build-using-namespace)
// bool controlState = false; 


// namespace SKKU {
// moveit_msgs::CartesianTrajectory ControlLoop::previous_msg;
// int operator_call_count_ = 0;
// bool isDirectoryCreated = false;
// std::string dataDirectory = "";
// namespace fs = boost::filesystem;
// int fail;
// TrajectoryGen trajectory_gen_;
// TrajectoryGen::PlanParam plan;
// TrajectoryGen::TraParam tra;
// float distance_threshold = 50; 
// std::vector<uint64_t> loopTimes;

// void TrajectoryGen::init(moveit_msgs::CartesianTrajectory msg, moveit_msgs::CartesianTrajectory prev_msg, float current_position[NUMBER_OF_JOINT], int operator_call_count_) {
//     // 로봇의 출발점과 도착점을 담을 배열
//     // [X, Y, Z 위치 3개] + [쿼터니언 x, y, z, w 4개]
//     float start_point[7]; 
//     float goal[7];
//     float tra_time = 0.0;

//     // 현재 위치 복사 및 자세 변환 (Euler -> Quaternion) (박사님 로직 유지 - 완벽함)
//     // 현재 로봇의 위치(X, Y, Z) 값은 그대로 start_point의 0, 1, 2번 방에 복사
//     for (int i = 0; i < 3; i++) start_point[i] = current_position[i];
//     // current_position의 3, 4, 5번 방에는 로봇의 현재 자세가 Roll, Pitch, Yaw(오일러 각, Degree 단위)
//     // 수학 라이브러리인 Eigen을 사용하기 위해, 각도(Degree)에 M_PI / 180.0f를 곱해 라디안(Radian)으로 변환
//     Eigen::AngleAxisf rollAngle(current_position[3] * M_PI / 180.0f, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf pitchAngle(current_position[4] * M_PI / 180.0f, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf yawAngle(current_position[5] * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
//     // 만들어진 축별 회전을 곱해서(yaw * pitch * roll 순서) 쿼터니언(q_start)으로 합성
//     Eigen::Quaternionf q_start = yawAngle * pitchAngle * rollAngle;
//     // 합성된 쿼터니언의 x, y, z, w 값을 start_point 배열의 3, 4, 5, 6번 방에 차례대로 넣
//     start_point[3] = q_start.x(); start_point[4] = q_start.y(); start_point[5] = q_start.z(); start_point[6] = q_start.w();

//     // 목표 지점 설정 (파이썬에서 보낸 Quat 그대로 복사)
//     // 외부(ROS/Python 등)에서 날아온 메시지
//     // 이 메시지는 이미 위치(Position) 3개와 쿼터니언 회전(Orientation) 4개로 구성되어 있으므로, 변환 작업 없이 goal 배열에 그대로 복사
//     goal[0] = msg.points[0].point.pose.position.x;
//     goal[1] = msg.points[0].point.pose.position.y;
//     goal[2] = msg.points[0].point.pose.position.z;
//     goal[3] = msg.points[0].point.pose.orientation.x;
//     goal[4] = msg.points[0].point.pose.orientation.y;
//     goal[5] = msg.points[0].point.pose.orientation.z;
//     goal[6] = msg.points[0].point.pose.orientation.w; 

//     // 5. Antipodal 보정 (부호 일치화 - 완벽함)
//     float dot = start_point[3]*goal[3] + start_point[4]*goal[4] + start_point[5]*goal[5] + start_point[6]*goal[6];
//     if (dot < 0.0f) {
//         for(int i = 3; i < 7; i++) start_point[i] *= -1.0f;
//     }

//     // 6. 계획 파라미터 할당
//     tra_time = msg.points[0].time_from_start.toSec();
//     plan.time = tra_time;

//     // [중요 수정] i < 7 까지 돌려야 vs[6], as[6] (w성분)까지 초기화됩니다.
//     for (int i = 0; i < 7; i++) {
//         plan.ps[i] = start_point[i];
//         plan.pf[i] = goal[i];
        
//         // 쿼터니언 w성분의 속도/가속도도 0으로 초기화해야 다항식이 깨지지 않습니다.
//         plan.vs[i] = 0.0f; plan.vf[i] = 0.0f; 
//         plan.as[i] = 0.0f; plan.af[i] = 0.0f;
//     }

//     // 7. 궤적 생성 실행
//     trajectory_gen_.TrajectoryPlan(&plan);
//     std::cout << "Full Quaternion Pipeline Initialized." << std::endl;
// }

// // std::vector<std::array<double, 6>> TrajectoryGen::quadraticInterpolation(const std::vector<std::array<double, 6>>& points, int newPointsNum) {
// //     std::vector<std::array<double, 6>> interpolatedPoints;
// //     int originalPointsNum = points.size();

// //     if (originalPointsNum < 4) {
// //         throw std::invalid_argument("Not enough points for cubic interpolation.");
// //     }

// //     int totalSegments = originalPointsNum - 1; // Number of segments between points
// //     int pointsPerSegment = newPointsNum / totalSegments;

// //     auto interpolate = [](const std::array<double, 6>& p0, const std::array<double, 6>& p1, const std::array<double, 6>& p2, const std::array<double, 6>& p3, double t) {
// //         std::array<double, 6> interpolatedPoint;
// //         double t2 = t * t;
// //         double t3 = t2 * t;
// //         for (int k = 0; k < 6; ++k) {
// //             interpolatedPoint[k] = 0.5 * (
// //                 (2 * p1[k]) +
// //                 (-p0[k] + p2[k]) * t +
// //                 (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * t2 +
// //                 (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * t3
// //             );
// //         }
// //         return interpolatedPoint;
// //     };

// //     for (int i = 1; i < totalSegments - 1; ++i) {
// //         interpolatedPoints.push_back(points[i]);
// //         for (int j = 1; j <= pointsPerSegment; ++j) {
// //             double t = static_cast<double>(j) / (pointsPerSegment + 1);
// //             interpolatedPoints.push_back(interpolate(points[i - 1], points[i], points[i + 1], points[i + 2], t));
// //         }
// //     }
// //     interpolatedPoints.push_back(points.back());
// //     return interpolatedPoints;
// // }

// std::vector<std::array<double, 7>> TrajectoryGen::quadraticInterpolation(const std::vector<std::array<double, 7>>& points, int newPointsNum) {
//         std::vector<std::array<double, 7>> interpolatedPoints;
//         int originalPointsNum = points.size();

//         if (originalPointsNum < 4) {
//             throw std::invalid_argument("Not enough points for cubic interpolation.");
//         }

//         int totalSegments = originalPointsNum - 1; 
//         int pointsPerSegment = newPointsNum / totalSegments;

//         auto interpolate = [](const std::array<double, 7>& p0, const std::array<double, 7>& p1, const std::array<double, 7>& p2, const std::array<double, 7>& p3, double t) {
//             std::array<double, 7> interpolatedPoint;
//             double t2 = t * t;
//             double t3 = t2 * t;
//             for (int k = 0; k < 7; ++k) { // 6에서 7로 변경
//                 interpolatedPoint[k] = 0.5 * (
//                     (2 * p1[k]) +
//                     (-p0[k] + p2[k]) * t +
//                     (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * t2 +
//                     (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * t3
//                 );
//             }
//             return interpolatedPoint;
//         };

//         for (int i = 1; i < totalSegments - 1; ++i) {
//             interpolatedPoints.push_back(points[i]);
//             for (int j = 1; j <= pointsPerSegment; ++j) {
//                 double t = static_cast<double>(j) / (pointsPerSegment + 1);
//                 interpolatedPoints.push_back(interpolate(points[i - 1], points[i], points[i + 1], points[i + 2], t));
//             }
//         }
//         interpolatedPoints.push_back(points.back());
//         return interpolatedPoints;
//     }

// // std::vector<std::array<double, 6>> TrajectoryGen::upsampleTrajectory(const moveit_msgs::CartesianTrajectory& msg, int newPointsNum) {
// //     int originalPointsNum = msg.points.size();
    
// //     std::vector<std::array<double, 6>> controlPoints(originalPointsNum);

// //     // 기존 경로 포인트 복사
// //     for (int i = 0; i < originalPointsNum; ++i) {
// //         controlPoints[i] = {
// //             msg.points[i].point.pose.position.x,
// //             msg.points[i].point.pose.position.y,
// //             msg.points[i].point.pose.position.z,
// //             msg.points[i].point.pose.orientation.x,
// //             msg.points[i].point.pose.orientation.y,
// //             msg.points[i].point.pose.orientation.z,
// //         };
// //     }

// //     // 보간 수행 (기존 quadraticInterpolation 함수 사용)
// //     std::vector<std::array<double, 6>> interpolatedPoints = quadraticInterpolation(controlPoints, newPointsNum);
    
    
// //     // Ease-in/out 가중치를 적용하여 앞부분과 마지막 부분을 부드럽게 보간
// //     auto easeInOutWeight = [](double t) -> double {
// //         return t * t * (3 - 2 * t);  // cubic Hermite interpolation (ease-in/out)
// //     };

// //     int extraPointsNum = 1000;  // 가속/감속 구간을 추가할 포인트 수
// //     std::vector<std::array<double, 6>> acceleration_segment, decceleration_segment;

// //     // if(std::abs(current_position[5]-interpolatedPoints[0][5])>100) {
// //     //         current_position[5] = interpolatedPoints[0][5];
// //     // }

// //     if (std::abs(current_position[5] - interpolatedPoints[0][5]) > 100.0) {
// //     // 현재 Yaw 각도와 보정 후 값 비교
// //     float adjusted_position_plus = current_position[5] + 180.0;
// //     float adjusted_position_minus = current_position[5] - 180.0;

// //     // 목표 각도와 가장 가까운 값으로 Yaw 보정
// //     if (std::abs(adjusted_position_plus - interpolatedPoints[0][5]) < std::abs(current_position[5] - interpolatedPoints[0][5])) {
// //             std::cout<<"Current Yaw Position : "<<current_position[5]<<", Initial Yaw Point : "<<interpolatedPoints[0][5]<<", Current Yaw Position + 180 : "<<adjusted_position_plus<<std::endl;
// //             current_position[5] = adjusted_position_plus;
            
// //         } else if (std::abs(adjusted_position_minus - interpolatedPoints[0][5]) < std::abs(current_position[5] - interpolatedPoints[0][5])) {
// //             std::cout<<"Current Yaw Position : "<<current_position[5]<<", Initial Yaw Point : "<<interpolatedPoints[0][5]<<", Current Yaw Position - 180 : "<<adjusted_position_minus<<std::endl;
// //             current_position[5] = adjusted_position_minus;
// //         }

// //     }
    
// //     for (int i = 0; i < extraPointsNum; ++i) {
// //         std::array<double, 6> interpolated_point;
// //         double t = static_cast<double>(i) / (extraPointsNum - 1);  // 0에서 1까지의 값
// //         double weight = easeInOutWeight(t);

// //         for (int j = 0; j < NUMBER_OF_JOINT; ++j) { 
// //             interpolated_point[j] = current_position[j] * (1.0 - weight) + interpolatedPoints[0][j] * weight;
// //         }

// //         // // Roll, Pitch, Yaw 값 설정 (Roll 고정, Pitch 고정, Yaw는 보간)
// //         // interpolated_point[3] = ROLL_FIXED; // Roll 고정
// //         // interpolated_point[4] = PITCH_FIXED;  // Pitch 고정

// //         // interpolated_point[5] = interpolatedPoints[0][5];
// //         acceleration_segment.push_back(interpolated_point);
// //     }

// //     int decelerationStartIndex = std::max(0, static_cast<int>(interpolatedPoints.size()) - 5);  // 마지막 5개 포인트 시작 인덱스
// //     interpolatedPoints.erase(interpolatedPoints.begin() + decelerationStartIndex, interpolatedPoints.end());  // 마지막 5개 포인트 제거

// //     for (int i = 0; i < extraPointsNum; ++i) {
// //         std::array<double, 6> interpolated_point;
// //         double t = static_cast<double>(i) / (extraPointsNum - 1);  // 0에서 1까지의 값
// //         double weight = easeInOutWeight(t);

// //         for (int j = 0; j < NUMBER_OF_JOINT; ++j) { 
// //             interpolated_point[j] = interpolatedPoints[decelerationStartIndex - 1][j] * (1.0 - weight) + controlPoints.back()[j] * weight;
// //         }

// //         // Roll, Pitch, Yaw 값 설정 (Roll 고정, Pitch 고정, Yaw는 보간)
// //         // interpolated_point[3] = ROLL_FIXED;  // Roll 고정
// //         // interpolated_point[4] = PITCH_FIXED;  // Pitch 고정
        
// //         decceleration_segment.push_back(interpolated_point);
// //     }

// //     std::vector<std::array<double, 6>> fullTrajectory;

// //     fullTrajectory.insert(fullTrajectory.end(), acceleration_segment.begin(), acceleration_segment.end());
// //     fullTrajectory.insert(fullTrajectory.end(), interpolatedPoints.begin(), interpolatedPoints.end());  
// //     fullTrajectory.insert(fullTrajectory.end(), decceleration_segment.begin(), decceleration_segment.end());  

// //     return fullTrajectory;
// // }

// std::vector<std::array<double, 7>> TrajectoryGen::upsampleTrajectory(const moveit_msgs::CartesianTrajectory& msg, int newPointsNum) {
//     int originalPointsNum = msg.points.size();
    
//     std::vector<std::array<double, 7>> controlPoints(originalPointsNum);

//     // [수정] w 성분까지 7개 모두 복사
//     for (int i = 0; i < originalPointsNum; ++i) {
//         controlPoints[i] = {
//             msg.points[i].point.pose.position.x,
//             msg.points[i].point.pose.position.y,
//             msg.points[i].point.pose.position.z,
//             msg.points[i].point.pose.orientation.x,
//             msg.points[i].point.pose.orientation.y,
//             msg.points[i].point.pose.orientation.z,
//             msg.points[i].point.pose.orientation.w  // 추가됨
//         };
//     }

//     std::vector<std::array<double, 7>> interpolatedPoints = quadraticInterpolation(controlPoints, newPointsNum);
    
//     auto easeInOutWeight = [](double t) -> double {
//         return t * t * (3 - 2 * t);  
//     };

//     int extraPointsNum = 1000;  
//     std::vector<std::array<double, 7>> acceleration_segment, decceleration_segment;

//     // [추가] 6차원 current_position을 안전하게 7차원(쿼터니언)으로 변환
//     float current_pos_7d[7];
//     for (int k = 0; k < 3; ++k) current_pos_7d[k] = current_position[k];
//     Eigen::AngleAxisf rollAngle(current_position[3] * M_PI / 180.0f, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf pitchAngle(current_position[4] * M_PI / 180.0f, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf yawAngle(current_position[5] * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
//     Eigen::Quaternionf q_curr = yawAngle * pitchAngle * rollAngle;
//     current_pos_7d[3] = q_curr.x(); current_pos_7d[4] = q_curr.y(); 
//     current_pos_7d[5] = q_curr.z(); current_pos_7d[6] = q_curr.w();

//     for (int i = 0; i < extraPointsNum; ++i) {
//         std::array<double, 7> interpolated_point;
//         double t = static_cast<double>(i) / (extraPointsNum - 1);  
//         double weight = easeInOutWeight(t);

//         // [수정] NUMBER_OF_JOINT(6) 대신 7로 변경 및 current_pos_7d 사용
//         for (int j = 0; j < 7; ++j) { 
//             interpolated_point[j] = current_pos_7d[j] * (1.0 - weight) + interpolatedPoints[0][j] * weight;
//         }
//         acceleration_segment.push_back(interpolated_point);
//     }

//     int decelerationStartIndex = std::max(0, static_cast<int>(interpolatedPoints.size()) - 5);  
//     interpolatedPoints.erase(interpolatedPoints.begin() + decelerationStartIndex, interpolatedPoints.end());  

//     for (int i = 0; i < extraPointsNum; ++i) {
//         std::array<double, 7> interpolated_point;
//         double t = static_cast<double>(i) / (extraPointsNum - 1);  
//         double weight = easeInOutWeight(t);

//         for (int j = 0; j < 7; ++j) { // 7로 변경
//             interpolated_point[j] = interpolatedPoints[decelerationStartIndex - 1][j] * (1.0 - weight) + controlPoints.back()[j] * weight;
//         }
//         decceleration_segment.push_back(interpolated_point);
//     }

//     std::vector<std::array<double, 7>> fullTrajectory;

//     fullTrajectory.insert(fullTrajectory.end(), acceleration_segment.begin(), acceleration_segment.end());
//     fullTrajectory.insert(fullTrajectory.end(), interpolatedPoints.begin(), interpolatedPoints.end());  
//     fullTrajectory.insert(fullTrajectory.end(), decceleration_segment.begin(), decceleration_segment.end());  

//     return fullTrajectory;
// }


// ControlLoop::ControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl) 
//     : PBIC(loop_time, Drfl) {
//     // total_trajectory_ = total_trajectory;
//     realtimeconfig_ = realtimeconfig;
//     loop_time_ = loop_time;
//     bool throw_on_error = realtimeconfig_ == RealtimeConfig::kEnforce;
//     std::string error_message;

//     if (!getCurrentScheduling(originalSetting_)) {
//         throw std::runtime_error("Failed to get current scheduling settings");
//     }

//     if (!setCurrentThreadToHighestSchedulerPriority(&error_message) && throw_on_error) {
//         throw std::runtime_error(error_message);
//     }
//     if (throw_on_error && !hasRealtimeKernel()) {
//         throw std::runtime_error("Error : Running kernel does not have realtime capabilities.");
//     }
//     std::cout << "High priority setting done" << std::endl;
//     //motion_id_ = startMotion();
// }

// ControlLoop::~ControlLoop() {
//     if (!setScheduling(originalSetting_)) {
//         std::cerr << "Failed to restore original scheduling settings" << std::endl;
//     } else {
//         std::cout << "Original scheduling settings restored successfully" << std::endl;
//     }
// }

// ImpedanceControlLoop::ImpedanceControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl)
//     : ControlLoop(msg, loop_time, realtimeconfig, Drfl) {}

// ImpedanceControlLoop::~ImpedanceControlLoop() {}

// PositionControlLoop::PositionControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl)
//     : ControlLoop(msg, loop_time, realtimeconfig, Drfl) {}

// PositionControlLoop::~PositionControlLoop() {}

// void ControlLoop::StateCheckingThread(ControlLoop* controlLoop) {
//     while (true) {
//         ROBOT_STATE state = Drfl_.get_robot_state();

//         // Check if the robot is in a state that requires initialization
//         if (state == STATE_SAFE_OFF || state == STATE_SAFE_STOP ||
//             state == STATE_RECOVERY || state == STATE_SAFE_STOP2||
//             state == STATE_SAFE_OFF2 || state == STATE_EMERGENCY_STOP) {
//             exitLoop = true;
//             Drfl_.set_robot_control(CONTROL_RESET_SAFET_STOP);     
//         }
//         else
//         {
//             exitLoop = false; 

//         }

//         std::this_thread::sleep_for(std::chrono::seconds(1)); // Sleep for some time before checking again
//     }
// }
    
// void PositionControlLoop::operator()(const moveit_msgs::CartesianTrajectory& msg) {
//     std::cout << "Position Goal-directed Mode called" << std::endl;
//     operator_call_count_++;
//     fail = 0;
//     control_mode_ = "Position goal mode";

//     if (msg.points.empty()) {
//         ROS_ERROR("Empty CartesianTrajectory received.");
//         fail = 2;
//         return;
//     }

//     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();

//     float current_joint[NUMBER_OF_JOINT] = {0,};
//     memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
//     memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//     constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
//     constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

//     auto unwrapToNearest = [](float angle_deg, float ref_deg) {
//         while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
//         while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
//         return angle_deg;
//     };

//     float goal_p[NUMBER_OF_JOINT] = {0,};
//     for (int i = 0; i < 6; ++i) {
//         goal_p[i] = current_position[i];
//     }

//     // 1) 목표 위치 반영
//     goal_p[0] = msg.points[0].point.pose.position.x;
//     goal_p[1] = msg.points[0].point.pose.position.y;
//     goal_p[2] = msg.points[0].point.pose.position.z;

//     // 2) 현재 자세(RPY deg) -> quaternion
//     Eigen::AngleAxisf roll_curr(current_position[3] * DEG2RAD, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf pitch_curr(current_position[4] * DEG2RAD, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf yaw_curr(current_position[5] * DEG2RAD, Eigen::Vector3f::UnitZ());
//     Eigen::Quaternionf q_curr = yaw_curr * pitch_curr * roll_curr;
//     q_curr.normalize();

//     // 3) 목표 quaternion 입력
//     Eigen::Quaternionf q_goal(
//         msg.points[0].point.pose.orientation.w,
//         msg.points[0].point.pose.orientation.x,
//         msg.points[0].point.pose.orientation.y,
//         msg.points[0].point.pose.orientation.z
//     );

//     if (q_goal.norm() < 1e-6f) {
//         ROS_WARN("Received near-zero quaternion. Using current orientation instead.");
//         q_goal = q_curr;
//     } else {
//         q_goal.normalize();
//     }

//     // 4) quaternion sign continuity
//     if (q_curr.coeffs().dot(q_goal.coeffs()) < 0.0f) {
//         q_goal.coeffs() *= -1.0f;
//     }

//     // 5) quaternion -> Euler(ZYX) -> degree
//     Eigen::Vector3f euler_rad = q_goal.toRotationMatrix().eulerAngles(2, 1, 0);

//     float roll_deg  = euler_rad[2] * RAD2DEG;
//     float pitch_deg = euler_rad[1] * RAD2DEG;
//     float yaw_deg   = euler_rad[0] * RAD2DEG;

//     // 6) 현재 자세 기준으로 가장 가까운 branch 선택
//     goal_p[3] = unwrapToNearest(roll_deg,  current_position[3]);
//     goal_p[4] = unwrapToNearest(pitch_deg, current_position[4]);
//     goal_p[5] = unwrapToNearest(yaw_deg,   current_position[5]);

//     // 필요하면 기존 lab helper를 여기서 추가로 적용
//     // goal_p[5] = trajectory_gen_.adjustAngle(goal_p[5]);

//     std::cout << "Goal pose XYZRPY(deg): "
//               << goal_p[0] << ", " << goal_p[1] << ", " << goal_p[2] << ", "
//               << goal_p[3] << ", " << goal_p[4] << ", " << goal_p[5] << std::endl;

//     // 7) IK
//     LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(goal_p, 2, COORDINATE_SYSTEM_BASE, 1);
//     if (res == nullptr) {
//         ROS_ERROR("IK failed for goal pose.");
//         fail = 2;
//         return;
//     }

//     float goal_joint[NUMBER_OF_JOINT] = {0,};
//     for (int i = 0; i < 6; ++i) {
//         goal_joint[i] = res->_fTargetPos[i];
//     }

//     // 8) Joint 6 ambiguity: 현재 joint와 가장 가까운 branch 선택
//     while (goal_joint[5] - current_joint[5] > 180.0f) goal_joint[5] -= 360.0f;
//     while (goal_joint[5] - current_joint[5] < -180.0f) goal_joint[5] += 360.0f;

//     float tTime = msg.points[0].time_from_start.toSec();

//     Drfl_.set_safety_mode(SAFETY_MODE_MANUAL, SAFETY_MODE_EVENT_MOVE);
//     Drfl_.set_robot_mode(ROBOT_MODE_MANUAL);

//     controlState = true;

//     ROS_INFO("Starting data saving...");
//     startDataSaving();

//     ROS_INFO("Calling movej(goal_joint)...");
//     bool success = Drfl_.movej(goal_joint, 60, 30, tTime);

//     ROS_INFO("Stopping data saving...");
//     stopDataSaving();
//     controlState = false;

//     if (!success) {
//         ROS_ERROR("movej failed.");
//         fail = 2;
//         return;
//     }

//     float final_position[NUMBER_OF_JOINT] = {0,};
//     robot_data = Drfl_.read_data_rt();
//     memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//     float distance = std::sqrt(
//         std::pow(final_position[0] - goal_p[0], 2) +
//         std::pow(final_position[1] - goal_p[1], 2) +
//         std::pow(final_position[2] - goal_p[2], 2)
//     );

//     if (distance > distance_threshold) {
//         ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
//         fail = 2;
//     } else {
//         ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
//         fail = 1;
//     }

//     previous_msg = msg;
//     return;
// }

// void ImpedanceControlLoop::operator()(const moveit_msgs::CartesianTrajectory& msg) {

//     std::cout << "Impedance Goal-directed Mode called" << std::endl;
//     fail = 0;
//     control_mode_ = "Impedance goal mode";
//     operator_call_count_++; 

//     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt(); // reading real time data 

//     float current_joint[NUMBER_OF_JOINT] = {0, };
//     memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
//     memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

//     Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE); // mode setting for control 
//     Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

//     float x_offset[NUMBER_OF_JOINT] = {0, };


//     sol_space = 0;
//     count = 0; 

//     /////////////////////////////////////////////////////////////
//     ///////////////External force offset detection///////////////
//     /////////////////////////////////////////////////////////////
//     //Check Fext Calculation
//     // std::thread gainsavingThread([this]() { gaindataSavingThread(); });
//     // GainMove();
//     // gainsavingThread.join();
//     // return;
//     /////////////////////////////////////////////////////////////
    
//     // std::cout << "INITIAL actual position for joint 6: " << current_joint[5] << std::endl;
//     ROS_INFO_STREAM("Full message content : " << msg);
    

//     robot_data = Drfl_.read_data_rt(); // reading real time data 
//     memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

//     Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE); // mode setting for control 
//     Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    
//     robot_data = Drfl_.read_data_rt(); // reading real time data 
//     memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));
//     memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

//     trajectory_gen_.init(msg,previous_msg,current_position,operator_call_count_);
    
//     Duration control_loop_time = Duration(loop_time_);

//     float st = static_cast<float>(loop_time_) / 1000;
    
//     // LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

//     // start_Motion(robot_state, prev, imp);

//     // auto start = std::chrono::high_resolution_clock::now();
        
//     // controlState = true; // control starts
//     // auto start_time  = std::chrono::high_resolution_clock::now();
//     // startDataSaving();
    
//     // while (spinMotion(robot_state, control_loop_time, desired, sol_space) && spinControl(robot_state, control_loop_time, control_command, desired, sol_space)){

//     //     if (!exitLoop && !g_nKill_dsr_control){
//     //     // startDataSaving();

//     //     Drfl_.torque_rt(control_command.tau_d, st);
//     //     // Drfl_.torque_rt(Drfl_.read_data_rt() -> gravity_torque, dt); // If Gravity only needs
//     //     auto current = std::chrono::high_resolution_clock::now();
        
//     //     Duration loop_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
        
//     //     loopTimes.push_back(loop_time.toMSec());


//     //     if (control_loop_time > loop_time) {
//     //     std::this_thread::sleep_for(control_loop_time() - loop_time());
//     //     }
//     //     // std::cout<< "Loop time : "<< loop_time.toMSec() <<std::endl;
//     //     start = std::chrono::high_resolution_clock::now();

//     //     }

//     //     else
//     //     {
//     //     fail = true;
//     //     std::cout << SKKU::fail<< std::endl;
//     //     break;
//     //     }
        
//     //     // stopDataSaving();
//     //     count ++;
//     // }

//     // stopDataSaving();
//     // saveLoopTimesToFile(dataDirectory);
    
//     LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

//     start_Motion(robot_state, prev, imp);

//     auto start = std::chrono::high_resolution_clock::now();

//     loopTimes.clear();
//     controlState = true;
//     auto start_time = std::chrono::high_resolution_clock::now();
//     startDataSaving();

//     while (true) {
//         // 매 제어주기마다 최신 robot state 갱신
//         robot_state = Drfl_.read_data_rt();

//         // motion / control 계산
//         if (!spinMotion(robot_state, control_loop_time, desired, sol_space) ||
//             !spinControl(robot_state, control_loop_time, control_command, desired, sol_space)) {
//             break;
//         }

//         // 종료 조건
//         if (exitLoop || g_nKill_dsr_control) {
//             fail = 2;
//             std::cout << SKKU::fail << std::endl;
//             break;
//         }

//         // 토크 명령 전송
//         Drfl_.torque_rt(control_command.tau_d, st);

//         // 루프 시간 측정
//         auto current = std::chrono::high_resolution_clock::now();
//         Duration loop_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
//         loopTimes.push_back(loop_time.toMSec());

//         // 주기 맞추기
//         if (control_loop_time > loop_time) {
//             std::this_thread::sleep_for(control_loop_time() - loop_time());
//         }

//         start = std::chrono::high_resolution_clock::now();
//         count++;
//     }

//     stopDataSaving();
//     saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
//     controlState = false;

//     // 루프 종료 후 최신 상태 다시 읽기
//     robot_data = Drfl_.read_data_rt();

//     auto finished_time  = std::chrono::high_resolution_clock::now();
//     auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(finished_time - start_time);
//     std::cout << "elapsed time: " << elapsed_time.count() << " ms" << std::endl;
//     controlState = false;
//     ///////////////////////////////////////

//     setScheduling(originalSetting_);
    
//     float final_position[NUMBER_OF_JOINT] = {0, };
//     memcpy(final_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

//     float distance = std::sqrt(
//         std::pow(final_position[0] - msg.points[0].point.pose.position.x, 2) +
//         std::pow(final_position[1] - msg.points[0].point.pose.position.y, 2) +
//         std::pow(final_position[2] - msg.points[0].point.pose.position.z, 2)
//     );

//     if (distance > distance_threshold) { 
//         ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
//         fail = 2;
//     }
//     else if (distance <= distance_threshold) {
//         ROS_INFO("Succeed reach the goal position: distance = %f", distance);
//         fail = 1;
//     }
    
//     previous_msg = msg;

//     return;
    
// }

// // void PositionControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {

// //     std::cout << "Position Push-path Mode called" << std::endl;
// //     fail = 0;
// //     control_mode_ = "Position path mode";
// //     operator_call_count_++; 

// //     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt(); // reading real time data 

// //     float current_joint[NUMBER_OF_JOINT] = {0, };
// //     memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));
// //     memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

// //     int pointsNum = msg.points.size();

// //     float (*xpos)[6] = new float[pointsNum][6];

// //     float tvel[2] = {1000, 1000};
// //     float tacc[2] = {1000, 1000};
// //     float tTime =  msg.points[0].time_from_start.toSec();

// //     for (int i = 0; i < pointsNum; ++i) {
// //         xpos[i][0] = msg.points[i].point.pose.position.x;
// //         xpos[i][1] = msg.points[i].point.pose.position.y;
// //         xpos[i][2] = msg.points[i].point.pose.position.z;
// //         xpos[i][3] = msg.points[i].point.pose.orientation.x;
// //         xpos[i][4] = msg.points[i].point.pose.orientation.y;
// //         xpos[i][5] = msg.points[i].point.pose.orientation.z;
// //     }

// //     std::cout << "Point size :" << pointsNum << std::endl;


// //     // solution space adjusting 
// //     if (current_joint[5] > 90) {
        
// //         std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
// //         float tvel[2] = { 70, 70 };
// //         float tacc[2] = { 120, 120 };
// //         current_position[2] += 100;
// //         Drfl_.movel(current_position,tvel,tacc);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_joint[5] -= 360;

// //         Drfl_.movej(current_joint, 60, 30);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_position[2] -= 100;

// //         Drfl_.movel(current_position,tvel,tacc);
// //     }
    
// //     else if (current_joint[5] < -90) {
        
// //         std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
// //         float tvel[2] = { 70, 70 };
// //         float tacc[2] = { 120, 120 };
// //         current_position[2] += 100;

// //         Drfl_.movel(current_position,tvel,tacc);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_joint[5] += 360;

// //         Drfl_.movej(current_joint, 60, 30);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_position[2] -= 100;

// //         Drfl_.movel(current_position,tvel,tacc);

// //     }
    
// //     else {
// //         std::cout << "No adjustment needed for joint 6: " << current_joint[5] << std::endl;
// //     }

// //     ///////////////////////////////////////
// //     controlState = true; // control starts

// //     ROS_INFO("Starting data saving...");
// //     std::thread savingThread([this]() { startDataSaving(); });

// //     ROS_INFO("Calling move function...");

// //     auto start_time = std::chrono::steady_clock::now();

// //     bool success = Drfl_.amovesx(xpos, pointsNum, tvel, tacc, tTime, MOVE_MODE_ABSOLUTE);


// //     savingThread.join();
// //     ROS_INFO("Stopping data saving...");
// //     stopDataSaving();

// //     controlState = false; // control ends
// //     ///////////////////////////////////////

// //     float final_position[NUMBER_OF_JOINT] = {0, }; //CHECKLIST

// //     robot_data = Drfl_.read_data_rt(); // reading real time data 

// //     memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

// //     float distance = std::sqrt(
// //         std::pow(final_position[0] - xpos[pointsNum-1][0], 2) +
// //         std::pow(final_position[1] - xpos[pointsNum-1][1], 2) +
// //         std::pow(final_position[2] - xpos[pointsNum-1][2], 2)
// //     );

// //     if (distance > distance_threshold) { // 2.5 cm
// //         ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
// //         fail = 2;

// //     } else if (distance <= distance_threshold) {
// //         ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
// //         fail = 1;
// //     }

// //     delete[] xpos;  
// //     previous_msg = msg;

    
// //     return;
// // }

// // void PositionControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {

// //     std::cout << "Position Push-path Mode called" << std::endl;
// //     fail = 0;
// //     control_mode_ = "Position path mode";
// //     operator_call_count_++; 

// //     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt(); // reading real time data 

// //     float current_joint[NUMBER_OF_JOINT] = {0, };
// //     memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));
// //     memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

// //     int pointsNum = msg.points.size();

// //     // 주의: 두산 amovesx 함수는 6차원(오일러)만 지원하므로 [6] 유지!
// //     float (*xpos)[6] = new float[pointsNum][6];

// //     float tvel[2] = {1000, 1000};
// //     float tacc[2] = {1000, 1000};
// //     float tTime =  msg.points[0].time_from_start.toSec();

// //     // ---------------------------------------------------------
// //     // [핵심 수정 구간] 쿼터니언(7D)을 받아서 오일러(6D, Degree)로 변환
// //     // ---------------------------------------------------------
// //     for (int i = 0; i < pointsNum; ++i) {
// //         xpos[i][0] = msg.points[i].point.pose.position.x;
// //         xpos[i][1] = msg.points[i].point.pose.position.y;
// //         xpos[i][2] = msg.points[i].point.pose.position.z;

// //         // 파이썬이 보낸 쿼터니언 조립 (Eigen 입력 순서: w, x, y, z)
// //         Eigen::Quaternionf q_msg(
// //             msg.points[i].point.pose.orientation.w,
// //             msg.points[i].point.pose.orientation.x,
// //             msg.points[i].point.pose.orientation.y,
// //             msg.points[i].point.pose.orientation.z
// //         );
// //         q_msg.normalize();

// //         // 쿼터니언 -> 회전행렬 -> 오일러(Z, Y, X 순서 = Yaw, Pitch, Roll)
// //         Eigen::Vector3f euler_rad = q_msg.toRotationMatrix().eulerAngles(2, 1, 0); 

// //         // 라디안을 두산 로봇이 요구하는 도(Degree) 단위로 변환하여 할당
// //         xpos[i][3] = euler_rad[2] * 180.0f / M_PI; // Roll
// //         xpos[i][4] = euler_rad[1] * 180.0f / M_PI; // Pitch
// //         xpos[i][5] = euler_rad[0] * 180.0f / M_PI; // Yaw
// //     }
// //     // ---------------------------------------------------------

// //     std::cout << "Point size :" << pointsNum << std::endl;


// //     // solution space adjusting 
// //     if (current_joint[5] > 90) {
        
// //         std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
// //         float tvel[2] = { 70, 70 };
// //         float tacc[2] = { 120, 120 };
// //         current_position[2] += 100;
// //         Drfl_.movel(current_position,tvel,tacc);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_joint[5] -= 360;

// //         Drfl_.movej(current_joint, 60, 30);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_position[2] -= 100;

// //         Drfl_.movel(current_position,tvel,tacc);
// //     }
    
// //     else if (current_joint[5] < -90) {
        
// //         std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
// //         float tvel[2] = { 70, 70 };
// //         float tacc[2] = { 120, 120 };
// //         current_position[2] += 100;

// //         Drfl_.movel(current_position,tvel,tacc);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_joint[5] += 360;

// //         Drfl_.movej(current_joint, 60, 30);

// //         robot_data = Drfl_.read_data_rt();

// //         memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

// //         current_position[2] -= 100;

// //         Drfl_.movel(current_position,tvel,tacc);

// //     }
    
// //     else {
// //         std::cout << "No adjustment needed for joint 6: " << current_joint[5] << std::endl;
// //     }

// //     ///////////////////////////////////////
// //     controlState = true; // control starts

// //     ROS_INFO("Starting data saving...");
// //     std::thread savingThread([this]() { startDataSaving(); });

// //     ROS_INFO("Calling move function...");

// //     auto start_time = std::chrono::steady_clock::now();

// //     bool success = Drfl_.amovesx(xpos, pointsNum, tvel, tacc, tTime, MOVE_MODE_ABSOLUTE);


// //     savingThread.join();
// //     ROS_INFO("Stopping data saving...");
// //     stopDataSaving();

// //     controlState = false; // control ends
// //     ///////////////////////////////////////

// //     float final_position[NUMBER_OF_JOINT] = {0, }; //CHECKLIST

// //     robot_data = Drfl_.read_data_rt(); // reading real time data 

// //     memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

// //     float distance = std::sqrt(
// //         std::pow(final_position[0] - xpos[pointsNum-1][0], 2) +
// //         std::pow(final_position[1] - xpos[pointsNum-1][1], 2) +
// //         std::pow(final_position[2] - xpos[pointsNum-1][2], 2)
// //     );

// //     if (distance > distance_threshold) { // 2.5 cm
// //         ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
// //         fail = 2;

// //     } else if (distance <= distance_threshold) {
// //         ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
// //         fail = 1;
// //     }

// //     delete[] xpos;  
// //     previous_msg = msg;

    
// //     return;
// // }

// // void PositionControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {
// //     std::cout << "Position Joint Path Mode called" << std::endl;
// //     fail = 0;
// //     control_mode_ = "Position joint path mode";
// //     operator_call_count_++;

// //     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt(); // 현재 로봇 상태 읽기

// //     float current_joint[NUMBER_OF_JOINT] = {0, };
// //     memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
// //     memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

// //     int pointsNum = msg.points.size();

// //     float (*jpos)[6] = new float[pointsNum][6];
// //     float jvel = 100;
// //     float jacc = 100;
// //     float tTime = msg.points[0].time_from_start.toSec();

// //     for (int i = 0; i < pointsNum; ++i) {
// //         jpos[i][0] = msg.points[i].point.pose.position.x;
// //         jpos[i][1] = msg.points[i].point.pose.position.y;
// //         jpos[i][2] = msg.points[i].point.pose.position.z;
// //         jpos[i][3] = msg.points[i].point.pose.orientation.x;
// //         jpos[i][4] = msg.points[i].point.pose.orientation.y;
// //         jpos[i][5] = msg.points[i].point.pose.orientation.z;
// //     }

// //     std::cout << "Calling amovesj with " << pointsNum << " points" << std::endl;

// //     controlState = true;
// //     ROS_INFO("Starting data saving...");
// //     std::thread savingThread([this]() { startDataSaving(); });

// //     auto start_time = std::chrono::steady_clock::now();

// //     bool success = Drfl_.amovesj(jpos, pointsNum, jvel, jacc, tTime, MOVE_MODE_ABSOLUTE);

// //     savingThread.join();
// //     ROS_INFO("Stopping data saving...");
// //     stopDataSaving();
// //     controlState = false;

// //     // float final_position[NUMBER_OF_JOINT] = {0, };
// //     // robot_data = Drfl_.read_data_rt();
// //     // memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

// //     // float distance = std::sqrt(
// //     //     std::pow(final_position[0] - jpos[pointsNum - 1][0], 2) +
// //     //     std::pow(final_position[1] - jpos[pointsNum - 1][1], 2) +
// //     //     std::pow(final_position[2] - jpos[pointsNum - 1][2], 2)
// //     // );

// //     // if (distance > distance_threshold) {
// //     //     ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
// //     //     fail = 2;
// //     // } else {
// //     //     ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
// //     //     fail = 1;
// //     // }

// //     delete[] jpos;
// //     previous_msg = msg;
// //     return;
// // }

// void PositionControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {
//     std::cout << "Position Push-path Mode called" << std::endl;
//     fail = 0;
//     control_mode_ = "Position path mode";
//     operator_call_count_++;

//     if (msg.points.empty()) {
//         ROS_ERROR("operator_path received empty trajectory.");
//         fail = 2;
//         return;
//     }

//     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();

//     float current_joint[NUMBER_OF_JOINT] = {0,};
//     memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
//     memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

//     const int pointsNum = static_cast<int>(msg.points.size());

//     // raw new/delete 대신 RAII 권장
//     std::unique_ptr<float[][6]> xpos(new float[pointsNum][6]);

//     float spline_vel[2] = {1000, 1000};
//     float spline_acc[2] = {1000, 1000};
//     // float tTime = msg.points[0].time_from_start.toSec();
//     float tTime = msg.points.back().time_from_start.toSec();

//     constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
//     constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

//     auto unwrapToNearest = [](float angle_deg, float ref_deg) {
//         while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
//         while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
//         return angle_deg;
//     };

//     // 현재 flange RPY를 quaternion으로 만들어 첫 기준점으로 사용
//     Eigen::AngleAxisf roll0(current_position[3] * DEG2RAD, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf pitch0(current_position[4] * DEG2RAD, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf yaw0(current_position[5] * DEG2RAD, Eigen::Vector3f::UnitZ());
//     Eigen::Quaternionf q_prev = yaw0 * pitch0 * roll0;
//     q_prev.normalize();

//     float prev_roll  = current_position[3];
//     float prev_pitch = current_position[4];
//     float prev_yaw   = current_position[5];

//     for (int i = 0; i < pointsNum; ++i) {
//         xpos[i][0] = msg.points[i].point.pose.position.x;
//         xpos[i][1] = msg.points[i].point.pose.position.y;
//         xpos[i][2] = msg.points[i].point.pose.position.z;

//         Eigen::Quaternionf q_msg(
//             msg.points[i].point.pose.orientation.w,
//             msg.points[i].point.pose.orientation.x,
//             msg.points[i].point.pose.orientation.y,
//             msg.points[i].point.pose.orientation.z
//         );

//         if (q_msg.norm() < 1e-6f) {
//             ROS_WARN("Waypoint %d has near-zero quaternion. Reusing previous orientation.", i);
//             q_msg = q_prev;
//         } else {
//             q_msg.normalize();
//         }

//         // quaternion sign continuity
//         if (q_prev.coeffs().dot(q_msg.coeffs()) < 0.0f) {
//             q_msg.coeffs() *= -1.0f;
//         }

//         Eigen::Vector3f euler_rad = q_msg.toRotationMatrix().eulerAngles(2, 1, 0);

//         float roll_deg  = euler_rad[2] * RAD2DEG;
//         float pitch_deg = euler_rad[1] * RAD2DEG;
//         float yaw_deg   = euler_rad[0] * RAD2DEG;

//         // Euler continuity
//         roll_deg  = unwrapToNearest(roll_deg,  prev_roll);
//         pitch_deg = unwrapToNearest(pitch_deg, prev_pitch);
//         yaw_deg   = unwrapToNearest(yaw_deg,   prev_yaw);

//         xpos[i][3] = roll_deg;
//         xpos[i][4] = pitch_deg;
//         xpos[i][5] = yaw_deg;

//         q_prev = q_msg;
//         prev_roll  = roll_deg;
//         prev_pitch = pitch_deg;
//         prev_yaw   = yaw_deg;
//     }

//     std::cout << "Point size: " << pointsNum << std::endl;

//     // yaw 진행 방향까지 같이 고려해서 solution-space adjust
//     float delta_yaw = xpos[pointsNum - 1][5] - xpos[0][5];

//     if (current_joint[5] > 90.0f && delta_yaw > 0.0f) {
//         std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
//         float prep_vel[2] = {70, 70};
//         float prep_acc[2] = {120, 120};

//         current_position[2] += 100;
//         Drfl_.movel(current_position, prep_vel, prep_acc);

//         robot_data = Drfl_.read_data_rt();
//         memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

//         current_joint[5] -= 360;
//         Drfl_.movej(current_joint, 60, 30);

//         robot_data = Drfl_.read_data_rt();
//         memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//         current_position[2] -= 100;
//         Drfl_.movel(current_position, prep_vel, prep_acc);
//     }
//     else if (current_joint[5] < -90.0f && delta_yaw < 0.0f) {
//         std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
//         float prep_vel[2] = {70, 70};
//         float prep_acc[2] = {120, 120};

//         current_position[2] += 100;
//         Drfl_.movel(current_position, prep_vel, prep_acc);

//         robot_data = Drfl_.read_data_rt();
//         memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

//         current_joint[5] += 360;
//         Drfl_.movej(current_joint, 60, 30);

//         robot_data = Drfl_.read_data_rt();
//         memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//         current_position[2] -= 100;
//         Drfl_.movel(current_position, prep_vel, prep_acc);
//     }
//     else {
//         std::cout << "No adjustment needed for joint 6: " << current_joint[5] << std::endl;
//     }

//     controlState = true;
//     ROS_INFO("Starting data saving...");
//     startDataSaving();

//     ROS_INFO("Calling amovesx...");
//     bool success = Drfl_.amovesx(xpos.get(), pointsNum, spline_vel, spline_acc, tTime, MOVE_MODE_ABSOLUTE);

//     if (!success) {
//         ROS_ERROR("amovesx failed to start.");
//         stopDataSaving();
//         controlState = false;
//         fail = 2;
//         return;
//     }

//     // amovesx는 async이므로 실제 종료까지 기다려야 로그가 안 잘림
//     int wait_ret = Drfl_.mwait();

//     ROS_INFO("Stopping data saving...");
//     stopDataSaving();
//     controlState = false;

//     if (wait_ret != 1) {
//         ROS_ERROR("mwait reported motion failure.");
//         fail = 2;
//         return;
//     }

//     float final_position[NUMBER_OF_JOINT] = {0,};
//     robot_data = Drfl_.read_data_rt();
//     memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//     float distance = std::sqrt(
//         std::pow(final_position[0] - xpos[pointsNum - 1][0], 2) +
//         std::pow(final_position[1] - xpos[pointsNum - 1][1], 2) +
//         std::pow(final_position[2] - xpos[pointsNum - 1][2], 2)
//     );

//     if (distance > distance_threshold) {
//         ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
//         fail = 2;
//     } else {
//         ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
//         fail = 1;
//     }

//     previous_msg = msg;
// }


// void ImpedanceControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {
    
//     if (msg.points.empty()) {
//         ROS_ERROR("Impedance operator_path received empty trajectory.");
//         fail = 2;
//         return;
//     }

//     std::cout << "Impedance Push-path Mode called" << std::endl;
//     fail = 0;
//     control_mode_ = "Impedance path mode";
//     operator_call_count_++;

//     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt(); // reading real time data 

//     float current_joint[NUMBER_OF_JOINT] = {0, };
//     memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
//     memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//     // alignPitchOrientation(current_joint,msg);

//     Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE); // mode setting for control 
//     Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    
//     int originalPointsNum = msg.points.size();

//     // double duration = msg.points[0].time_from_start.toSec();
//     double duration = msg.points.back().time_from_start.toSec();

//     int newPointsNum = static_cast<int>(duration * 1000/loop_time_);

//     count = 0;
//     sol_space = 0;
//     count_motion = 0;

//     Duration control_loop_time = Duration(loop_time_);

//     float st = static_cast<float>(loop_time_) / 1000;

//     LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

//     start_Motion(robot_state, prev, imp);
    
//     // float delta_yaw = msg.points.back().point.pose.orientation.z - msg.points[0].point.pose.orientation.z;

//     float delta_yaw =
//         quatMsgToYawDeg(msg.points.back().point.pose.orientation) -
//         quatMsgToYawDeg(msg.points.front().point.pose.orientation);

//     std::cout << "Initiating joint 6 position before adjustment: " << current_joint[5] << std::endl;
//     std::cout << "Trajectory deviation of yaw : " << delta_yaw << std::endl;

//     // solution space adjusting 
//     if (current_joint[5] > 90 && delta_yaw > 0) {
//         //
//         float tvel[2] = { 70, 70 };
//         float tacc[2] = { 120, 120 };
//         current_position[2] += 100;

//         float goal_joint[NUMBER_OF_JOINT];

//         LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);

//         for (int i = 0; i < 6; i++)
//         {
//         goal_joint[i] = res->_fTargetPos[i];
//         }

//         Drfl_.movej(goal_joint,60,30);


//         robot_data = Drfl_.read_data_rt();

//         memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

//         current_joint[5] -= 360;

//         Drfl_.movej(current_joint, 60, 30);


//         robot_data = Drfl_.read_data_rt();

//         memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

//         current_position[2] -= 100;

//         LPINVERSE_KINEMATIC_RESPONSE res2 = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);

//         for (int i = 0; i < 6; i++)
//         {
//         goal_joint[i] = res2->_fTargetPos[i];
//         }

//         Drfl_.movej(goal_joint,60,30);
//         // waitForMotionCompletionWithRetry(goal_joint, 0.01, 5000, 3); 
//         // Drfl_.movel(current_position,tvel,tacc);
//         ///
//         // current_joint[5] -= 360;
//         // std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
//         // Drfl_.movej(current_joint, 60, 30);
//         // waitForMotionCompletionWithRetry(current_joint, 0.01, 5000, 3); 
//     }
    
//     else if (current_joint[5] < -90 && delta_yaw < 0) {

//         float tvel[2] = { 70, 70 };
//         float tacc[2] = { 120, 120 };
//         current_position[2] += 100;

//         float goal_joint[NUMBER_OF_JOINT];

//         LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);

//         for (int i = 0; i < 6; i++)
//         {
//         goal_joint[i] = res->_fTargetPos[i];
//         }

//         Drfl_.movej(goal_joint,60,30);
//         // waitForMotionCompletionWithRetry(goal_joint, 0.01, 5000, 3); 

//         robot_data = Drfl_.read_data_rt();

//         memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));

//         current_joint[5] += 360;

//         Drfl_.movej(current_joint, 60, 30);
//         // waitForMotionCompletionWithRetry(current_joint, 0.01, 5000, 3); 

//         robot_data = Drfl_.read_data_rt();

//         memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

//         current_position[2] -= 100;

//         LPINVERSE_KINEMATIC_RESPONSE res2 = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);

//         for (int i = 0; i < 6; i++)
//         {
//         goal_joint[i] = res2->_fTargetPos[i];
//         }

//         Drfl_.movej(goal_joint,60,30);
//         // waitForMotionCompletionWithRetry(goal_joint, 0.01, 5000, 3); 
        
//         //

//         // current_joint[5] += 360;
//         // std::cout << "Adjusted final joint 6 position: " << current_joint[5] << " - Moving joint..." << std::endl;
//         // Drfl_.movej(current_joint, 60, 30);
//         // waitForMotionCompletionWithRetry(current_joint, 0.01, 5000, 3); 
//     }
    
//     else {
//         std::cout << "No adjustment needed for joint 6: " << current_joint[5] << std::endl;
//     }
    
//     robot_data = Drfl_.read_data_rt(); // reading real time data 
//     memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//     auto start = std::chrono::high_resolution_clock::now();

//     // float x = 0;
//     float tvel[2] = { 70, 70 };
//     float tacc[2] = { 120, 120 };
//     float initiate_pos[NUMBER_OF_JOINT] = {0, };

//     // Original initiate pos
//     initiate_pos[0] = msg.points[0].point.pose.position.x;
//     initiate_pos[1] = msg.points[0].point.pose.position.y;
//     initiate_pos[2] = msg.points[0].point.pose.position.z;
//     initiate_pos[3] = current_position[3];
//     initiate_pos[4] = current_position[4];
//     // initiate_pos[5] = msg.points[0].point.pose.orientation.z;


//     float distance = std::sqrt(
//         std::pow(initiate_pos[0] - current_position[0], 2) +
//         std::pow(initiate_pos[1] - current_position[1], 2) +
//         std::pow(initiate_pos[2] - current_position[2], 2)
//     );

//     if (distance > distance_threshold) {
//         ROS_ERROR("Error: The distance between current position and trajectory start point exceeds the threshold (distance = %f)", distance);
//         fail = 2;
//         return;
//     }

//     robot_data = Drfl_.read_data_rt(); // reading real time data 
    
//     memcpy(current_joint,robot_data->actual_joint_position,NUMBER_OF_JOINT*sizeof(float));
//     memcpy(current_position,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));

//     trajectory_gen_.CurvePoints_ = trajectory_gen_.upsampleTrajectory(msg, newPointsNum);

    
//     Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE); // mode setting for control 
//     Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    
//     controlState = true; 


//     while (spinMotion_path(robot_state, control_loop_time, desired, sol_space) && spinControl(robot_state, control_loop_time, control_command, desired, sol_space)) {
        
//         robot_state = Drfl_.read_data_rt();
        
//         if (!exitLoop && !g_nKill_dsr_control) {

//             Drfl_.torque_rt(control_command.tau_d, st);

//             auto current = std::chrono::high_resolution_clock::now();
//             auto loop_time = std::chrono::duration_cast<std::chrono::milliseconds>(current - start);

            

//             if (std::chrono::milliseconds(static_cast<int64_t>(control_loop_time.toMSec())) > loop_time) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(control_loop_time.toMSec())) - loop_time);
//             }

//             start = std::chrono::high_resolution_clock::now();
            
//         } else {
//             fail = true;
//             std::cout << SKKU::fail << std::endl;
//             break;
//         }

//         stopDataSaving();
//         count++;
//     }

//     previous_msg = msg;

//     controlState = false;

//     setScheduling(originalSetting_);
//     float final_position[NUMBER_OF_JOINT] = {0, };
//     memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//     distance = std::sqrt(
//         std::pow(final_position[0] - msg.points[originalPointsNum - 1].point.pose.position.x, 2) +
//         std::pow(final_position[1] - msg.points[originalPointsNum - 1].point.pose.position.y, 2) +
//         std::pow(final_position[2] - msg.points[originalPointsNum - 1].point.pose.position.z, 2)
//     );

//     if (distance > distance_threshold) { // 2.5 cm
//         ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
//         fail = 2;
//     }
//     else if (distance <= distance_threshold) {
//         ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
//         fail = 1;
//     }

//     return;
// }



// // void ControlLoop::GainMove(){//주의 : GRIPPER 가 쫙 펴진 상태라면 WORKSPACE 범위를 줄여야 함. gripper가 어느 정도 접혀진 상태에서 만든 workspace임.
// // //workspace new

// // int step = 5;
// // float tTime = 5;
// // float tvel[2] = { 70, 70 };
// // float tacc[2] = { 120, 120 };
// // float ztop = 700;
// // float zbottom = 390;

// // // TABLE
// // // float check_X1[6] = {-375,  520,  700,  0,  -180, 3.42};
// // // float check_X2[6] = {325,   520,  700,  0,  -180, 3.42};
// // // float check_X3[6] = {325,   940,  700,  0,  -180, 3.42};
// // // float check_X4[6] = {-375,   940,  700,  0,  -180, 3.42};
// // // float zdp = (700-230) / step;

// // // TUB
// // float check_X1[6] = {500,  -180,  600,  0,  -180, 3.42};
// // float check_X2[6] = {770,  -180,  600,  0,  -180, 3.42};
// // float check_X3[6] = {770,   180,  600,  0,  -180, 3.42};
// // float check_X4[6] = {500,   180,  600,  0,  -180, 3.42};
// // float zdp = (ztop - zbottom) / step;

// // float xdp = (check_X3[0] - check_X1[0])/ (2*step);
// // float ydp = (check_X3[1] - check_X1[1]) / (2*step);


// // for (int i = 0; i < step+1; ++i)
// // {
// //     check_X1[2] = ztop - i*zdp;
// //     check_X2[2] = ztop - i*zdp;
// //     check_X3[2] = ztop - i*zdp;
// //     check_X4[2] = ztop - i*zdp;

// //     for (int j = 0; j < step; ++j)
// //     {
// //     // Drfl_.movejx(check_X1,2,0,0,tTime);
// //     // Drfl_.movejx(check_X2,2,0,0,tTime);
// //     // Drfl_.movejx(check_X3,2,0,0,tTime);
// //     // Drfl_.movejx(check_X4,2,0,0,tTime);
// //     std::cout<<"Moving X1 :"<<i+1<<"th height, "<<j+1<<"th step !"<<std::endl;
// //     Drfl_.movel(check_X1,tvel,tacc);
// //     std::cout<<"Moving X2 :"<<i+1<<"th height, "<<j+1<<"th step !"<<std::endl;
// //     Drfl_.movel(check_X2,tvel,tacc);
// //     std::cout<<"Moving X3 :"<<i+1<<"th height, "<<j+1<<"th step !"<<std::endl;
// //     Drfl_.movel(check_X3,tvel,tacc);
// //     std::cout<<"Moving X4 :"<<i+1<<"th height, "<<j+1<<"th step !"<<std::endl;
// //     Drfl_.movel(check_X4,tvel,tacc);
// //     std::cout<<"Moving X1 :"<<i+1<<"th height, "<<j+1<<"th step !"<<std::endl;
// //     Drfl_.movel(check_X1,tvel,tacc);
// //     check_X1[0] += xdp; check_X1[1] += ydp;
// //     check_X2[0] -= xdp; check_X2[1] += ydp;
// //     check_X3[0] -= xdp; check_X3[1] -= ydp;
// //     check_X4[0] += xdp; check_X4[1] -= ydp;

// //     }
// //     Drfl_.movejx(check_X1,2,5,10);
// //     check_X1[0] -= step*xdp; check_X1[1] -= step*ydp;
// //     check_X2[0] += step*xdp; check_X2[1] -= step*ydp;
// //     check_X3[0] += step*xdp; check_X3[1] += step*ydp;
// //     check_X4[0] -= step*xdp; check_X4[1] += step*ydp;
// // }


// // gaincheckloop = true;
// // return; 
// // }

// void ControlLoop::GainMove() {
//     // General settings
//     float step = 4;
//     float tTime = 5;
//     float tvel[2] = { 70, 70 };
//     float tacc[2] = { 120, 120 };

//     // TUB and TABLE configurations
//     struct Config {
//         float ztop;
//         float check_X1[6];
//         float check_X2[6];
//         float check_X3[6];
//         float check_X4[6];
//         float zdp;
//         float xdp;
//         float ydp;
//     };

//     std::vector<Config> configurations;

//     // TUB configuration
//     // configurations.push_back({
//     //     {500, -180, 600, 0, -180, 3.42},  // X1
//     //     {770, -180, 600, 0, -180, 3.42},  // X2
//     //     {770, 180, 600, 0, -180, 3.42},   // X3
//     //     {500, 180, 600, 0, -180, 3.42},   // X4
//     //     (700 - 390) / step,               // zdp
//     //     (770 - 500) / (2 * step),         // xdp
//     //     (180 - -180) / (2 * step)         // ydp
//     //     });

//     configurations.push_back({
//         600,
//         {300, -400, 600, 0, -180, 3.42},  // X1
//         {900, -400, 600, 0, -180, 3.42},  // X2
//         {900, 400, 600, 0, -180, 3.42},   // X3
//         {300, 400, 600, 0, -180, 3.42},   // X4
//         (600 - 360) / step,               // zdp
//         (900 - 300) / (2 * step),         // xdp
//         (400  + 400) / (2 * step)         // ydp
//         });

//     // TABLE configuration
//     // configurations.push_back({
//     //     700,
//     //     {400, -300, 700, 0, -180, 3.42},  // X1
//     //     {1000, -300, 700, 0, -180, 3.42},   // X2
//     //     {1000, 350, 700, 0, -180, 3.42},   // X3
//     //     {400, 350, 700, 0, -180, 3.42},  // X4
//     //     (700 - 360) / step,               // zdp
//     //     (1000 - 400) / (2 * step),        // xdp
//     //     (350 + 300) / (2 * step)          // ydp
//     //     });

//     // Execute gainmove for both configurations
//     for (const auto& config : configurations) {
//         float X1[6], X2[6], X3[6], X4[6];
//         float ztop = config.ztop;
//         memcpy(X1, config.check_X1, sizeof(config.check_X1));
//         memcpy(X2, config.check_X2, sizeof(config.check_X2));
//         memcpy(X3, config.check_X3, sizeof(config.check_X3));
//         memcpy(X4, config.check_X4, sizeof(config.check_X4));

//         for (int i = 0; i < step + 1; ++i) {
//             X1[2] = ztop - i * config.zdp;
//             X2[2] = ztop - i * config.zdp;
//             X3[2] = ztop - i * config.zdp;
//             X4[2] = ztop - i * config.zdp;

//             for (int j = 0; j < step; ++j) {
//                 std::cout << "Moving X1: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
//                 Drfl_.movel(X1, tvel, tacc);

//                 std::cout << "Moving X2: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
//                 Drfl_.movel(X2, tvel, tacc);

//                 std::cout << "Moving X3: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
//                 Drfl_.movel(X3, tvel, tacc);

//                 std::cout << "Moving X4: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
//                 Drfl_.movel(X4, tvel, tacc);

//                 std::cout << "Returning to X1: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
//                 Drfl_.movel(X1, tvel, tacc);

//                 X1[0] += config.xdp;
//                 X1[1] += config.ydp;
//                 X2[0] -= config.xdp;
//                 X2[1] += config.ydp;
//                 X3[0] -= config.xdp;
//                 X3[1] -= config.ydp;
//                 X4[0] += config.xdp;
//                 X4[1] -= config.ydp;
//             }

//             Drfl_.movejx(X1, 2, 5, 10);
//             X1[0] -= step * config.xdp;
//             X1[1] -= step * config.ydp;
//             X2[0] += step * config.xdp;
//             X2[1] -= step * config.ydp;
//             X3[0] += step * config.xdp;
//             X3[1] += step * config.ydp;
//             X4[0] -= step * config.xdp;
//             X4[1] += step * config.ydp;
//         }
//     }

//     gaincheckloop = true;
//     return;
// }

// void ControlLoop::returnToHome() {

//     // Move to home position
//     Drfl_.movej(home_abs, 60, 30);
    
//     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
//     memcpy(home_abs,robot_data->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));
//     Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
//     Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
// }

// //Help function
// namespace {
// constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

// float unwrapNear(float angle_deg, float ref_deg) {
//     while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
//     while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
//     return angle_deg;
// }

// Eigen::Quaternionf quatFromPose7(const std::array<float, 7>& pose7) {
//     Eigen::Quaternionf q(pose7[6], pose7[3], pose7[4], pose7[5]); // w,x,y,z
//     if (q.norm() < 1e-6f) return Eigen::Quaternionf::Identity();
//     q.normalize();
//     return q;
// }

// float quatMsgToYawDeg(const geometry_msgs::Quaternion& qmsg) {
//     Eigen::Quaternionf q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
//     if (q.norm() < 1e-6f) return 0.0f;
//     q.normalize();
//     Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
//     return euler[0] * RAD2DEG; // yaw
// }

// void fillEulerDummyForIK(const Trajectory& src_quat, Trajectory& dst_euler) {
//     dst_euler = src_quat;

//     static bool first = true;
//     static float prev_rpy[3] = {0.f, 0.f, 0.f};

//     Eigen::Quaternionf q = quatFromPose7(src_quat.pos_d);
//     Eigen::Vector3f euler_zyx = q.toRotationMatrix().eulerAngles(2, 1, 0);

//     float roll  = euler_zyx[2] * RAD2DEG;
//     float pitch = euler_zyx[1] * RAD2DEG;
//     float yaw   = euler_zyx[0] * RAD2DEG;

//     if (!first) {
//         roll  = unwrapNear(roll,  prev_rpy[0]);
//         pitch = unwrapNear(pitch, prev_rpy[1]);
//         yaw   = unwrapNear(yaw,   prev_rpy[2]);
//     } else {
//         first = false;
//     }

//     prev_rpy[0] = roll;
//     prev_rpy[1] = pitch;
//     prev_rpy[2] = yaw;

//     dst_euler.pos_d[3] = roll;
//     dst_euler.pos_d[4] = pitch;
//     dst_euler.pos_d[5] = yaw;

//     // MotionGenerator가 quaternion rate를 Euler rate로 오해하지 않도록
//     dst_euler.vel_d[3] = 0.0f;
//     dst_euler.vel_d[4] = 0.0f;
//     dst_euler.vel_d[5] = 0.0f;
//     dst_euler.acc_d[3] = 0.0f;
//     dst_euler.acc_d[4] = 0.0f;
//     dst_euler.acc_d[5] = 0.0f;
// }
// }

// // bool ControlLoop::spinMotion(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Desired& desired, int sol_space) {
// //     // 1. 현재 루프 시간 계산 (0초부터 시작)
// //     tra.time = static_cast<double>(count) * loop_time_ / 1000.0;
    
// //     trajectory_gen_.setLoopTime(loop_time_);
// //     bool correction_flag = false; // 보정 구간을 없앴으므로 항상 false

// //     // 2. 트라젝토리 실행 (계획된 시간 동안만 수행)
// //     if (tra.time <= plan.time) {
// //         // 즉시 원래 트라젝토리 포인트 생성
// //         trajectory_gen_.TrajectoryGenerator(&plan, &tra);

// //         // 7개 성분(위치 3 + 쿼터니언 4)을 trajectory 구조체에 할당
// //         for (int i = 0; i < 7; ++i) {
// //             trajectory.pos_d[i] = tra.pos[i];
// //             trajectory.vel_d[i] = tra.vel[i];
// //             trajectory.acc_d[i] = tra.acc[i];
// //         }
// //         Trajectory dummy_traj_6d = trajectory;
// //         // MotionGenerator를 통해 조인트 목표값 계산
// //         auto [output, is_singular] = MotionGenerator(dummy_traj_6d, robot_state, prev, imp, sol_space, correction_flag, operator_call_count_);
        
// //         if (is_singular) {
// //             std::cout << "Singularity occurred! Exiting loop." << std::endl;
// //             return false;
// //         }
        
// //         desired.q_d = output;
// //         return true; // 제어 루프 유지
// //     }
// //     // 3. 트라젝토리 종료 (시간 초과 시 즉시 탈출)
// //     else {
// //         std::cout << "Trajectory duration reached. Motion finished." << std::endl;
// //         return false; 
// //     }
// // }

// bool ControlLoop::spinMotion(const LPRT_OUTPUT_DATA_LIST& robot_state,
//                              SKKU::Duration time_step,
//                              Desired& desired,
//                              int sol_space) {
//     tra.time = static_cast<double>(count) * loop_time_ / 1000.0;
//     trajectory_gen_.setLoopTime(loop_time_);
//     bool correction_flag = false;

//     if (tra.time <= plan.time) {
//         trajectory_gen_.TrajectoryGenerator(&plan, &tra);

//         for (int i = 0; i < 7; ++i) {
//             trajectory.pos_d[i] = tra.pos[i];
//             trajectory.vel_d[i] = tra.vel[i];
//             trajectory.acc_d[i] = tra.acc[i];
//         }

//         Trajectory dummy_traj_for_ik;
//         fillEulerDummyForIK(trajectory, dummy_traj_for_ik);

//         auto [output, is_singular] =
//             MotionGenerator(dummy_traj_for_ik, robot_state, prev, imp,
//                             sol_space, correction_flag, operator_call_count_);

//         if (is_singular) {
//             std::cout << "Singularity occurred! Exiting loop." << std::endl;
//             return false;
//         }

//         desired.q_d = output;
//         return true;
//     }

//     std::cout << "Trajectory duration reached. Motion finished." << std::endl;
//     return false;
// }

// // bool ControlLoop::spinMotion_path(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Desired& desired, int sol_space) {
// //     bool correction_flag = false;
// //     if (count < trajectory_gen_.CurvePoints_.size()) {
// //         auto& point = trajectory_gen_.CurvePoints_[count];

// //         // for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
// //         //     trajectory.pos_d[i] = point[i];
// //         //     trajectory.vel_d[i] = 0; 
// //         //     trajectory.acc_d[i] = 0; 
// //         // }

// //         for (int i = 0; i < 7; ++i) { // NUMBER_OF_JOINT를 7로 변경!
// //         trajectory.pos_d[i] = point[i]; // w 성분(위치) 할당
// //         trajectory.vel_d[i] = 0;        // w 성분(속도) 0으로 안전하게 초기화
// //         trajectory.acc_d[i] = 0;        // w 성분(가속도) 0으로 안전하게 초기화
// //     }

// //         auto [output, is_singular] = MotionGenerator(trajectory, robot_state, prev, imp, sol_space,correction_flag,operator_call_count_);
// //         if (is_singular) {
// //             return false;  
// //         }
// //         desired.q_d = output;

// //         return !desired.motion_finished;
// //     } else {
// //         return desired.motion_finished;
// //     }
// // }

// bool ControlLoop::spinMotion_path(const LPRT_OUTPUT_DATA_LIST& robot_state,
//                                   SKKU::Duration time_step,
//                                   Desired& desired,
//                                   int sol_space) {
//     bool correction_flag = false;

//     if (count < trajectory_gen_.CurvePoints_.size()) {
//         auto& point = trajectory_gen_.CurvePoints_[count];

//         for (int i = 0; i < 7; ++i) {
//             trajectory.pos_d[i] = point[i];
//             trajectory.vel_d[i] = 0.0f;
//             trajectory.acc_d[i] = 0.0f;
//         }

//         Trajectory dummy_traj_for_ik;
//         fillEulerDummyForIK(trajectory, dummy_traj_for_ik);

//         auto [output, is_singular] =
//             MotionGenerator(dummy_traj_for_ik, robot_state, prev, imp,
//                             sol_space, correction_flag, operator_call_count_);

//         if (is_singular) {
//             return false;
//         }

//         desired.q_d = output;
//         return !desired.motion_finished;
//     }

//     return desired.motion_finished;
// }

// bool ControlLoop::spinControl(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Torques& command, Desired& desired, int sol_space) {
//     //pthread_mutex_lock(&mutex);

//     Torques control_output = ControlGenerator(trajectory,desired, robot_state, errors,count);

//     /*
//     if (limit_rate_) {
//         control_output.tau_d = limitRate(kMaxTorqueRate, control_output.tau_J, robot_state.tau_J_d);
//     }
//     */

//     for (int i = 0; i < 6; i++) {
//         command.tau_d[i] = control_output.tau_d[i];
//     }

//     //checkFinite(command.tau_d);
//     // pthread_mutex_unlock(&mutex);

    
//     return !command.motion_finished;

// }
// void ControlLoop::saveLoopTimesToFile(const std::string& filePath) {
//     std::ofstream loopTimeFile(filePath, std::ios::app);
//     if (loopTimeFile.is_open()) {
//         for (const auto& time : loopTimes) {
//             loopTimeFile << time << "\n";
//         }
//         loopTimeFile.close();
//         std::cout << "Loop times saved to " << filePath << std::endl;
//     } else {
//         std::cerr << "Failed to open " << filePath << std::endl;
//     }
// }
// void ControlLoop::logData(const std::string& fileName, const float* data, int dataSize) {
//     // std::cout<<"OPERATOR_COUNT : "<< operator_call_count_<<", isDirCreated : "<<isDirectoryCreated<<std::endl;
//     if (!isDirectoryCreated && (operator_call_count_ == 1)) {
//         createNewDataDirectory();
//         isDirectoryCreated = true;  // 디렉토리 생성 후 플래그를 true로 변경
//     }

//     const std::string fullPath = dataDirectory +  "/" +fileName;

//     std::ofstream file(fullPath, std::ios::app);  // 항상 append 모드로 열기
    
//     if (file.is_open()) {
//         for (int i = 0; i < dataSize - 1; ++i) {
//             file << data[i] << "\t";
//         }
//         // Write the last element without the delimiter
//         file << data[dataSize - 1] << std::endl;
//         file.close();
//     } else {
//         std::cerr << "Unable to open file: " << fullPath << " | Error: " << strerror(errno) << std::endl;
//     }

// }

// void ControlLoop::logMatrixData(const std::string& fileName, const float matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT], int rows, int cols) {

//     if (!isDirectoryCreated && (operator_call_count_ == 1)) {
//         createNewDataDirectory();
//         isDirectoryCreated = true;  // 디렉토리 생성 후 플래그를 true로 변경
//     }

//     const std::string fullPath = dataDirectory + "/" + fileName;

//     std::ofstream file(fullPath, std::ios::app);  // 항상 append 모드로 열기
    
//     if (file.is_open()) {
//         for (int i = 0; i < rows; ++i) {
//             for (int j = 0; j < cols; ++j) {
//                 file << std::setw(10) << matrix[i][j];
//                 if (j < cols - 1) {
//                     file << "\t";
//                 }
//             }
//             file << std::endl;
//         }
//         file.close();
//     } else {
//         std::cerr << "Unable to open file: " << fullPath << " | Error: " << strerror(errno) << std::endl;
//     }
// }

// void ControlLoop::logMatrixData3x3(const std::string& fileName, const float matrix[3][3], int rows, int cols) {
//     if (!isDirectoryCreated && (operator_call_count_ == 1)) {
//         createNewDataDirectory();
//         isDirectoryCreated = true;  // 디렉토리 생성 후 플래그를 true로 변경
//     }

//     const std::string fullPath = dataDirectory + "/" + fileName;

//     std::ofstream file(fullPath, std::ios::app);  // 항상 append 모드로 열기
    
//     if (file.is_open()) {
//         for (int i = 0; i < rows; ++i) {
//             for (int j = 0; j < cols; ++j) {
//                 file << std::setw(10) << matrix[i][j];
//                 if (j < cols - 1) {
//                     file << "\t";
//                 }
//             }
//             file << std::endl;
//         }
//         file.close();
//     } else {
//         std::cerr << "Unable to open file: " << fullPath << " | Error: " << strerror(errno) << std::endl;
//     }
// }


// // void ControlLoop::gaindataSavingThread() {
// //     float gravity_torque[NUMBER_OF_JOINT] = {0,};
// //     float actual_position[NUMBER_OF_JOINT] = {0,};
// //     float raw_torque[NUMBER_OF_JOINT] = {0,};
// //     float external_torque[NUMBER_OF_JOINT] = {0,};
// //     float traj_position[NUMBER_OF_JOINT] = {0,};
// //     float actual_positionj[NUMBER_OF_JOINT] = {0,};
// //     float impedance_position[NUMBER_OF_JOINT] = {0,};
// //     float F_external[NUMBER_OF_JOINT] = {0,};
// //     float position_command[NUMBER_OF_JOINT] = {0,};
// //     float F_impedance[NUMBER_OF_JOINT] = {0,};
// //     Duration control_loop_time = Duration(loop_time_);
// //     float F_external_box[NUMBER_OF_JOINT] = {0,};
// //     float actual_position2[NUMBER_OF_JOINT] = {0,};
// //     float joint_error[NUMBER_OF_JOINT] = {0,};
// //     float position_error[NUMBER_OF_JOINT] = {0,};

// //     float time[1] = {0,};

// //     auto start = std::chrono::high_resolution_clock::now();
// //     bool correction_flag = false;
// //     while (!gaincheckloop){
// //         /////Fext의 값을 가져오기 위해서 필요한 부분//////////////////
// //         LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();
// //         MotionGenerator(trajectory, robot_state, prev, imp, sol_space,correction_flag,operator_call_count_);
// //         ///////////////////////////////////////////////////////

// //         LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
// //         memcpy(gravity_torque, robot_data->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(actual_position2, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(raw_torque, robot_data->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(external_torque, robot_data->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(actual_positionj, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
// //         LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
// //         for(int i=0; i<6; i++){
// //             actual_position[i] = res->_fPosition[i];
// //         }
// //         convertToArray(trajectory.pos_d, traj_position);
// //         for (int i = 0; i<6 ; i++){
// //         impedance_position[i] = imp.pos_m(i);
// //         position_command[i] = desired.q_d[i];
// //         F_external[i] = F.Fext[i];
// //         F_impedance[i] = F.Fimp[i];
// //         joint_error[i] = position_command[i] - actual_positionj[i];

// //         time[0] += dt;

// //         }

// //         logData("time.txt",time,1);
        
// //         logData("task_position.txt", actual_position, NUMBER_OF_JOINT);
// //         // logData("task_trajectory.txt", traj_position, NUMBER_OF_JOINT);
// //         logData("task_trajectory.txt", traj_position_6d, NUMBER_OF_JOINT); // ⬅️ 이렇게 수정!
    
// //         logData("joint_position.txt", actual_positionj, NUMBER_OF_JOINT);
// //         logData("joint_command.txt", position_command, NUMBER_OF_JOINT);

// //         logData("raw_torque.txt", raw_torque, NUMBER_OF_JOINT);
// //         logData("command_torque.txt", control_command.tau_d, NUMBER_OF_JOINT);
// //         logData("gravity_torque.txt", gravity_torque, NUMBER_OF_JOINT);
// //         logData("external_torque.txt", external_torque, NUMBER_OF_JOINT);
// //         logData("force_external.txt",F_external, NUMBER_OF_JOINT);
        
    

// //         auto current = std::chrono::high_resolution_clock::now();
        
// //         Duration save_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
    
// //         if (control_loop_time > save_time) {
// //         std::this_thread::sleep_for(control_loop_time() - save_time());
// //         }
// //         start = std::chrono::high_resolution_clock::now();
// //     }
// // }

// void ControlLoop::gaindataSavingThread() {
//     float gravity_torque[NUMBER_OF_JOINT] = {0,};
//     float actual_position[NUMBER_OF_JOINT] = {0,};
//     float raw_torque[NUMBER_OF_JOINT] = {0,};
//     float external_torque[NUMBER_OF_JOINT] = {0,};
    
//     // ---------------------------------------------------------
//     // [수정 1] 궤적 배열 크기를 6(NUMBER_OF_JOINT)에서 7로 확장
//     // ---------------------------------------------------------
//     float traj_position[7] = {0,}; 
    
//     float actual_positionj[NUMBER_OF_JOINT] = {0,};
//     float impedance_position[NUMBER_OF_JOINT] = {0,};
//     float F_external[NUMBER_OF_JOINT] = {0,};
//     float position_command[NUMBER_OF_JOINT] = {0,};
//     float F_impedance[NUMBER_OF_JOINT] = {0,};
//     Duration control_loop_time = Duration(loop_time_);
//     float F_external_box[NUMBER_OF_JOINT] = {0,};
//     float actual_position2[NUMBER_OF_JOINT] = {0,};
//     float joint_error[NUMBER_OF_JOINT] = {0,};
//     float position_error[NUMBER_OF_JOINT] = {0,};

//     float time[1] = {0,};

//     auto start = std::chrono::high_resolution_clock::now();
//     bool correction_flag = false;
//     while (!gaincheckloop){
//         /////Fext의 값을 가져오기 위해서 필요한 부분//////////////////
//         LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();
//         MotionGenerator(trajectory, robot_state, prev, imp, sol_space,correction_flag,operator_call_count_);
//         ///////////////////////////////////////////////////////

//         LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
//         memcpy(gravity_torque, robot_data->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(actual_position2, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(raw_torque, robot_data->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(external_torque, robot_data->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(actual_positionj, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
//         LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
//         for(int i=0; i<6; i++){
//             actual_position[i] = res->_fPosition[i];
//         }
        
//         // 이제 traj_position이 7칸이므로 7차원 데이터가 안전하게 복사됨
//         convertToArray(trajectory.pos_d, traj_position);
        
//         // ---------------------------------------------------------
//         // [수정 2] 7차원 쿼터니언을 6차원 오일러(Degree)로 변환 (로깅용)
//         // ---------------------------------------------------------
//         float traj_position_6d[6] = {0,};
//         traj_position_6d[0] = traj_position[0]; // X
//         traj_position_6d[1] = traj_position[1]; // Y
//         traj_position_6d[2] = traj_position[2]; // Z

//         // 쿼터니언 조립 (주의: Eigen 순서는 w, x, y, z)
//         Eigen::Quaternionf q_gain_log(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
//         q_gain_log.normalize();
//         Eigen::Vector3f euler_gain = q_gain_log.toRotationMatrix().eulerAngles(2, 1, 0);

//         traj_position_6d[3] = euler_gain[2] * 180.0f / M_PI; // Roll
//         traj_position_6d[4] = euler_gain[1] * 180.0f / M_PI; // Pitch
//         traj_position_6d[5] = euler_gain[0] * 180.0f / M_PI; // Yaw
//         // ---------------------------------------------------------

//         for (int i = 0; i<6 ; i++){
//         impedance_position[i] = imp.pos_m(i);
//         position_command[i] = desired.q_d[i];
//         F_external[i] = F.Fext[i];
//         F_impedance[i] = F.Fimp[i];
//         joint_error[i] = position_command[i] - actual_positionj[i];

//         time[0] += dt;

//         }

//         logData("time.txt",time,1);
        
//         logData("task_position.txt", actual_position, NUMBER_OF_JOINT);
//         // 방금 만든 6차원 오일러 배열(traj_position_6d)을 텍스트 파일에 기록!
//         logData("task_trajectory.txt", traj_position_6d, NUMBER_OF_JOINT); 
    
//         logData("joint_position.txt", actual_positionj, NUMBER_OF_JOINT);
//         logData("joint_command.txt", position_command, NUMBER_OF_JOINT);

//         logData("raw_torque.txt", raw_torque, NUMBER_OF_JOINT);
//         logData("command_torque.txt", control_command.tau_d, NUMBER_OF_JOINT);
//         logData("gravity_torque.txt", gravity_torque, NUMBER_OF_JOINT);
//         logData("external_torque.txt", external_torque, NUMBER_OF_JOINT);
//         logData("force_external.txt",F_external, NUMBER_OF_JOINT);
        
    

//         auto current = std::chrono::high_resolution_clock::now();
        
//         Duration save_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
    
//         if (control_loop_time > save_time) {
//         std::this_thread::sleep_for(control_loop_time() - save_time());
//         }
//         start = std::chrono::high_resolution_clock::now();
//     }
// }

// // void ControlLoop::dataSaving() {
// //     float trq_g[NUMBER_OF_JOINT] = {0,};
// //     float actual_position[NUMBER_OF_JOINT] = {0,};
// //     float actual_velocity[NUMBER_OF_JOINT] = {0,};
// //     float trq_raw[NUMBER_OF_JOINT] = {0,};
// //     float trq_act[NUMBER_OF_JOINT] = {0,};
// //     // float traj_position[NUMBER_OF_JOINT] = {0,};
// //     // float traj_velocity[NUMBER_OF_JOINT] = {0,};
// //     // float traj_acceleration[NUMBER_OF_JOINT] = {0,};

// //     float traj_position[7] = {0,};
// //     float traj_velocity[7] = {0,};
// //     float traj_acceleration[7] = {0,};

// //     float actual_positionj[NUMBER_OF_JOINT] = {0,};
// //     float actual_velocityj[NUMBER_OF_JOINT] = {0,};
// //     float accelerationj[NUMBER_OF_JOINT] = {0,};
// //     float filtered_accelerationj[NUMBER_OF_JOINT] = {0,}; 
// //     float impedance_position[NUMBER_OF_JOINT] = {0,};
// //     float F_external[NUMBER_OF_JOINT] = {0,};
// //     float F_DBIC[NUMBER_OF_JOINT] = {0,};
// //     float F_rest[NUMBER_OF_JOINT] = {0,};
// //     float F_coriolis[NUMBER_OF_JOINT] = {0,};
// //     float position_command[NUMBER_OF_JOINT] = {0,};
// //     float trq_force[NUMBER_OF_JOINT] = {0,};
// //     float F_impedance[NUMBER_OF_JOINT] = {0,};
// //     Duration control_loop_time = Duration(loop_time_);
// //     float F_external_box[NUMBER_OF_JOINT] = {0,};
// //     float trq_ext[NUMBER_OF_JOINT] = {0,};
// //     float trq_ext_auto[NUMBER_OF_JOINT] = {0,};
// //     float trq_ext_cal[NUMBER_OF_JOINT] = {0,};
// //     float sensor_FT[NUMBER_OF_JOINT] = {0,};
// //     float sensor_FT_matched[NUMBER_OF_JOINT] = {0,};
// //     float actual_position2[NUMBER_OF_JOINT] = {0,};
// //     float joint_error[NUMBER_OF_JOINT] = {0,};
// //     float position_error[NUMBER_OF_JOINT] = {0,};
// //     float time[1] = {0,};
// //     float operator_count[1] = {static_cast<float>(operator_call_count_)};
// //     float controlMode[1];
// //     float alpha = 0.1;
// //     float actual_quat[4] = {0,};       // 현재 쿼터니언 저장용
// //     float traj_quat[4] = {0,};         // 목표 쿼터니언 저장용
// //     float orientation_error[3] = {0,};  // 쿼터니언 오차 벡터 저장용

// //     std::unordered_map<std::string, int> modeMap = {
// //         {"Position goal mode", 0},
// //         {"Impedance goal mode", 1},
// //         {"Position path mode", 2},
// //         {"Impedance path mode", 3}
// //     };

// //     auto it = modeMap.find(control_mode_);

// //     if (it != modeMap.end()) {
// //         switch (it->second) {
// //             case 0:
// //                 controlMode[0] = 0.0f;
// //                 break;
// //             case 1:
// //                 controlMode[0] = 1.0f;
// //                 break;
// //             case 2:
// //                 controlMode[0] = 2.0f;
// //                 break;
// //             case 3:
// //                 controlMode[0] = 3.0f;
// //                 break;
// //             default:
// //                 controlMode[0] = -1.0f; // Handle unexpected values
// //                 break;
// //         }
// //     } else {
// //         controlMode[0] = -1.0f; // Handle the case where control_mode_ is not found in the map
// //     }

// //     static Eigen::Quaternionf q_act_prev = Eigen::Quaternionf::Identity();
// //     static Eigen::Quaternionf q_des_prev = Eigen::Quaternionf::Identity();
// //     static bool is_first_run = true;

// //     // Matrix 
// //     float massMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
// //     float coriolisMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
// //     float jacobianMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
// //     float rotationMatrix[3][3] = {{0,}};
    
    
// //     auto start = std::chrono::high_resolution_clock::now();

// //     while (data_saving_running_){
        
// //         LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
        
// //         memcpy(trq_g, robot_data->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(actual_position2, robot_data->actual_tcp_position, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(trq_raw, robot_data->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(actual_positionj, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(actual_velocityj, robot_data->actual_joint_velocity, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(actual_velocity, robot_data->actual_flange_velocity, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(F_external_box, robot_data->external_tcp_force, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(trq_ext, robot_data->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(trq_force, robot_data->raw_force_torque, NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(trq_act, robot_data->actual_joint_torque, NUMBER_OF_JOINT * sizeof(float));


// //         for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
// //             accelerationj[i] = (actual_velocityj[i] - previous_velocityj[i]) / dt; // (v_now - v_prev) / delta_time
            
// //             // Apply an exponential moving average filter for smoothing
// //             filtered_accelerationj[i] = alpha * accelerationj[i] + (1 - alpha) * filtered_accelerationj[i];

// //             // Update the previous velocity for the next iteration
// //             previous_velocityj[i] = actual_velocityj[i];
// //         }
        
        
// //         memcpy(massMatrix, robot_data->mass_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(coriolisMatrix, robot_data->coriolis_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
// //         memcpy(jacobianMatrix, robot_data->jacobian_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
// //         float(*result)[3] = Drfl_.get_current_rotm();
// //         LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
// //         float gripper_torque[NUMBER_OF_JOINT] = {0,};

// //         LPROBOT_FORCE lpForce = Drfl_.get_external_torque();
// //         Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_ext2(lpForce->_fForce);

// //         // Swap rows 1-3 with rows 4-6 in Jacobian matrix
// //         Eigen::Map<Eigen::Matrix<float, 6, 6>> J(reinterpret_cast<float*>(jacobianMatrix));
// //         Eigen::Matrix<float, 6, 6> J_Tinv = J.transpose().inverse();

// //         float jacobian_transepose_inverse[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0}};

// //         for(int i=0; i<6; i++){
// //         actual_position[i] = res->_fPosition[i];
// //         // actual_velocity[i] = res->[i];
// //         }
// //         convertToArray(trajectory.pos_d, traj_position);
// //         convertToArray(trajectory.vel_d, traj_velocity);
// //         convertToArray(trajectory.acc_d, traj_acceleration);
// //         for (int i=0; i<3; i++)
// //         {
// //             for (int j=0; j<3; j++)
// //             {
// //                 rotationMatrix[i][j] = result[i][j];
// //             }
        
// //         }

// //         for (int i = 0; i<6 ; i++){
// //         impedance_position[i] = imp.pos_m(i);
// //         position_command[i] = desired.q_d[i];
// //         F_external[i] = F.Fext[i];
// //         F_impedance[i] = F.Fimp[i];
// //         F_DBIC[i] = F.F_DBIC[i];
// //         F_coriolis[i] = F.F_coriolis[i];
// //         F_rest[i] = F.F_rest[i];
// //         joint_error[i] = position_command[i] - actual_positionj[i];
// //         position_error[i] = impedance_position[i] - actual_position[i];
// //         time[0] += dt;
// //         gripper_torque[i] = trq_gg[i];
// //         trq_ext_auto[i] = trq_ext2[i];
// //         trq_ext_cal[i] = trq_raw[i]-trq_g[i];
// //         sensor_FT[i] = sensor_data.AFT_wrench_[i];
// //         sensor_FT_matched[i] = sensor_data.AFT_wrench_matched[i];
// //         }


// //         logData("filtered_acceleration.txt", filtered_accelerationj, NUMBER_OF_JOINT);
// //         logData("time.txt",time,1);
// //         logData("Control mode.txt", controlMode, 1); // 모드 정보 저장
// //         logData("gravity_torque.txt", trq_g, NUMBER_OF_JOINT);  
// //         logData("task_position.txt", actual_position, NUMBER_OF_JOINT);
// //         logData("actual_velocity.txt", actual_velocity, NUMBER_OF_JOINT);
// //         logData("task_position2.txt", actual_position2, NUMBER_OF_JOINT);
// //         logData("raw_joint_torque.txt", trq_raw, NUMBER_OF_JOINT);
// //         logData("task_trajectory.txt", traj_position, NUMBER_OF_JOINT);
// //         logData("task_velocity.txt", traj_velocity, NUMBER_OF_JOINT);
// //         logData("task_acceleration.txt", traj_acceleration, NUMBER_OF_JOINT);
// //         logData("joint_position.txt", actual_positionj, NUMBER_OF_JOINT);
// //         logData("joint_velocity.txt", actual_velocityj, NUMBER_OF_JOINT);
// //         logData("command_torque.txt", control_command.tau_d, NUMBER_OF_JOINT);
// //         logData("impedance_position.txt", impedance_position, NUMBER_OF_JOINT);
// //         logData("joint_command.txt", position_command, NUMBER_OF_JOINT);
// //         logData("force_external.txt",F_external, NUMBER_OF_JOINT);
// //         logData("force_dbic.txt",F_DBIC, NUMBER_OF_JOINT);
// //         logData("force_rest.txt",F_rest, NUMBER_OF_JOINT);
// //         logData("force_coriolis.txt",F_coriolis, NUMBER_OF_JOINT);
// //         logData("actual_joint_torque.txt",trq_act, NUMBER_OF_JOINT);
// //         logData("force_external_box.txt",F_external_box, NUMBER_OF_JOINT);
// //         logData("force_impedance.txt",F_impedance, NUMBER_OF_JOINT);
// //         logData("joint_error.txt",joint_error, NUMBER_OF_JOINT);
// //         logData("task_position_error.txt",position_error, NUMBER_OF_JOINT);
// //         logData("external_joint_torque.txt",trq_ext, NUMBER_OF_JOINT);
// //         logData("external_joint_torque_auto.txt",trq_ext_auto, NUMBER_OF_JOINT);
// //         logData("external_joint_torque_cal.txt",trq_ext_cal, NUMBER_OF_JOINT);
// //         logData("raw_force_torque.txt",trq_force, NUMBER_OF_JOINT);
// //         logData("gripper_torque.txt",gripper_torque, NUMBER_OF_JOINT);
// //         logData("sensor_FT.txt",sensor_FT, NUMBER_OF_JOINT);
// //         logData("sensor_FT_matched.txt",sensor_FT_matched, NUMBER_OF_JOINT);
// //         logData("operator_count.txt", operator_count, 1);
// //         logMatrixData("mass_matrix.txt", massMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
// //         logMatrixData("coriolis_matrix.txt", coriolisMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
// //         logMatrixData("jacobian_matrix.txt", jacobianMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
// //         logMatrixData3x3("rotation_matrix.txt", rotationMatrix, 3, 3);
        
// //         // // 1. 현재 RPY -> Quaternion 변환
// //         // Eigen::AngleAxisf rollAngle(actual_position[3] * M_PI / 180.0f, Eigen::Vector3f::UnitX());
// //         // Eigen::AngleAxisf pitchAngle(actual_position[4] * M_PI / 180.0f, Eigen::Vector3f::UnitY());
// //         // Eigen::AngleAxisf yawAngle(actual_position[5] * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
// //         // Eigen::Quaternionf q_act = yawAngle * pitchAngle * rollAngle;

// //         // // 2. 목표 RPY -> Quaternion 변환
// //         // Eigen::AngleAxisf rollDes(traj_position[3] * M_PI / 180.0f, Eigen::Vector3f::UnitX());
// //         // Eigen::AngleAxisf pitchDes(traj_position[4] * M_PI / 180.0f, Eigen::Vector3f::UnitY());
// //         // Eigen::AngleAxisf yawDes(traj_position[5] * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
// //         // Eigen::Quaternionf q_des = yawDes * pitchDes * rollDes;

// //         // 1. 현재 로봇 자세 (안전한 3x3 회전행렬을 쿼터니언으로 직접 변환)
// //         float(*rotm_ptr)[3] = Drfl_.get_current_rotm();
// //         Eigen::Matrix3f R_act;
// //         R_act << rotm_ptr[0][0], rotm_ptr[0][1], rotm_ptr[0][2],
// //                  rotm_ptr[1][0], rotm_ptr[1][1], rotm_ptr[1][2],
// //                  rotm_ptr[2][0], rotm_ptr[2][1], rotm_ptr[2][2];
// //         Eigen::Quaternionf q_act(R_act);
// //         q_act.normalize(); // 수치적 안정성을 위해 정규화

// //         // 2. 목표 궤적 자세 (각도 변환 없이 7차원 배열에서 직접 꺼내 조립)
// //         // 주의: Eigen 쿼터니언 생성자 입력 순서는 (w, x, y, z) 이므로,
// //         // 배열 인덱스 (6, 3, 4, 5) 순서로 넣어주어야 합니다!
// //         Eigen::Quaternionf q_des(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
// //         q_des.normalize(); // 수치적 안정성을 위해 정규화

// //         if (is_first_run) {
// //             q_act_prev = q_act;
// //             q_des_prev = q_des;
// //             is_first_run = false;
// //         } else {
// //             // 현재 값과 이전 값의 내적(dot)이 음수라면 부호가 뒤집힌 것이므로 반전시킴
// //             if (q_act.coeffs().dot(q_act_prev.coeffs()) < 0.0f) q_act.coeffs() *= -1.0f;
// //             if (q_des.coeffs().dot(q_des_prev.coeffs()) < 0.0f) q_des.coeffs() *= -1.0f;
            
// //             q_act_prev = q_act;
// //             q_des_prev = q_des;
// //         }
        
// //         // 3. 쿼터니언 기반 오차 계산 (Antipodal 보정 포함)
// //         if (q_des.coeffs().dot(q_act.coeffs()) < 0.0f) q_act.coeffs() << -q_act.coeffs();
// //         Eigen::Quaternionf q_err(q_act.inverse() * q_des);
// //         Eigen::Vector3f e_rot = q_act * q_err.vec();

// //         // 4. 저장용 배열에 복사
// //         actual_quat[0] = q_act.x(); actual_quat[1] = q_act.y(); actual_quat[2] = q_act.z(); actual_quat[3] = q_act.w();
// //         traj_quat[0] = q_des.x();   traj_quat[1] = q_des.y();   traj_quat[2] = q_des.z();   traj_quat[3] = q_des.w();
// //         orientation_error[0] = e_rot.x(); orientation_error[1] = e_rot.y(); orientation_error[2] = e_rot.z();

// //         // 5. 파일로 기록
// //         logData("actual_quaternion.txt", actual_quat, 4);
// //         logData("traj_quaternion.txt", traj_quat, 4);
// //         logData("quat_orientation_error.txt", orientation_error, 3);
        
// //         auto current = std::chrono::high_resolution_clock::now();
        
// //         Duration save_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
    
// //         if (control_loop_time > save_time) {
// //         std::this_thread::sleep_for(control_loop_time() - save_time());
// //         }
// //         start = std::chrono::high_resolution_clock::now();
// //     }
// // }

// void ControlLoop::dataSaving() {
//     float trq_g[NUMBER_OF_JOINT] = {0,};
//     float actual_position[NUMBER_OF_JOINT] = {0,};
//     float actual_velocity[NUMBER_OF_JOINT] = {0,};
//     float trq_raw[NUMBER_OF_JOINT] = {0,};
//     float trq_act[NUMBER_OF_JOINT] = {0,};

//     // ---------------------------------------------------------
//     // [수정 1] 7칸짜리 배열 초기화 문법 오류 수정 (중요!)
//     // ---------------------------------------------------------
//     float traj_position[7] = {0,};
//     float traj_velocity[7] = {0,};     // 반드시 {0,} 를 붙여야 쓰레기값이 안 들어갑니다.
//     float traj_acceleration[7] = {0,}; // 반드시 {0,} 를 붙여야 쓰레기값이 안 들어갑니다.

//     float actual_positionj[NUMBER_OF_JOINT] = {0,};
//     float actual_velocityj[NUMBER_OF_JOINT] = {0,};
//     float accelerationj[NUMBER_OF_JOINT] = {0,};
//     float filtered_accelerationj[NUMBER_OF_JOINT] = {0,}; 
//     float impedance_position[NUMBER_OF_JOINT] = {0,};
//     float F_external[NUMBER_OF_JOINT] = {0,};
//     float F_DBIC[NUMBER_OF_JOINT] = {0,};
//     float F_rest[NUMBER_OF_JOINT] = {0,};
//     float F_coriolis[NUMBER_OF_JOINT] = {0,};
//     float position_command[NUMBER_OF_JOINT] = {0,};
//     float trq_force[NUMBER_OF_JOINT] = {0,};
//     float F_impedance[NUMBER_OF_JOINT] = {0,};
//     Duration control_loop_time = Duration(loop_time_);
//     float F_external_box[NUMBER_OF_JOINT] = {0,};
//     float trq_ext[NUMBER_OF_JOINT] = {0,};
//     float trq_ext_auto[NUMBER_OF_JOINT] = {0,};
//     float trq_ext_cal[NUMBER_OF_JOINT] = {0,};
//     float sensor_FT[NUMBER_OF_JOINT] = {0,};
//     float sensor_FT_matched[NUMBER_OF_JOINT] = {0,};
//     float actual_position2[NUMBER_OF_JOINT] = {0,};
//     float joint_error[NUMBER_OF_JOINT] = {0,};
//     float position_error[NUMBER_OF_JOINT] = {0,};
//     float time[1] = {0,};
//     float operator_count[1] = {static_cast<float>(operator_call_count_)};
//     float controlMode[1];
//     float alpha = 0.1;
//     float actual_quat[4] = {0,};       // 현재 쿼터니언 저장용
//     float traj_quat[4] = {0,};         // 목표 쿼터니언 저장용
//     float orientation_error[3] = {0,};  // 쿼터니언 오차 벡터 저장용

//     std::unordered_map<std::string, int> modeMap = {
//         {"Position goal mode", 0},
//         {"Impedance goal mode", 1},
//         {"Position path mode", 2},
//         {"Impedance path mode", 3}
//     };

//     auto it = modeMap.find(control_mode_);

//     if (it != modeMap.end()) {
//         switch (it->second) {
//             case 0:
//                 controlMode[0] = 0.0f;
//                 break;
//             case 1:
//                 controlMode[0] = 1.0f;
//                 break;
//             case 2:
//                 controlMode[0] = 2.0f;
//                 break;
//             case 3:
//                 controlMode[0] = 3.0f;
//                 break;
//             default:
//                 controlMode[0] = -1.0f; // Handle unexpected values
//                 break;
//         }
//     } else {
//         controlMode[0] = -1.0f; // Handle the case where control_mode_ is not found in the map
//     }

//     static Eigen::Quaternionf q_act_prev = Eigen::Quaternionf::Identity();
//     static Eigen::Quaternionf q_des_prev = Eigen::Quaternionf::Identity();
//     static bool is_first_run = true;

//     // Matrix 
//     float massMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
//     float coriolisMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
//     float jacobianMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
//     float rotationMatrix[3][3] = {{0,}};
    
    
//     auto start = std::chrono::high_resolution_clock::now();

//     while (data_saving_running_){
        
//         LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
        
//         memcpy(trq_g, robot_data->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(actual_position2, robot_data->actual_tcp_position, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(trq_raw, robot_data->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(actual_positionj, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(actual_velocityj, robot_data->actual_joint_velocity, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(actual_velocity, robot_data->actual_flange_velocity, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(F_external_box, robot_data->external_tcp_force, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(trq_ext, robot_data->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(trq_force, robot_data->raw_force_torque, NUMBER_OF_JOINT * sizeof(float));
//         memcpy(trq_act, robot_data->actual_joint_torque, NUMBER_OF_JOINT * sizeof(float));


//         for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
//             accelerationj[i] = (actual_velocityj[i] - previous_velocityj[i]) / dt; // (v_now - v_prev) / delta_time
            
//             // Apply an exponential moving average filter for smoothing
//             filtered_accelerationj[i] = alpha * accelerationj[i] + (1 - alpha) * filtered_accelerationj[i];

//             // Update the previous velocity for the next iteration
//             previous_velocityj[i] = actual_velocityj[i];
//         }
        
        
//         memcpy(massMatrix, robot_data->mass_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
//         memcpy(coriolisMatrix, robot_data->coriolis_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
//         memcpy(jacobianMatrix, robot_data->jacobian_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
//         float(*result)[3] = Drfl_.get_current_rotm();
//         LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
//         float gripper_torque[NUMBER_OF_JOINT] = {0,};

//         LPROBOT_FORCE lpForce = Drfl_.get_external_torque();
//         Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_ext2(lpForce->_fForce);

//         // Swap rows 1-3 with rows 4-6 in Jacobian matrix
//         Eigen::Map<Eigen::Matrix<float, 6, 6>> J(reinterpret_cast<float*>(jacobianMatrix));
//         Eigen::Matrix<float, 6, 6> J_Tinv = J.transpose().inverse();

//         float jacobian_transepose_inverse[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0}};

//         for(int i=0; i<6; i++){
//         actual_position[i] = res->_fPosition[i];
//         // actual_velocity[i] = res->[i];
//         }
//         convertToArray(trajectory.pos_d, traj_position);
//         convertToArray(trajectory.vel_d, traj_velocity);
//         convertToArray(trajectory.acc_d, traj_acceleration);

//         // ---------------------------------------------------------
//         // [수정 2] 7차원 쿼터니언을 6차원 오일러(Degree)로 변환 (로깅용)
//         // ---------------------------------------------------------
//         float traj_position_6d[6] = {0,};
//         traj_position_6d[0] = traj_position[0]; 
//         traj_position_6d[1] = traj_position[1]; 
//         traj_position_6d[2] = traj_position[2]; 

//         Eigen::Quaternionf q_traj_log(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
//         q_traj_log.normalize();
        
//         // 원하시는 euler_traj_log 변수명으로 변경
//         Eigen::Vector3f euler_traj_log = q_traj_log.toRotationMatrix().eulerAngles(2, 1, 0);

//         // 아래 3줄도 모두 euler_traj_log에서 값을 가져오도록 이름 통일!
//         traj_position_6d[3] = euler_traj_log[2] * 180.0f / M_PI; // Roll
//         traj_position_6d[4] = euler_traj_log[1] * 180.0f / M_PI; // Pitch
//         traj_position_6d[5] = euler_traj_log[0] * 180.0f / M_PI; // Yaw
//         // ---------------------------------------------------------

//         for (int i=0; i<3; i++)
//         {
//             for (int j=0; j<3; j++)
//             {
//                 rotationMatrix[i][j] = result[i][j];
//             }
        
//         }

//         for (int i = 0; i<6 ; i++){
//         impedance_position[i] = imp.pos_m(i);
//         position_command[i] = desired.q_d[i];
//         F_external[i] = F.Fext[i];
//         F_impedance[i] = F.Fimp[i];
//         F_DBIC[i] = F.F_DBIC[i];
//         F_coriolis[i] = F.F_coriolis[i];
//         F_rest[i] = F.F_rest[i];
//         joint_error[i] = position_command[i] - actual_positionj[i];
//         position_error[i] = impedance_position[i] - actual_position[i];
//         time[0] += dt;
//         gripper_torque[i] = trq_gg[i];
//         trq_ext_auto[i] = trq_ext2[i];
//         trq_ext_cal[i] = trq_raw[i]-trq_g[i];
//         sensor_FT[i] = sensor_data.AFT_wrench_[i];
//         sensor_FT_matched[i] = sensor_data.AFT_wrench_matched[i];
//         }


//         logData("filtered_acceleration.txt", filtered_accelerationj, NUMBER_OF_JOINT);
//         logData("time.txt",time,1);
//         logData("Control mode.txt", controlMode, 1); 
//         logData("gravity_torque.txt", trq_g, NUMBER_OF_JOINT);  
//         logData("task_position.txt", actual_position, NUMBER_OF_JOINT);
//         logData("actual_velocity.txt", actual_velocity, NUMBER_OF_JOINT);
//         logData("task_position2.txt", actual_position2, NUMBER_OF_JOINT);
//         logData("raw_joint_torque.txt", trq_raw, NUMBER_OF_JOINT);

//         // [수정 3] 번역된 6차원 배열을 텍스트 파일에 저장!
//         logData("task_trajectory.txt", traj_position_6d, NUMBER_OF_JOINT); 
        
//         logData("task_velocity.txt", traj_velocity, NUMBER_OF_JOINT);
//         logData("task_acceleration.txt", traj_acceleration, NUMBER_OF_JOINT);
//         logData("joint_position.txt", actual_positionj, NUMBER_OF_JOINT);
//         logData("joint_velocity.txt", actual_velocityj, NUMBER_OF_JOINT);
//         logData("command_torque.txt", control_command.tau_d, NUMBER_OF_JOINT);
//         logData("impedance_position.txt", impedance_position, NUMBER_OF_JOINT);
//         logData("joint_command.txt", position_command, NUMBER_OF_JOINT);
//         logData("force_external.txt",F_external, NUMBER_OF_JOINT);
//         logData("force_dbic.txt",F_DBIC, NUMBER_OF_JOINT);
//         logData("force_rest.txt",F_rest, NUMBER_OF_JOINT);
//         logData("force_coriolis.txt",F_coriolis, NUMBER_OF_JOINT);
//         logData("actual_joint_torque.txt",trq_act, NUMBER_OF_JOINT);
//         logData("force_external_box.txt",F_external_box, NUMBER_OF_JOINT);
//         logData("force_impedance.txt",F_impedance, NUMBER_OF_JOINT);
//         logData("joint_error.txt",joint_error, NUMBER_OF_JOINT);
//         logData("task_position_error.txt",position_error, NUMBER_OF_JOINT);
//         logData("external_joint_torque.txt",trq_ext, NUMBER_OF_JOINT);
//         logData("external_joint_torque_auto.txt",trq_ext_auto, NUMBER_OF_JOINT);
//         logData("external_joint_torque_cal.txt",trq_ext_cal, NUMBER_OF_JOINT);
//         logData("raw_force_torque.txt",trq_force, NUMBER_OF_JOINT);
//         logData("gripper_torque.txt",gripper_torque, NUMBER_OF_JOINT);
//         logData("sensor_FT.txt",sensor_FT, NUMBER_OF_JOINT);
//         logData("sensor_FT_matched.txt",sensor_FT_matched, NUMBER_OF_JOINT);
//         logData("operator_count.txt", operator_count, 1);
//         logMatrixData("mass_matrix.txt", massMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
//         logMatrixData("coriolis_matrix.txt", coriolisMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
//         logMatrixData("jacobian_matrix.txt", jacobianMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
//         logMatrixData3x3("rotation_matrix.txt", rotationMatrix, 3, 3);

//         // 1. 현재 로봇 자세 (안전한 3x3 회전행렬을 쿼터니언으로 직접 변환)
//         float(*rotm_ptr)[3] = Drfl_.get_current_rotm();
//         Eigen::Matrix3f R_act;
//         R_act << rotm_ptr[0][0], rotm_ptr[0][1], rotm_ptr[0][2],
//                     rotm_ptr[1][0], rotm_ptr[1][1], rotm_ptr[1][2],
//                     rotm_ptr[2][0], rotm_ptr[2][1], rotm_ptr[2][2];
//         Eigen::Quaternionf q_act(R_act);
//         q_act.normalize(); // 수치적 안정성을 위해 정규화

//         // 2. 목표 궤적 자세 (각도 변환 없이 7차원 배열에서 직접 꺼내 조립)
//         // 주의: Eigen 쿼터니언 생성자 입력 순서는 (w, x, y, z) 이므로,
//         // 배열 인덱스 (6, 3, 4, 5) 순서로 넣어주어야 합니다!
//         Eigen::Quaternionf q_des(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
//         q_des.normalize(); // 수치적 안정성을 위해 정규화

//         if (is_first_run) {
//             q_act_prev = q_act;
//             q_des_prev = q_des;
//             is_first_run = false;
//         } else {
//             // 현재 값과 이전 값의 내적(dot)이 음수라면 부호가 뒤집힌 것이므로 반전시킴
//             if (q_act.coeffs().dot(q_act_prev.coeffs()) < 0.0f) q_act.coeffs() *= -1.0f;
//             if (q_des.coeffs().dot(q_des_prev.coeffs()) < 0.0f) q_des.coeffs() *= -1.0f;
            
//             q_act_prev = q_act;
//             q_des_prev = q_des;
//         }
        
//         // 3. 쿼터니언 기반 오차 계산 (Antipodal 보정 포함)
//         if (q_des.coeffs().dot(q_act.coeffs()) < 0.0f) {
//             q_des.coeffs() *= -1.0f;
//         }
//         Eigen::Quaternionf q_err(q_act.inverse() * q_des);
//         Eigen::Vector3f e_rot = q_act * q_err.vec();

//         // 4. 저장용 배열에 복사
//         actual_quat[0] = q_act.x(); actual_quat[1] = q_act.y(); actual_quat[2] = q_act.z(); actual_quat[3] = q_act.w();
//         traj_quat[0] = q_des.x();   traj_quat[1] = q_des.y();   traj_quat[2] = q_des.z();   traj_quat[3] = q_des.w();
//         orientation_error[0] = e_rot.x(); orientation_error[1] = e_rot.y(); orientation_error[2] = e_rot.z();

//         // 5. 파일로 기록
//         logData("actual_quaternion.txt", actual_quat, 4);
//         logData("traj_quaternion.txt", traj_quat, 4);
//         logData("quat_orientation_error.txt", orientation_error, 3);
        
//         auto current = std::chrono::high_resolution_clock::now();
        
//         Duration save_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
    
//         if (control_loop_time > save_time) {
//         std::this_thread::sleep_for(control_loop_time() - save_time());
//         }
//         start = std::chrono::high_resolution_clock::now();
//     }
// }

// void ControlLoop::startDataSaving() {
//     if (!data_saving_running_) {
//         data_saving_running_ = true;
//         data_saving_thread_ = std::thread(&ControlLoop::dataSaving, this);
//     }
// }

// void ControlLoop::stopDataSaving() {
//     if (data_saving_running_) {
//         data_saving_running_ = false;
//         if (data_saving_thread_.joinable()) {
//             data_saving_thread_.join();
//         }
//     }
// }

// void ControlLoop::convertToArray(const std::array<float, 6>& stdArray, float floatArray[6]) {
//     std::copy(stdArray.begin(), stdArray.end(), floatArray);
// }

// // 기존 6칸짜리 함수 아래에 7칸짜리 함수를 새로 추가
// void ControlLoop::convertToArray(const std::array<float, 7>& stdArray, float floatArray[7]) {
//     std::copy(stdArray.begin(), stdArray.end(), floatArray);
// }

// void TrajectoryGen::TrajectoryPlan(PlanParam* plan)
// {
//     // [수정] 배열 크기를 6에서 7로 확장
//     float ps[7], vs[7], as[7]; 
//     float pf[7], vf[7], af[7];
//     float tf;

//     tf = plan->time;

//     // [수정] 루프 범위를 7(3 pos + 4 quat)로 변경
//     for(int i = 0; i < 7; i++)
//     {
//         ps[i] = plan->ps[i];
//         vs[i] = plan->vs[i];
//         as[i] = plan->as[i];
//         pf[i] = plan->pf[i];
//         vf[i] = plan->vf[i];
//         af[i] = plan->af[i];
//     }

//     for(int i = 0; i < 7; i++)
//     {
//         plan->A0[i] = ps[i];
//         plan->A1[i] = vs[i];
//         plan->A2[i] = as[i] / 2.0f;
//         plan->A3[i] = (20.0f*pf[i] - 20.0f*ps[i] - (8.0f*vf[i] + 12.0f*vs[i])*tf - (3.0f*as[i] - af[i])*tf*tf) / (2.0f*tf*tf*tf);
//         plan->A4[i] = (30.0f*ps[i] - 30.0f*pf[i] + (14.0f*vf[i] + 16.0f*vs[i])*tf + (3.0f*as[i] - 2.0f*af[i])*tf*tf) / (2.0f*tf*tf*tf*tf);
//         plan->A5[i] = (12.0f*pf[i] - 12.0f*ps[i] - (6.0f*vf[i] + 6.0f*vs[i])*tf - (as[i] - af[i])*tf*tf) / (2.0f*tf*tf*tf*tf*tf);
//     }
// }

// void TrajectoryGen::TrajectoryGenerator(PlanParam *plan, TraParam *tra)
// {
//     // [수정] 계수 배열 크기 7로 확장
//     double A0[7], A1[7], A2[7], A3[7], A4[7], A5[7];
//     double t = tra->time;

//     if (t <= plan->time) {
//         for(int i = 0; i < 7; i++)
//         {
//             A0[i] = plan->A0[i];
//             A1[i] = plan->A1[i];
//             A2[i] = plan->A2[i];
//             A3[i] = plan->A3[i];
//             A4[i] = plan->A4[i];
//             A5[i] = plan->A5[i];
//         }
    
//         for(int i = 0; i < 7; i++)
//         {
//             tra->pos[i] = A0[i] + A1[i]*t + A2[i]*t*t + A3[i]*t*t*t + A4[i]*t*t*t*t + A5[i]*t*t*t*t*t;
//             tra->vel[i] = A1[i] + 2.0*A2[i]*t + 3.0*A3[i]*t*t + 4.0*A4[i]*t*t*t + 5.0*A5[i]*t*t*t*t;
//             tra->acc[i] = 2.0*A2[i] + 6.0*A3[i]*t + 12.0*A4[i]*t*t + 20.0*A5[i]*t*t*t;
//         }

//         // [중요 추가] 쿼터니언 정규화 (Normalization)
//         // 다항식 계산 결과로 나온 쿼터니언 성분(인덱스 3~6)의 크기를 1로 맞춰줍니다.
//         double quat_norm = std::sqrt(tra->pos[3]*tra->pos[3] + tra->pos[4]*tra->pos[4] + 
//                                     tra->pos[5]*tra->pos[5] + tra->pos[6]*tra->pos[6]);
//         if (quat_norm > 1e-6) {
//             for (int i = 3; i < 7; i++) {
//                 tra->pos[i] /= quat_norm;
//             }
//         }
//     }
//     else {
//         // [수정] 종료 지점도 7개 성분 모두 복사
//         for (int i = 0; i < 7; i++) {
//             tra->pos[i] = plan->pf[i];
//             tra->vel[i] = plan->vf[i];
//             tra->acc[i] = plan->af[i];
//         }
//     }
// }

// void ControlLoop::createNewDataDirectory() {
//     // 현재 시간을 기반으로 새로운 디렉토리 경로 생성
//     auto now = std::chrono::system_clock::now();
//     auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
//     std::tm* now_tm = std::localtime(&in_time_t);
    
//     // 날짜를 YYMMDD 형식으로 생성
//     std::stringstream dateStream;
//     dateStream << std::put_time(now_tm, "%y%m%d");
//     std::string datePart = dateStream.str();

//     // 시간을 HHMM 형식으로 생성
//     std::stringstream timeStream;
//     timeStream << std::put_time(now_tm, "%H%M");
//     std::string timePart = timeStream.str();

//     // 최종 디렉토리 경로
//     dataDirectory = "/home/rbl/catkin_ws/data/" + datePart +"_data" +"/" + timePart;
//     // dataDirectory = "/home/rbl/catkin_ws/data/sensor_validation";
//     // 디렉토리 생성
//     if (!fs::create_directories(dataDirectory)) {
//         std::cerr << "Failed to create directory: " << dataDirectory << std::endl;
//     } else {
//         std::cout << "Directory created: " << dataDirectory << std::endl;
//     }
// }


// }


#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

extern bool g_nKill_dsr_control;

#include <skku_tools/control_loop.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Quaternion.h>
#include <../../include/skku_control/dsr_hw_interface.h>

std::mutex mtx;
std::condition_variable cv;
std::thread stateThread;
std::thread savingThread;
std::atomic<bool> stopFlag(false);
float home_abs[NUMBER_OF_JOINT] = {0.00, 0.00, 90.00, 0.0, 90.0, 0.00};
float current_position[NUMBER_OF_JOINT] = {0, };
#define DESIRED_TIME 10

double working_mode;
// `using std::string_literals::operator""s` produces a GCC warning that cannot be disabled, so we
// have to use `using namespace ...`.
// See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=65923#c0
using namespace std::string_literals;  // NOLINT(google-build-using-namespace)
bool controlState = false;

namespace SKKU {

moveit_msgs::CartesianTrajectory ControlLoop::previous_msg;
int operator_call_count_ = 0;
bool isDirectoryCreated = false;
std::string dataDirectory = "";
namespace fs = boost::filesystem;
int fail;
TrajectoryGen trajectory_gen_;
TrajectoryGen::PlanParam plan;
TrajectoryGen::TraParam tra;
float distance_threshold = 50;
std::vector<uint64_t> loopTimes;

namespace {
constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);
constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;

float unwrapNear(float angle_deg, float ref_deg) {
    while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

void normalizeQuatPoint(std::array<double, 7>& p) {
    double n = std::sqrt(p[3] * p[3] + p[4] * p[4] + p[5] * p[5] + p[6] * p[6]);
    if (n > 1e-9) {
        p[3] /= n;
        p[4] /= n;
        p[5] /= n;
        p[6] /= n;
    } else {
        p[3] = 0.0;
        p[4] = 0.0;
        p[5] = 0.0;
        p[6] = 1.0;
    }
}

void alignQuatSign(std::array<double, 7>& curr, const std::array<double, 7>& prev) {
    double dot = curr[3] * prev[3] + curr[4] * prev[4] + curr[5] * prev[5] + curr[6] * prev[6];
    if (dot < 0.0) {
        for (int k = 3; k < 7; ++k) {
            curr[k] *= -1.0;
        }
    }
}

Eigen::Quaternionf quatFromPose7(const std::array<float, 7>& pose7) {
    Eigen::Quaternionf q(pose7[6], pose7[3], pose7[4], pose7[5]);  // w,x,y,z
    if (q.norm() < 1e-6f) return Eigen::Quaternionf::Identity();
    q.normalize();
    return q;
}

float quatMsgToYawDeg(const geometry_msgs::Quaternion& qmsg) {
    Eigen::Quaternionf q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
    if (q.norm() < 1e-6f) return 0.0f;
    q.normalize();
    Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler[0] * RAD2DEG;
}

void fillEulerDummyForIK(const Trajectory& src_quat,
                         Trajectory& dst_euler,
                         bool reset,
                         const float ref_rpy[3]) {
    dst_euler = src_quat;

    static bool initialized = false;
    static float prev_rpy[3] = {0.f, 0.f, 0.f};

    if (reset || !initialized) {
        prev_rpy[0] = ref_rpy[0];
        prev_rpy[1] = ref_rpy[1];
        prev_rpy[2] = ref_rpy[2];
        initialized = true;
    }

    Eigen::Quaternionf q = quatFromPose7(src_quat.pos_d);
    Eigen::Vector3f euler_zyx = q.toRotationMatrix().eulerAngles(2, 1, 0);

    float roll = unwrapNear(euler_zyx[2] * RAD2DEG, prev_rpy[0]);
    float pitch = unwrapNear(euler_zyx[1] * RAD2DEG, prev_rpy[1]);
    float yaw = unwrapNear(euler_zyx[0] * RAD2DEG, prev_rpy[2]);

    prev_rpy[0] = roll;
    prev_rpy[1] = pitch;
    prev_rpy[2] = yaw;

    dst_euler.pos_d[3] = roll;
    dst_euler.pos_d[4] = pitch;
    dst_euler.pos_d[5] = yaw;

    dst_euler.vel_d[3] = 0.0f;
    dst_euler.vel_d[4] = 0.0f;
    dst_euler.vel_d[5] = 0.0f;
    dst_euler.acc_d[3] = 0.0f;
    dst_euler.acc_d[4] = 0.0f;
    dst_euler.acc_d[5] = 0.0f;
}
}  // namespace

void TrajectoryGen::init(moveit_msgs::CartesianTrajectory msg,
                         moveit_msgs::CartesianTrajectory prev_msg,
                         float current_position[NUMBER_OF_JOINT],
                         int operator_call_count_) {
    (void)prev_msg;
    (void)operator_call_count_;

    float start_point[7];
    float goal[7];
    float tra_time = 0.0f;

    for (int i = 0; i < 3; i++) start_point[i] = current_position[i];

    Eigen::AngleAxisf rollAngle(current_position[3] * DEG2RAD, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(current_position[4] * DEG2RAD, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(current_position[5] * DEG2RAD, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q_start = yawAngle * pitchAngle * rollAngle;
    q_start.normalize();

    start_point[3] = q_start.x();
    start_point[4] = q_start.y();
    start_point[5] = q_start.z();
    start_point[6] = q_start.w();

    goal[0] = msg.points[0].point.pose.position.x;
    goal[1] = msg.points[0].point.pose.position.y;
    goal[2] = msg.points[0].point.pose.position.z;
    goal[3] = msg.points[0].point.pose.orientation.x;
    goal[4] = msg.points[0].point.pose.orientation.y;
    goal[5] = msg.points[0].point.pose.orientation.z;
    goal[6] = msg.points[0].point.pose.orientation.w;

    float dot = start_point[3] * goal[3] +
                start_point[4] * goal[4] +
                start_point[5] * goal[5] +
                start_point[6] * goal[6];
    if (dot < 0.0f) {
        for (int i = 3; i < 7; i++) start_point[i] *= -1.0f;
    }

    tra_time = msg.points[0].time_from_start.toSec();
    plan.time = tra_time;

    for (int i = 0; i < 7; i++) {
        plan.ps[i] = start_point[i];
        plan.pf[i] = goal[i];
        plan.vs[i] = 0.0f;
        plan.vf[i] = 0.0f;
        plan.as[i] = 0.0f;
        plan.af[i] = 0.0f;
    }

    trajectory_gen_.TrajectoryPlan(&plan);
    std::cout << "Full Quaternion Pipeline Initialized." << std::endl;
}

std::vector<std::array<double, 7>> TrajectoryGen::quadraticInterpolation(
    const std::vector<std::array<double, 7>>& points, int newPointsNum) {
    int originalPointsNum = static_cast<int>(points.size());
    if (originalPointsNum < 4) {
        return points;
    }

    std::vector<std::array<double, 7>> interpolatedPoints;
    int totalSegments = originalPointsNum - 1;
    int pointsPerSegment = std::max(0, newPointsNum / totalSegments);

    auto interpolate = [](const std::array<double, 7>& p0,
                          const std::array<double, 7>& p1,
                          const std::array<double, 7>& p2,
                          const std::array<double, 7>& p3,
                          double t) {
        std::array<double, 7> interpolatedPoint;
        double t2 = t * t;
        double t3 = t2 * t;

        for (int k = 0; k < 7; ++k) {
            interpolatedPoint[k] = 0.5 * ((2 * p1[k]) +
                                          (-p0[k] + p2[k]) * t +
                                          (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * t2 +
                                          (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * t3);
        }

        normalizeQuatPoint(interpolatedPoint);
        return interpolatedPoint;
    };

    for (int i = 1; i < totalSegments - 1; ++i) {
        interpolatedPoints.push_back(points[i]);
        for (int j = 1; j <= pointsPerSegment; ++j) {
            double t = static_cast<double>(j) / (pointsPerSegment + 1);
            interpolatedPoints.push_back(interpolate(points[i - 1], points[i], points[i + 1], points[i + 2], t));
        }
    }

    interpolatedPoints.push_back(points.back());
    return interpolatedPoints;
}

std::vector<std::array<double, 7>> TrajectoryGen::upsampleTrajectory(
    const moveit_msgs::CartesianTrajectory& msg, int newPointsNum) {
    int originalPointsNum = static_cast<int>(msg.points.size());
    if (originalPointsNum == 0) {
        return {};
    }

    std::vector<std::array<double, 7>> controlPoints(originalPointsNum);
    for (int i = 0; i < originalPointsNum; ++i) {
        controlPoints[i] = {
            msg.points[i].point.pose.position.x,
            msg.points[i].point.pose.position.y,
            msg.points[i].point.pose.position.z,
            msg.points[i].point.pose.orientation.x,
            msg.points[i].point.pose.orientation.y,
            msg.points[i].point.pose.orientation.z,
            msg.points[i].point.pose.orientation.w};
        normalizeQuatPoint(controlPoints[i]);
    }

    for (int i = 1; i < originalPointsNum; ++i) {
        alignQuatSign(controlPoints[i], controlPoints[i - 1]);
    }

    std::vector<std::array<double, 7>> interpolatedPoints =
        quadraticInterpolation(controlPoints, std::max(newPointsNum, originalPointsNum));

    if (interpolatedPoints.empty()) {
        return controlPoints;
    }

    float current_pos_7d[7];
    for (int k = 0; k < 3; ++k) current_pos_7d[k] = current_position[k];

    Eigen::AngleAxisf rollAngle(current_position[3] * DEG2RAD, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(current_position[4] * DEG2RAD, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(current_position[5] * DEG2RAD, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q_curr = yawAngle * pitchAngle * rollAngle;
    q_curr.normalize();

    current_pos_7d[3] = q_curr.x();
    current_pos_7d[4] = q_curr.y();
    current_pos_7d[5] = q_curr.z();
    current_pos_7d[6] = q_curr.w();

    std::array<double, 7> current_pos_7d_arr = {
        current_pos_7d[0], current_pos_7d[1], current_pos_7d[2],
        current_pos_7d[3], current_pos_7d[4], current_pos_7d[5], current_pos_7d[6]
    };
    alignQuatSign(interpolatedPoints[0], current_pos_7d_arr);

    auto easeInOutWeight = [](double t) { return t * t * (3 - 2 * t); };

    int extraPointsNum = 1000;
    std::vector<std::array<double, 7>> acceleration_segment, decceleration_segment;

    for (int i = 0; i < extraPointsNum; ++i) {
        std::array<double, 7> interpolated_point;
        double t = static_cast<double>(i) / (extraPointsNum - 1);
        double weight = easeInOutWeight(t);

        for (int j = 0; j < 7; ++j) {
            interpolated_point[j] =
                current_pos_7d[j] * (1.0 - weight) + interpolatedPoints[0][j] * weight;
        }
        normalizeQuatPoint(interpolated_point);
        acceleration_segment.push_back(interpolated_point);
    }

    if (interpolatedPoints.size() < 2) {
        return interpolatedPoints;
    }

    int decelerationStartIndex = std::max(1, static_cast<int>(interpolatedPoints.size()) - 5);
    std::array<double, 7> decel_start = interpolatedPoints[decelerationStartIndex - 1];
    interpolatedPoints.erase(interpolatedPoints.begin() + decelerationStartIndex, interpolatedPoints.end());

    for (int i = 0; i < extraPointsNum; ++i) {
        std::array<double, 7> interpolated_point;
        double t = static_cast<double>(i) / (extraPointsNum - 1);
        double weight = easeInOutWeight(t);

        for (int j = 0; j < 7; ++j) {
            interpolated_point[j] =
                decel_start[j] * (1.0 - weight) + controlPoints.back()[j] * weight;
        }
        normalizeQuatPoint(interpolated_point);
        decceleration_segment.push_back(interpolated_point);
    }

    std::vector<std::array<double, 7>> fullTrajectory;
    fullTrajectory.insert(fullTrajectory.end(), acceleration_segment.begin(), acceleration_segment.end());
    fullTrajectory.insert(fullTrajectory.end(), interpolatedPoints.begin(), interpolatedPoints.end());
    fullTrajectory.insert(fullTrajectory.end(), decceleration_segment.begin(), decceleration_segment.end());

    return fullTrajectory;
}

ControlLoop::ControlLoop(moveit_msgs::CartesianTrajectory msg,
                         u_int64_t loop_time,
                         RealtimeConfig realtimeconfig,
                         DRAFramework::CDRFLEx& Drfl)
    : PBIC(loop_time, Drfl) {
    (void)msg;
    realtimeconfig_ = realtimeconfig;
    loop_time_ = loop_time;
    bool throw_on_error = realtimeconfig_ == RealtimeConfig::kEnforce;
    std::string error_message;

    if (!getCurrentScheduling(originalSetting_)) {
        throw std::runtime_error("Failed to get current scheduling settings");
    }

    if (!setCurrentThreadToHighestSchedulerPriority(&error_message) && throw_on_error) {
        throw std::runtime_error(error_message);
    }
    if (throw_on_error && !hasRealtimeKernel()) {
        throw std::runtime_error("Error : Running kernel does not have realtime capabilities.");
    }
    std::cout << "High priority setting done" << std::endl;
}

ControlLoop::~ControlLoop() {
    if (!setScheduling(originalSetting_)) {
        std::cerr << "Failed to restore original scheduling settings" << std::endl;
    } else {
        std::cout << "Original scheduling settings restored successfully" << std::endl;
    }
}

ImpedanceControlLoop::ImpedanceControlLoop(moveit_msgs::CartesianTrajectory msg,
                                           u_int64_t loop_time,
                                           RealtimeConfig realtimeconfig,
                                           DRAFramework::CDRFLEx& Drfl)
    : ControlLoop(msg, loop_time, realtimeconfig, Drfl) {}

ImpedanceControlLoop::~ImpedanceControlLoop() {}

PositionControlLoop::PositionControlLoop(moveit_msgs::CartesianTrajectory msg,
                                         u_int64_t loop_time,
                                         RealtimeConfig realtimeconfig,
                                         DRAFramework::CDRFLEx& Drfl)
    : ControlLoop(msg, loop_time, realtimeconfig, Drfl) {}

PositionControlLoop::~PositionControlLoop() {}

void ControlLoop::StateCheckingThread(ControlLoop* controlLoop) {
    (void)controlLoop;
    while (true) {
        ROBOT_STATE state = Drfl_.get_robot_state();

        if (state == STATE_SAFE_OFF || state == STATE_SAFE_STOP ||
            state == STATE_RECOVERY || state == STATE_SAFE_STOP2 ||
            state == STATE_SAFE_OFF2 || state == STATE_EMERGENCY_STOP) {
            exitLoop = true;
            Drfl_.set_robot_control(CONTROL_RESET_SAFET_STOP);
        } else {
            exitLoop = false;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void PositionControlLoop::operator()(const moveit_msgs::CartesianTrajectory& msg) {
    std::cout << "Position Goal-directed Mode called" << std::endl;
    operator_call_count_++;
    fail = 0;
    control_mode_ = "Position goal mode";

    if (msg.points.empty()) {
        ROS_ERROR("Empty CartesianTrajectory received.");
        fail = 2;
        return;
    }

    LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();

    float current_joint[NUMBER_OF_JOINT] = {0,};
    memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    auto unwrapToNearest = [](float angle_deg, float ref_deg) {
        while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
        while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
        return angle_deg;
    };

    float goal_p[NUMBER_OF_JOINT] = {0,};
    for (int i = 0; i < 6; ++i) {
        goal_p[i] = current_position[i];
    }

    goal_p[0] = msg.points[0].point.pose.position.x;
    goal_p[1] = msg.points[0].point.pose.position.y;
    goal_p[2] = msg.points[0].point.pose.position.z;

    Eigen::AngleAxisf roll_curr(current_position[3] * DEG2RAD, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch_curr(current_position[4] * DEG2RAD, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw_curr(current_position[5] * DEG2RAD, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q_curr = yaw_curr * pitch_curr * roll_curr;
    q_curr.normalize();

    Eigen::Quaternionf q_goal(msg.points[0].point.pose.orientation.w,
                              msg.points[0].point.pose.orientation.x,
                              msg.points[0].point.pose.orientation.y,
                              msg.points[0].point.pose.orientation.z);

    if (q_goal.norm() < 1e-6f) {
        ROS_WARN("Received near-zero quaternion. Using current orientation instead.");
        q_goal = q_curr;
    } else {
        q_goal.normalize();
    }

    if (q_curr.coeffs().dot(q_goal.coeffs()) < 0.0f) {
        q_goal.coeffs() *= -1.0f;
    }

    Eigen::Vector3f euler_rad = q_goal.toRotationMatrix().eulerAngles(2, 1, 0);

    float roll_deg = euler_rad[2] * RAD2DEG;
    float pitch_deg = euler_rad[1] * RAD2DEG;
    float yaw_deg = euler_rad[0] * RAD2DEG;

    goal_p[3] = unwrapToNearest(roll_deg, current_position[3]);
    goal_p[4] = unwrapToNearest(pitch_deg, current_position[4]);
    goal_p[5] = unwrapToNearest(yaw_deg, current_position[5]);

    std::cout << "Goal pose XYZRPY(deg): "
              << goal_p[0] << ", " << goal_p[1] << ", " << goal_p[2] << ", "
              << goal_p[3] << ", " << goal_p[4] << ", " << goal_p[5] << std::endl;

    LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(goal_p, 2, COORDINATE_SYSTEM_BASE, 1);
    if (res == nullptr) {
        ROS_ERROR("IK failed for goal pose.");
        fail = 2;
        return;
    }

    float goal_joint[NUMBER_OF_JOINT] = {0,};
    for (int i = 0; i < 6; ++i) {
        goal_joint[i] = res->_fTargetPos[i];
    }

    while (goal_joint[5] - current_joint[5] > 180.0f) goal_joint[5] -= 360.0f;
    while (goal_joint[5] - current_joint[5] < -180.0f) goal_joint[5] += 360.0f;

    float tTime = msg.points[0].time_from_start.toSec();

    Drfl_.set_safety_mode(SAFETY_MODE_MANUAL, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_MANUAL);

    controlState = true;
    ROS_INFO("Starting data saving...");
    startDataSaving();

    ROS_INFO("Calling movej(goal_joint)...");
    bool success = Drfl_.movej(goal_joint, 60, 30, tTime);

    ROS_INFO("Stopping data saving...");
    stopDataSaving();
    controlState = false;

    if (!success) {
        ROS_ERROR("movej failed.");
        fail = 2;
        return;
    }

    float final_position[NUMBER_OF_JOINT] = {0,};
    robot_data = Drfl_.read_data_rt();
    memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    float distance = std::sqrt(
        std::pow(final_position[0] - goal_p[0], 2) +
        std::pow(final_position[1] - goal_p[1], 2) +
        std::pow(final_position[2] - goal_p[2], 2));

    if (distance > distance_threshold) {
        ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
        fail = 2;
    } else {
        ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
        fail = 1;
    }

    previous_msg = msg;
}

void ImpedanceControlLoop::operator()(const moveit_msgs::CartesianTrajectory& msg) {
    std::cout << "Impedance Goal-directed Mode called" << std::endl;
    fail = 0;
    control_mode_ = "Impedance goal mode";
    operator_call_count_++;

    LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();

    float current_joint[NUMBER_OF_JOINT] = {0,};
    memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    sol_space = 0;
    count = 0;

    ROS_INFO_STREAM("Full message content : " << msg);

    robot_data = Drfl_.read_data_rt();
    memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    robot_data = Drfl_.read_data_rt();
    memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

    trajectory_gen_.init(msg, previous_msg, current_position, operator_call_count_);

    Duration control_loop_time = Duration(loop_time_);
    float st = static_cast<float>(loop_time_) / 1000.0f;

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();
    start_Motion(robot_state, prev, imp);

    auto start = std::chrono::high_resolution_clock::now();

    loopTimes.clear();
    controlState = true;
    auto start_time = std::chrono::high_resolution_clock::now();
    startDataSaving();

    while (true) {
        robot_state = Drfl_.read_data_rt();

        if (!spinMotion(robot_state, control_loop_time, desired, sol_space) ||
            !spinControl(robot_state, control_loop_time, control_command, desired, sol_space)) {
            break;
        }

        if (exitLoop || g_nKill_dsr_control) {
            fail = 2;
            std::cout << SKKU::fail << std::endl;
            break;
        }

        Drfl_.torque_rt(control_command.tau_d, st);

        auto current = std::chrono::high_resolution_clock::now();
        Duration loop_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
        loopTimes.push_back(loop_time.toMSec());

        if (control_loop_time > loop_time) {
            std::this_thread::sleep_for(control_loop_time() - loop_time());
        }

        start = std::chrono::high_resolution_clock::now();
        count++;
    }

    stopDataSaving();
    saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    controlState = false;

    robot_data = Drfl_.read_data_rt();

    auto finished_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(finished_time - start_time);
    std::cout << "elapsed time: " << elapsed_time.count() << " ms" << std::endl;

    setScheduling(originalSetting_);

    float final_position[NUMBER_OF_JOINT] = {0,};
    memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    float distance = std::sqrt(
        std::pow(final_position[0] - msg.points[0].point.pose.position.x, 2) +
        std::pow(final_position[1] - msg.points[0].point.pose.position.y, 2) +
        std::pow(final_position[2] - msg.points[0].point.pose.position.z, 2));

    if (distance > distance_threshold) {
        ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
        fail = 2;
    } else {
        ROS_INFO("Succeed reach the goal position: distance = %f", distance);
        fail = 1;
    }

    previous_msg = msg;
}

void PositionControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {
    std::cout << "Position Push-path Mode called" << std::endl;
    fail = 0;
    control_mode_ = "Position path mode";
    operator_call_count_++;

    if (msg.points.empty()) {
        ROS_ERROR("operator_path received empty trajectory.");
        fail = 2;
        return;
    }

    LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();

    float current_joint[NUMBER_OF_JOINT] = {0,};
    memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

    const int pointsNum = static_cast<int>(msg.points.size());

    std::unique_ptr<float[][6]> xpos(new float[pointsNum][6]);

    float spline_vel[2] = {1000, 1000};
    float spline_acc[2] = {1000, 1000};
    float tTime = msg.points.back().time_from_start.toSec();

    auto unwrapToNearest = [](float angle_deg, float ref_deg) {
        while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
        while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
        return angle_deg;
    };

    Eigen::AngleAxisf roll0(current_position[3] * DEG2RAD, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitch0(current_position[4] * DEG2RAD, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yaw0(current_position[5] * DEG2RAD, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q_prev = yaw0 * pitch0 * roll0;
    q_prev.normalize();

    float prev_roll = current_position[3];
    float prev_pitch = current_position[4];
    float prev_yaw = current_position[5];

    for (int i = 0; i < pointsNum; ++i) {
        xpos[i][0] = msg.points[i].point.pose.position.x;
        xpos[i][1] = msg.points[i].point.pose.position.y;
        xpos[i][2] = msg.points[i].point.pose.position.z;

        Eigen::Quaternionf q_msg(msg.points[i].point.pose.orientation.w,
                                 msg.points[i].point.pose.orientation.x,
                                 msg.points[i].point.pose.orientation.y,
                                 msg.points[i].point.pose.orientation.z);

        if (q_msg.norm() < 1e-6f) {
            ROS_WARN("Waypoint %d has near-zero quaternion. Reusing previous orientation.", i);
            q_msg = q_prev;
        } else {
            q_msg.normalize();
        }

        if (q_prev.coeffs().dot(q_msg.coeffs()) < 0.0f) {
            q_msg.coeffs() *= -1.0f;
        }

        Eigen::Vector3f euler_rad = q_msg.toRotationMatrix().eulerAngles(2, 1, 0);

        float roll_deg = euler_rad[2] * RAD2DEG;
        float pitch_deg = euler_rad[1] * RAD2DEG;
        float yaw_deg = euler_rad[0] * RAD2DEG;

        roll_deg = unwrapToNearest(roll_deg, prev_roll);
        pitch_deg = unwrapToNearest(pitch_deg, prev_pitch);
        yaw_deg = unwrapToNearest(yaw_deg, prev_yaw);

        xpos[i][3] = roll_deg;
        xpos[i][4] = pitch_deg;
        xpos[i][5] = yaw_deg;

        q_prev = q_msg;
        prev_roll = roll_deg;
        prev_pitch = pitch_deg;
        prev_yaw = yaw_deg;
    }

    std::cout << "Point size: " << pointsNum << std::endl;

    float delta_yaw = xpos[pointsNum - 1][5] - xpos[0][5];

    if (current_joint[5] > 90.0f && delta_yaw > 0.0f) {
        std::cout << "Adjusted final joint 6 position: " << current_joint[5]
                  << " - Moving joint..." << std::endl;
        float prep_vel[2] = {70, 70};
        float prep_acc[2] = {120, 120};

        current_position[2] += 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

        current_joint[5] -= 360;
        Drfl_.movej(current_joint, 60, 30);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

        current_position[2] -= 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);
    } else if (current_joint[5] < -90.0f && delta_yaw < 0.0f) {
        std::cout << "Adjusted final joint 6 position: " << current_joint[5]
                  << " - Moving joint..." << std::endl;
        float prep_vel[2] = {70, 70};
        float prep_acc[2] = {120, 120};

        current_position[2] += 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

        current_joint[5] += 360;
        Drfl_.movej(current_joint, 60, 30);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

        current_position[2] -= 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);
    } else {
        std::cout << "No adjustment needed for joint 6: " << current_joint[5] << std::endl;
    }

    controlState = true;
    ROS_INFO("Starting data saving...");
    startDataSaving();

    ROS_INFO("Calling amovesx...");
    bool success = Drfl_.amovesx(xpos.get(), pointsNum, spline_vel, spline_acc, tTime, MOVE_MODE_ABSOLUTE);

    if (!success) {
        ROS_ERROR("amovesx failed to start.");
        stopDataSaving();
        controlState = false;
        fail = 2;
        return;
    }

    int wait_ret = Drfl_.mwait();

    ROS_INFO("Stopping data saving...");
    stopDataSaving();
    controlState = false;

    if (wait_ret != 1) {
        ROS_ERROR("mwait reported motion failure.");
        fail = 2;
        return;
    }

    float final_position[NUMBER_OF_JOINT] = {0,};
    robot_data = Drfl_.read_data_rt();
    memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    float distance = std::sqrt(
        std::pow(final_position[0] - xpos[pointsNum - 1][0], 2) +
        std::pow(final_position[1] - xpos[pointsNum - 1][1], 2) +
        std::pow(final_position[2] - xpos[pointsNum - 1][2], 2));

    if (distance > distance_threshold) {
        ROS_ERROR("Failed to reach the goal position: distance = %f", distance);
        fail = 2;
    } else {
        ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance);
        fail = 1;
    }

    previous_msg = msg;
}

void ImpedanceControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {
    if (msg.points.empty()) {
        ROS_ERROR("Impedance operator_path received empty trajectory.");
        fail = 2;
        return;
    }

    std::cout << "Impedance Push-path Mode called" << std::endl;
    fail = 0;
    control_mode_ = "Impedance path mode";
    operator_call_count_++;

    LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();

    float current_joint[NUMBER_OF_JOINT] = {0,};
    memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    const int originalPointsNum = static_cast<int>(msg.points.size());
    const double duration = msg.points.back().time_from_start.toSec();

    if (duration <= 0.0) {
        ROS_ERROR("Invalid path duration: %.6f", duration);
        fail = 2;
        return;
    }

    const int newPointsNum = std::max(static_cast<int>(duration * 1000.0 / loop_time_), originalPointsNum);

    count = 0;
    sol_space = 0;
    count_motion = 0;

    Duration control_loop_time = Duration(loop_time_);
    float st = static_cast<float>(loop_time_) / 1000.0f;

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();
    start_Motion(robot_state, prev, imp);

    float delta_yaw =
        quatMsgToYawDeg(msg.points.back().point.pose.orientation) -
        quatMsgToYawDeg(msg.points.front().point.pose.orientation);

    std::cout << "Initiating joint 6 position before adjustment: " << current_joint[5] << std::endl;
    std::cout << "Trajectory deviation of yaw : " << delta_yaw << std::endl;

    if (current_joint[5] > 90.0f && delta_yaw > 0.0f) {
        float prep_vel[2] = {70, 70};
        float prep_acc[2] = {120, 120};
        current_position[2] += 100.0f;

        float goal_joint[NUMBER_OF_JOINT] = {0,};
        LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; ++i) {
            goal_joint[i] = res->_fTargetPos[i];
        }
        Drfl_.movej(goal_joint, 60, 30);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

        current_joint[5] -= 360.0f;
        Drfl_.movej(current_joint, 60, 30);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

        current_position[2] -= 100.0f;

        LPINVERSE_KINEMATIC_RESPONSE res2 = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; ++i) {
            goal_joint[i] = res2->_fTargetPos[i];
        }
        Drfl_.movej(goal_joint, 60, 30);
    } else if (current_joint[5] < -90.0f && delta_yaw < 0.0f) {
        float prep_vel[2] = {70, 70};
        float prep_acc[2] = {120, 120};
        current_position[2] += 100.0f;

        float goal_joint[NUMBER_OF_JOINT] = {0,};
        LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; ++i) {
            goal_joint[i] = res->_fTargetPos[i];
        }
        Drfl_.movej(goal_joint, 60, 30);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

        current_joint[5] += 360.0f;
        Drfl_.movej(current_joint, 60, 30);

        robot_data = Drfl_.read_data_rt();
        memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

        current_position[2] -= 100.0f;

        LPINVERSE_KINEMATIC_RESPONSE res2 = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; ++i) {
            goal_joint[i] = res2->_fTargetPos[i];
        }
        Drfl_.movej(goal_joint, 60, 30);
    } else {
        std::cout << "No adjustment needed for joint 6: " << current_joint[5] << std::endl;
    }

    robot_data = Drfl_.read_data_rt();
    memcpy(current_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_joint, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

    float distance_to_start = std::sqrt(
        std::pow(msg.points[0].point.pose.position.x - current_position[0], 2) +
        std::pow(msg.points[0].point.pose.position.y - current_position[1], 2) +
        std::pow(msg.points[0].point.pose.position.z - current_position[2], 2));

    if (distance_to_start > distance_threshold) {
        ROS_ERROR("Error: The distance between current position and trajectory start point exceeds the threshold (distance = %f)", distance_to_start);
        fail = 2;
        return;
    }

    try {
        trajectory_gen_.CurvePoints_ = trajectory_gen_.upsampleTrajectory(msg, newPointsNum);
    } catch (const std::exception& e) {
        ROS_ERROR("upsampleTrajectory failed: %s", e.what());
        fail = 2;
        return;
    }

    if (trajectory_gen_.CurvePoints_.empty()) {
        ROS_ERROR("upsampleTrajectory produced empty CurvePoints_.");
        fail = 2;
        return;
    }

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    auto start = std::chrono::high_resolution_clock::now();
    auto start_time = std::chrono::high_resolution_clock::now();

    loopTimes.clear();
    controlState = true;
    startDataSaving();

    while (true) {
        robot_state = Drfl_.read_data_rt();

        if (!spinMotion_path(robot_state, control_loop_time, desired, sol_space) ||
            !spinControl(robot_state, control_loop_time, control_command, desired, sol_space)) {
            break;
        }

        if (exitLoop || g_nKill_dsr_control) {
            fail = 2;
            std::cout << SKKU::fail << std::endl;
            break;
        }

        Drfl_.torque_rt(control_command.tau_d, st);

        auto current = std::chrono::high_resolution_clock::now();
        auto loop_time = std::chrono::duration_cast<std::chrono::milliseconds>(current - start);
        loopTimes.push_back(static_cast<uint64_t>(loop_time.count()));

        auto control_loop_ms = std::chrono::milliseconds(static_cast<int64_t>(control_loop_time.toMSec()));

        if (control_loop_ms > loop_time) {
            std::this_thread::sleep_for(control_loop_ms - loop_time);
        }

        start = std::chrono::high_resolution_clock::now();
        count++;
    }

    ROS_INFO("Stopping data saving...");
    stopDataSaving();
    saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    controlState = false;

    robot_data = Drfl_.read_data_rt();

    auto finished_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(finished_time - start_time);
    std::cout << "elapsed time: " << elapsed_time.count() << " ms" << std::endl;

    setScheduling(originalSetting_);

    float final_position[NUMBER_OF_JOINT] = {0,};
    memcpy(final_position, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    float distance_to_goal = std::sqrt(
        std::pow(final_position[0] - msg.points[originalPointsNum - 1].point.pose.position.x, 2) +
        std::pow(final_position[1] - msg.points[originalPointsNum - 1].point.pose.position.y, 2) +
        std::pow(final_position[2] - msg.points[originalPointsNum - 1].point.pose.position.z, 2));

    if (fail != 2) {
        if (distance_to_goal > distance_threshold) {
            ROS_ERROR("Failed to reach the goal position: distance = %f", distance_to_goal);
            fail = 2;
        } else {
            ROS_INFO("Succeeded in reaching the goal position: distance = %f", distance_to_goal);
            fail = 1;
        }
    }

    previous_msg = msg;
}

void ControlLoop::GainMove() {
    float step = 4;
    float tvel[2] = {70, 70};
    float tacc[2] = {120, 120};

    struct Config {
        float ztop;
        float check_X1[6];
        float check_X2[6];
        float check_X3[6];
        float check_X4[6];
        float zdp;
        float xdp;
        float ydp;
    };

    std::vector<Config> configurations;
    configurations.push_back({
        600,
        {300, -400, 600, 0, -180, 3.42},
        {900, -400, 600, 0, -180, 3.42},
        {900, 400, 600, 0, -180, 3.42},
        {300, 400, 600, 0, -180, 3.42},
        (600 - 360) / step,
        (900 - 300) / (2 * step),
        (400 + 400) / (2 * step)
    });

    for (const auto& config : configurations) {
        float X1[6], X2[6], X3[6], X4[6];
        float ztop = config.ztop;
        memcpy(X1, config.check_X1, sizeof(config.check_X1));
        memcpy(X2, config.check_X2, sizeof(config.check_X2));
        memcpy(X3, config.check_X3, sizeof(config.check_X3));
        memcpy(X4, config.check_X4, sizeof(config.check_X4));

        for (int i = 0; i < step + 1; ++i) {
            X1[2] = ztop - i * config.zdp;
            X2[2] = ztop - i * config.zdp;
            X3[2] = ztop - i * config.zdp;
            X4[2] = ztop - i * config.zdp;

            for (int j = 0; j < step; ++j) {
                std::cout << "Moving X1: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
                Drfl_.movel(X1, tvel, tacc);

                std::cout << "Moving X2: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
                Drfl_.movel(X2, tvel, tacc);

                std::cout << "Moving X3: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
                Drfl_.movel(X3, tvel, tacc);

                std::cout << "Moving X4: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
                Drfl_.movel(X4, tvel, tacc);

                std::cout << "Returning to X1: " << i + 1 << "th height, " << j + 1 << "th step!" << std::endl;
                Drfl_.movel(X1, tvel, tacc);

                X1[0] += config.xdp;
                X1[1] += config.ydp;
                X2[0] -= config.xdp;
                X2[1] += config.ydp;
                X3[0] -= config.xdp;
                X3[1] -= config.ydp;
                X4[0] += config.xdp;
                X4[1] -= config.ydp;
            }

            Drfl_.movejx(X1, 2, 5, 10);
            X1[0] -= step * config.xdp;
            X1[1] -= step * config.ydp;
            X2[0] += step * config.xdp;
            X2[1] -= step * config.ydp;
            X3[0] += step * config.xdp;
            X3[1] += step * config.ydp;
            X4[0] -= step * config.xdp;
            X4[1] += step * config.ydp;
        }
    }

    gaincheckloop = true;
}

void ControlLoop::returnToHome() {
    Drfl_.movej(home_abs, 60, 30);

    LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
    memcpy(home_abs, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
}

bool ControlLoop::spinMotion(const LPRT_OUTPUT_DATA_LIST& robot_state,
                             SKKU::Duration time_step,
                             Desired& desired,
                             int sol_space) {
    (void)time_step;
    tra.time = static_cast<double>(count) * loop_time_ / 1000.0;
    trajectory_gen_.setLoopTime(loop_time_);
    bool correction_flag = false;

    if (tra.time <= plan.time) {
        trajectory_gen_.TrajectoryGenerator(&plan, &tra);

        for (int i = 0; i < 7; ++i) {
            trajectory.pos_d[i] = tra.pos[i];
            trajectory.vel_d[i] = tra.vel[i];
            trajectory.acc_d[i] = tra.acc[i];
        }

        Trajectory dummy_traj_for_ik;
        float ref_rpy[3] = {
            robot_state->actual_flange_position[3],
            robot_state->actual_flange_position[4],
            robot_state->actual_flange_position[5]
        };
        fillEulerDummyForIK(trajectory, dummy_traj_for_ik, count == 0, ref_rpy);

        auto [output, is_singular] =
            MotionGenerator(dummy_traj_for_ik, robot_state, prev, imp,
                            sol_space, correction_flag, operator_call_count_);

        if (is_singular) {
            std::cout << "Singularity occurred! Exiting loop." << std::endl;
            return false;
        }

        desired.q_d = output;
        return true;
    }

    std::cout << "Trajectory duration reached. Motion finished." << std::endl;
    return false;
}

bool ControlLoop::spinMotion_path(const LPRT_OUTPUT_DATA_LIST& robot_state,
                                  SKKU::Duration time_step,
                                  Desired& desired,
                                  int sol_space) {
    (void)time_step;
    bool correction_flag = false;

    if (count < trajectory_gen_.CurvePoints_.size()) {
        auto& point = trajectory_gen_.CurvePoints_[count];

        for (int i = 0; i < 7; ++i) {
            trajectory.pos_d[i] = point[i];
            trajectory.vel_d[i] = 0.0f;
            trajectory.acc_d[i] = 0.0f;
        }

        Trajectory dummy_traj_for_ik;
        float ref_rpy[3] = {
            robot_state->actual_flange_position[3],
            robot_state->actual_flange_position[4],
            robot_state->actual_flange_position[5]
        };
        fillEulerDummyForIK(trajectory, dummy_traj_for_ik, count == 0, ref_rpy);

        auto [output, is_singular] =
            MotionGenerator(dummy_traj_for_ik, robot_state, prev, imp,
                            sol_space, correction_flag, operator_call_count_);

        if (is_singular) {
            return false;
        }

        desired.q_d = output;
        return !desired.motion_finished;
    }

    return desired.motion_finished;
}

bool ControlLoop::spinControl(const LPRT_OUTPUT_DATA_LIST& robot_state,
                              SKKU::Duration time_step,
                              Torques& command,
                              Desired& desired,
                              int sol_space) {
    (void)time_step;
    (void)sol_space;

    Torques control_output = ControlGenerator(trajectory, desired, robot_state, errors, count);

    for (int i = 0; i < 6; i++) {
        command.tau_d[i] = control_output.tau_d[i];
    }

    return !command.motion_finished;
}

void ControlLoop::saveLoopTimesToFile(const std::string& filePath) {
    if (filePath.empty()) return;

    std::ofstream loopTimeFile(filePath, std::ios::app);
    if (loopTimeFile.is_open()) {
        for (const auto& time : loopTimes) {
            loopTimeFile << time << "\n";
        }
        loopTimeFile.close();
        std::cout << "Loop times saved to " << filePath << std::endl;
    } else {
        std::cerr << "Failed to open " << filePath << std::endl;
    }
}

void ControlLoop::logData(const std::string& fileName, const float* data, int dataSize) {
    if (!isDirectoryCreated && (operator_call_count_ >= 1)) {
        createNewDataDirectory();
        isDirectoryCreated = true;
    }

    const std::string fullPath = dataDirectory + "/" + fileName;
    std::ofstream file(fullPath, std::ios::app);

    if (file.is_open()) {
        for (int i = 0; i < dataSize - 1; ++i) {
            file << data[i] << "\t";
        }
        file << data[dataSize - 1] << std::endl;
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath
                  << " | Error: " << strerror(errno) << std::endl;
    }
}

void ControlLoop::logMatrixData(const std::string& fileName,
                                const float matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT],
                                int rows,
                                int cols) {
    if (!isDirectoryCreated && (operator_call_count_ >= 1)) {
        createNewDataDirectory();
        isDirectoryCreated = true;
    }

    const std::string fullPath = dataDirectory + "/" + fileName;
    std::ofstream file(fullPath, std::ios::app);

    if (file.is_open()) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file << std::setw(10) << matrix[i][j];
                if (j < cols - 1) file << "\t";
            }
            file << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath
                  << " | Error: " << strerror(errno) << std::endl;
    }
}

void ControlLoop::logMatrixData3x3(const std::string& fileName,
                                   const float matrix[3][3],
                                   int rows,
                                   int cols) {
    if (!isDirectoryCreated && (operator_call_count_ >= 1)) {
        createNewDataDirectory();
        isDirectoryCreated = true;
    }

    const std::string fullPath = dataDirectory + "/" + fileName;
    std::ofstream file(fullPath, std::ios::app);

    if (file.is_open()) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file << std::setw(10) << matrix[i][j];
                if (j < cols - 1) file << "\t";
            }
            file << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << fullPath
                  << " | Error: " << strerror(errno) << std::endl;
    }
}

void ControlLoop::gaindataSavingThread() {
    float gravity_torque[NUMBER_OF_JOINT] = {0,};
    float actual_position[NUMBER_OF_JOINT] = {0,};
    float raw_torque[NUMBER_OF_JOINT] = {0,};
    float external_torque[NUMBER_OF_JOINT] = {0,};
    float traj_position[7] = {0,};

    float actual_positionj[NUMBER_OF_JOINT] = {0,};
    float impedance_position[NUMBER_OF_JOINT] = {0,};
    float F_external[NUMBER_OF_JOINT] = {0,};
    float position_command[NUMBER_OF_JOINT] = {0,};
    float F_impedance[NUMBER_OF_JOINT] = {0,};
    Duration control_loop_time = Duration(loop_time_);
    float actual_position2[NUMBER_OF_JOINT] = {0,};
    float joint_error[NUMBER_OF_JOINT] = {0,};
    float time[1] = {0,};

    auto start = std::chrono::high_resolution_clock::now();
    bool correction_flag = false;

    while (!gaincheckloop) {
        LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

        Trajectory dummy_traj_for_ik;
        float ref_rpy[3] = {
            robot_state->actual_flange_position[3],
            robot_state->actual_flange_position[4],
            robot_state->actual_flange_position[5]
        };
        fillEulerDummyForIK(trajectory, dummy_traj_for_ik, true, ref_rpy);

        MotionGenerator(dummy_traj_for_ik, robot_state, prev, imp,
                        sol_space, correction_flag, operator_call_count_);

        LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
        memcpy(gravity_torque, robot_data->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_position2, robot_data->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
        memcpy(raw_torque, robot_data->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(external_torque, robot_data->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_positionj, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

        LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
        for (int i = 0; i < 6; i++) {
            actual_position[i] = res->_fPosition[i];
        }

        convertToArray(trajectory.pos_d, traj_position);

        float traj_position_6d[6] = {0,};
        traj_position_6d[0] = traj_position[0];
        traj_position_6d[1] = traj_position[1];
        traj_position_6d[2] = traj_position[2];

        Eigen::Quaternionf q_gain_log(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
        q_gain_log.normalize();
        Eigen::Vector3f euler_gain = q_gain_log.toRotationMatrix().eulerAngles(2, 1, 0);

        traj_position_6d[3] = euler_gain[2] * RAD2DEG;
        traj_position_6d[4] = euler_gain[1] * RAD2DEG;
        traj_position_6d[5] = euler_gain[0] * RAD2DEG;

        for (int i = 0; i < 6; i++) {
            impedance_position[i] = imp.pos_m(i);
            position_command[i] = desired.q_d[i];
            F_external[i] = F.Fext[i];
            F_impedance[i] = F.Fimp[i];
            joint_error[i] = position_command[i] - actual_positionj[i];
        }
        time[0] += dt;

        logData("time.txt", time, 1);
        logData("task_position.txt", actual_position, NUMBER_OF_JOINT);
        logData("task_trajectory.txt", traj_position_6d, NUMBER_OF_JOINT);
        logData("joint_position.txt", actual_positionj, NUMBER_OF_JOINT);
        logData("joint_command.txt", position_command, NUMBER_OF_JOINT);
        logData("raw_torque.txt", raw_torque, NUMBER_OF_JOINT);
        logData("command_torque.txt", control_command.tau_d, NUMBER_OF_JOINT);
        logData("gravity_torque.txt", gravity_torque, NUMBER_OF_JOINT);
        logData("external_torque.txt", external_torque, NUMBER_OF_JOINT);
        logData("force_external.txt", F_external, NUMBER_OF_JOINT);

        auto current = std::chrono::high_resolution_clock::now();
        Duration save_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));

        if (control_loop_time > save_time) {
            std::this_thread::sleep_for(control_loop_time() - save_time());
        }
        start = std::chrono::high_resolution_clock::now();
    }
}

void ControlLoop::dataSaving() {
    float trq_g[NUMBER_OF_JOINT] = {0,};
    float actual_position[NUMBER_OF_JOINT] = {0,};
    float actual_velocity[NUMBER_OF_JOINT] = {0,};
    float trq_raw[NUMBER_OF_JOINT] = {0,};
    float trq_act[NUMBER_OF_JOINT] = {0,};

    float traj_position[7] = {0,};
    float traj_velocity[7] = {0,};
    float traj_acceleration[7] = {0,};

    float actual_positionj[NUMBER_OF_JOINT] = {0,};
    float actual_velocityj[NUMBER_OF_JOINT] = {0,};
    float accelerationj[NUMBER_OF_JOINT] = {0,};
    float filtered_accelerationj[NUMBER_OF_JOINT] = {0,};
    float impedance_position[NUMBER_OF_JOINT] = {0,};
    float F_external[NUMBER_OF_JOINT] = {0,};
    float F_DBIC[NUMBER_OF_JOINT] = {0,};
    float F_rest[NUMBER_OF_JOINT] = {0,};
    float F_coriolis[NUMBER_OF_JOINT] = {0,};
    float position_command[NUMBER_OF_JOINT] = {0,};
    float trq_force[NUMBER_OF_JOINT] = {0,};
    float F_impedance[NUMBER_OF_JOINT] = {0,};
    Duration control_loop_time = Duration(loop_time_);
    float F_external_box[NUMBER_OF_JOINT] = {0,};
    float trq_ext[NUMBER_OF_JOINT] = {0,};
    float trq_ext_auto[NUMBER_OF_JOINT] = {0,};
    float trq_ext_cal[NUMBER_OF_JOINT] = {0,};
    float sensor_FT[NUMBER_OF_JOINT] = {0,};
    float sensor_FT_matched[NUMBER_OF_JOINT] = {0,};
    float actual_position2[NUMBER_OF_JOINT] = {0,};
    float joint_error[NUMBER_OF_JOINT] = {0,};
    float position_error[NUMBER_OF_JOINT] = {0,};
    float time[1] = {0,};
    float operator_count[1] = {static_cast<float>(operator_call_count_)};
    float controlMode[1];
    float alpha = 0.1f;
    float actual_quat[4] = {0,};
    float traj_quat[4] = {0,};
    float orientation_error[3] = {0,};

    std::unordered_map<std::string, int> modeMap = {
        {"Position goal mode", 0},
        {"Impedance goal mode", 1},
        {"Position path mode", 2},
        {"Impedance path mode", 3}
    };

    auto it = modeMap.find(control_mode_);
    if (it != modeMap.end()) {
        controlMode[0] = static_cast<float>(it->second);
    } else {
        controlMode[0] = -1.0f;
    }

    static Eigen::Quaternionf q_act_prev = Eigen::Quaternionf::Identity();
    static Eigen::Quaternionf q_des_prev = Eigen::Quaternionf::Identity();
    static bool is_first_run = true;
    static int last_operator_count = -1;

    float massMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
    float coriolisMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
    float jacobianMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
    float rotationMatrix[3][3] = {{0,}};

    auto start = std::chrono::high_resolution_clock::now();

    while (data_saving_running_) {
        LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();

        memcpy(trq_g, robot_data->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_position2, robot_data->actual_tcp_position, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_raw, robot_data->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_positionj, robot_data->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_velocityj, robot_data->actual_joint_velocity, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_velocity, robot_data->actual_flange_velocity, NUMBER_OF_JOINT * sizeof(float));
        memcpy(F_external_box, robot_data->external_tcp_force, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_ext, robot_data->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_force, robot_data->raw_force_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_act, robot_data->actual_joint_torque, NUMBER_OF_JOINT * sizeof(float));

        for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
            accelerationj[i] = (actual_velocityj[i] - previous_velocityj[i]) / dt;
            filtered_accelerationj[i] =
                alpha * accelerationj[i] + (1.0f - alpha) * filtered_accelerationj[i];
            previous_velocityj[i] = actual_velocityj[i];
        }

        memcpy(massMatrix, robot_data->mass_matrix,
               NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        memcpy(coriolisMatrix, robot_data->coriolis_matrix,
               NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        memcpy(jacobianMatrix, robot_data->jacobian_matrix,
               NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        float(*result)[3] = Drfl_.get_current_rotm();
        LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
        float gripper_torque[NUMBER_OF_JOINT] = {0,};

        LPROBOT_FORCE lpForce = Drfl_.get_external_torque();
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_ext2(lpForce->_fForce);

        for (int i = 0; i < 6; i++) {
            actual_position[i] = res->_fPosition[i];
        }

        convertToArray(trajectory.pos_d, traj_position);
        convertToArray(trajectory.vel_d, traj_velocity);
        convertToArray(trajectory.acc_d, traj_acceleration);

        float traj_position_6d[6] = {0,};
        traj_position_6d[0] = traj_position[0];
        traj_position_6d[1] = traj_position[1];
        traj_position_6d[2] = traj_position[2];

        Eigen::Quaternionf q_traj_log(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
        q_traj_log.normalize();
        Eigen::Vector3f euler_traj_log = q_traj_log.toRotationMatrix().eulerAngles(2, 1, 0);

        traj_position_6d[3] = euler_traj_log[2] * RAD2DEG;
        traj_position_6d[4] = euler_traj_log[1] * RAD2DEG;
        traj_position_6d[5] = euler_traj_log[0] * RAD2DEG;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                rotationMatrix[i][j] = result[i][j];
            }
        }

        for (int i = 0; i < 6; i++) {
            impedance_position[i] = imp.pos_m(i);
            position_command[i] = desired.q_d[i];
            F_external[i] = F.Fext[i];
            F_impedance[i] = F.Fimp[i];
            F_DBIC[i] = F.F_DBIC[i];
            F_coriolis[i] = F.F_coriolis[i];
            F_rest[i] = F.F_rest[i];
            joint_error[i] = position_command[i] - actual_positionj[i];
            position_error[i] = impedance_position[i] - actual_position[i];
            gripper_torque[i] = trq_gg[i];
            trq_ext_auto[i] = trq_ext2[i];
            trq_ext_cal[i] = trq_raw[i] - trq_g[i];
            sensor_FT[i] = sensor_data.AFT_wrench_[i];
            sensor_FT_matched[i] = sensor_data.AFT_wrench_matched[i];
        }
        time[0] += dt;

        logData("filtered_acceleration.txt", filtered_accelerationj, NUMBER_OF_JOINT);
        logData("time.txt", time, 1);
        logData("Control mode.txt", controlMode, 1);
        logData("gravity_torque.txt", trq_g, NUMBER_OF_JOINT);
        logData("task_position.txt", actual_position, NUMBER_OF_JOINT);
        logData("actual_velocity.txt", actual_velocity, NUMBER_OF_JOINT);
        logData("task_position2.txt", actual_position2, NUMBER_OF_JOINT);
        logData("raw_joint_torque.txt", trq_raw, NUMBER_OF_JOINT);
        logData("task_trajectory.txt", traj_position_6d, NUMBER_OF_JOINT);
        logData("task_velocity.txt", traj_velocity, NUMBER_OF_JOINT);
        logData("task_acceleration.txt", traj_acceleration, NUMBER_OF_JOINT);
        logData("joint_position.txt", actual_positionj, NUMBER_OF_JOINT);
        logData("joint_velocity.txt", actual_velocityj, NUMBER_OF_JOINT);
        logData("command_torque.txt", control_command.tau_d, NUMBER_OF_JOINT);
        logData("impedance_position.txt", impedance_position, NUMBER_OF_JOINT);
        logData("joint_command.txt", position_command, NUMBER_OF_JOINT);
        logData("force_external.txt", F_external, NUMBER_OF_JOINT);
        logData("force_dbic.txt", F_DBIC, NUMBER_OF_JOINT);
        logData("force_rest.txt", F_rest, NUMBER_OF_JOINT);
        logData("force_coriolis.txt", F_coriolis, NUMBER_OF_JOINT);
        logData("actual_joint_torque.txt", trq_act, NUMBER_OF_JOINT);
        logData("force_external_box.txt", F_external_box, NUMBER_OF_JOINT);
        logData("force_impedance.txt", F_impedance, NUMBER_OF_JOINT);
        logData("joint_error.txt", joint_error, NUMBER_OF_JOINT);
        logData("task_position_error.txt", position_error, NUMBER_OF_JOINT);
        logData("external_joint_torque.txt", trq_ext, NUMBER_OF_JOINT);
        logData("external_joint_torque_auto.txt", trq_ext_auto, NUMBER_OF_JOINT);
        logData("external_joint_torque_cal.txt", trq_ext_cal, NUMBER_OF_JOINT);
        logData("raw_force_torque.txt", trq_force, NUMBER_OF_JOINT);
        logData("gripper_torque.txt", gripper_torque, NUMBER_OF_JOINT);
        logData("sensor_FT.txt", sensor_FT, NUMBER_OF_JOINT);
        logData("sensor_FT_matched.txt", sensor_FT_matched, NUMBER_OF_JOINT);
        logData("operator_count.txt", operator_count, 1);
        logMatrixData("mass_matrix.txt", massMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
        logMatrixData("coriolis_matrix.txt", coriolisMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
        logMatrixData("jacobian_matrix.txt", jacobianMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
        logMatrixData3x3("rotation_matrix.txt", rotationMatrix, 3, 3);

        float(*rotm_ptr)[3] = Drfl_.get_current_rotm();
        Eigen::Matrix3f R_act;
        R_act << rotm_ptr[0][0], rotm_ptr[0][1], rotm_ptr[0][2],
                 rotm_ptr[1][0], rotm_ptr[1][1], rotm_ptr[1][2],
                 rotm_ptr[2][0], rotm_ptr[2][1], rotm_ptr[2][2];
        Eigen::Quaternionf q_act(R_act);
        q_act.normalize();

        Eigen::Quaternionf q_des(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
        q_des.normalize();

        if (last_operator_count != operator_call_count_) {
            is_first_run = true;
            last_operator_count = operator_call_count_;
        }

        if (is_first_run) {
            q_act_prev = q_act;
            q_des_prev = q_des;
            is_first_run = false;
        } else {
            if (q_act.coeffs().dot(q_act_prev.coeffs()) < 0.0f) q_act.coeffs() *= -1.0f;
            if (q_des.coeffs().dot(q_des_prev.coeffs()) < 0.0f) q_des.coeffs() *= -1.0f;

            q_act_prev = q_act;
            q_des_prev = q_des;
        }

        if (q_des.coeffs().dot(q_act.coeffs()) < 0.0f) {
            q_des.coeffs() *= -1.0f;
        }
        Eigen::Quaternionf q_err(q_act.inverse() * q_des);
        Eigen::Vector3f e_rot = 2.0f * (q_act * q_err.vec());

        actual_quat[0] = q_act.x();
        actual_quat[1] = q_act.y();
        actual_quat[2] = q_act.z();
        actual_quat[3] = q_act.w();

        traj_quat[0] = q_des.x();
        traj_quat[1] = q_des.y();
        traj_quat[2] = q_des.z();
        traj_quat[3] = q_des.w();

        orientation_error[0] = e_rot.x();
        orientation_error[1] = e_rot.y();
        orientation_error[2] = e_rot.z();

        logData("actual_quaternion.txt", actual_quat, 4);
        logData("traj_quaternion.txt", traj_quat, 4);
        logData("quat_orientation_error.txt", orientation_error, 3);

        auto current = std::chrono::high_resolution_clock::now();
        Duration save_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));

        if (control_loop_time > save_time) {
            std::this_thread::sleep_for(control_loop_time() - save_time());
        }
        start = std::chrono::high_resolution_clock::now();
    }
}

void ControlLoop::startDataSaving() {
    if (!data_saving_running_) {
        data_saving_running_ = true;
        data_saving_thread_ = std::thread(&ControlLoop::dataSaving, this);
    }
}

void ControlLoop::stopDataSaving() {
    if (data_saving_running_) {
        data_saving_running_ = false;
        if (data_saving_thread_.joinable()) {
            data_saving_thread_.join();
        }
    }
}

void ControlLoop::convertToArray(const std::array<float, 6>& stdArray, float floatArray[6]) {
    std::copy(stdArray.begin(), stdArray.end(), floatArray);
}

void ControlLoop::convertToArray(const std::array<float, 7>& stdArray, float floatArray[7]) {
    std::copy(stdArray.begin(), stdArray.end(), floatArray);
}

void TrajectoryGen::TrajectoryPlan(PlanParam* plan) {
    float ps[7], vs[7], as[7];
    float pf[7], vf[7], af[7];
    float tf = plan->time;

    for (int i = 0; i < 7; i++) {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for (int i = 0; i < 7; i++) {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i] / 2.0f;
        plan->A3[i] =
            (20.0f * pf[i] - 20.0f * ps[i] - (8.0f * vf[i] + 12.0f * vs[i]) * tf -
             (3.0f * as[i] - af[i]) * tf * tf) /
            (2.0f * tf * tf * tf);
        plan->A4[i] =
            (30.0f * ps[i] - 30.0f * pf[i] + (14.0f * vf[i] + 16.0f * vs[i]) * tf +
             (3.0f * as[i] - 2.0f * af[i]) * tf * tf) /
            (2.0f * tf * tf * tf * tf);
        plan->A5[i] =
            (12.0f * pf[i] - 12.0f * ps[i] - (6.0f * vf[i] + 6.0f * vs[i]) * tf -
             (as[i] - af[i]) * tf * tf) /
            (2.0f * tf * tf * tf * tf * tf);
    }
}

void TrajectoryGen::TrajectoryGenerator(PlanParam* plan, TraParam* tra) {
    double A0[7], A1[7], A2[7], A3[7], A4[7], A5[7];
    double t = tra->time;

    if (t <= plan->time) {
        for (int i = 0; i < 7; i++) {
            A0[i] = plan->A0[i];
            A1[i] = plan->A1[i];
            A2[i] = plan->A2[i];
            A3[i] = plan->A3[i];
            A4[i] = plan->A4[i];
            A5[i] = plan->A5[i];
        }

        for (int i = 0; i < 7; i++) {
            tra->pos[i] = A0[i] + A1[i] * t + A2[i] * t * t + A3[i] * t * t * t +
                          A4[i] * t * t * t * t + A5[i] * t * t * t * t * t;
            tra->vel[i] = A1[i] + 2.0 * A2[i] * t + 3.0 * A3[i] * t * t +
                          4.0 * A4[i] * t * t * t + 5.0 * A5[i] * t * t * t * t;
            tra->acc[i] =
                2.0 * A2[i] + 6.0 * A3[i] * t + 12.0 * A4[i] * t * t + 20.0 * A5[i] * t * t * t;
        }

        double quat_norm = std::sqrt(tra->pos[3] * tra->pos[3] + tra->pos[4] * tra->pos[4] +
                                     tra->pos[5] * tra->pos[5] + tra->pos[6] * tra->pos[6]);
        if (quat_norm > 1e-6) {
            for (int i = 3; i < 7; i++) {
                tra->pos[i] /= quat_norm;
            }
        }
    } else {
        for (int i = 0; i < 7; i++) {
            tra->pos[i] = plan->pf[i];
            tra->vel[i] = plan->vf[i];
            tra->acc[i] = plan->af[i];
        }
    }
}

void ControlLoop::createNewDataDirectory() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::tm* now_tm = std::localtime(&in_time_t);

    std::stringstream dateStream;
    dateStream << std::put_time(now_tm, "%y%m%d");
    std::string datePart = dateStream.str();

    std::stringstream timeStream;
    timeStream << std::put_time(now_tm, "%H%M");
    std::string timePart = timeStream.str();

    dataDirectory = "/home/rbl/catkin_ws/data/" + datePart + "_data/" + timePart;

    if (!fs::create_directories(dataDirectory)) {
        std::cerr << "Failed to create directory: " << dataDirectory << std::endl;
    } else {
        std::cout << "Directory created: " << dataDirectory << std::endl;
    }
}

}  // namespace SKKU