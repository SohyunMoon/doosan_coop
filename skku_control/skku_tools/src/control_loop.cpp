#include <algorithm>
#include <cerrno>
#include <cstring>
#include <exception>
#include <fstream>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
extern bool g_nKill_dsr_control;
#include <skku_tools/control_loop.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <condition_variable>
#include <../../include/skku_control/dsr_hw_interface.h>
#include <unordered_map>
#include <future>


std::mutex mtx;
std::condition_variable cv;
std::thread stateThread;
std::thread savingThread;
std::atomic<bool> stopFlag(false);
float home_abs[NUMBER_OF_JOINT] = {0.00, 0.00, 90.00, 0.0, 90.0, 0.00};
float current_position[NUMBER_OF_JOINT] = {0, };
#define DESIRED_TIME 10

double working_mode;
using namespace std::string_literals;  // NOLINT(google-build-using-namespace)
bool controlState = false; 

// =========================================================================
// Helper Functions
// =========================================================================
namespace {
    constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
    constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

    inline Eigen::Quaternionf normalizeQuat(Eigen::Quaternionf q) {
        if (q.norm() < 1e-6f) {
            return Eigen::Quaternionf::Identity();
        }
        q.normalize();
        return q;
    }

    inline bool isZeroQuatMsg(const geometry_msgs::Quaternion& q_msg) {
        return std::abs(q_msg.x) < 1e-12 &&
               std::abs(q_msg.y) < 1e-12 &&
               std::abs(q_msg.z) < 1e-12 &&
               std::abs(q_msg.w) < 1e-12;
    }

    inline float unwrapNear(float angle_deg, float ref_deg) {
        while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
        while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
        return angle_deg;
    }

    inline Eigen::Quaternionf quatFromEulerDeg(float roll_deg, float pitch_deg, float yaw_deg) {
        Eigen::AngleAxisf rollAngle (roll_deg  * DEG2RAD, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch_deg * DEG2RAD, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle  (yaw_deg   * DEG2RAD, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        return normalizeQuat(q);
    }

    inline Eigen::Quaternionf quatFromMsg(const geometry_msgs::Quaternion& q_msg) {
        return normalizeQuat(Eigen::Quaternionf(q_msg.w, q_msg.x, q_msg.y, q_msg.z));
    }

    inline Eigen::Quaternionf quatFromPose7(const std::array<float, 7>& pose7) {
        return normalizeQuat(Eigen::Quaternionf(pose7[6], pose7[3], pose7[4], pose7[5]));
    }

    inline void alignQuatHemisphere(Eigen::Quaternionf& q, const Eigen::Quaternionf& ref) {
        if (q.coeffs().dot(ref.coeffs()) < 0.0f) {
            q.coeffs() *= -1.0f;
        }
    }

    inline std::array<float, 3> quatToEulerDegZYX(const Eigen::Quaternionf& q_in) {
        Eigen::Quaternionf q = normalizeQuat(q_in);
        Eigen::Vector3f euler_zyx = q.toRotationMatrix().eulerAngles(2, 1, 0);

        float roll_deg  = euler_zyx[2] * RAD2DEG;
        float pitch_deg = euler_zyx[1] * RAD2DEG;
        float yaw_deg   = euler_zyx[0] * RAD2DEG;

        return {roll_deg, pitch_deg, yaw_deg};
    }

    inline std::array<float, 3> quatToEulerDegZYXNear(
        const Eigen::Quaternionf& q_in,
        float ref_roll_deg,
        float ref_pitch_deg,
        float ref_yaw_deg) {

        auto rpy = quatToEulerDegZYX(q_in);
        rpy[0] = unwrapNear(rpy[0], ref_roll_deg);
        rpy[1] = unwrapNear(rpy[1], ref_pitch_deg);
        rpy[2] = unwrapNear(rpy[2], ref_yaw_deg);
        return rpy;
    }

    inline void writeQuatXYZW(const Eigen::Quaternionf& q_in, float* dst_xyzw) {
        Eigen::Quaternionf q = normalizeQuat(q_in);
        dst_xyzw[0] = q.x();
        dst_xyzw[1] = q.y();
        dst_xyzw[2] = q.z();
        dst_xyzw[3] = q.w();
    }

    inline double quatMsgToYawDeg(const geometry_msgs::Quaternion& q_msg) {
        return static_cast<double>(quatToEulerDegZYX(quatFromMsg(q_msg))[2]);
    }
    //new0317 ~
    inline Eigen::Quaternionf quatFromRotm(const float (*R)[3]) {
        Eigen::Matrix3f m;
        m << R[0][0], R[0][1], R[0][2],
             R[1][0], R[1][1], R[1][2],
             R[2][0], R[2][1], R[2][2];
        Eigen::Quaternionf q(m);
        q.normalize();
        return q;
    }

    inline Eigen::Matrix3f skew(const Eigen::Vector3f& r) {
        Eigen::Matrix3f S;
        S <<     0.f, -r.z(),  r.y(),
              r.z(),     0.f, -r.x(),
             -r.y(),  r.x(),    0.f;
        return S;
    }

    inline Eigen::Vector3f quatLog(const Eigen::Quaternionf& q_in) {
        Eigen::Quaternionf q = normalizeQuat(q_in);
        if (q.w() < 0.0f) q.coeffs() *= -1.0f;

        const float vnorm = q.vec().norm();
        if (vnorm < 1e-8f) return Eigen::Vector3f::Zero();

        const float angle = 2.0f * std::atan2(vnorm, q.w());
        return (angle / vnorm) * q.vec();
    }

    inline Eigen::Vector3f quatLogError(const Eigen::Quaternionf& q_d,
                                        const Eigen::Quaternionf& q) {
        Eigen::Quaternionf q_err = q_d * q.conjugate();
        return quatLog(q_err);
    }

    inline Eigen::Isometry3f poseToIso(const Eigen::Vector3f& p,
                                       const Eigen::Quaternionf& q) {
        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        T.linear() = normalizeQuat(q).toRotationMatrix();
        T.translation() = p;
        return T;
    } //~new0317
    inline void printPose6(const std::string& tag, const float* p) {
        std::cout << tag
                << " [x y z r p y] = "
                << p[0] << ", " << p[1] << ", " << p[2] << ", "
                << p[3] << ", " << p[4] << ", " << p[5] << std::endl;
    }

    inline void printQuat4(const std::string& tag, float x, float y, float z, float w) {
        std::cout << tag
                << " [x y z w] = "
                << x << ", " << y << ", " << z << ", " << w << std::endl;
    }

    inline Eigen::Quaternionf quatFromEulerXYZDegSequence(float x_deg, float y_deg, float z_deg) {
        Eigen::AngleAxisf xAngle(x_deg * DEG2RAD, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf yAngle(y_deg * DEG2RAD, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf zAngle(z_deg * DEG2RAD, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = xAngle * yAngle * zAngle;
        return normalizeQuat(q);
    }

    inline Eigen::Quaternionf quatFromEulerZYZDegSequence(float z1_deg, float y_deg, float z2_deg) {
        Eigen::AngleAxisf z1Angle(z1_deg * DEG2RAD, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yAngle (y_deg  * DEG2RAD, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf z2Angle(z2_deg * DEG2RAD, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = z1Angle * yAngle * z2Angle;
        return normalizeQuat(q);
    }
    inline Eigen::Quaternionf quatFromEulerZYZ2DegSequence(float z1_deg, float y_deg, float z2_deg) {
        Eigen::AngleAxisf z1Angle(z1_deg * DEG2RAD, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yAngle (y_deg  * DEG2RAD, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf z2Angle(z2_deg * DEG2RAD, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = z2Angle * yAngle * z1Angle;
        return normalizeQuat(q);
    }    

    inline void pose6ToQuatAssumingXYZ(const float pose6[6], float quat_xyzw[4]) {
        Eigen::Quaternionf q = quatFromEulerDeg(pose6[3], pose6[4], pose6[5]);
        writeQuatXYZW(q, quat_xyzw);
    }
    inline void pose6ToQuatAssumingZYZ(const float pose6[6], float quat_xyzw[4]) {
        Eigen::Quaternionf q = quatFromEulerZYZDegSequence(pose6[3], pose6[4], pose6[5]);
        writeQuatXYZW(q, quat_xyzw);
    }
    inline void pose6ToQuatAssumingZYX(const float pose6[6], float quat_xyzw[4]) {
        Eigen::Quaternionf q = quatFromEulerXYZDegSequence(pose6[3], pose6[4], pose6[5]);
        writeQuatXYZW(q, quat_xyzw);
    }
    inline void pose6ToQuatAssumingZYZ2(const float pose6[6], float quat_xyzw[4]) {
        Eigen::Quaternionf q = quatFromEulerZYZ2DegSequence(pose6[3], pose6[4], pose6[5]);
        writeQuatXYZW(q, quat_xyzw);
    }    

} // end of anonymous namespace

namespace SKKU {

moveit_msgs::CartesianTrajectory ControlLoop::previous_msg;
int operator_call_count_ = 0;
bool isDirectoryCreated = false;
std::string dataDirectory = "";
namespace fs = boost::filesystem;
int fail;
//TrajectoryGen trajectory_gen_; new0317
TrajectoryGen::PlanParam plan;
TrajectoryGen::TraParam tra;
float distance_threshold = 50; 
std::vector<uint64_t> loopTimes;
// new0330 DBIC debug logs shared from impedance_controller.cpp
std::atomic<float> g_ref_v_d_log[3];
std::atomic<float> g_s_v_log[3];
std::atomic<float> g_ref_w_d_log[3];
std::atomic<float> g_s_w_log[3];
namespace {
    bool g_fill_euler_dummy_first = true;
    float g_fill_euler_dummy_prev_rpy[3] = {0.f, 0.f, 0.f};

    void resetFillEulerDummyForIKState() {
        g_fill_euler_dummy_first = true;
        g_fill_euler_dummy_prev_rpy[0] = 0.f;
        g_fill_euler_dummy_prev_rpy[1] = 0.f;
        g_fill_euler_dummy_prev_rpy[2] = 0.f;
    }
    //new0406
    bool g_pbic_goal_motion_finished = false;
    bool g_pbic_goal_motion_failed = false;
    double g_pbic_goal_elapsed_sec = 0.0;

    void resetPbicGoalSpinMotionState() {
        g_pbic_goal_motion_finished = false;
        g_pbic_goal_motion_failed = false;
        g_pbic_goal_elapsed_sec = 0.0;
    }

    void fillEulerDummyForIK(const SKKU::Trajectory& src_quat, SKKU::Trajectory& dst_euler) {
        dst_euler = src_quat;

        Eigen::Quaternionf q = quatFromPose7(src_quat.pos_d);

        std::array<float, 3> rpy;
        if (g_fill_euler_dummy_first) {
            rpy = quatToEulerDegZYX(q);
            g_fill_euler_dummy_first = false;
        } else {
            rpy = quatToEulerDegZYXNear(
                q,
                g_fill_euler_dummy_prev_rpy[0],
                g_fill_euler_dummy_prev_rpy[1],
                g_fill_euler_dummy_prev_rpy[2]);
        }

        g_fill_euler_dummy_prev_rpy[0] = rpy[0];
        g_fill_euler_dummy_prev_rpy[1] = rpy[1];
        g_fill_euler_dummy_prev_rpy[2] = rpy[2];

        dst_euler.pos_d[3] = rpy[0];
        dst_euler.pos_d[4] = rpy[1];
        dst_euler.pos_d[5] = rpy[2];

        dst_euler.vel_d[3] = 0.0f;
        dst_euler.vel_d[4] = 0.0f;
        dst_euler.vel_d[5] = 0.0f;

        dst_euler.acc_d[3] = 0.0f;
        dst_euler.acc_d[4] = 0.0f;
        dst_euler.acc_d[5] = 0.0f;
    }
}

void TrajectoryGen::init(moveit_msgs::CartesianTrajectory msg,
                         moveit_msgs::CartesianTrajectory prev_msg,
                         float current_position[NUMBER_OF_JOINT],
                         int operator_call_count_) {
    (void)prev_msg;
    (void)operator_call_count_;

    if (msg.points.empty()) {
        throw std::invalid_argument("TrajectoryGen::init received empty trajectory.");
    }

    float start_point[7] = {0.f, };
    float goal[7] = {0.f, };
    float tra_time = 0.0f;

    for (int i = 0; i < 3; ++i) {
        start_point[i] = current_position[i];
    }

    Eigen::Quaternionf q_start =
        quatFromEulerDeg(current_position[3], current_position[4], current_position[5]);
    writeQuatXYZW(q_start, &start_point[3]);

    goal[0] = msg.points[0].point.pose.position.x;
    goal[1] = msg.points[0].point.pose.position.y;
    goal[2] = msg.points[0].point.pose.position.z;

    Eigen::Quaternionf q_goal =
        isZeroQuatMsg(msg.points[0].point.pose.orientation)
        ? q_start
        : quatFromMsg(msg.points[0].point.pose.orientation);

    alignQuatHemisphere(q_goal, q_start);

    goal[3] = q_goal.x();
    goal[4] = q_goal.y();
    goal[5] = q_goal.z();
    goal[6] = q_goal.w();

    tra_time = std::max(1e-3f, static_cast<float>(msg.points[0].time_from_start.toSec()));
    plan.time = tra_time;

    for (int i = 0; i < 7; ++i) {
        plan.ps[i] = start_point[i];
        plan.pf[i] = goal[i];
        plan.vs[i] = 0.0f;
        plan.vf[i] = 0.0f;
        plan.as[i] = 0.0f;
        plan.af[i] = 0.0f;
    }

    TrajectoryPlan(&plan);
    std::cout << "Quaternion goal trajectory initialized." << std::endl;
}

std::vector<std::array<double, 7>> TrajectoryGen::quadraticInterpolation(const std::vector<std::array<double, 7>>& points, int newPointsNum) {
    std::vector<std::array<double, 7>> interpolatedPoints;
    int originalPointsNum = points.size();

    if (originalPointsNum < 4) {
        throw std::invalid_argument("Not enough points for cubic interpolation.");
    }

    int totalSegments = originalPointsNum - 1; 
    int pointsPerSegment = newPointsNum / totalSegments;

    auto interpolate = [](const std::array<double, 7>& p0, const std::array<double, 7>& p1, const std::array<double, 7>& p2, const std::array<double, 7>& p3, double t) {
        std::array<double, 7> interpolatedPoint;
        double t2 = t * t;
        double t3 = t2 * t;
        for (int k = 0; k < 7; ++k) { 
            interpolatedPoint[k] = 0.5 * (
                (2 * p1[k]) +
                (-p0[k] + p2[k]) * t +
                (2 * p0[k] - 5 * p1[k] + 4 * p2[k] - p3[k]) * t2 +
                (-p0[k] + 3 * p1[k] - 3 * p2[k] + p3[k]) * t3
            );
        }
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

std::vector<std::array<double, 7>> TrajectoryGen::upsampleTrajectory(const moveit_msgs::CartesianTrajectory& msg, int newPointsNum) {
    int originalPointsNum = msg.points.size();
    std::vector<std::array<double, 7>> controlPoints(originalPointsNum);

    Eigen::Quaternionf q_prev =
        quatFromEulerDeg(current_position[3], current_position[4], current_position[5]);

    for (int i = 0; i < originalPointsNum; ++i) {
        Eigen::Quaternionf q_i =
            isZeroQuatMsg(msg.points[i].point.pose.orientation)
            ? q_prev
            : quatFromMsg(msg.points[i].point.pose.orientation);

        alignQuatHemisphere(q_i, q_prev);

        controlPoints[i] = {
            msg.points[i].point.pose.position.x,
            msg.points[i].point.pose.position.y,
            msg.points[i].point.pose.position.z,
            static_cast<double>(q_i.x()),
            static_cast<double>(q_i.y()),
            static_cast<double>(q_i.z()),
            static_cast<double>(q_i.w())
        };

        q_prev = q_i;
    }

    std::vector<std::array<double, 7>> interpolatedPoints = quadraticInterpolation(controlPoints, newPointsNum);
    
    auto easeInOutWeight = [](double t) -> double {
        return t * t * (3 - 2 * t);  
    };

    int extraPointsNum = 1000;  
    std::vector<std::array<double, 7>> acceleration_segment, decceleration_segment;

    float current_pos_7d[7];
    for (int k = 0; k < 3; ++k) current_pos_7d[k] = current_position[k];
        Eigen::Quaternionf q_curr =
            quatFromEulerDeg(current_position[3], current_position[4], current_position[5]);

        writeQuatXYZW(q_curr, &current_pos_7d[3]);

    for (int i = 0; i < extraPointsNum; ++i) {
        std::array<double, 7> interpolated_point;
        double t = static_cast<double>(i) / (extraPointsNum - 1);  
        double weight = easeInOutWeight(t);

        for (int j = 0; j < 7; ++j) { 
            interpolated_point[j] = current_pos_7d[j] * (1.0 - weight) + interpolatedPoints[0][j] * weight;
        }
        acceleration_segment.push_back(interpolated_point);
    }

    int decelerationStartIndex = std::max(0, static_cast<int>(interpolatedPoints.size()) - 5);  
    interpolatedPoints.erase(interpolatedPoints.begin() + decelerationStartIndex, interpolatedPoints.end());  

    for (int i = 0; i < extraPointsNum; ++i) {
        std::array<double, 7> interpolated_point;
        double t = static_cast<double>(i) / (extraPointsNum - 1);  
        double weight = easeInOutWeight(t);

        for (int j = 0; j < 7; ++j) { 
            interpolated_point[j] = interpolatedPoints[decelerationStartIndex - 1][j] * (1.0 - weight) + controlPoints.back()[j] * weight;
        }
        decceleration_segment.push_back(interpolated_point);
    }

    std::vector<std::array<double, 7>> fullTrajectory;
    fullTrajectory.insert(fullTrajectory.end(), acceleration_segment.begin(), acceleration_segment.end());
    fullTrajectory.insert(fullTrajectory.end(), interpolatedPoints.begin(), interpolatedPoints.end());  
    fullTrajectory.insert(fullTrajectory.end(), decceleration_segment.begin(), decceleration_segment.end());  

    return fullTrajectory;
}

void TrajectoryGen::TrajectoryPlan(PlanParam* plan)
{
    //new0317
    if (plan->time <= 1e-6f) {
        throw std::invalid_argument("TrajectoryPlan: plan->time must be > 0.");
    }//

    float ps[7], vs[7], as[7]; 
    float pf[7], vf[7], af[7];
    float tf = plan->time;

    for(int i = 0; i < 7; i++)
    {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for(int i = 0; i < 7; i++)
    {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i] / 2.0f;
        plan->A3[i] = (20.0f*pf[i] - 20.0f*ps[i] - (8.0f*vf[i] + 12.0f*vs[i])*tf - (3.0f*as[i] - af[i])*tf*tf) / (2.0f*tf*tf*tf);
        plan->A4[i] = (30.0f*ps[i] - 30.0f*pf[i] + (14.0f*vf[i] + 16.0f*vs[i])*tf + (3.0f*as[i] - 2.0f*af[i])*tf*tf) / (2.0f*tf*tf*tf*tf);
        plan->A5[i] = (12.0f*pf[i] - 12.0f*ps[i] - (6.0f*vf[i] + 6.0f*vs[i])*tf - (as[i] - af[i])*tf*tf) / (2.0f*tf*tf*tf*tf*tf);
    }
}

void TrajectoryGen::TrajectoryGenerator(PlanParam *plan, TraParam *tra)
{
    double A0[7], A1[7], A2[7], A3[7], A4[7], A5[7];
    double t = tra->time;

    if (t <= plan->time) {
        for(int i = 0; i < 7; i++)
        {
            A0[i] = plan->A0[i];
            A1[i] = plan->A1[i];
            A2[i] = plan->A2[i];
            A3[i] = plan->A3[i];
            A4[i] = plan->A4[i];
            A5[i] = plan->A5[i];
        }
    
        for(int i = 0; i < 7; i++)
        {
            tra->pos[i] = A0[i] + A1[i]*t + A2[i]*t*t + A3[i]*t*t*t + A4[i]*t*t*t*t + A5[i]*t*t*t*t*t;
            tra->vel[i] = A1[i] + 2.0*A2[i]*t + 3.0*A3[i]*t*t + 4.0*A4[i]*t*t*t + 5.0*A5[i]*t*t*t*t;
            tra->acc[i] = 2.0*A2[i] + 6.0*A3[i]*t + 12.0*A4[i]*t*t + 20.0*A5[i]*t*t*t;
        }

        double quat_norm = std::sqrt(tra->pos[3]*tra->pos[3] + tra->pos[4]*tra->pos[4] + 
                                    tra->pos[5]*tra->pos[5] + tra->pos[6]*tra->pos[6]);
        if (quat_norm > 1e-6) {
            for (int i = 3; i < 7; i++) {
                tra->pos[i] /= quat_norm;
            }
        }
    }
    else {
        for (int i = 0; i < 7; i++) {
            tra->pos[i] = plan->pf[i];
            tra->vel[i] = plan->vf[i];
            tra->acc[i] = plan->af[i];
        }
    }
}
//new0317
void TrajectoryGen::initDBICGoal(const Eigen::Vector3f& p0_m,
                                 const Eigen::Quaternionf& q0,
                                 const moveit_msgs::CartesianTrajectory& msg) {
    dbic_mode_ = DBICMode::kGoal;
    dbic_path_samples_.clear();

    dbic_p0_ = p0_m;
    dbic_q0_ = normalizeQuat(q0);

    dbic_pf_ << static_cast<float>(msg.points[0].point.pose.position.x) * 1e-3f,
                static_cast<float>(msg.points[0].point.pose.position.y) * 1e-3f,
                static_cast<float>(msg.points[0].point.pose.position.z) * 1e-3f;

    if (isZeroQuatMsg(msg.points[0].point.pose.orientation)) {
        dbic_qf_ = dbic_q0_;
    } else {
        dbic_qf_ = quatFromMsg(msg.points[0].point.pose.orientation);
        alignQuatHemisphere(dbic_qf_, dbic_q0_);
    }

    // dbic_T_ = std::max(1e-3, msg.points[0].time_from_start.toSec());

    // 실제 사용자가 보낸 goal time을 그대로 사용한다.
    const double T_cmd = msg.points[0].time_from_start.toSec();
    dbic_T_ = std::max(1e-3, T_cmd);

    std::cout << "[DBIC INIT] requested_T=" << T_cmd
              << " dbic_T_=" << dbic_T_ << std::endl;

}

TaskRef TrajectoryGen::sampleDBICGoal(double t_sec, double dt_sec) const {
    auto clamp01 = [](double x) {
        return std::max(0.0, std::min(1.0, x));
    };

    auto samplePoseOnly = [&](double ts, Eigen::Vector3f& p, Eigen::Quaternionf& q) {
        double tau = clamp01(ts / dbic_T_);

        double s =
            10.0 * std::pow(tau, 3) -
            15.0 * std::pow(tau, 4) +
             6.0 * std::pow(tau, 5);

        p = dbic_p0_ + static_cast<float>(s) * (dbic_pf_ - dbic_p0_);
        q = dbic_q0_.slerp(static_cast<float>(s), dbic_qf_);
        q.normalize();
    };

    TaskRef ref;

    Eigen::Vector3f p_m, p_0, p_p;
    Eigen::Quaternionf q_m, q_0, q_p;

    samplePoseOnly(std::max(0.0, t_sec - dt_sec), p_m, q_m);
    samplePoseOnly(t_sec,                          p_0, q_0);
    samplePoseOnly(std::min(dbic_T_, t_sec + dt_sec), p_p, q_p);

    ref.p_d = p_0;
    ref.v_d = (p_p - p_m) / static_cast<float>(2.0 * dt_sec);
    ref.a_d = (p_p - 2.0f * p_0 + p_m) / static_cast<float>(dt_sec * dt_sec);
    ref.q_d = q_0;

    Eigen::Quaternionf dq_p = q_0.conjugate() * q_p;
    Eigen::Quaternionf dq_m = q_m.conjugate() * q_0;
    if (dq_p.w() < 0.f) dq_p.coeffs() *= -1.f;
    if (dq_m.w() < 0.f) dq_m.coeffs() *= -1.f;

    Eigen::Vector3f w_p = quatLog(dq_p) / static_cast<float>(dt_sec);
    Eigen::Vector3f w_m = quatLog(dq_m) / static_cast<float>(dt_sec);

    ref.w_d     = 0.5f * (w_p + w_m);
    ref.alpha_d = (w_p - w_m) / static_cast<float>(dt_sec);

    if (t_sec >= dbic_T_) {
        ref.p_d = dbic_pf_;
        ref.q_d = dbic_qf_;
        ref.v_d.setZero();
        ref.a_d.setZero();
        ref.w_d.setZero();
        ref.alpha_d.setZero();
    }

    ref.motion_finished = (t_sec > dbic_T_);

    return ref;
}

void TrajectoryGen::initDBICPath(const Eigen::Vector3f& p0_m,
                                 const Eigen::Quaternionf& q0,
                                 const moveit_msgs::CartesianTrajectory& msg,
                                 double dt_sec) {
    dbic_mode_ = DBICMode::kPath;
    dbic_path_samples_.clear();

    if (msg.points.empty()) return;

    std::vector<double> ts;
    std::vector<Eigen::Vector3f> ps;
    std::vector<Eigen::Quaternionf> qs;

    ts.push_back(0.0);
    ps.push_back(p0_m);
    qs.push_back(normalizeQuat(q0));

    for (size_t i = 0; i < msg.points.size(); ++i) {
        double ti = msg.points[i].time_from_start.toSec();
        if (ti <= ts.back()) ti = ts.back() + dt_sec;

        Eigen::Vector3f p;
        p << static_cast<float>(msg.points[i].point.pose.position.x) * 1e-3f,
             static_cast<float>(msg.points[i].point.pose.position.y) * 1e-3f,
             static_cast<float>(msg.points[i].point.pose.position.z) * 1e-3f;

        Eigen::Quaternionf q;
        if (isZeroQuatMsg(msg.points[i].point.pose.orientation)) {
            q = qs.back();
        } else {
            q = quatFromMsg(msg.points[i].point.pose.orientation);
            alignQuatHemisphere(q, qs.back());
        }

        ts.push_back(ti);
        ps.push_back(p);
        qs.push_back(q);
    }

    const double total_T = ts.back();
    const size_t num_samples =
        std::max<size_t>(2, static_cast<size_t>(std::ceil(total_T / dt_sec)) + 1);

    std::vector<double> sample_times(num_samples, 0.0);
    dbic_path_samples_.resize(num_samples);

    auto samplePoseOnly = [&](double t_query, Eigen::Vector3f& p, Eigen::Quaternionf& q) {
        if (t_query <= ts.front()) {
            p = ps.front();
            q = qs.front();
            return;
        }
        if (t_query >= ts.back()) {
            p = ps.back();
            q = qs.back();
            return;
        }

        size_t k = 0;
        while (k + 1 < ts.size() && ts[k + 1] < t_query) ++k;

        const double seg_dt = std::max(1e-6, ts[k + 1] - ts[k]);
        const float u = static_cast<float>((t_query - ts[k]) / seg_dt);

        p = (1.0f - u) * ps[k] + u * ps[k + 1];
        q = qs[k].slerp(u, qs[k + 1]);
        q.normalize();
    };

    for (size_t i = 0; i < num_samples; ++i) {
        sample_times[i] = std::min(total_T, static_cast<double>(i) * dt_sec);
        samplePoseOnly(sample_times[i], dbic_path_samples_[i].p_d, dbic_path_samples_[i].q_d);
    }

    for (size_t i = 0; i < num_samples; ++i) {
        const size_t i_prev = (i == 0) ? 0 : i - 1;
        const size_t i_next = (i + 1 >= num_samples) ? num_samples - 1 : i + 1;
        const double dt_local = std::max(1e-6, sample_times[i_next] - sample_times[i_prev]);

        dbic_path_samples_[i].v_d =
            (dbic_path_samples_[i_next].p_d - dbic_path_samples_[i_prev].p_d)
            / static_cast<float>(dt_local);

        Eigen::Quaternionf dq =
            dbic_path_samples_[i_prev].q_d.conjugate() * dbic_path_samples_[i_next].q_d;
        if (dq.w() < 0.0f) dq.coeffs() *= -1.0f;
        dq.normalize();

        dbic_path_samples_[i].w_d =
            quatLog(dq) / static_cast<float>(dt_local);
    }

    for (size_t i = 0; i < num_samples; ++i) {
        const size_t i_prev = (i == 0) ? 0 : i - 1;
        const size_t i_next = (i + 1 >= num_samples) ? num_samples - 1 : i + 1;
        const double dt_local = std::max(1e-6, sample_times[i_next] - sample_times[i_prev]);

        dbic_path_samples_[i].a_d =
            (dbic_path_samples_[i_next].v_d - dbic_path_samples_[i_prev].v_d)
            / static_cast<float>(dt_local);

        dbic_path_samples_[i].alpha_d =
            (dbic_path_samples_[i_next].w_d - dbic_path_samples_[i_prev].w_d)
            / static_cast<float>(dt_local);
    }

    // dbic_path_samples_.back().motion_finished = true;

    // 마지막 샘플도 한 번은 출력되게 한다.
    dbic_path_samples_.back().motion_finished = false;
    
}

TaskRef TrajectoryGen::sampleDBICPath(size_t idx) const {
    if (dbic_path_samples_.empty()) {
        TaskRef out;
        out.motion_finished = true;
        return out;
    }

    if (idx >= dbic_path_samples_.size()) {
        TaskRef out = dbic_path_samples_.back();
        out.motion_finished = true;
        return out;
    }

    return dbic_path_samples_[idx];
}//

ControlLoop::ControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl) 
    : PBIC(loop_time, Drfl) {
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
    : ControlLoop(msg, loop_time, realtimeconfig, Drfl)
{
    // 현재 실험은 DBIC
    // setImpedanceImplMode(ImpedanceImplMode::kDBIC);
    setImpedanceImplMode(ImpedanceImplMode::kPBIC_TDC);

    ////////////////////////// Flange ////////////////////////// 
    setTaskPointMode(TaskPointMode::kFlange);

    // flange 기준 실험에서는 추가 tool offset을 쓰지 않는다
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    setToolTransform(T);

    //////////////////////////  TCP ////////////////////////// 
    // // EE를 실제 TCP 기준으로 제어
    // setTaskPointMode(TaskPointMode::kTCP);

    // // ------------------------------------------------------------
    // // flange -> actual EE(TCP) transform
    // // 아래 숫자는 반드시 실제 장착값으로 바꿔야 한다.
    // // 단위: meter / rad
    // // ------------------------------------------------------------
    // Eigen::Isometry3f T = Eigen::Isometry3f::Identity();

    // // 예시: flange에서 TCP까지 local 
    // const float tcp_x = 0.0f;
    // const float tcp_y = 0.0f;
    // const float tcp_z = 0.0f;

    // // 예시 회전 오프셋: 실제 EE frame이 flange와 다르면 여기를 수정
    // const float tcp_roll_rad  = 0.0f;
    // const float tcp_pitch_rad = 0.0f;
    // const float tcp_yaw_rad   = 0.0f;

    // T.translation() << tcp_x, tcp_y, tcp_z;

    // Eigen::AngleAxisf rx(tcp_roll_rad,  Eigen::Vector3f::UnitX());
    // Eigen::AngleAxisf ry(tcp_pitch_rad, Eigen::Vector3f::UnitY());
    // Eigen::AngleAxisf rz(tcp_yaw_rad,   Eigen::Vector3f::UnitZ());
    // T.linear() = (rz * ry * rx).toRotationMatrix();

    // setToolTransform(T);
}

ImpedanceControlLoop::~ImpedanceControlLoop() {}

PositionControlLoop::PositionControlLoop(moveit_msgs::CartesianTrajectory msg, u_int64_t loop_time, RealtimeConfig realtimeconfig, DRAFramework::CDRFLEx& Drfl)
    : ControlLoop(msg, loop_time, realtimeconfig, Drfl) {}

PositionControlLoop::~PositionControlLoop() {}

void ControlLoop::StateCheckingThread(ControlLoop* controlLoop) {
    while (true) {
        ROBOT_STATE state = Drfl_.get_robot_state();
        if (state == STATE_SAFE_OFF || state == STATE_SAFE_STOP ||
            state == STATE_RECOVERY || state == STATE_SAFE_STOP2||
            state == STATE_SAFE_OFF2 || state == STATE_EMERGENCY_STOP) {
            exitLoop = true;
            Drfl_.set_robot_control(CONTROL_RESET_SAFET_STOP);    
        }
        else {
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

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

    float current_joint[NUMBER_OF_JOINT] = {0,};
    memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    // constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
    // constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

    float goal_p[NUMBER_OF_JOINT] = {0,};
    for (int i = 0; i < 6; ++i) {
        goal_p[i] = current_position[i];
    }

    goal_p[0] = msg.points[0].point.pose.position.x;
    goal_p[1] = msg.points[0].point.pose.position.y;
    goal_p[2] = msg.points[0].point.pose.position.z;

    Eigen::Quaternionf q_curr =
        quatFromEulerDeg(current_position[3], current_position[4], current_position[5]);

    Eigen::Quaternionf q_goal = quatFromMsg(msg.points[0].point.pose.orientation);

    if (isZeroQuatMsg(msg.points[0].point.pose.orientation)) {
        q_goal = q_curr;
    }

    alignQuatHemisphere(q_goal, q_curr);

    auto goal_rpy = quatToEulerDegZYXNear(
        q_goal,
        current_position[3],
        current_position[4],
        current_position[5]
    );

    goal_p[3] = goal_rpy[0];
    goal_p[4] = goal_rpy[1];
    goal_p[5] = goal_rpy[2];

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
    startDataSaving();

    bool success = Drfl_.movej(goal_joint, 60, 30, tTime);

    stopDataSaving();
    controlState = false;

    if (!success) {
        fail = 2;
        return;
    }

    float final_position[NUMBER_OF_JOINT] = {0,};
    robot_state = Drfl_.read_data_rt();
    memcpy(final_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    float distance = std::sqrt(
        std::pow(final_position[0] - goal_p[0], 2) +
        std::pow(final_position[1] - goal_p[1], 2) +
        std::pow(final_position[2] - goal_p[2], 2)
    );

    if (distance > distance_threshold) fail = 2;
    else fail = 1;

    previous_msg = msg;
}

void PositionControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {
    std::cout << "Position Push-path Mode called" << std::endl;
    fail = 0;
    control_mode_ = "Position path mode";
    operator_call_count_++;

    if (msg.points.empty()) {
        fail = 2;
        return;
    }

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

    float current_joint[NUMBER_OF_JOINT] = {0,};
    memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

    const int pointsNum = static_cast<int>(msg.points.size());
    std::unique_ptr<float[][6]> xpos(new float[pointsNum][6]);

    float spline_vel[2] = {1000, 1000};
    float spline_acc[2] = {1000, 1000};
    float tTime = msg.points.back().time_from_start.toSec();

    // constexpr float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
    // constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

    Eigen::Quaternionf q_prev =
        quatFromEulerDeg(current_position[3], current_position[4], current_position[5]);

    float prev_roll  = current_position[3];
    float prev_pitch = current_position[4];
    float prev_yaw   = current_position[5];

    for (int i = 0; i < pointsNum; ++i) {
        xpos[i][0] = msg.points[i].point.pose.position.x;
        xpos[i][1] = msg.points[i].point.pose.position.y;
        xpos[i][2] = msg.points[i].point.pose.position.z;

        Eigen::Quaternionf q_msg = quatFromMsg(msg.points[i].point.pose.orientation);

        if (isZeroQuatMsg(msg.points[i].point.pose.orientation)) {
            q_msg = q_prev;
        }

        alignQuatHemisphere(q_msg, q_prev);

        auto rpy = quatToEulerDegZYXNear(q_msg, prev_roll, prev_pitch, prev_yaw);

        xpos[i][3] = rpy[0];
        xpos[i][4] = rpy[1];
        xpos[i][5] = rpy[2];

        q_prev = q_msg;
        prev_roll  = rpy[0];
        prev_pitch = rpy[1];
        prev_yaw   = rpy[2];
    }

    float delta_yaw = xpos[pointsNum - 1][5] - xpos[0][5];

    if (current_joint[5] > 90.0f && delta_yaw > 0.0f) {
        float prep_vel[2] = {70, 70};
        float prep_acc[2] = {120, 120};

        current_position[2] += 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

        current_joint[5] -= 360;
        Drfl_.movej(current_joint, 60, 30);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

        current_position[2] -= 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);
    }
    else if (current_joint[5] < -90.0f && delta_yaw < 0.0f) {
        float prep_vel[2] = {70, 70};
        float prep_acc[2] = {120, 120};

        current_position[2] += 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

        current_joint[5] += 360;
        Drfl_.movej(current_joint, 60, 30);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

        current_position[2] -= 100;
        Drfl_.movel(current_position, prep_vel, prep_acc);
    }

    controlState = true;
    startDataSaving();

    bool success = Drfl_.amovesx(xpos.get(), pointsNum, spline_vel, spline_acc, tTime, MOVE_MODE_ABSOLUTE);

    if (!success) {
        stopDataSaving();
        controlState = false;
        fail = 2;
        return;
    }

    int wait_ret = Drfl_.mwait();

    stopDataSaving();
    controlState = false;

    if (wait_ret != 1) {
        fail = 2;
        return;
    }

    float final_position[NUMBER_OF_JOINT] = {0,};
    robot_state = Drfl_.read_data_rt();
    memcpy(final_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    float distance = std::sqrt(
        std::pow(final_position[0] - xpos[pointsNum - 1][0], 2) +
        std::pow(final_position[1] - xpos[pointsNum - 1][1], 2) +
        std::pow(final_position[2] - xpos[pointsNum - 1][2], 2)
    );

    if (distance > distance_threshold) fail = 2;
    else fail = 1;

    previous_msg = msg;
}

// =========================================================================
// [추가] PositionControlLoop::operator_jpath() (조인트 경로 제어 모드)
// =========================================================================
void PositionControlLoop::operator_jpath(const moveit_msgs::CartesianTrajectory& msg) {
    std::cout << "Position Joint Path Mode called" << std::endl;
    fail = 0;
    control_mode_ = "Position joint path mode";
    operator_call_count_++;

    if (msg.points.empty()) {
        ROS_ERROR("operator_jpath received empty trajectory.");
        fail = 2;
        return;
    }

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

    float current_joint[NUMBER_OF_JOINT] = {0, };
    memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));

    int pointsNum = msg.points.size();
    std::unique_ptr<float[][6]> jpos(new float[pointsNum][6]);
    float jvel = 100;
    float jacc = 100;
    float tTime = msg.points.back().time_from_start.toSec();

    // CartesianTrajectory 메시지를 재사용하여 조인트 각도를 전달받는 로직 (원본 유지)
    for (int i = 0; i < pointsNum; ++i) {
        jpos[i][0] = msg.points[i].point.pose.position.x;
        jpos[i][1] = msg.points[i].point.pose.position.y;
        jpos[i][2] = msg.points[i].point.pose.position.z;
        jpos[i][3] = msg.points[i].point.pose.orientation.x;
        jpos[i][4] = msg.points[i].point.pose.orientation.y;
        jpos[i][5] = msg.points[i].point.pose.orientation.z;
    }

    std::cout << "Calling amovesj with " << pointsNum << " points" << std::endl;

    controlState = true;
    ROS_INFO("Starting data saving...");
    startDataSaving();

    // amovesj 실행 (조인트 공간 경로 이동)
    bool success = Drfl_.amovesj(jpos.get(), pointsNum, jvel, jacc, tTime, MOVE_MODE_ABSOLUTE);

    if (!success) {
        ROS_ERROR("amovesj failed to start.");
        stopDataSaving();
        controlState = false;
        fail = 2;
        return;
    }

    // 비동기 함수이므로 도착할 때까지 대기
    int wait_ret = Drfl_.mwait();

    ROS_INFO("Stopping data saving...");
    stopDataSaving();
    controlState = false;

    if (wait_ret != 1) {
        ROS_ERROR("mwait reported motion failure.");
        fail = 2;
        return;
    }

    fail = 1; // 성공
    previous_msg = msg;
}
//new0406(old version)
// void ImpedanceControlLoop::runPBICGoal(const moveit_msgs::CartesianTrajectory& msg) {
//     std::cout << "\n======================================================\n";
//     std::cout << "[INFO] Impedance Goal-directed Mode called" << std::endl;
//     fail = 0;
//     control_mode_ = "PBIC goal mode";
//     operator_call_count_++;
//     sol_space = 0;
//     count = 0;
//     count_motion = 0;

//     Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE); 
//     Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
//     std::this_thread::sleep_for(std::chrono::milliseconds(10)); 

//     LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt(); 

//     float current_joint[NUMBER_OF_JOINT] = {0, };
//     memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
//     memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

//     // ----------------------------------------------------------------------------------
//     // 🟢 [TEST POINT 1] 입력값(Input) vs 현재 상태(Current) 확인
//     // ----------------------------------------------------------------------------------
//     std::cout << "[TEST POINT 1] Initial Check\n";
//     std::cout << " - Start Pos (X,Y,Z) : " << current_position[0] << ", " << current_position[1] << ", " << current_position[2] << "\n";
//     std::cout << " - Input Goal(X,Y,Z) : " << msg.points[0].point.pose.position.x << ", " 
//                                           << msg.points[0].point.pose.position.y << ", " 
//                                           << msg.points[0].point.pose.position.z << "\n";
//     std::cout << "------------------------------------------------------\n";
//     resetFillEulerDummyForIKState();
//     trajectory_gen_.init(msg, previous_msg, current_position, operator_call_count_);
    
//     Duration control_loop_time = Duration(loop_time_);
//     float st = static_cast<float>(loop_time_) / 1000;

//     start_Motion(robot_state, prev, imp);

//     auto start = std::chrono::high_resolution_clock::now();
//     loopTimes.clear();
//     controlState = true;
//     auto start_time = std::chrono::high_resolution_clock::now();
//     startDataSaving();

//     while (true) {
//         robot_state = Drfl_.read_data_rt();

//         if (!spinMotion(robot_state, control_loop_time, desired, sol_space) ||
//             !spinControl(robot_state, control_loop_time, control_command, desired, sol_space)) {
//             break;
//         }

//         // ----------------------------------------------------------------------------------
//         // 🟡 [TEST POINT 2] 실시간 생성 궤적(Desired) vs 실제 로봇 상태(Actual) 확인
//         // 주의: 1ms마다 출력하면 제어기가 뻗으므로 500 카운트(0.5초)마다 1번만 출력합니다.
//         // ----------------------------------------------------------------------------------
//         if (count % 500 == 0) {
//             std::cout << "[TEST POINT 2] Loop Count: " << count << " (Time: " << (count * st) << " sec)\n";
//             // desired.q_d 가 조인트 각도인지 위치인지에 따라 출력이 달라질 수 있습니다. (여기선 조인트로 가정)
//             std::cout << " - Desired (q_d 0~2): " << desired.q_d[0] << ", " << desired.q_d[1] << ", " << desired.q_d[2] << "\n";
//             std::cout << " - Actual  (Act 0~2): " << robot_state->actual_joint_position[0] << ", " 
//                                                   << robot_state->actual_joint_position[1] << ", " 
//                                                   << robot_state->actual_joint_position[2] << "\n";
//         }

//         if (exitLoop || g_nKill_dsr_control) {
//             fail = 2;
//             break;
//         }

//         Drfl_.torque_rt(control_command.tau_d, st);

//         auto current = std::chrono::high_resolution_clock::now();
//         Duration loop_time(std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
//         loopTimes.push_back(loop_time.toMSec());

//         if (control_loop_time > loop_time) {
//             std::this_thread::sleep_for(control_loop_time() - loop_time());
//         }
//         start = std::chrono::high_resolution_clock::now();
//         count++;
//     }

//     stopDataSaving();
//     saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
//     controlState = false;

//     robot_state = Drfl_.read_data_rt(); 
    
//     auto finished_time  = std::chrono::high_resolution_clock::now();
//     auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(finished_time - start_time);
//     std::cout << "------------------------------------------------------\n";
//     std::cout << "[INFO] Control Loop Finished. Elapsed time: " << elapsed_time.count() << " ms" << std::endl;

//     setScheduling(originalSetting_);
    
//     float final_position[NUMBER_OF_JOINT] = {0, };
//     memcpy(final_position, robot_state->actual_flange_position, NUMBER_OF_JOINT*sizeof(float)); 

//     float distance = std::sqrt(
//         std::pow(final_position[0] - msg.points[0].point.pose.position.x, 2) +
//         std::pow(final_position[1] - msg.points[0].point.pose.position.y, 2) +
//         std::pow(final_position[2] - msg.points[0].point.pose.position.z, 2)
//     );

//     // ----------------------------------------------------------------------------------
//     // 🔴 [TEST POINT 3] 최종 도착 위치(Final) vs 원래 목표(Input Goal) 오차 확인
//     // ----------------------------------------------------------------------------------
//     std::cout << "[TEST POINT 3] Final Result\n";
//     std::cout << " - Final Pos (X,Y,Z) : " << final_position[0] << ", " << final_position[1] << ", " << final_position[2] << "\n";
//     std::cout << " - Input Goal(X,Y,Z) : " << msg.points[0].point.pose.position.x << ", " 
//                                           << msg.points[0].point.pose.position.y << ", " 
//                                           << msg.points[0].point.pose.position.z << "\n";
//     std::cout << " => Final Distance Error: " << distance << " mm\n";
//     std::cout << "======================================================\n\n";

//     if (distance > distance_threshold) fail = 2;
//     else fail = 1;
    
//     previous_msg = msg;
// }

void ImpedanceControlLoop::runPBICGoal(const moveit_msgs::CartesianTrajectory& msg) {
    std::cout << "\n======================================================\n";
    std::cout << "[INFO] PB-IC Goal Mode called" << std::endl;

    fail = 0;
    control_mode_ = "PBIC goal mode";
    operator_call_count_++;
    sol_space = 0;
    count = 0;
    count_motion = 0;

    if (msg.points.empty()) {
        ROS_ERROR("Empty CartesianTrajectory received.");
        fail = 2;
        return;
    }

    // logRequestedGoalPose(this, msg);

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

    // DBIC 쪽 state reset은 getTaskState quaternion continuity / force filter 초기화용
    resetDBICControllerState();

    TaskState current_task_state =
        getTaskState(robot_state, task_point_mode_, T_flange_tcp_);

    // ------------------------------------------------------------
    // DBIC와 동일한 nominal goal trajectory 생성
    // ------------------------------------------------------------
    trajectory_gen_.initDBICGoal(current_task_state.p, current_task_state.q, msg);

    // ------------------------------------------------------------
    // PBIC outer model state 초기화
    // ------------------------------------------------------------
    resetPBICControllerState(current_task_state, robot_state);

    Duration control_loop_time = Duration(loop_time_);
    float st = static_cast<float>(loop_time_) / 1000.0f;

    auto start = std::chrono::high_resolution_clock::now();
    auto start_time = std::chrono::high_resolution_clock::now();

    loopTimes.clear();
    controlState = true;

    constexpr bool kEnablePbicDataSaving = true;
    if (kEnablePbicDataSaving) {
        startDataSaving();
    }

    TaskRef ref_tcp;
    TaskRef ref_task;

    bool entered_hold_phase = false;

    while (true) {
        robot_state = Drfl_.read_data_rt();

        // 1) DBIC nominal reference
        bool motion_ok = spinMotionDBIC(control_loop_time, ref_tcp, ref_task);

        // 2) PBIC outer model: ref_task -> x_m -> desired.q_d
        bool model_ok = spinMotionPBIC(robot_state, control_loop_time, ref_task, desired, sol_space);

        // 3) PBIC-TDC inner control: desired.q_d -> tau
        bool control_ok = spinControlPBIC(robot_state, control_loop_time, control_command, desired);

        if (exitLoop || g_nKill_dsr_control) {
            fail = 2;
            break;
        }

        if (!model_ok) {
            fail = 2;
            std::cout << "[PBIC BREAK] MotionGeneratorPBIC failed.\n";
            break;
        }

        if (!control_ok) {
            fail = 2;
            std::cout << "[PBIC BREAK] ControlGeneratorPBIC failed.\n";
            break;
        }

        // 마지막 reference도 실제 로봇에 한 번은 보낸다
        Drfl_.torque_rt(control_command.tau_d, st);

        auto current = std::chrono::high_resolution_clock::now();
        Duration loop_time(
            std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
        loopTimes.push_back(loop_time.toMSec());

        if (control_loop_time > loop_time) {
            std::this_thread::sleep_for(control_loop_time() - loop_time());
        }

        start = std::chrono::high_resolution_clock::now();
        count++;

        // nominal trajectory가 끝났으면 hold phase로 이동
        if (!motion_ok) {
            entered_hold_phase = true;
            std::cout << "[PBIC BREAK] nominal trajectory finished.\n";
            break;
        }
    }

    // ------------------------------------------------------------
    // Hold / settling phase
    // 마지막 desired.q_d를 잠시 유지하여 수렴 확인
    // ------------------------------------------------------------
    if (entered_hold_phase && fail == 0) {
        const float joint_tol_deg = 0.5f;
        const double hold_timeout_sec = 3.0;

        const int hold_steps = static_cast<int>(
            std::ceil(hold_timeout_sec / (static_cast<double>(loop_time_) * 1e-3)));

        std::cout << "[PBIC HOLD] start" << std::endl;

        start = std::chrono::high_resolution_clock::now();

        for (int hold_count = 0; hold_count < hold_steps; ++hold_count) {
            robot_state = Drfl_.read_data_rt();

            bool control_ok =
                spinControlPBIC(robot_state, control_loop_time, control_command, desired);

            if (!control_ok || exitLoop || g_nKill_dsr_control) {
                fail = 2;
                break;
            }

            Drfl_.torque_rt(control_command.tau_d, st);

            float max_joint_err_deg = 0.0f;
            for (int i = 0; i < 6; ++i) {
                float delta = desired.q_d[i] - robot_state->actual_joint_position[i];
                while (delta > 180.0f) delta -= 360.0f;
                while (delta < -180.0f) delta += 360.0f;
                max_joint_err_deg = std::max(max_joint_err_deg, std::fabs(delta));
            }

            auto current = std::chrono::high_resolution_clock::now();
            Duration loop_time(
                std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
            loopTimes.push_back(loop_time.toMSec());

            if (control_loop_time > loop_time) {
                std::this_thread::sleep_for(control_loop_time() - loop_time());
            }

            start = std::chrono::high_resolution_clock::now();

            if (max_joint_err_deg < joint_tol_deg) {
                std::cout << "[PBIC HOLD] settled, max_joint_err_deg="
                          << max_joint_err_deg << std::endl;
                break;
            }
        }
    }

    if (kEnablePbicDataSaving) {
        stopDataSaving();
        saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    }
    controlState = false;

    robot_state = Drfl_.read_data_rt();

    auto finished_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(finished_time - start_time);

    // DBIC와 동일한 nominal final goal 기준으로 final error check
    TaskState final_state =
        getTaskState(robot_state, task_point_mode_, T_flange_tcp_);

    TaskRef final_tcp =
        trajectory_gen_.sampleDBICGoal(
            msg.points[0].time_from_start.toSec(),
            static_cast<double>(loop_time_) * 1e-3);

    TaskRef final_task = convertRefToTaskPoint(final_tcp);

    const float distance_mm = 1000.0f * (final_state.p - final_task.p_d).norm();

    std::cout << "------------------------------------------------------\n";
    std::cout << "[INFO] PB-IC Finished. Elapsed time: "
              << elapsed_time.count() << " ms\n";
    std::cout << "[INFO] Final position error: " << distance_mm << " mm\n";
    std::cout << "======================================================\n\n";

    setScheduling(originalSetting_);

    if (fail != 2) {
        if (distance_mm > distance_threshold) fail = 2;
        else fail = 1;
    }

    previous_msg = msg;
}

void ImpedanceControlLoop::runPBICPath(const moveit_msgs::CartesianTrajectory& msg) {
    if (msg.points.empty()) {
        fail = 2;
        return;
    }

    std::cout << "Impedance Push-path Mode called" << std::endl;
    fail = 0;
    control_mode_ = "PBIC path mode";
    operator_call_count_++;

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt(); 
    float current_joint[NUMBER_OF_JOINT] = {0, };
    memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
    memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE); 
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    
    int originalPointsNum = msg.points.size();
    double duration = msg.points.back().time_from_start.toSec();
    int newPointsNum = static_cast<int>(duration * 1000/loop_time_);

    count = 0;
    sol_space = 0;
    count_motion = 0;
    resetFillEulerDummyForIKState();

    Duration control_loop_time = Duration(loop_time_);
    float st = static_cast<float>(loop_time_) / 1000;

    robot_state = Drfl_.read_data_rt();
    start_Motion(robot_state, prev, imp);
    
    float yaw_start = static_cast<float>(quatMsgToYawDeg(msg.points.front().point.pose.orientation));
    float yaw_end   = static_cast<float>(quatMsgToYawDeg(msg.points.back().point.pose.orientation));

    yaw_end = unwrapNear(yaw_end, yaw_start);
    float delta_yaw = yaw_end - yaw_start;

    if (current_joint[5] > 90 && delta_yaw > 0) {
        float goal_joint[NUMBER_OF_JOINT];
        current_position[2] += 100;
        LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; i++) goal_joint[i] = res->_fTargetPos[i];
        Drfl_.movej(goal_joint,60,30);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT*sizeof(float));
        current_joint[5] -= 360;
        Drfl_.movej(current_joint, 60, 30);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT*sizeof(float));
        current_position[2] -= 100;
        LPINVERSE_KINEMATIC_RESPONSE res2 = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; i++) goal_joint[i] = res2->_fTargetPos[i];
        Drfl_.movej(goal_joint,60,30);
    }
    else if (current_joint[5] < -90 && delta_yaw < 0) {
        float goal_joint[NUMBER_OF_JOINT];
        current_position[2] += 100;
        LPINVERSE_KINEMATIC_RESPONSE res = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; i++) goal_joint[i] = res->_fTargetPos[i];
        Drfl_.movej(goal_joint,60,30);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT*sizeof(float));
        current_joint[5] += 360;
        Drfl_.movej(current_joint, 60, 30);

        robot_state = Drfl_.read_data_rt();
        memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT*sizeof(float));
        current_position[2] -= 100;
        LPINVERSE_KINEMATIC_RESPONSE res2 = Drfl_.ikin(current_position, 2, COORDINATE_SYSTEM_BASE, 1);
        for (int i = 0; i < 6; i++) goal_joint[i] = res2->_fTargetPos[i];
        Drfl_.movej(goal_joint,60,30);
    }
    
    robot_state = Drfl_.read_data_rt(); 
    memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    auto start = std::chrono::high_resolution_clock::now();
    float initiate_pos[NUMBER_OF_JOINT] = {0, };

    initiate_pos[0] = msg.points[0].point.pose.position.x;
    initiate_pos[1] = msg.points[0].point.pose.position.y;
    initiate_pos[2] = msg.points[0].point.pose.position.z;
    initiate_pos[3] = current_position[3];
    initiate_pos[4] = current_position[4];

    float distance = std::sqrt(
        std::pow(initiate_pos[0] - current_position[0], 2) +
        std::pow(initiate_pos[1] - current_position[1], 2) +
        std::pow(initiate_pos[2] - current_position[2], 2)
    );

    if (distance > distance_threshold) {
        fail = 2;
        return;
    }

    robot_state = Drfl_.read_data_rt(); 
    memcpy(current_joint, robot_state->actual_joint_position, NUMBER_OF_JOINT*sizeof(float));
    memcpy(current_position, robot_state->actual_flange_position, NUMBER_OF_JOINT*sizeof(float));

    trajectory_gen_.CurvePoints_ = trajectory_gen_.upsampleTrajectory(msg, newPointsNum);

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE); 
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    controlState = true; 
    loopTimes.clear();
    startDataSaving();

    while (spinMotion_path(robot_state, control_loop_time, desired, sol_space) && spinControl(robot_state, control_loop_time, control_command, desired, sol_space)) {
        robot_state = Drfl_.read_data_rt();
        
        if (!exitLoop && !g_nKill_dsr_control) {
            Drfl_.torque_rt(control_command.tau_d, st);
            auto current = std::chrono::high_resolution_clock::now();
            auto loop_time = std::chrono::duration_cast<std::chrono::milliseconds>(current - start);

            if (std::chrono::milliseconds(static_cast<int64_t>(control_loop_time.toMSec())) > loop_time) {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(control_loop_time.toMSec())) - loop_time);
            }
            start = std::chrono::high_resolution_clock::now();
        } else {
            fail = true;
            break;
        }

        
        count++;
    }
    stopDataSaving();
    saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    previous_msg = msg;
    controlState = false;

    setScheduling(originalSetting_);
    float final_position[NUMBER_OF_JOINT] = {0, };
    memcpy(final_position, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));

    distance = std::sqrt(
        std::pow(final_position[0] - msg.points[originalPointsNum - 1].point.pose.position.x, 2) +
        std::pow(final_position[1] - msg.points[originalPointsNum - 1].point.pose.position.y, 2) +
        std::pow(final_position[2] - msg.points[originalPointsNum - 1].point.pose.position.z, 2)
    );

    if (distance > distance_threshold) fail = 2;
    else fail = 1;
}

void ImpedanceControlLoop::runDBICGoal(const moveit_msgs::CartesianTrajectory& msg) {
    std::cout << "\n======================================================\n";
    std::cout << "[INFO] DB-IC Goal Mode called" << std::endl;

    fail = 0;
    control_mode_ = "DBIC goal mode";
    operator_call_count_++;
    count = 0;

    if (msg.points.empty()) {
        ROS_ERROR("Empty CartesianTrajectory received.");
        fail = 2;
        return;
    }

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

    resetDBICControllerState();

    TaskState current_task_state =
        getTaskState(robot_state, task_point_mode_, T_flange_tcp_);

    trajectory_gen_.initDBICGoal(current_task_state.p, current_task_state.q, msg);

    Duration control_loop_time = Duration(loop_time_);
    float st = static_cast<float>(loop_time_) / 1000.0f;

    auto start = std::chrono::high_resolution_clock::now();
    auto start_time = std::chrono::high_resolution_clock::now();
//new0322
    // loopTimes.clear();
    // controlState = true;
    // startDataSaving();

    loopTimes.clear();
    controlState = true;

    constexpr bool kEnableDbicDataSaving = false;
    if (kEnableDbicDataSaving) {
        startDataSaving();
    }    
//
    TaskRef ref_tcp;
    TaskRef ref_task;

    // hold phase용 상태 변수는 while 바깥에서 선언해야 한다.
    bool entered_hold_phase = false;
    TaskRef hold_ref_task{};

    while (true) {
        robot_state = Drfl_.read_data_rt();

        bool motion_ok  = spinMotionDBIC(control_loop_time, ref_tcp, ref_task);
        bool control_ok = spinControlDBIC(robot_state, control_loop_time, control_command, ref_task);

        if (exitLoop || g_nKill_dsr_control) {
            fail = 2;
            break;
        }

        // 마지막 trajectory sample도 실제 로봇에 한 번은 보낸다.
        Drfl_.torque_rt(control_command.tau_d, st);

        auto current = std::chrono::high_resolution_clock::now();
        Duration loop_time(
            std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
        loopTimes.push_back(loop_time.toMSec());

        if (control_loop_time > loop_time) {
            std::this_thread::sleep_for(control_loop_time() - loop_time());
        }

        start = std::chrono::high_resolution_clock::now();
        count++;   // main loop에서는 필요: spinMotionDBIC()의 t_sec를 전진시킴

        if (!control_ok) {
            fail = 2;
            std::cout << "[DBIC BREAK] motion_ok=" << motion_ok
                      << ", control_ok=" << control_ok
                      << ", count=" << count
                      << ", t_sec=" << (static_cast<double>(count) * loop_time_ * 1e-3)
                      << std::endl;
            break;
        }

        // trajectory 시간이 끝났으면 final reference를 hold phase로 넘긴다.
        if (!motion_ok) {
            entered_hold_phase = true;
            hold_ref_task = ref_task;

            // hold 단계에서는 "정지한 최종 목표"를 유지해야 하므로
            // 속도/가속도 reference는 0으로 만든다.
            hold_ref_task.v_d.setZero();
            hold_ref_task.a_d.setZero();
            hold_ref_task.w_d.setZero();
            hold_ref_task.alpha_d.setZero();
            hold_ref_task.motion_finished = false;

            std::cout << "[DBIC BREAK] motion_ok=" << motion_ok
                      << ", control_ok=" << control_ok
                      << ", count=" << count
                      << ", t_sec=" << (static_cast<double>(count) * loop_time_ * 1e-3)
                      << std::endl;
            break;
        }
    }

    // ------------------------------------------------------------
    // Hold / settling phase
    // ------------------------------------------------------------
    if (entered_hold_phase && fail == 0) {
        const float pos_tol_m = 0.005f;      // 5 mm
        const float rot_tol_rad = 0.02f;     // 약 1.15 deg
        const double hold_timeout_sec = 3.0;

        const int hold_steps = static_cast<int>(
            std::ceil(hold_timeout_sec / (static_cast<double>(loop_time_) * 1e-3)));

        std::cout << "[DBIC HOLD] start" << std::endl;

        // hold loop는 별도의 주기 타이밍만 관리하면 된다.
        start = std::chrono::high_resolution_clock::now();

        for (int hold_count = 0; hold_count < hold_steps; ++hold_count) {
            robot_state = Drfl_.read_data_rt();

            TaskState s_hold =
                getTaskState(robot_state, task_point_mode_, T_flange_tcp_);

            const float pos_err_m =
                (hold_ref_task.p_d - s_hold.p).norm();
            const float rot_err_rad =
                quatLogError(hold_ref_task.q_d, s_hold.q).norm();

            bool control_ok =
                spinControlDBIC(robot_state, control_loop_time, control_command, hold_ref_task);

            if (!control_ok || exitLoop || g_nKill_dsr_control) {
                fail = 2;
                break;
            }

            Drfl_.torque_rt(control_command.tau_d, st);

            auto current = std::chrono::high_resolution_clock::now();
            Duration loop_time(
                std::chrono::duration_cast<std::chrono::milliseconds>(current - start));
            loopTimes.push_back(loop_time.toMSec());

            if (control_loop_time > loop_time) {
                std::this_thread::sleep_for(control_loop_time() - loop_time());
            }

            start = std::chrono::high_resolution_clock::now();

            // hold loop에서는 count++ 하지 않는다.
            // 여기서는 trajectory time을 전진시키는 게 아니라,
            // final reference를 고정한 채 수렴만 시키는 단계다.

            if (pos_err_m < pos_tol_m && rot_err_rad < rot_tol_rad) {
                std::cout << "[DBIC HOLD] settled, pos_err_mm="
                          << pos_err_m * 1000.0f
                          << ", rot_err_rad=" << rot_err_rad
                          << std::endl;
                break;
            }
        }
    }
//new0322
    // stopDataSaving();
    // saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    // controlState = false;
    if (kEnableDbicDataSaving) {
        stopDataSaving();
        saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    }
    controlState = false;

    robot_state = Drfl_.read_data_rt();

    auto finished_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(finished_time - start_time);

    TaskState final_state =
        getTaskState(robot_state, task_point_mode_, T_flange_tcp_);

    TaskRef final_tcp =
        trajectory_gen_.sampleDBICGoal(msg.points[0].time_from_start.toSec(),
                                       static_cast<double>(loop_time_) * 1e-3);
    TaskRef final_task = convertRefToTaskPoint(final_tcp);

    const float distance_mm = 1000.0f * (final_state.p - final_task.p_d).norm();

    std::cout << "------------------------------------------------------\n";
    std::cout << "[INFO] DB-IC Finished. Elapsed time: "
              << elapsed_time.count() << " ms\n";
    std::cout << "[INFO] Final position error: " << distance_mm << " mm\n";
    std::cout << "======================================================\n\n";

    setScheduling(originalSetting_);

    // 이미 중간에 fail=2가 났으면 그 상태를 유지해야 한다.
    if (fail != 2) {
        if (distance_mm > distance_threshold) fail = 2;
        else fail = 1;
    }

    previous_msg = msg;
}

void ImpedanceControlLoop::runDBICPath(const moveit_msgs::CartesianTrajectory& msg) {
    if (msg.points.empty()) {
        fail = 2;
        return;
    }

    std::cout << "[INFO] DB-IC Path Mode called" << std::endl;

    fail = 0;
    control_mode_ = "DBIC path mode";
    operator_call_count_++;
    count = 0;

    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();

    resetDBICControllerState();

    TaskState current_task_state =
        getTaskState(robot_state, task_point_mode_, T_flange_tcp_);

    trajectory_gen_.initDBICPath(current_task_state.p,
                                 current_task_state.q,
                                 msg,
                                 static_cast<double>(loop_time_) * 1e-3);

    Duration control_loop_time = Duration(loop_time_);
    float st = static_cast<float>(loop_time_) / 1000.0f;

    auto start = std::chrono::high_resolution_clock::now();
    auto start_time = std::chrono::high_resolution_clock::now();
//new0322
    // loopTimes.clear();
    // controlState = true;
    // startDataSaving();
    loopTimes.clear();
    controlState = true;

    constexpr bool kEnableDbicDataSaving = false;
    if (kEnableDbicDataSaving) {
        startDataSaving();
    }

    TaskRef ref_tcp;
    TaskRef ref_task;

    while (true) {
        robot_state = Drfl_.read_data_rt();

        if (!spinMotionDBIC(control_loop_time, ref_tcp, ref_task) ||
            !spinControlDBIC(robot_state, control_loop_time, control_command, ref_task)) {
            break;
        }

        if (exitLoop || g_nKill_dsr_control) {
            fail = 2;
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
//new0322
    // stopDataSaving();
    // saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    // controlState = false;
    if (kEnableDbicDataSaving) {
        stopDataSaving();
        saveLoopTimesToFile(dataDirectory + "/loop_times.txt");
    }
    controlState = false;
    
    robot_state = Drfl_.read_data_rt();

    auto finished_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(finished_time - start_time);

    TaskState final_state =
        getTaskState(robot_state, task_point_mode_, T_flange_tcp_);

    TaskRef final_tcp =
        trajectory_gen_.sampleDBICPath(trajectory_gen_.dbic_path_samples_.size() - 1);
    TaskRef final_task = convertRefToTaskPoint(final_tcp);

    const float distance_mm = 1000.0f * (final_state.p - final_task.p_d).norm();

    std::cout << "[INFO] DB-IC Path Finished. Elapsed time: "
              << elapsed_time.count() << " ms\n";
    std::cout << "[INFO] Final path endpoint error: " << distance_mm << " mm\n";

    setScheduling(originalSetting_);

    if (distance_mm > distance_threshold) fail = 2;
    else fail = 1;

    previous_msg = msg;
}

void ImpedanceControlLoop::operator()(const moveit_msgs::CartesianTrajectory& msg) {
    switch (impedance_impl_mode_) {
        case ImpedanceImplMode::kDBIC:
            runDBICGoal(msg);
            break;
        case ImpedanceImplMode::kPBIC_TDC:
        default:
            runPBICGoal(msg);
            break;
    }
}

void ImpedanceControlLoop::operator_path(const moveit_msgs::CartesianTrajectory& msg) {
    switch (impedance_impl_mode_) {
        case ImpedanceImplMode::kDBIC:
            runDBICPath(msg);
            break;
        case ImpedanceImplMode::kPBIC_TDC:
        default:
            runPBICPath(msg);
            break;
    }
}
//new0406(old version)
// bool ControlLoop::spinMotion(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Desired& desired, int sol_space) {
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
//     return false;
// }
bool ControlLoop::spinMotion(const LPRT_OUTPUT_DATA_LIST& robot_state,
                             SKKU::Duration time_step,
                             Desired& desired,
                             int sol_space) {
    (void)time_step;
    bool correction_flag = false;

    // 기본값 리셋
    g_pbic_goal_motion_finished = false;
    g_pbic_goal_motion_failed = false;

    // ------------------------------------------------------------------
    // PBIC goal mode:
    // DBIC와 동일한 nominal task-space goal trajectory를 사용하고,
    // 그 ref_task를 legacy PBIC MotionGenerator가 먹을 수 있는
    // trajectory 구조체(mm + quaternion pose, translation vel/acc only)로 변환한다.
    //
    // 전제:
    // - 현재 PBIC는 flange 기준(TaskPointMode::kFlange)에서 사용한다.
    // - MotionGenerator()는 legacy unit(mm/deg)과 fillEulerDummyForIK() 경로를 기대한다.
    // ------------------------------------------------------------------
    const bool use_dbic_nominal_for_pbic_goal =
        (control_mode_ == "PBIC goal mode") &&
        (trajectory_gen_.dbic_mode_ == TrajectoryGen::DBICMode::kGoal);

    if (use_dbic_nominal_for_pbic_goal) {
        const double t_sec  = g_pbic_goal_elapsed_sec;   // wall-clock 기준
        const double dt_sec = static_cast<double>(loop_time_) * 1e-3;

        TaskRef ref_tcp  = trajectory_gen_.sampleDBICGoal(t_sec, dt_sec);
        TaskRef ref_task = convertRefToTaskPoint(ref_tcp);

        // --------------------------------------------------------------
        // MotionGenerator() / dataSaving()가 기대하는 legacy trajectory format
        // pos_d : [x(mm), y(mm), z(mm), qx, qy, qz, qw]
        // vel_d : translation만 사용 [mm/s], orientation rate는 fillEulerDummyForIK에서 0 처리
        // acc_d : translation만 사용 [mm/s^2], orientation acc는 fillEulerDummyForIK에서 0 처리
        // --------------------------------------------------------------
        trajectory.pos_d[0] = ref_task.p_d(0) * 1000.0f;
        trajectory.pos_d[1] = ref_task.p_d(1) * 1000.0f;
        trajectory.pos_d[2] = ref_task.p_d(2) * 1000.0f;
        trajectory.pos_d[3] = ref_task.q_d.x();
        trajectory.pos_d[4] = ref_task.q_d.y();
        trajectory.pos_d[5] = ref_task.q_d.z();
        trajectory.pos_d[6] = ref_task.q_d.w();

        trajectory.vel_d[0] = ref_task.v_d(0) * 1000.0f;
        trajectory.vel_d[1] = ref_task.v_d(1) * 1000.0f;
        trajectory.vel_d[2] = ref_task.v_d(2) * 1000.0f;
        trajectory.vel_d[3] = 0.0f;
        trajectory.vel_d[4] = 0.0f;
        trajectory.vel_d[5] = 0.0f;
        trajectory.vel_d[6] = 0.0f;

        trajectory.acc_d[0] = ref_task.a_d(0) * 1000.0f;
        trajectory.acc_d[1] = ref_task.a_d(1) * 1000.0f;
        trajectory.acc_d[2] = ref_task.a_d(2) * 1000.0f;
        trajectory.acc_d[3] = 0.0f;
        trajectory.acc_d[4] = 0.0f;
        trajectory.acc_d[5] = 0.0f;
        trajectory.acc_d[6] = 0.0f;

        Trajectory dummy_traj_for_ik;
        fillEulerDummyForIK(trajectory, dummy_traj_for_ik);

        auto [output, is_singular] =
            MotionGenerator(dummy_traj_for_ik,
                            robot_state,
                            prev,
                            imp,
                            sol_space,
                            correction_flag,
                            operator_call_count_);

        if (is_singular) {
            g_pbic_goal_motion_failed = true;
            std::cout << "PBIC spinMotion(): singularity / IK branch jump detected." << std::endl;
            return false;
        }

        desired.q_d = output;

        // sampleDBICGoal()은 t > T 에서 motion_finished = true
        g_pbic_goal_motion_finished = ref_task.motion_finished;
        return !g_pbic_goal_motion_finished;
    }

    // ------------------------------------------------------------------
    // legacy PBIC / 기존 path / 기존 goal fallback
    // ------------------------------------------------------------------
    tra.time = static_cast<double>(count) * loop_time_ / 1000.0;
    trajectory_gen_.setLoopTime(loop_time_);

    if (tra.time <= plan.time) {
        trajectory_gen_.TrajectoryGenerator(&plan, &tra);

        for (int i = 0; i < 7; ++i) {
            trajectory.pos_d[i] = tra.pos[i];
            trajectory.vel_d[i] = tra.vel[i];
            trajectory.acc_d[i] = tra.acc[i];
        }

        Trajectory dummy_traj_for_ik;
        fillEulerDummyForIK(trajectory, dummy_traj_for_ik);

        auto [output, is_singular] =
            MotionGenerator(dummy_traj_for_ik,
                            robot_state,
                            prev,
                            imp,
                            sol_space,
                            correction_flag,
                            operator_call_count_);

        if (is_singular) {
            std::cout << "Singularity occurred! Exiting loop." << std::endl;
            return false;
        }

        desired.q_d = output;
        return true;
    }

    return false;
}


bool ControlLoop::spinMotion_path(const LPRT_OUTPUT_DATA_LIST& robot_state,
                                  SKKU::Duration time_step,
                                  Desired& desired,
                                  int sol_space) {
    bool correction_flag = false;

    const int total = static_cast<int>(trajectory_gen_.CurvePoints_.size());
    if (count >= total) {
        return desired.motion_finished;
    }

    auto clamp_idx = [&](int idx) {
        return std::max(0, std::min(idx, total - 1));
    };

    const auto& pm1 = trajectory_gen_.CurvePoints_[clamp_idx(count - 1)];
    const auto& p0  = trajectory_gen_.CurvePoints_[clamp_idx(count)];
    const auto& p1  = trajectory_gen_.CurvePoints_[clamp_idx(count + 1)];
    const auto& p2  = trajectory_gen_.CurvePoints_[clamp_idx(count + 2)];

    const float dt_s = static_cast<float>(loop_time_) / 1000.0f;

    for (int i = 0; i < 7; ++i) {
        trajectory.pos_d[i] = static_cast<float>(p0[i]);
    }

    // translation velocity / acceleration only
    for (int i = 0; i < 3; ++i) {
        if (total == 1) {
            trajectory.vel_d[i] = 0.0f;
            trajectory.acc_d[i] = 0.0f;
        } else if (count == 0) {
            trajectory.vel_d[i] = static_cast<float>((p1[i] - p0[i]) / dt_s);
            trajectory.acc_d[i] = static_cast<float>((p2[i] - 2.0 * p1[i] + p0[i]) / (dt_s * dt_s));
        } else if (count == total - 1) {
            const auto& pm2 = trajectory_gen_.CurvePoints_[clamp_idx(count - 2)];
            trajectory.vel_d[i] = static_cast<float>((p0[i] - pm1[i]) / dt_s);
            trajectory.acc_d[i] = static_cast<float>((p0[i] - 2.0 * pm1[i] + pm2[i]) / (dt_s * dt_s));
        } else {
            trajectory.vel_d[i] = static_cast<float>((p1[i] - pm1[i]) / (2.0 * dt_s));
            trajectory.acc_d[i] = static_cast<float>((p1[i] - 2.0 * p0[i] + pm1[i]) / (dt_s * dt_s));
        }
    }

    // quaternion component derivative는 angular velocity가 아니므로
    // PBIC legacy path에서는 orientation vel/acc는 0 유지
    for (int i = 3; i < 7; ++i) {
        trajectory.vel_d[i] = 0.0f;
        trajectory.acc_d[i] = 0.0f;
    }

    Trajectory dummy_traj_for_ik;
    fillEulerDummyForIK(trajectory, dummy_traj_for_ik);

    auto [output, is_singular] =
        MotionGenerator(dummy_traj_for_ik, robot_state, prev, imp,
                        sol_space, correction_flag, operator_call_count_);

    if (is_singular) {
        return false;
    }

    desired.q_d = output;
    return !desired.motion_finished;
}

bool ControlLoop::spinControl(const LPRT_OUTPUT_DATA_LIST& robot_state, SKKU::Duration time_step, Torques& command, Desired& desired, int sol_space) {
    Torques control_output = ControlGenerator(trajectory, desired, robot_state, errors, count);
    for (int i = 0; i < 6; i++) {
        command.tau_d[i] = control_output.tau_d[i];
    }
    return !command.motion_finished;
}
//new0317
TaskRef ControlLoop::convertRefToTaskPoint(const TaskRef& tcp_ref) const {
    if (task_point_mode_ == TaskPointMode::kTCP) {
        return tcp_ref;
    }

    TaskRef out = tcp_ref;

    Eigen::Isometry3f T_B_TCP = poseToIso(tcp_ref.p_d, tcp_ref.q_d);
    Eigen::Isometry3f T_B_F   = T_B_TCP * T_flange_tcp_.inverse();

    out.p_d = T_B_F.translation();
    out.q_d = Eigen::Quaternionf(T_B_F.linear());
    out.q_d.normalize();

    const Eigen::Vector3f r = T_B_TCP.translation() - T_B_F.translation();

    out.w_d = tcp_ref.w_d;
    out.v_d = tcp_ref.v_d - tcp_ref.w_d.cross(r);
    out.alpha_d = tcp_ref.alpha_d;
    out.a_d = tcp_ref.a_d
            - tcp_ref.alpha_d.cross(r)
            - tcp_ref.w_d.cross(tcp_ref.w_d.cross(r));

    out.motion_finished = tcp_ref.motion_finished;
    return out;
}

bool ControlLoop::spinMotionDBIC(SKKU::Duration time_step,
                                 TaskRef& ref_tcp,
                                 TaskRef& ref_task) {
    (void)time_step;

    const double t_sec = static_cast<double>(count) * loop_time_ * 1e-3;
    const double dt_sec = static_cast<double>(loop_time_) * 1e-3;

    if (trajectory_gen_.dbic_mode_ == TrajectoryGen::DBICMode::kGoal) {
        ref_tcp = trajectory_gen_.sampleDBICGoal(t_sec, dt_sec);
    } else if (trajectory_gen_.dbic_mode_ == TrajectoryGen::DBICMode::kPath) {
        ref_tcp = trajectory_gen_.sampleDBICPath(static_cast<size_t>(count));
    } else {
        return false;
    }

    ref_task = convertRefToTaskPoint(ref_tcp);

    // logging용 trajectory 갱신
    trajectory.pos_d[0] = ref_task.p_d(0) * 1000.0f;
    trajectory.pos_d[1] = ref_task.p_d(1) * 1000.0f;
    trajectory.pos_d[2] = ref_task.p_d(2) * 1000.0f;
    trajectory.pos_d[3] = ref_task.q_d.x();
    trajectory.pos_d[4] = ref_task.q_d.y();
    trajectory.pos_d[5] = ref_task.q_d.z();
    trajectory.pos_d[6] = ref_task.q_d.w();

    trajectory.vel_d[0] = ref_task.v_d(0) * 1000.0f;
    trajectory.vel_d[1] = ref_task.v_d(1) * 1000.0f;
    trajectory.vel_d[2] = ref_task.v_d(2) * 1000.0f;
    trajectory.vel_d[3] = ref_task.w_d(0) * RAD2DEG;
    trajectory.vel_d[4] = ref_task.w_d(1) * RAD2DEG;
    trajectory.vel_d[5] = ref_task.w_d(2) * RAD2DEG;
    trajectory.vel_d[6] = 0.0f;

    trajectory.acc_d[0] = ref_task.a_d(0) * 1000.0f;
    trajectory.acc_d[1] = ref_task.a_d(1) * 1000.0f;
    trajectory.acc_d[2] = ref_task.a_d(2) * 1000.0f;
    trajectory.acc_d[3] = ref_task.alpha_d(0) * RAD2DEG;
    trajectory.acc_d[4] = ref_task.alpha_d(1) * RAD2DEG;
    trajectory.acc_d[5] = ref_task.alpha_d(2) * RAD2DEG;
    trajectory.acc_d[6] = 0.0f;

    return !ref_task.motion_finished;
}

bool ControlLoop::spinControlDBIC(const LPRT_OUTPUT_DATA_LIST& robot_state,
                                  SKKU::Duration time_step,
                                  Torques& control_command,
                                  const TaskRef& ref_task) {
    (void)time_step;

    Torques control_output =
        ControlGeneratorDBIC(ref_task, robot_state, task_point_mode_, T_flange_tcp_);

    for (int i = 0; i < 6; ++i) {
        control_command.tau_d[i] = control_output.tau_d[i];
    }

    return !control_command.motion_finished;
}
//new0407
bool ControlLoop::spinMotionPBIC(const LPRT_OUTPUT_DATA_LIST& robot_state,
                                 SKKU::Duration time_step,
                                 const TaskRef& ref_task,
                                 Desired& desired,
                                 int& sol_space) {
    (void)time_step;

    auto [output, is_singular] =
        MotionGeneratorPBIC(ref_task,
                            robot_state,
                            task_point_mode_,
                            T_flange_tcp_,
                            sol_space);

    if (is_singular) {
        return false;
    }

    desired.q_d = output;
    return true;
}

bool ControlLoop::spinControlPBIC(const LPRT_OUTPUT_DATA_LIST& robot_state,
                                  SKKU::Duration time_step,
                                  Torques& control_command,
                                  Desired& desired) {
    (void)time_step;

    Torques control_output =
        ControlGeneratorPBIC(desired, robot_state, errors, count);

    for (int i = 0; i < 6; ++i) {
        control_command.tau_d[i] = control_output.tau_d[i];
    }

    return !control_command.motion_finished;
}

void ControlLoop::saveLoopTimesToFile(const std::string& filePath) {
    std::ofstream loopTimeFile(filePath, std::ios::app);
    if (loopTimeFile.is_open()) {
        for (const auto& time : loopTimes) {
            loopTimeFile << time << "\n";
        }
        loopTimeFile.close();
    }
}

void ControlLoop::logData(const std::string& fileName, const float* data, int dataSize) {
    if (!isDirectoryCreated && (operator_call_count_ == 1)) {
        createNewDataDirectory();
        isDirectoryCreated = true;  
    }
    const std::string fullPath = dataDirectory +  "/" +fileName;
    std::ofstream file(fullPath, std::ios::app);  
    if (file.is_open()) {
        for (int i = 0; i < dataSize - 1; ++i) {
            file << data[i] << "\t";
        }
        file << data[dataSize - 1] << std::endl;
        file.close();
    }
}

void ControlLoop::logMatrixData(const std::string& fileName, const float matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT], int rows, int cols) {
    if (!isDirectoryCreated && (operator_call_count_ == 1)) {
        createNewDataDirectory();
        isDirectoryCreated = true;  
    }
    const std::string fullPath = dataDirectory + "/" + fileName;
    std::ofstream file(fullPath, std::ios::app);  
    if (file.is_open()) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file << std::setw(10) << matrix[i][j];
                if (j < cols - 1) {
                    file << "\t";
                }
            }
            file << std::endl;
        }
        file.close();
    }
}

void ControlLoop::logMatrixData3x3(const std::string& fileName, const float matrix[3][3], int rows, int cols) {
    if (!isDirectoryCreated && (operator_call_count_ == 1)) {
        createNewDataDirectory();
        isDirectoryCreated = true;  
    }
    const std::string fullPath = dataDirectory + "/" + fileName;
    std::ofstream file(fullPath, std::ios::app);  
    if (file.is_open()) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file << std::setw(10) << matrix[i][j];
                if (j < cols - 1) {
                    file << "\t";
                }
            }
            file << std::endl;
        }
        file.close();
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
    float F_external_box[NUMBER_OF_JOINT] = {0,};
    float actual_position2[NUMBER_OF_JOINT] = {0,};
    float joint_error[NUMBER_OF_JOINT] = {0,};
    float position_error[NUMBER_OF_JOINT] = {0,};
    float time[1] = {0,};

    auto start = std::chrono::high_resolution_clock::now();
    bool correction_flag = false;
    while (!gaincheckloop){
        LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();
        MotionGenerator(trajectory, robot_state, prev, imp, sol_space,correction_flag,operator_call_count_);

        memcpy(gravity_torque, robot_state->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_position2, robot_state->actual_flange_position, NUMBER_OF_JOINT * sizeof(float));
        memcpy(raw_torque, robot_state->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(external_torque, robot_state->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_positionj, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
        LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
        for(int i=0; i<6; i++){
            actual_position[i] = res->_fPosition[i];
        }
        
        convertToArray(trajectory.pos_d, traj_position);
        
        float traj_position_6d[6] = {0,};
        traj_position_6d[0] = traj_position[0]; 
        traj_position_6d[1] = traj_position[1]; 
        traj_position_6d[2] = traj_position[2]; 

        Eigen::Quaternionf q_gain_log(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
        auto gain_rpy = quatToEulerDegZYX(q_gain_log);

        traj_position_6d[3] = gain_rpy[0];
        traj_position_6d[4] = gain_rpy[1];
        traj_position_6d[5] = gain_rpy[2];
        /*new0317
        for (int i = 0; i<6 ; i++){
        impedance_position[i] = imp.pos_m(i);
        position_command[i] = desired.q_d[i];
        F_external[i] = F.Fext[i];
        F_impedance[i] = F.Fimp[i];
        joint_error[i] = position_command[i] - actual_positionj[i];
        time[0] += dt;
        }*/
        //new0317
        for (int i = 0; i < 6; ++i) {
            impedance_position[i] = imp.pos_m(i);
            position_command[i] = desired.q_d[i];
            F_external[i] = F.Fext[i];
            F_impedance[i] = F.Fimp[i];
            joint_error[i] = position_command[i] - actual_positionj[i];
        }
        time[0] += dt;
        //

        logData("time.txt",time,1);
        logData("task_position.txt", actual_position, NUMBER_OF_JOINT);
        logData("task_trajectory.txt", traj_position_6d, NUMBER_OF_JOINT); 
    
        logData("joint_position.txt", actual_positionj, NUMBER_OF_JOINT);
        logData("joint_command.txt", position_command, NUMBER_OF_JOINT);
        logData("raw_torque.txt", raw_torque, NUMBER_OF_JOINT);
        logData("command_torque.txt", control_command.tau_d, NUMBER_OF_JOINT);
        logData("gravity_torque.txt", gravity_torque, NUMBER_OF_JOINT);
        logData("external_torque.txt", external_torque, NUMBER_OF_JOINT);
        logData("force_external.txt",F_external, NUMBER_OF_JOINT);

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
    float F_task_log[NUMBER_OF_JOINT] = {0,};   // 추가
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
    float alpha = 0.1;
    float actual_quat[4] = {0,};       
    float traj_quat[4] = {0,};         
    float orientation_error[3] = {0,};  

    //new0324
    float raw_actual_flange_position[NUMBER_OF_JOINT] = {0,};
    float raw_actual_tcp_position[NUMBER_OF_JOINT] = {0,};

    float actual_flange_quat_assuming_zyz[4] = {0,};
    float actual_flange_quat_assuming_xyz[4] = {0,};

    float actual_flange_quat_assuming_zyz2[4] = {0,};
    float actual_flange_quat_assuming_zyx[4] = {0,};

    float actual_motor_torque[NUMBER_OF_JOINT] = {0,};
    float target_motor_torque[NUMBER_OF_JOINT] = {0,};

    float actual_flange_position[NUMBER_OF_TASK] = {0,};
    float actual_tcp_position[NUM_TASK] = {0,};

    float Raw_external_force[NUMBER_OF_TASK] = {0,};
    float error[NUMBER_OF_TASK] = {0,};
    float error_dot[NUMBER_OF_TASK] = {0,};

    float ref_v_d_log[3] = {0,};
    float s_v_log[3] = {0,};
    float ref_w_d_log[3] = {0,};
    float s_w_log[3] = {0,};
    //

    /*new0317
    std::unordered_map<std::string, int> modeMap = {
        {"Position goal mode", 0},
        {"Impedance goal mode", 1},
        {"Position path mode", 2},
        {"Impedance path mode", 3}
    };*/
    //new0317
    std::unordered_map<std::string, int> modeMap = {
        {"Position goal mode", 0},
        {"PBIC goal mode", 1},
        {"Position path mode", 2},
        {"PBIC path mode", 3},
        {"DBIC goal mode", 4},
        {"DBIC path mode", 5}
    };
    //
    auto it = modeMap.find(control_mode_);

    if (it != modeMap.end()) {
        // switch (it->second) {
        //     case 0: controlMode[0] = 0.0f; break;
        //     case 1: controlMode[0] = 1.0f; break;
        //     case 2: controlMode[0] = 2.0f; break;
        //     case 3: controlMode[0] = 3.0f; break;
        //     default: controlMode[0] = -1.0f; break;
        // }

        switch (it->second) {
            case 0: controlMode[0] = 0.0f; break;
            case 1: controlMode[0] = 1.0f; break;
            case 2: controlMode[0] = 2.0f; break;
            case 3: controlMode[0] = 3.0f; break;
            case 4: controlMode[0] = 4.0f; break;
            case 5: controlMode[0] = 5.0f; break;
            default: controlMode[0] = -1.0f; break;
        }
    } else {
        controlMode[0] = -1.0f; 
    }

    static Eigen::Quaternionf q_act_prev = Eigen::Quaternionf::Identity();
    static Eigen::Quaternionf q_des_prev = Eigen::Quaternionf::Identity();
    static bool is_first_run = true;

    float massMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
    float coriolisMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
    float jacobianMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
    float rotationMatrix[3][3] = {{0,}};
    
    auto start = std::chrono::high_resolution_clock::now();

    while (data_saving_running_){
        
        LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();
        //new0317
        const bool is_dbic_mode =
            (control_mode_ == "DBIC goal mode" ||
             control_mode_ == "DBIC path mode");

        TaskState s_task;
        if (is_dbic_mode) {
            s_task = getTaskState(robot_state, task_point_mode_, T_flange_tcp_);
        }
        //
        memcpy(trq_g, robot_state->gravity_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_position2, robot_state->actual_tcp_position, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_raw, robot_state->raw_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_positionj, robot_state->actual_joint_position, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_velocityj, robot_state->actual_joint_velocity, NUMBER_OF_JOINT * sizeof(float));
        memcpy(actual_velocity, robot_state->actual_flange_velocity, NUMBER_OF_JOINT * sizeof(float));
        memcpy(F_external_box, robot_state->external_tcp_force, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_ext, robot_state->external_joint_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_force, robot_state->raw_force_torque, NUMBER_OF_JOINT * sizeof(float));
        memcpy(trq_act, robot_state->actual_joint_torque, NUMBER_OF_JOINT * sizeof(float));

        for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
            accelerationj[i] = (actual_velocityj[i] - previous_velocityj[i]) / dt; 
            filtered_accelerationj[i] = alpha * accelerationj[i] + (1 - alpha) * filtered_accelerationj[i];
            previous_velocityj[i] = actual_velocityj[i];
        }
        
        memcpy(massMatrix, robot_state->mass_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        memcpy(coriolisMatrix, robot_state->coriolis_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        memcpy(jacobianMatrix, robot_state->jacobian_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
        float(*result)[3] = Drfl_.get_current_rotm();

        /*new0317
        LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
        float gripper_torque[NUMBER_OF_JOINT] = {0,};

        LPROBOT_FORCE lpForce = Drfl_.get_external_torque();
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_ext2(lpForce->_fForce);

        Eigen::Map<Eigen::Matrix<float, 6, 6>> J(reinterpret_cast<float*>(jacobianMatrix));
        Eigen::Matrix<float, 6, 6> J_Tinv = J.transpose().inverse();

        for(int i=0; i<6; i++){
            actual_position[i] = res->_fPosition[i];
        }*/
        //new0317
        float gripper_torque[NUMBER_OF_JOINT] = {0,};

        LPROBOT_FORCE lpForce = Drfl_.get_external_torque();
        Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_ext2(lpForce->_fForce);

        if (is_dbic_mode) {
            actual_position[0] = s_task.p(0) * 1000.0f;
            actual_position[1] = s_task.p(1) * 1000.0f;
            actual_position[2] = s_task.p(2) * 1000.0f;

            auto act_rpy = quatToEulerDegZYX(s_task.q);
            actual_position[3] = act_rpy[0];
            actual_position[4] = act_rpy[1];
            actual_position[5] = act_rpy[2];

            actual_velocity[0] = s_task.v(0) * 1000.0f;
            actual_velocity[1] = s_task.v(1) * 1000.0f;
            actual_velocity[2] = s_task.v(2) * 1000.0f;
            actual_velocity[3] = s_task.w(0) * RAD2DEG;
            actual_velocity[4] = s_task.w(1) * RAD2DEG;
            actual_velocity[5] = s_task.w(2) * RAD2DEG;

            for (int i = 0; i < 6; ++i) {
                F_external_box[i] = s_task.F_env_on_robot(i);
                actual_position2[i] = actual_position[i];
            }

            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    jacobianMatrix[i][j] = s_task.J(i, j);
                }
            }
        } else {
            LPROBOT_POSE res = Drfl_.fkin(actual_positionj, COORDINATE_SYSTEM_WORLD);
            for (int i = 0; i < 6; ++i) {
                actual_position[i] = res->_fPosition[i];
            }
        }
        //
        //new0324
        pose6ToQuatAssumingZYZ(raw_actual_flange_position, actual_flange_quat_assuming_zyz);
        pose6ToQuatAssumingXYZ(raw_actual_flange_position, actual_flange_quat_assuming_xyz);

        pose6ToQuatAssumingZYZ2(raw_actual_flange_position, actual_flange_quat_assuming_zyz2);
        pose6ToQuatAssumingZYX(raw_actual_flange_position, actual_flange_quat_assuming_zyx);
        //
        convertToArray(trajectory.pos_d, traj_position);
        convertToArray(trajectory.vel_d, traj_velocity);
        convertToArray(trajectory.acc_d, traj_acceleration);

        float traj_position_6d[6] = {0,};
        traj_position_6d[0] = traj_position[0]; 
        traj_position_6d[1] = traj_position[1]; 
        traj_position_6d[2] = traj_position[2]; 

        Eigen::Quaternionf q_traj_log(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
        auto traj_rpy = quatToEulerDegZYX(q_traj_log);

        traj_position_6d[3] = traj_rpy[0];
        traj_position_6d[4] = traj_rpy[1];
        traj_position_6d[5] = traj_rpy[2];

        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                rotationMatrix[i][j] = result[i][j];
            }
        }
        // //new0317
        // if (is_dbic_mode) {
        //     Eigen::Matrix3f R_task = s_task.q.toRotationMatrix();
        //     for (int i = 0; i < 3; ++i) {
        //         for (int j = 0; j < 3; ++j) {
        //             rotationMatrix[i][j] = R_task(i, j);
        //         }
        //     }
        // } else {
        //     for (int i = 0; i < 3; ++i) {
        //         for (int j = 0; j < 3; ++j) {
        //             rotationMatrix[i][j] = result[i][j];
        //         }
        //     }
        // }

        const bool is_pbic_mode =
            (control_mode_ == "PBIC goal mode" ||
             control_mode_ == "PBIC path mode");

        for (int i = 0; i < 6; ++i) {
            if (is_pbic_mode) {
                impedance_position[i] = imp.pos_m(i);
                position_command[i] = desired.q_d[i];
                joint_error[i] = position_command[i] - actual_positionj[i];
                position_error[i] = impedance_position[i] - actual_position[i];
            } else {
                // DBIC에서는 desired joint / impedance model이 없으므로
                // desired task trajectory를 impedance_position log에 넣는다.
                impedance_position[i] = traj_position_6d[i];
                position_command[i] = 0.0f;
                joint_error[i] = 0.0f;
                position_error[i] = traj_position_6d[i] - actual_position[i];
            }

            F_external[i] = F.Fext[i];
            F_impedance[i] = F.Fimp[i];
            F_task_log[i] = F.F_task[i];      // 추가
            F_DBIC[i] = F.F_DBIC[i];
            F_coriolis[i] = F.F_coriolis[i];
            F_rest[i] = F.F_rest[i];

            gripper_torque[i] = trq_gg[i];
            trq_ext_auto[i] = trq_ext2[i];
            trq_ext_cal[i] = trq_raw[i] - trq_g[i];
            sensor_FT[i] = sensor_data.AFT_wrench_[i];
            sensor_FT_matched[i] = sensor_data.AFT_wrench_matched[i];

            error[i] = F.error[i];
            error_dot[i] = F.error_dot[i];
        }
        for (int i = 0; i < 3; ++i) {
            ref_v_d_log[i] = g_ref_v_d_log[i].load(std::memory_order_relaxed);
            s_v_log[i] = g_s_v_log[i].load(std::memory_order_relaxed);
            ref_w_d_log[i] = g_ref_w_d_log[i].load(std::memory_order_relaxed);
            s_w_log[i] = g_s_w_log[i].load(std::memory_order_relaxed);
        }

        time[0] += dt;//     

        //new0324
        logData("ref_v_d.txt", ref_v_d_log, 3);
        logData("s_v.txt", s_v_log, 3);
        logData("ref_w_d.txt", ref_w_d_log, 3);
        logData("s_w.txt", s_w_log, 3);

        logData("error.txt", error, NUMBER_OF_TASK);
        logData("error_dot.txt", error_dot, NUM_TASK);             
        logData("actual_flange_position.txt", actual_flange_position, NUMBER_OF_JOINT);
        logData("actual_tcp_position.txt", actual_tcp_position, NUM_TASK);                
        logData("actual_motor_torque.txt", actual_motor_torque, NUMBER_OF_JOINT);
        logData("target_motor_torque.txt", target_motor_torque, NUMBER_OF_JOINT);
        logData("actual_flange_quaternion_assuming_ZYZ.txt", actual_flange_quat_assuming_zyz, 4);
        logData("actual_flange_quaternion_assuming_XYZ.txt", actual_flange_quat_assuming_xyz, 4);
        logData("actual_flange_quaternion_assuming_ZYZ2.txt", actual_flange_quat_assuming_zyz2, 4);
        logData("actual_flange_quaternion_assuming_ZYX.txt", actual_flange_quat_assuming_zyx, 4);
        logData("Raw_external_force.txt", Raw_external_force, NUMBER_OF_TASK);
        //
        logData("filtered_acceleration.txt", filtered_accelerationj, NUMBER_OF_JOINT);
        logData("time.txt",time,1);
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
        logData("force_external.txt",F_external, NUMBER_OF_JOINT);
        logData("force_task.txt",F_task_log, NUMBER_OF_JOINT);   // 추가
        logData("force_dbic.txt",F_DBIC, NUMBER_OF_JOINT);
        logData("force_rest.txt",F_rest, NUMBER_OF_JOINT);
        logData("force_coriolis.txt",F_coriolis, NUMBER_OF_JOINT);
        logData("actual_joint_torque.txt",trq_act, NUMBER_OF_JOINT);
        logData("force_external_box.txt",F_external_box, NUMBER_OF_JOINT);
        logData("force_impedance.txt",F_impedance, NUMBER_OF_JOINT);
        logData("joint_error.txt",joint_error, NUMBER_OF_JOINT);
        logData("task_position_error.txt",position_error, NUMBER_OF_JOINT);
        logData("external_joint_torque.txt",trq_ext, NUMBER_OF_JOINT);
        logData("external_joint_torque_auto.txt",trq_ext_auto, NUMBER_OF_JOINT);
        logData("external_joint_torque_cal.txt",trq_ext_cal, NUMBER_OF_JOINT);
        logData("raw_force_torque.txt",trq_force, NUMBER_OF_JOINT);
        logData("gripper_torque.txt",gripper_torque, NUMBER_OF_JOINT);
        logData("sensor_FT.txt",sensor_FT, NUMBER_OF_JOINT);
        logData("sensor_FT_matched.txt",sensor_FT_matched, NUMBER_OF_JOINT);
        logData("operator_count.txt", operator_count, 1);
        logMatrixData("mass_matrix.txt", massMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
        logMatrixData("coriolis_matrix.txt", coriolisMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
        logMatrixData("jacobian_matrix.txt", jacobianMatrix, NUMBER_OF_JOINT, NUMBER_OF_JOINT);
        logMatrixData3x3("rotation_matrix.txt", rotationMatrix, 3, 3);

        /*new0317
        float(*rotm_ptr)[3] = Drfl_.get_current_rotm();
        Eigen::Matrix3f R_act;
        R_act << rotm_ptr[0][0], rotm_ptr[0][1], rotm_ptr[0][2],
                    rotm_ptr[1][0], rotm_ptr[1][1], rotm_ptr[1][2],
                    rotm_ptr[2][0], rotm_ptr[2][1], rotm_ptr[2][2];
        Eigen::Quaternionf q_act(R_act);
        q_act.normalize();
        */
        //new0317
        Eigen::Quaternionf q_act;
        if (is_dbic_mode) {
            q_act = s_task.q;
        } else {
            float(*rotm_ptr)[3] = Drfl_.get_current_rotm();
            Eigen::Matrix3f R_act;
            R_act << rotm_ptr[0][0], rotm_ptr[0][1], rotm_ptr[0][2],
                     rotm_ptr[1][0], rotm_ptr[1][1], rotm_ptr[1][2],
                     rotm_ptr[2][0], rotm_ptr[2][1], rotm_ptr[2][2];
            q_act = Eigen::Quaternionf(R_act);
            q_act.normalize();
        }//

        Eigen::Quaternionf q_des(traj_position[6], traj_position[3], traj_position[4], traj_position[5]);
        q_des.normalize(); 

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
        Eigen::Vector3f e_rot = q_act * q_err.vec();

        actual_quat[0] = q_act.x(); actual_quat[1] = q_act.y(); actual_quat[2] = q_act.z(); actual_quat[3] = q_act.w();
        traj_quat[0] = q_des.x();   traj_quat[1] = q_des.y();   traj_quat[2] = q_des.z();   traj_quat[3] = q_des.w();
        orientation_error[0] = e_rot.x(); orientation_error[1] = e_rot.y(); orientation_error[2] = e_rot.z();

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

void ControlLoop::GainMove() {
    float step = 4;
    float tTime = 5;
    float tvel[2] = { 70, 70 };
    float tacc[2] = { 120, 120 };

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
        {300, -400, 600, 0, -180, 3.42},  // X1
        {900, -400, 600, 0, -180, 3.42},  // X2
        {900, 400, 600, 0, -180, 3.42},   // X3
        {300, 400, 600, 0, -180, 3.42},   // X4
        (600 - 360) / step,               // zdp
        (900 - 300) / (2 * step),         // xdp
        (400  + 400) / (2 * step)         // ydp
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
                Drfl_.movel(X1, tvel, tacc);
                Drfl_.movel(X2, tvel, tacc);
                Drfl_.movel(X3, tvel, tacc);
                Drfl_.movel(X4, tvel, tacc);
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
    return;
}

void ControlLoop::returnToHome() {
    Drfl_.movej(home_abs, 60, 30);
    LPRT_OUTPUT_DATA_LIST robot_state = Drfl_.read_data_rt();
    memcpy(home_abs,robot_state->actual_flange_position,NUMBER_OF_JOINT*sizeof(float));
    Drfl_.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);
    Drfl_.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
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

    dataDirectory = "/home/rbl/catkin_ws/data/" + datePart +"_data" +"/" + timePart;
    if (!fs::create_directories(dataDirectory)) {
        std::cerr << "Failed to create directory: " << dataDirectory << std::endl;
    } else {
        std::cout << "Directory created: " << dataDirectory << std::endl;
    }
}

} // end of SKKU namespace