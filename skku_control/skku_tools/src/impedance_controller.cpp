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
    //new0330
    inline Eigen::Quaternionf quatFromEulerZYZDeg(float z1_deg, float y_deg, float z2_deg) {
        Eigen::AngleAxisf z1Angle(z1_deg * DEG2RAD, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf yAngle (y_deg  * DEG2RAD, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf z2Angle(z2_deg * DEG2RAD, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q = z1Angle * yAngle * z2Angle;
        return normalizeQuat(q);
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
//new0412
    constexpr float RAD2DEG = 180.0f / static_cast<float>(M_PI);

    inline Eigen::Quaternionf quatFromPose7(const std::array<float, 7>& pose7) {
        return normalizeQuat(Eigen::Quaternionf(
            pose7[6], pose7[3], pose7[4], pose7[5]));
    }

    inline float unwrapNearDeg(float angle_deg, float ref_deg) {
        while (angle_deg - ref_deg > 180.0f) angle_deg -= 360.0f;
        while (angle_deg - ref_deg < -180.0f) angle_deg += 360.0f;
        return angle_deg;
    }

    inline std::array<float, 3> quatToEulerZYZDeg(const Eigen::Quaternionf& q_in) {
        Eigen::Quaternionf q = normalizeQuat(q_in);
        Eigen::Vector3f zyz = q.toRotationMatrix().eulerAngles(2, 1, 2);

        return {
            zyz[0] * RAD2DEG,
            zyz[1] * RAD2DEG,
            zyz[2] * RAD2DEG
        };
    }

    inline std::array<float, 3> quatToEulerZYZDegNear(
        const Eigen::Quaternionf& q_in,
        float ref_z1_deg,
        float ref_y_deg,
        float ref_z2_deg) {

        auto zyz = quatToEulerZYZDeg(q_in);

        zyz[0] = unwrapNearDeg(zyz[0], ref_z1_deg);
        zyz[1] = unwrapNearDeg(zyz[1], ref_y_deg);
        zyz[2] = unwrapNearDeg(zyz[2], ref_z2_deg);

        float y_abs = std::fmod(std::fabs(zyz[1]), 360.0f);
        if (y_abs > 180.0f) y_abs = 360.0f - y_abs;

        // ZYZ singularity 근처에서는 Z 분해를 이전 값 근처로 유지
        if (y_abs < 1.0e-4f || std::fabs(y_abs - 180.0f) < 1.0e-4f) {
            zyz[0] = ref_z1_deg;
            zyz[2] = ref_z2_deg;
        }

        return zyz;
    }

    inline Eigen::Quaternionf quatDerivativeWorld(const Eigen::Quaternionf& q,
                                                const Eigen::Vector3f& w_world) {
        Eigen::Quaternionf omega_q(0.0f, w_world.x(), w_world.y(), w_world.z());
        Eigen::Quaternionf dq = omega_q * q;
        dq.coeffs() *= 0.5f;
        return dq;
    }

    inline Eigen::Quaternionf quatFromCoeffVec(const Eigen::Vector4f& coeffs_xyzw) {
        return normalizeQuat(Eigen::Quaternionf(
            coeffs_xyzw[3], coeffs_xyzw[0], coeffs_xyzw[1], coeffs_xyzw[2]));
    }

    struct PoseQuatState {
        Eigen::Vector3f p = Eigen::Vector3f::Zero();
        Eigen::Vector3f v = Eigen::Vector3f::Zero();
        Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
        Eigen::Vector3f w = Eigen::Vector3f::Zero();
    };

    struct PoseQuatDeriv {
        Eigen::Vector3f dp = Eigen::Vector3f::Zero();
        Eigen::Vector3f dv = Eigen::Vector3f::Zero();
        Eigen::Vector4f dq = Eigen::Vector4f::Zero();  // [x,y,z,w]
        Eigen::Vector3f dw = Eigen::Vector3f::Zero();
    };

    inline PoseQuatState addState(const PoseQuatState& s,
                                const PoseQuatDeriv& k,
                                float h) {
        PoseQuatState out = s;
        out.p = s.p + h * k.dp;
        out.v = s.v + h * k.dv;
        out.w = s.w + h * k.dw;
        out.q = quatFromCoeffVec(s.q.coeffs() + h * k.dq);
        return out;
    }

    inline PoseQuatDeriv evalPoseQuatDeriv(
        const PoseQuatState& s,
        const Eigen::Vector3f& p_d,
        const Eigen::Vector3f& v_d,
        const Eigen::Vector3f& a_d,
        const Eigen::Quaternionf& q_d_in,
        const Eigen::Vector3f& w_d,
        const Eigen::Vector3f& alpha_d,
        const Eigen::Vector3f& Fext_lin,
        const Eigen::Vector3f& Text_rot,
        const Eigen::Vector3f& Mlin_inv,
        const Eigen::Vector3f& Blin,
        const Eigen::Vector3f& Klin,
        const Eigen::Vector3f& Mrot_inv,
        const Eigen::Vector3f& Brot,
        const Eigen::Vector3f& Krot) {

        PoseQuatDeriv k;
        k.dp = s.v;

        k.dv = a_d
            + Mlin_inv.cwiseProduct(
                Blin.cwiseProduct(v_d - s.v)
                + Klin.cwiseProduct(p_d - s.p)
                - Fext_lin);

        Eigen::Quaternionf q_d = q_d_in;
        alignQuatHemisphere(q_d, s.q);
        const Eigen::Vector3f e_R = quatLogError(q_d, s.q);

        k.dw = alpha_d
            + Mrot_inv.cwiseProduct(
                Brot.cwiseProduct(w_d - s.w)
                + Krot.cwiseProduct(e_R)
                - Text_rot);

        k.dq = quatDerivativeWorld(s.q, s.w).coeffs();
        return k;
    }
    inline void syncImpedanceLegacyMirror(SKKU::Impedance& imp,
                                        float ref_z1_deg,
                                        float ref_y_deg,
                                        float ref_z2_deg) {
        auto zyz = quatToEulerZYZDegNear(imp.q_m, ref_z1_deg, ref_y_deg, ref_z2_deg);

        imp.pos_m.setZero();
        imp.vel_m.setZero();
        imp.acc_m.setZero();

        imp.pos_m(0) = imp.p_m(0);
        imp.pos_m(1) = imp.p_m(1);
        imp.pos_m(2) = imp.p_m(2);
        imp.pos_m(3) = zyz[0];
        imp.pos_m(4) = zyz[1];
        imp.pos_m(5) = zyz[2];

        imp.vel_m(0) = imp.v_m(0);
        imp.vel_m(1) = imp.v_m(1);
        imp.vel_m(2) = imp.v_m(2);

        // 주의: tail은 ZYZ Euler rate가 아니라 angular-velocity component를 deg/s로 기록
        imp.vel_m(3) = imp.w_m(0) * RAD2DEG;
        imp.vel_m(4) = imp.w_m(1) * RAD2DEG;
        imp.vel_m(5) = imp.w_m(2) * RAD2DEG;

        imp.acc_m(0) = imp.a_m(0);
        imp.acc_m(1) = imp.a_m(1);
        imp.acc_m(2) = imp.a_m(2);

        imp.acc_m(3) = imp.alpha_m(0) * RAD2DEG;
        imp.acc_m(4) = imp.alpha_m(1) * RAD2DEG;
        imp.acc_m(5) = imp.alpha_m(2) * RAD2DEG;
    }

    inline void rungeKuttaPoseQuaternion(
        SKKU::Impedance& imp,
        const Eigen::Vector3f& p_d,
        const Eigen::Vector3f& v_d,
        const Eigen::Vector3f& a_d,
        const Eigen::Quaternionf& q_d_in,
        const Eigen::Vector3f& w_d,
        const Eigen::Vector3f& alpha_d,
        const Eigen::Matrix<float, 6, 1>& F_ext,
        const Eigen::Matrix<float, 6, 6>& M,
        const Eigen::Matrix<float, 6, 6>& B,
        const Eigen::Matrix<float, 6, 6>& K,
        const Eigen::Matrix<float, 6, 6>& M_inv,
        float dt,
        int n) {

        PoseQuatState s;
        s.p = imp.p_m;
        s.v = imp.v_m;
        s.q = imp.q_m;
        s.w = imp.w_m;

        const Eigen::Vector3f Fext_lin = -1.0f*F_ext.head<3>();
        const Eigen::Vector3f Text_rot = -1.0f*F_ext.tail<3>();

        const Eigen::Vector3f Mlin_inv(
            M_inv(0, 0), M_inv(1, 1), M_inv(2, 2));
        const Eigen::Vector3f Blin(
            B(0, 0), B(1, 1), B(2, 2));
        const Eigen::Vector3f Klin(
            K(0, 0), K(1, 1), K(2, 2));

        // 기존 rotational gains는 degree 기반으로 튜닝돼 있으므로
        // quaternion / angular velocity(rad)로 계산할 때 rad 기준으로 환산
        const Eigen::Vector3f Mrot_inv(
            M_inv(3, 3),
            M_inv(4, 4),
            M_inv(5, 5));
        const Eigen::Vector3f Brot(
            B(3, 3),
            B(4, 4),
            B(5, 5));
        const Eigen::Vector3f Krot(
            K(3, 3),
            K(4, 4),
            K(5, 5));

        const float h = dt / static_cast<float>(n);

        for (int i = 0; i < n; ++i) {
            const PoseQuatDeriv k1 = evalPoseQuatDeriv(
                s, p_d, v_d, a_d, q_d_in, w_d, alpha_d,
                Fext_lin, Text_rot, Mlin_inv, Blin, Klin, Mrot_inv, Brot, Krot);

            const PoseQuatDeriv k2 = evalPoseQuatDeriv(
                addState(s, k1, 0.5f * h),
                p_d, v_d, a_d, q_d_in, w_d, alpha_d,
                Fext_lin, Text_rot, Mlin_inv, Blin, Klin, Mrot_inv, Brot, Krot);

            const PoseQuatDeriv k3 = evalPoseQuatDeriv(
                addState(s, k2, 0.5f * h),
                p_d, v_d, a_d, q_d_in, w_d, alpha_d,
                Fext_lin, Text_rot, Mlin_inv, Blin, Klin, Mrot_inv, Brot, Krot);

            const PoseQuatDeriv k4 = evalPoseQuatDeriv(
                addState(s, k3, h),
                p_d, v_d, a_d, q_d_in, w_d, alpha_d,
                Fext_lin, Text_rot, Mlin_inv, Blin, Klin, Mrot_inv, Brot, Krot);

            s.p += (h / 6.0f) * (k1.dp + 2.0f * k2.dp + 2.0f * k3.dp + k4.dp);
            s.v += (h / 6.0f) * (k1.dv + 2.0f * k2.dv + 2.0f * k3.dv + k4.dv);
            s.w += (h / 6.0f) * (k1.dw + 2.0f * k2.dw + 2.0f * k3.dw + k4.dw);
            s.q  = quatFromCoeffVec(
                s.q.coeffs() + (h / 6.0f) * (k1.dq + 2.0f * k2.dq + 2.0f * k3.dq + k4.dq));
        }

        const PoseQuatDeriv kf = evalPoseQuatDeriv(
            s, p_d, v_d, a_d, q_d_in, w_d, alpha_d,
            Fext_lin, Text_rot, Mlin_inv, Blin, Klin, Mrot_inv, Brot, Krot);

        imp.p_m = s.p;
        imp.v_m = s.v;
        imp.a_m = kf.dv;

        imp.q_m = s.q;
        imp.w_m = s.w;
        imp.alpha_m = kf.dw;
    }    
}//


namespace SKKU
{
    namespace fs = boost::filesystem;
    // new0330 Shared debug logs consumed by ControlLoop::dataSaving()
    extern std::atomic<float> g_ref_task_pose_log[6];
    extern std::atomic<float> g_s_task_pose_log[6];
    extern std::atomic<float> g_ref_task_vel_log[6];
    extern std::atomic<float> g_s_task_vel_log[6];
    extern std::atomic<float> g_ref_task_acc_log[6];
    extern std::atomic<float> g_s_task_acc_log[6]; 
    extern std::atomic<float> g_imp_task_pose_log[6];
    extern std::atomic<float> g_imp_task_vel_log[6];
    extern std::atomic<float> g_imp_task_acc_log[6]; 
    extern std::atomic<float> g_qdot_des_log[6];
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
        // K_gains = {3*imp_k, 3*imp_k, imp_k, imp_k, imp_k, imp_k};
        K_gains = {1.0f*imp_k, 1.0f*imp_k, 1.0f*imp_k, 75.0f * imp_k, 75.0f * imp_k, 75.0f * imp_k};
        for (int i = 0; i < 6; ++i)
        {
            // B_gains[i] = 2 * sqrt(K_gains[i] * M_gains[i]); // 2 critical dmaped
            B_gains[i] = 1.4 * sqrt(K_gains[i] * M_gains[i]); // 4 Overdmaped
            
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


        // // ------------------------------------------------------------
        // // DBIC tuning for current z-lift free-space test
        // // z translation authority를 키우고, rotation coupling은 약하게 둔다.
        // // ------------------------------------------------------------
        // Md_.setZero();
        // Bd_.setZero();
        // Kd_.setZero();


        // // 행렬에 들어가는 6개의 숫자들은 차례대로 [X, Y, Z, Roll, Pitch, Yaw] 축을 의미
        // Md_.diagonal() << 5.0f, 5.0f, 2.0f, 0.20f, 0.20f, 0.20f;
        // Kd_.diagonal() << 200.0f, 200.0f, 200.0f, 20.0f, 20.0f, 20.0f;

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
        // Kd_.diagonal() << 10000.0f, 8000.0f, 8000.0f, 200.0f, 200.0f, 200.0f;
        // Bd_.diagonal() << 1000.0f, 1000.0f, 1000.0f, 40.0f, 40.0f, 40.0f;
        Md_.setZero();
        Bd_.setZero();
        Kd_.setZero();

        for (int i = 0; i < 3; ++i) {
            Md_(i, i) = M_gains[i];  // mm 기준
            Bd_(i, i) = B_gains[i];
            Kd_(i, i) = K_gains[i];
        }

        for (int i = 3; i < 6; ++i) {
            Md_(i, i) = M_gains[i];  // rad 기준
            Bd_(i, i) = B_gains[i];
            Kd_(i, i) = K_gains[i];
        }

        Md_inv_ = Md_.inverse();

        dt = static_cast<float>(loop_time) / 1000.0f;
    }

    //로그용 게인값
    void PBIC::getGainLogValues(float M_log[6],
                                float B_log[6],
                                float K_log[6],
                                float K1_log[6],
                                float K2_log[6],
                                float M_hat_log[6]) const
    {
        for (int i = 0; i < 6; ++i) {
            M_log[i] = M(i, i);
            B_log[i] = B(i, i);
            K_log[i] = K(i, i);

            K1_log[i] = K1[i];
            K2_log[i] = K2[i];
            M_hat_log[i] = M_hat[i];
        }
    }


    void PBIC::start_Motion(LPRT_OUTPUT_DATA_LIST &robot_state, Prev &prev, Impedance &imp)
    {
        robot_state = Drfl_.read_data_rt();

        std::copy(robot_state->actual_flange_position,
                robot_state->actual_flange_position + 6,
                begin(prev.xPrev));

        std::fill(prev.vPrev.begin(), prev.vPrev.end(), 0.0f);
        std::fill(prev.F_extPrev.begin(), prev.F_extPrev.end(), 0.0f);

        imp.p_m << robot_state->actual_flange_position[0],
                robot_state->actual_flange_position[1],
                robot_state->actual_flange_position[2];

        imp.v_m.setZero();
        imp.a_m.setZero();

        imp.q_m = quatFromEulerZYZDeg(robot_state->actual_flange_position[3],
                                    robot_state->actual_flange_position[4],
                                    robot_state->actual_flange_position[5]);

        imp.w_m.setZero();
        imp.alpha_m.setZero();

        syncImpedanceLegacyMirror(imp,
                                robot_state->actual_flange_position[3],
                                robot_state->actual_flange_position[4],
                                robot_state->actual_flange_position[5]);
    }
    //

    // void PBIC::resetDBICControllerState()
    // {
    //     has_prev_J_dbic_ = false;
    //     J_prev_dbic_.setZero();
    //     Jdot_qdot_prev_.setZero();
    //     Fe_filt_dbic_.setZero();
    //     // Fe_filt.setZero(); // <--- Fe_filt 로 수정!
    // }
    //new0401
    static Eigen::Quaternionf g_qF_prev = Eigen::Quaternionf::Identity();
    static bool g_is_qF_init = false;
    //
    void PBIC::resetDBICControllerState()
    {
        has_prev_J_dbic_ = false;
        J_prev_dbic_.setZero();
        Jdot_qdot_prev_.setZero();
        has_prev_task_vel_dbic_ = false;
        prev_task_v_dbic_.setZero();
        prev_task_w_dbic_.setZero();
        has_prev_edot_dbic_ = false;
        prev_edot_dbic_.setZero();
        has_xdd_actual_lpf_dbic_ = false;
        xdd_actual_filt_dbic_.setZero();
        Fe_filt_dbic_.setZero();
        Fe_bias_dbic_.setZero();
        Fe_bias_accum_dbic_.setZero();
        Fe_bias_count_dbic_ = 0;
        fe_bias_ready_dbic_ = false;
        has_prev_qdot_dbic_ = false;
        prev_qdot_dbic_.setZero();
        qddot_filt_dbic_.setZero();

        tau_prev_dbic_.setZero();
        //new0401
        g_is_qF_init = false;
        g_qF_prev = Eigen::Quaternionf::Identity();
    }

    //new0317
    TaskState PBIC::getTaskState(const LPRT_OUTPUT_DATA_LIST robot_state,
                             TaskPointMode task_point_mode,
                             const Eigen::Isometry3f& T_flange_tcp) {
    TaskState s;

    Eigen::Vector3f pF;
    pF << robot_state->actual_flange_position[0] * 1e-3f,
          robot_state->actual_flange_position[1] * 1e-3f,
          robot_state->actual_flange_position[2] * 1e-3f;
    //new0330      

    // float (*rotm_ptr)[3] = Drfl_.get_current_rotm();
    // Eigen::Quaternionf qF = quatFromRotm(rotm_ptr);

    Eigen::Isometry3f T_B_F = Eigen::Isometry3f::Identity();
    //new0330
    Eigen::Quaternionf qF = quatFromEulerZYZDeg(
        robot_state->actual_flange_position[3],
        robot_state->actual_flange_position[4],
        robot_state->actual_flange_position[5]
    );
    //new0401
    if (!g_is_qF_init) {
        g_qF_prev = qF;
        g_is_qF_init = true;
    } else {
        alignQuatHemisphere(qF, g_qF_prev);
        g_qF_prev = qF; // 다음 프레임을 위해 저장
    }

    float (*rotm_ptr)[3] = Drfl_.get_current_rotm();
    Eigen::Matrix3f R_B_F;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_B_F(i, j) = rotm_ptr[i][j];
        }
    }

    sensor_data.matchAFTWrench(R_B_F);

    T_B_F.translation() = pF;

    Eigen::Isometry3f T_B_TCP = T_B_F * T_flange_tcp;
    Eigen::Vector3f r_F_to_TCP = T_B_TCP.translation() - T_B_F.translation();

    Eigen::Vector3f vF;
    vF << robot_state->actual_flange_velocity[0] * 1e-3f,
          robot_state->actual_flange_velocity[1] * 1e-3f,
          robot_state->actual_flange_velocity[2] * 1e-3f;

    Eigen::Vector3f wF;
    wF << robot_state->actual_flange_velocity[3] * DEG2RAD,
          robot_state->actual_flange_velocity[4] * DEG2RAD,
          robot_state->actual_flange_velocity[5] * DEG2RAD;

    if (task_point_mode == TaskPointMode::kTCP) {
        s.p = T_B_TCP.translation();
        // s.q = Eigen::Quaternionf(T_B_TCP.linear());
        s.q = qF;
        s.q.normalize();
        s.v = vF + wF.cross(r_F_to_TCP);
        s.w = wF;
    } else {
        s.p = T_B_F.translation();
        s.q = qF;
        s.q.normalize();
        s.v = vF;
        s.w = wF;
    }
    Eigen::Matrix<float, 6, 6> J_raw = mapMat6(robot_state->jacobian_matrix);

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

    // // external_tcp_force는 TCP point, base/world frame, environment-on-robot 가정
    // Eigen::Matrix<float, 6, 1> wrench_tcp;

    if (kRawJacobianIsFlange) {
        if (task_point_mode == TaskPointMode::kTCP) {
            s.J = twistShiftMatrix(r_F_to_TCP) * J_raw;
        } else {
            s.J = J_raw;
        }
    } else {
        if (task_point_mode == TaskPointMode::kTCP) {
            s.J = J_raw;
        } else {
            s.J = twistShiftMatrix(-r_F_to_TCP) * J_raw;
        }
    }

    // ------------------------------------------------------------
    // DBIC는 SI 단위(m, rad, m/s, rad/s)로 계산한다.
    // actual_joint_velocity는 raw joint unit이므로 rad/s로 변환해서 사용
    // ------------------------------------------------------------
        Eigen::Matrix<float, 6, 1> qdot_task =
        mapJointVelDegToRad(robot_state->actual_joint_velocity);

    Eigen::Matrix<float, 6, 1> twist_task = s.J * qdot_task;

    s.v = twist_task.head<3>();   // [m/s]
    s.w = twist_task.tail<3>();   // [rad/s]

    if (has_prev_task_vel_dbic_) {
        s.a = (s.v - prev_task_v_dbic_) / dt;
        s.alpha = (s.w - prev_task_w_dbic_) / dt;
    } else {
        s.a.setZero();
        s.alpha.setZero();
        has_prev_task_vel_dbic_ = true;
    }

    prev_task_v_dbic_ = s.v;
    prev_task_w_dbic_ = s.w;

    // external_tcp_force는 TCP point, base/world frame, environment-on-robot 가정
    Eigen::Matrix<float, 6, 1> wrench_tcp;

    for (int i = 0; i < 6; ++i) {
        wrench_tcp(i) = robot_state->external_tcp_force[i];
    }

    if (task_point_mode == TaskPointMode::kTCP) {
        s.F_env_on_robot = wrench_tcp;
    } else {
        s.F_env_on_robot.head<3>() = wrench_tcp.head<3>();
        s.F_env_on_robot.tail<3>() =
            wrench_tcp.tail<3>() + r_F_to_TCP.cross(wrench_tcp.head<3>());
    }

    return s;
    }
    
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
        //qdot 코드
        Eigen::Matrix<float, 6, 1> qddot =
            Eigen::Matrix<float, 6, 1>::Zero();

        if (has_prev_qdot_dbic_) {
            Eigen::Matrix<float, 6, 1> qddot_raw =
                (qdot - prev_qdot_dbic_) / dt;

            const float alpha_qddot = 0.2f;
            qddot_filt_dbic_ =
                alpha_qddot * qddot_raw +
                (1.0f - alpha_qddot) * qddot_filt_dbic_;

            qddot = qddot_filt_dbic_;
        } else {
            has_prev_qdot_dbic_ = true;
        }

        prev_qdot_dbic_ = qdot;
        // ------------------------------------------------------------
        // Force measurement conditioning
        // paper에서는 calibrated force sensor를 사용하지만,
        // 현재 시스템은 raw external_tcp_force를 바로 쓰므로
        // bias 제거 + LPF를 적용해서 free-space jitter를 줄인다.
        // ------------------------------------------------------------
        Eigen::Matrix<float, 6, 1> Fe_meas = -s.F_env_on_robot;
        Eigen::Matrix<float, 6, 1> Fe_paper = Eigen::Matrix<float, 6, 1>::Zero();

        // if (!fe_bias_ready_dbic_) {
        //     Fe_bias_accum_dbic_ += Fe_meas;
        //     Fe_bias_count_dbic_++;

        //     if (Fe_bias_count_dbic_ >= 50) {   // 약 0.2초 @ 250Hz
        //         Fe_bias_dbic_ = Fe_bias_accum_dbic_ / static_cast<float>(Fe_bias_count_dbic_);
        //         fe_bias_ready_dbic_ = true;
        //         Fe_filt_dbic_.setZero();
        //     }
        // } else {
        //     Fe_meas -= Fe_bias_dbic_;
        //     Fe_filt_dbic_ = 0.05f * Fe_meas + 0.95f * Fe_filt_dbic_;
        //     Fe_paper = Fe_filt_dbic_;

            // 외력 = 0 experiment setting
            // Fe_paper = Eigen::Matrix<float, 6, 1>::Zero();
        

        Eigen::Matrix<float, 6, 1> e    = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> edot = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> edot_raw = Eigen::Matrix<float, 6, 1>::Zero();        

        e.head<3>()    = (ref.p_d - s.p) * 1000.0f;
        edot_raw.head<3>() = (ref.v_d - s.v) * 1000.0f;

        e.tail<3>()    = quatLogError(ref.q_d, s.q);
        edot_raw.tail<3>() = ref.w_d - s.w;
        // new0330 Debug logs for edot decomposition

        // ------------------------------------------------------------
        // edot filtering
        //   1) slew-rate limit
        //   2) deadband
        //   3) low-pass filter
        // ------------------------------------------------------------
        static bool edot_filter_init = false;
        static Eigen::Matrix<float, 6, 1> edot_prev_limited = Eigen::Matrix<float, 6, 1>::Zero();
        static Eigen::Matrix<float, 6, 1> edot_filt_state   = Eigen::Matrix<float, 6, 1>::Zero();

        Eigen::Matrix<float, 6, 1> edot_limited = edot_raw;

        // 작은 떨림 제거용 deadband
        // Eigen::Matrix<float, 6, 1> edot_deadband;
        // edot_deadband << 0.0015f, 0.0015f, 0.0015f,
        //                 0.0100f, 0.0100f, 0.0100f;
        Eigen::Matrix<float, 6, 1> edot_deadband;
        edot_deadband << 0.00f, 0.00f, 0.00f,
                        0.000f, 0.000f, 0.000f;


        // 축별 LPF 계수 (작을수록 더 부드러움)
        Eigen::Matrix<float, 6, 1> edot_alpha;
        edot_alpha << 0.2f, 0.2f, 0.2f,
                    1.0f, 1.0f, 1.0f;

        // edot 변화율 제한 (단위: linear = mm/s^2, angular = rad/s^2)
        // const float linear_edot_slew_rate  = 5.0f;
        // const float angular_edot_slew_rate = 20.0f;
        const float linear_edot_slew_rate  = 100000000.0f;
        const float angular_edot_slew_rate = 20000000.0f;

        Eigen::Matrix<float, 6, 1> edot_delta_limit;
        edot_delta_limit << linear_edot_slew_rate  * dt,
                            linear_edot_slew_rate  * dt,
                            linear_edot_slew_rate  * dt,
                            angular_edot_slew_rate * dt,
                            angular_edot_slew_rate * dt,
                            angular_edot_slew_rate * dt;

        if (!edot_filter_init) {
            edot_prev_limited = edot_raw;
            edot_filt_state   = edot_raw;
            edot_filter_init  = true;
        }

        for (int i = 0; i < 6; ++i) {
            float delta = edot_raw(i) - edot_prev_limited(i);

            if (delta >  edot_delta_limit(i)) delta =  edot_delta_limit(i);
            if (delta < -edot_delta_limit(i)) delta = -edot_delta_limit(i);

            edot_limited(i) = edot_prev_limited(i) + delta;

            if (std::fabs(edot_limited(i)) < edot_deadband(i)) {
                edot_limited(i) = 0.0f;
            }

            edot_filt_state(i) =
                edot_alpha(i) * edot_limited(i) +
                (1.0f - edot_alpha(i)) * edot_filt_state(i);

            edot(i) = edot_filt_state(i);
        }

        edot_prev_limited = edot_limited;            

 

        Eigen::Matrix<float, 6, 1> xdd_d = Eigen::Matrix<float, 6, 1>::Zero();
        xdd_d.head<3>() = ref.a_d * 1000.0f;
        xdd_d.tail<3>() = ref.alpha_d;

        Eigen::Matrix<float, 6, 1> xdd_actual_raw = Eigen::Matrix<float, 6, 1>::Zero();
        xdd_actual_raw.head<3>() = s.a * 1000.0f;
        xdd_actual_raw.tail<3>() = s.alpha;

        Eigen::Matrix<float, 6, 1> xdd_actual_alpha;
        xdd_actual_alpha << 0.10f, 0.10f, 0.05f,
                            0.10f, 0.10f, 0.10f;

        if (!has_xdd_actual_lpf_dbic_) {
            xdd_actual_filt_dbic_ = xdd_actual_raw;
            has_xdd_actual_lpf_dbic_ = true;
        } else {
            for (int i = 0; i < 6; ++i) {
                xdd_actual_filt_dbic_(i) =
                    xdd_actual_alpha(i) * xdd_actual_raw(i) +
                    (1.0f - xdd_actual_alpha(i)) * xdd_actual_filt_dbic_(i);
            }
        }
        s.a = xdd_actual_filt_dbic_.head<3>();

        Eigen::Matrix<float, 6, 1> xdd_actual_filt = xdd_actual_filt_dbic_;
        Eigen::Matrix<float, 6, 1> e2dot = xdd_d - xdd_actual_filt;

        Eigen::Matrix<float, 6, 1> ref_task_pose_log;
        Eigen::Matrix<float, 6, 1> s_task_pose_log;

        Eigen::Matrix<float, 6, 1> ref_task_vel_log;
        Eigen::Matrix<float, 6, 1> s_task_vel_log;

        Eigen::Matrix<float, 6, 1> ref_task_acc_log;
        Eigen::Matrix<float, 6, 1> s_task_acc_log;

        // pose
        ref_task_pose_log.head<3>() = ref.p_d * 1000.0f;  // m -> mm
        s_task_pose_log.head<3>() = s.p * 1000.0f;        // m -> mm

        // orientation은 3축 rotation vector [rad]로 저장
        ref_task_pose_log.tail<3>() = quatLog(ref.q_d);
        s_task_pose_log.tail<3>() = quatLog(s.q);

        // velocity
        ref_task_vel_log.head<3>() = ref.v_d * 1000.0f;   // m/s -> mm/s
        s_task_vel_log.head<3>() = s.v * 1000.0f;         // m/s -> mm/s
        ref_task_vel_log.tail<3>() = ref.w_d;             // rad/s
        s_task_vel_log.tail<3>() = s.w;                   // rad/s

        // acceleration
        ref_task_acc_log = xdd_d;             // 이미 [mm/s^2, rad/s^2]
        s_task_acc_log = xdd_actual_filt;     // e2dot 계산에 실제로 쓴 filtered actual acc

        for (int i = 0; i < 6; ++i) {
            g_ref_task_pose_log[i].store(ref_task_pose_log(i), std::memory_order_relaxed);
            g_s_task_pose_log[i].store(s_task_pose_log(i), std::memory_order_relaxed);

            g_ref_task_vel_log[i].store(ref_task_vel_log(i), std::memory_order_relaxed);
            g_s_task_vel_log[i].store(s_task_vel_log(i), std::memory_order_relaxed);

            g_ref_task_acc_log[i].store(ref_task_acc_log(i), std::memory_order_relaxed);
            g_s_task_acc_log[i].store(s_task_acc_log(i), std::memory_order_relaxed);
        }
    
        // DB-IC core
        // Eigen::Matrix<float, 6, 1> u_d =
        //     xdd_d + Md_inv_ * (Bd_ * edot + Kd_ * e - Fe_paper);

        // ------------------------------------------------------------
        // Jdot*qdot는 raw finite difference를 그대로 쓰면 매우 noisy하므로
        // 저역통과 형태로 한 번 smoothing 한다.
        // ------------------------------------------------------------
        Eigen::Matrix<float, 6, 1> Jdot_qdot = Eigen::Matrix<float, 6, 1>::Zero();
        if (has_prev_J_dbic_) {
            Eigen::Matrix<float, 6, 6> Jdot = (s.J - J_prev_dbic_) / dt;
            Eigen::Matrix<float, 6, 1> Jdot_qdot_raw = Jdot * qdot;
            Jdot_qdot = 0.1f * Jdot_qdot_raw + 0.9f * Jdot_qdot_prev_;
            Jdot_qdot_prev_ = Jdot_qdot;
        } else {
            Jdot_qdot_prev_.setZero();
        }

        J_prev_dbic_ = s.J;
        has_prev_J_dbic_ = true;

        // ------------------------------------------------------------
        // exact inverse / damped inverse를 분기하면 joint-space 해가 튄다.
        // 항상 같은 형태의 damped pseudo inverse를 써서 joint coordination을 부드럽게 만든다.
        // ------------------------------------------------------------
        Eigen::Matrix<float, 6, 6> J_inv = dampedPseudoInverse(s.J, 5e-3f);

        Eigen::Matrix<float, 6, 1> Nhat = Hhat * qddot + Cmat * qdot + g;

        // Eigen::Matrix<float, 6, 1> tau =
        //     Hhat * J_inv * (u_d - Jdot_qdot)
        //     + Nhat
        //     + s.J.transpose() * Fe_paper;F
        //new0330

        //FT sensor use
        auto ft_matched = sensor_data.getMatchedAFTWrench();

        Eigen::Matrix<float, 6, 1> Fft_raw;
        for (int i = 0; i < 6; ++i) {
            Fft_raw(i) = ft_matched[i];
        }

        static int ft_valid_count = 0;
        static bool ft_ready = false;

        const bool ft_sensor_valid = Fft_raw.cwiseAbs().maxCoeff() > 1.0e-6f;
        if (ft_sensor_valid) {
            ++ft_valid_count;
        } else {
            ft_valid_count = 0;
            ft_ready = false;
        }

        if (ft_valid_count >= 5) {
            ft_ready = true;
        }

        Eigen::Matrix<float, 6, 1> Fext_raw =
            Eigen::Matrix<float, 6, 1>::Zero();

        if (ft_ready) {
            Eigen::Matrix<float, 6, 1> Fext_extra_offset;
            Fext_extra_offset << 0.0f, 0.0f, -0.75f, 0.0f, 0.0f, 0.0f;
            Fext_raw = Fft_raw - Fext_extra_offset;
        }

        //external_joint_torque use

        Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_ext_joint(
            robot_state->external_joint_torque
        );

        Eigen::Matrix<float, 6, 1> Fext_joint_raw =
            1.0f * J_inv.transpose() * trq_ext_joint;

        // YAML의 F_offset_gain이 PBIC::F_offset에 들어가 있으니까 이걸 빼서 로그용으로 저장
        Eigen::Matrix<float, 6, 1> Fext_joint_log =
            Fext_joint_raw - F_offset;

        // Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_raw(robot_state->external_joint_torque);

        // Eigen::Matrix<float, 6, 1> Fext_raw = -1.0f * J_inv.transpose() * trq_raw;

        // float Fsensor[6] = {17.09f, -15.47f, 1.50f, 3.52f, -1.32f, 2.06f};
        // Eigen::Map<const Eigen::Matrix<float, 6, 1>> Fsensoroffset(Fsensor);
        // // Fext_raw = Fext_raw - Fsensoroffset;
        // Fext_raw = Fext_raw;

        // --------------------------------------------------
        // spike suppression + LPF
        // --------------------------------------------------
        static bool fext_filter_init = false;
        static Eigen::Matrix<float, 6, 1> Fext_prev = Eigen::Matrix<float, 6, 1>::Zero();
        static Eigen::Matrix<float, 6, 1> Fext_filt = Eigen::Matrix<float, 6, 1>::Zero();

        if (!ft_ready) {
            fext_filter_init = false;
            Fext_prev.setZero();
            Fext_filt.setZero();
        }

        Eigen::Matrix<float, 6, 1> Fext = Fext_raw;

        // 축별 절대 제한값 [Fx,Fy,Fz,Mx,My,Mz]
        // 시작은 보수적으로 두고 나중에 조정
        Eigen::Matrix<float, 6, 1> fext_abs_limit;
        fext_abs_limit << 400.0f, 400.0f, 200.0f, 50.0f, 50.0f, 50.0f;

        // 변화율 제한값 [N/s, Nm/s]
        const float force_slew_rate  = 2000.0f;  // translational
        const float torque_slew_rate = 200.0f;   // rotational

        Eigen::Matrix<float, 6, 1> fext_delta_limit;
        fext_delta_limit << force_slew_rate * dt,
                            force_slew_rate * dt,
                            force_slew_rate * dt,
                            torque_slew_rate * dt,
                            torque_slew_rate * dt,
                            torque_slew_rate * dt;

        if (!fext_filter_init) {
            Fext_prev = Fext;
            Fext_filt = Fext;
            fext_filter_init = true;
        }

        // 1) 프레임 간 급격한 점프 제한
        for (int i = 0; i < 6; ++i) {
            float delta = Fext(i) - Fext_prev(i);

            if (delta >  fext_delta_limit(i)) delta =  fext_delta_limit(i);
            if (delta < -fext_delta_limit(i)) delta = -fext_delta_limit(i);

            Fext(i) = Fext_prev(i) + delta;

            // 2) 절대 크기 제한
            if (Fext(i) >  fext_abs_limit(i)) Fext(i) =  fext_abs_limit(i);
            if (Fext(i) < -fext_abs_limit(i)) Fext(i) = -fext_abs_limit(i);
        }

        // 3) 저역통과필터
        const float alpha_fext = 1.00f; // 작을수록 더 부드러움
        Fext_filt = alpha_fext * Fext + (1.0f - alpha_fext) * Fext_filt;

        Fext_prev = Fext;

        //
        Eigen::Matrix<float, 6, 1> Fmass = Md_ * e2dot;
        Eigen::Matrix<float, 6, 1> Fspring = Kd_ * e;
        Eigen::Matrix<float, 6, 1> Fdamp   = Bd_ * edot;
        Eigen::Matrix<float, 6, 1> Fdbic   = Fspring + Fdamp - Fe_paper;
        Eigen::Matrix<float, 6, 1> Fimp   = Fspring + Fdamp + Fmass;
        // Eigen::Matrix<float, 6, 1> Fext   = -1 * J_inv.transpose()*trq_raw;
        // float Fsesnor[6] = {17.09f, -15.47f, 1.50f, 3.52f , -1.32f, 2.06f};
        // Eigen::Map<const Eigen::Matrix<float, 6, 1>>Fsensoroffset(Fsesnor);
        // Fext = Fext-Fsensoroffset;
        //new0330
        // Eigen::Matrix<float, 6, 1> tau =
        // s.J.transpose()*(Fimp)
        // + Nhat
        // + Fext;

        //236.9+126.9

        Eigen::Matrix<float, 6, 1> tau =
        s.J.transpose()*(Fimp+Fext_filt-F_offset)
        + Nhat;


        auto clampf = [](float v, float lo, float hi) {
            return (v < lo) ? lo : ((v > hi) ? hi : v);
        };

        // safety wrapper
        const std::array<float, 6> tau_rate_limit_per_sec = {
            1500.0f, 1500.0f, 1200.0f, 250.0f, 250.0f, 250.0f
        };

        
        for (int i = 0; i < 6; ++i) {
            F.F_mass[i] = Fmass(i);
            F.F_rest[i] = Fspring(i);
            F.F_coriolis[i] = Fdamp(i);
            // F.Fext[i] = s.F_env_on_robot(i);
            F.Fext[i] = Fext_filt(i);
            F.Fext_joint[i] = Fext_joint_log(i);
            F.Fimp[i] = Fimp(i);
            errors.e[i] = e(i);
            errors.de[i] = edot(i);
            errors.dde[i] = e2dot(i);
            errors.e_integral[i] = 0.0f;

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
        float actual_velocityj[6] = {0,};
        float trq_gravity[6] = {0,};

        memcpy(joint, robot_state->actual_joint_position, sizeof(float) * 6);
        memcpy(actual_velocityj, robot_state->actual_joint_velocity, sizeof(float) * 6);
        memcpy(trq_gravity, robot_state->gravity_torque, sizeof(float) * 6);

        const std::array<float, 6> torque_limits = {519.0f, 519.0f, 244.5f, 75.0f, 75.0f, 75.0f};

        Eigen::Matrix<float, 6, 6> J = mapMat6(robot_state->jacobian_matrix);
        Eigen::Matrix<float, 6, 1> tool_weight_tau = J.transpose() * (-F_offset);

        Eigen::Matrix<float, 6, 1> xdot_imp = Eigen::Matrix<float, 6, 1>::Zero();

        // imp.v_m은 PBIC에서 mm/s
        xdot_imp(0) = imp.v_m(0) * 1e-3f;
        xdot_imp(1) = imp.v_m(1) * 1e-3f;
        xdot_imp(2) = imp.v_m(2) * 1e-3f;

        // imp.w_m은 rad/s
        xdot_imp(3) = imp.w_m(0);
        xdot_imp(4) = imp.w_m(1);
        xdot_imp(5) = imp.w_m(2);

        Eigen::Matrix<float, 6, 6> J_inv = dampedPseudoInverse(J, 5e-3f);

        // J는 qdot [rad/s] -> twist [m/s, rad/s]
        // 따라서 qdot_des_rad는 rad/s
        Eigen::Matrix<float, 6, 1> qdot_des_rad = J_inv * xdot_imp;

        // actual_velocityj는 deg/s라서 맞춰줌
        Eigen::Matrix<float, 6, 1> qdot_des_deg = qdot_des_rad * RAD2DEG;

        for (int i = 0; i < 6; ++i) {
            g_qdot_des_log[i].store(qdot_des_deg(i), std::memory_order_relaxed);
        }

        for (int i = 0; i < 6; ++i)
        {
            err[i] = desired.q_d[i] - joint[i];

            // joint6 초기 branch jump 완화용 기존 로직 유지
            // if (i == 5 && count <= 500) {
            //     float scaling_factor = static_cast<float>(count) / 500.0f;
            //     err[5] *= scaling_factor;
            // }

            // angle wrap
            if (err[i] >= 350.0f) {
                err[i] -= 360.0f;
            } else if (err[i] <= -350.0f) {
                err[i] += 360.0f;
            }

            const float qdot_des = qdot_des_deg(i);
            float derr_raw = qdot_des - actual_velocityj[i];

            // if (i == 5 && count <= 500) {
            //     float scaling_factor = static_cast<float>(count) / 500.0f;
            //     derr_raw *= scaling_factor;
            // }

            // derivative LPF
            // derivative LPF
            const std::array<float, 6> derr_alpha_joint = {
                0.1f,  // J1
                0.1f,  // J2
                0.1f,  // J3
                0.1f,  // J4
                0.1f,  // J5
                0.1f   // J6
            };

            float derr_alpha = derr_alpha_joint[i];

            derr[i] = derr_alpha * derr_raw + (1.0f - derr_alpha) * derrPrev(i);

            // derr[i] = 0.2f * derr_raw + 0.8f * derrPrev(i);

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
                + trq_gravity[i]
                + tool_weight_tau(i); 

            //test-deadband

            // J5 directional breakaway minimum torque compensation
            // float tau_pbic = torque.tau_d[i];

            // if (i == 4) {  // J5
            //     static float tau_comp_prev = 0.0f;

            //     if (count == 0) {
            //         tau_comp_prev = 0.0f;
            //     }

            //     const float tau_dead = 5.5f;          // 추정 deadband [Nm]
            //     const float tau_eps = 0.05f;          // 아주 작은 torque는 무시
            //     const float vel_release = 1.5f;       // 움직이면 보상 줄임 [deg/s]
            //     const float comp_rate = 150.0f;       // Nm/s

            //     float direction = 0.0f;

            //     if (std::fabs(tau_pbic) > tau_eps) {
            //         direction = (tau_pbic > 0.0f) ? 1.0f : -1.0f;
            //     } else if (std::fabs(qdot_des) > 0.02f) {
            //         direction = (qdot_des > 0.0f) ? 1.0f : -1.0f;
            //     }

            //     float tau_comp_cmd = 0.0f;

            //     if (direction != 0.0f &&
            //         std::fabs(actual_velocityj[i]) < vel_release) {
            //         tau_comp_cmd = direction * tau_dead;
            //     }

            //     const float max_step = comp_rate * dt;
            //     float delta = tau_comp_cmd - tau_comp_prev;

            //     if (delta > max_step) delta = max_step;
            //     if (delta < -max_step) delta = -max_step;

            //     tau_comp_prev += delta;

            //     torque.tau_d[i] = tau_pbic + tau_comp_prev;
            // }

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

        return torque;
    }

    // NOTE:
    // 이 함수는 PBIC outer loop용 IK target generator이다.
    // 현재 DBIC goal/path 실험에서는 ControlGeneratorDBIC() 경로를 사용하므로
    // 여기의 Euler dummy orientation은 현재 DBIC photo issue의 직접 원인이 아니다.
    //new0411
    // std::pair<std::array<float, 6>, bool> PBIC::MotionGenerator(Trajectory &trajectory, const LPRT_OUTPUT_DATA_LIST robot_state, Prev &prev, Impedance &imp, int sol_space, bool correction_flag,int operator_call_count_)
    // std::pair<std::array<float, 6>, bool> PBIC::MotionGenerator(Trajectory &trajectory,
    //                                                             const LPRT_OUTPUT_DATA_LIST robot_state,
    //                                                             Prev &prev,
    //                                                             Impedance &imp,
    //                                                             int& sol_space,
    //                                                             bool correction_flag,
    //                                                             int operator_call_count_,
    //                                                             TaskPointMode task_point_mode,
    //                                                             const Eigen::Isometry3f& T_flange_tcp)//
    // {
    //     static int singularity_counter = 0;
    //     bool is_singular = false;
    //     std::array<float, 6> des = {0, };
    //     LPRT_OUTPUT_DATA_LIST robot_data = Drfl_.read_data_rt();
    //     Eigen::Map<const Eigen::Matrix<float, 6, 1>> q(robot_state->actual_joint_position);
    //     Eigen::Map<const Eigen::Matrix<float, 6, 1>> x(robot_state->actual_flange_position);
    //     Eigen::Map<Eigen::Matrix<float, 6, 1>> xPrev(prev.xPrev.data());
    //     Eigen::Map<Eigen::Matrix<float, 6, 1>> vPrev(prev.vPrev.data());
    //     Eigen::Map<Eigen::Matrix<float, 6, 1>> qPrev(prev.qPrev.data());
    
    //     Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_raw(robot_state->actual_joint_torque);
    //     Eigen::Map<Eigen::Matrix<float, 6, 1>> trq_ext(robot_state->external_joint_torque);
    //     Eigen::Map<const Eigen::Matrix<float, 6, 1>> trq_g(robot_state->gravity_torque);
    //     Eigen::Map<Eigen::Matrix<float, 6, 1>> F_extPrev(prev.F_extPrev.data());
    //     // Eigen::Map<Eigen::Matrix<float, 6, 1>> pos(trajectory.pos_d.data());
    //     // Eigen::Map<Eigen::Matrix<float, 6, 1>> vel(trajectory.vel_d.data());
    //     // Eigen::Map<Eigen::Matrix<float, 6, 1>> acc(trajectory.acc_d.data());

    //     // // ✅ 물리적 크기인 7차원으로 우선 매핑
    //     // Eigen::Map<Eigen::Matrix<float, 7, 1>> pos(trajectory.pos_d.data());
    //     // Eigen::Map<Eigen::Matrix<float, 7, 1>> vel(trajectory.vel_d.data());
    //     // Eigen::Map<Eigen::Matrix<float, 7, 1>> acc(trajectory.acc_d.data());

    //     // 여기서 trajectory는 fillEulerDummyForIK()를 거친 Euler dummy trajectory입니다.
    //     // 따라서 앞의 6개만 [x, y, z, roll, pitch, yaw]로 사용해야 합니다.
    //     Eigen::Map<const Eigen::Matrix<float, 6, 1>> pos_euler(trajectory.pos_d.data());
    //     Eigen::Map<const Eigen::Matrix<float, 6, 1>> vel_euler(trajectory.vel_d.data());
    //     Eigen::Map<const Eigen::Matrix<float, 6, 1>> acc_euler(trajectory.acc_d.data());

    //     Eigen::Matrix<float, 6, 6> JPrev;
    //     float qd_sing[NUMBER_OF_JOINT] = {0,};
    //     Eigen::Matrix<float, 6, 1> v = 0.05 * (x - xPrev) / dt + 0.95 * vPrev;
    //     float q_input_array[NUMBER_OF_JOINT]  = {0,};
    //     float trq_ext_input_array[NUMBER_OF_JOINT] = {0,};
    //     float task_p_input_array[NUMBER_OF_JOINT] = {0,};
    //     float jacobianMatrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT] = {{0,}};
    //     float F_box[NUMBER_OF_JOINT] = {0,};


    //     memcpy(q_input_array, robot_state->actual_joint_position, sizeof(float) * 6);
    //     memcpy(trq_ext_input_array, robot_state->external_joint_torque, sizeof(float) * 6);
    //     memcpy(task_p_input_array, robot_state->target_tcp_position, sizeof(float) * 6);
    //     memcpy(jacobianMatrix, robot_data->jacobian_matrix, NUMBER_OF_JOINT * NUMBER_OF_JOINT * sizeof(float));
    //     memcpy(F_box, robot_data->external_tcp_force, NUMBER_OF_JOINT * sizeof(float));

    //     Eigen::Matrix<float, 6, 6> J;

    //     for (int i = 0; i < NUMBER_OF_JOINT; ++i) {
    //         for (int j = 0; j < NUMBER_OF_JOINT; ++j) {
    //             J(i,j) = jacobianMatrix[i][j];
    //         }
    //         // F_ext(i) = F_box[i];
    //         // 노이즈로 인한 토크 발산을 막기 위해 로우패스 필터(LPF) 적용 복구
    //         F_ext(i) = 0.1f * F_box[i] + 0.9f * F_extPrev(i);
    //         F_ext(i) = 0;
    //     }

    //     Eigen::Matrix<float, 1, 6> q_input_matrix;
    //     Eigen::Matrix<float, 1, 6> task_p_input_matrix;
    //     Eigen::Matrix<float, 1, 6> trq_ext_input_matrix;

    //     for (int i = 0; i < 6; ++i) {
    //         q_input_matrix(0, i) = q_input_array[i];
    //         task_p_input_matrix(0, i) = task_p_input_array[i];
    //         trq_ext_input_matrix(0, i) = trq_ext_input_array[i];
    //     }

    //     // F_estim = F_estimate(q_input_matrix, task_p_input_matrix, trq_ext_input_matrix);
        
    //     // external force estimation 
    //     // F_ext = 0.1 * (J.transpose().inverse() * (trq_ext) - F_offset) + 0.9 * F_extPrev; // w/ Gripper 
    //     // F_ext = 0.1 * (J.transpose().inverse() * (trq_ext) - F_estim) + 0.9 * F_extPrev; // w/ Gripper learning
    //     // F_ext = 0.1 * (J.transpose().inverse() * (trq_ext)) + 0.9 * F_extPrev; // w/o Gripper 

    //     // F_ext = (J.transpose().inverse() * (trq_ext)) - F_offset;``````

    //     // Remove moment
    //     // F_ext[3] = 0;
    //     // F_ext[4] = 0;
    //     // F_ext[5] = 0;
    //     // 

    //     // 2. utilize force sensor
    //     for (int i = 0 ; i < 6 ; i++) {
    //         F_sensor[i] = sensor_data.AFT_wrench_[i];
    //     }
        
    //     // consider adhere frame
    //     float(*result)[3] = Drfl_.get_current_rotm();

    //     F_sensor_matched[0] = -F_sensor[0];
    //     F_sensor_matched[1] = -F_sensor[1];
    //     F_sensor_matched[2] = F_sensor[2];
    //     F_sensor_matched[3] = -F_sensor[3];
    //     F_sensor_matched[4] = -F_sensor[4];
    //     F_sensor_matched[5] = F_sensor[5];

    //     Eigen::Matrix3f rotationMatrix;
    //     for (int i = 0; i < 3; ++i) {
    //         for (int j = 0; j < 3; ++j) {
    //             rotationMatrix(i, j) = result[i][j];
    //         }
    //     }

    
    //     Eigen::Vector3f forceVector(F_sensor_matched[0], F_sensor_matched[1], F_sensor_matched[2]);
    //     Eigen::Vector3f torqueVector(F_sensor_matched[3], F_sensor_matched[4], F_sensor_matched[5]);

    //     // Step 3: Multiply the vectors by the rotation matrix
    //     Eigen::Vector3f rotatedForce = rotationMatrix * forceVector;
    //     Eigen::Vector3f rotatedTorque = rotationMatrix * torqueVector;

    //     F_sensor_matched[0] = rotatedForce(0);  // Rotated x-force
    //     F_sensor_matched[1] = rotatedForce(1);  // Rotated y-force
    //     F_sensor_matched[2] = rotatedForce(2);  // Rotated z-force

    //     F_sensor_matched[3] = rotatedTorque(0);  // Rotated x-torque
    //     F_sensor_matched[4] = rotatedTorque(1);  // Rotated y-torque
    //     F_sensor_matched[5] = rotatedTorque(2);  // Rotated z-torque
        
    //     // F_ext = 0.2 * F_sensor + 0.8 * F_extPrev - F_offset; // Sensor value -> External force 

    //     for (int i = 0 ; i < 6 ; i++) {
    //         sensor_data.AFT_wrench_matched[i] = F_sensor_matched[i]; 
    //     }
    //     //new0411
    //     // ------------------------------------------------------------------
    //     // external_joint_torque -> task wrench
    //     // DBIC-style filtering for PBIC
    //     // ------------------------------------------------------------------
    //     TaskState s = getTaskState(robot_state, task_point_mode, T_flange_tcp);

    //     static int pbic_fext_motion_id = -1;
    //     static bool pbic_fext_filter_init = false;
    //     static Eigen::Matrix<float, 6, 1> Fext_prev = Eigen::Matrix<float, 6, 1>::Zero();
    //     static Eigen::Matrix<float, 6, 1> Fext_filt = Eigen::Matrix<float, 6, 1>::Zero();

    //     if (pbic_fext_motion_id != operator_call_count_) {
    //         pbic_fext_motion_id = operator_call_count_;
    //         pbic_fext_filter_init = false;
    //         Fext_prev.setZero();
    //         Fext_filt.setZero();
    //     }

    //     Eigen::Matrix<float, 6, 6> J_inv = dampedPseudoInverse(s.J, 5e-3f);
    //     Eigen::Matrix<float, 6, 1> Fext_raw = -1.0f * J_inv.transpose() * trq_ext;

    //     Eigen::Matrix<float, 6, 1> Fext = Fext_raw;

    //     Eigen::Matrix<float, 6, 1> fext_abs_limit;
    //     fext_abs_limit << 400.0f, 400.0f, 200.0f, 50.0f, 50.0f, 50.0f;

    //     const float force_slew_rate  = 2000.0f;
    //     const float torque_slew_rate = 200.0f;

    //     Eigen::Matrix<float, 6, 1> fext_delta_limit;
    //     fext_delta_limit << force_slew_rate * dt,
    //                         force_slew_rate * dt,
    //                         force_slew_rate * dt,
    //                         torque_slew_rate * dt,
    //                         torque_slew_rate * dt,
    //                         torque_slew_rate * dt;

    //     if (!pbic_fext_filter_init) {
    //         Fext_prev = Fext;
    //         Fext_filt = Fext;
    //         pbic_fext_filter_init = true;
    //     }

    //     for (int i = 0; i < 6; ++i) {
    //         float delta = Fext(i) - Fext_prev(i);

    //         if (delta >  fext_delta_limit(i)) delta =  fext_delta_limit(i);
    //         if (delta < -fext_delta_limit(i)) delta = -fext_delta_limit(i);

    //         Fext(i) = Fext_prev(i) + delta;

    //         if (Fext(i) >  fext_abs_limit(i)) Fext(i) =  fext_abs_limit(i);
    //         if (Fext(i) < -fext_abs_limit(i)) Fext(i) = -fext_abs_limit(i);
    //     }

    //     const float alpha_fext = 0.80f;
    //     Fext_filt = alpha_fext * Fext + (1.0f - alpha_fext) * Fext_filt;
    //     Fext_prev = Fext;

    //     F_ext = Fext_filt;
    //     //
    //     Eigen::Matrix<float, 6, 1> imp_C = Eigen::Matrix<float, 6, 1>::Zero();

    //     // imp_C = M_inv * (M * acc + B * vel + K * pos + F_ext); // considering external force
    //     // // imp_C = M_inv * (M * acc + B * vel + K * pos); // Not considering external force 

    //     // rungeKutta(t_start, imp.pos_m, imp.vel_m, imp_C);

    //     // imp.acc_m = M_inv * (-1 * B * imp.vel_m - K * imp.pos_m) + imp_C;
    //     // F_imp = M * (acc - imp.acc_m) + B * (vel - imp.vel_m) + K * (pos - imp.pos_m);

    //     // ✅ .head(6)을 사용해 앞의 6칸(X,Y,Z,Roll,Pitch,Yaw)만 추출하여 연산
    //     // imp_C = M_inv * (M * acc.head(6) + B * vel.head(6) + K * pos.head(6) + F_ext); 

    //     // rungeKutta(t_start, imp.pos_m, imp.vel_m, imp_C);

    //     // imp.acc_m = M_inv * (-1 * B * imp.vel_m - K * imp.pos_m) + imp_C;
        
    //     // // ✅ 여기도 .head(6) 적용
    //     // F_imp = M * (acc.head(6) - imp.acc_m) + B * (vel.head(6) - imp.vel_m) + K * (pos.head(6) - imp.pos_m);

    //     // imp_C = M_inv * (M * acc_euler + B * vel_euler + K * pos_euler + F_ext);

    //     // PBIC outer impedance model
    //     // xdd_m = xdd_d + M^{-1}[ B(xd_dot - x_m_dot) + K(xd - x_m) - F_int ]
    //     // F_ext는 environment-on-robot 기준으로 사용

    //     imp_C = M_inv * (M * acc_euler + B * vel_euler + K * pos_euler - F_ext);

    //     rungeKutta(t_start, imp.pos_m, imp.vel_m, imp_C);

    //     imp.acc_m = M_inv * (-1 * B * imp.vel_m - K * imp.pos_m) + imp_C;

    //     F_imp = M * (acc_euler - imp.acc_m)
    //         + B * (vel_euler - imp.vel_m)
    //         + K * (pos_euler - imp.pos_m);

    //     for (int i = 0; i < 6; i++)
    //     {
    //         F.Fext[i] = F_ext(i);
    //         F.Fimp[i] = F_imp(i); 
    //     }

    //     // transform Eigen::Matrix to float[6]
        
    //     float x_d[6] = {0,};
    //     float x_d2[6] = {0,};
        
    //     Eigen::VectorXf::Map(&x_d[0], 6) = imp.pos_m; // Impedance mode
    

    //     // trajectory는 이미 fillEulerDummyForIK()를 거쳐
    //     // quaternion -> continuous Euler dummy 로 변환된 상태다.
    //     // 따라서 여기의 pos_euler(3..5)는 raw quaternion이 아니라
    //     // [roll, pitch, yaw] [deg] 값이다.
    //     x_d[3] = pos_euler(3);
    //     x_d[4] = pos_euler(4);
    //     x_d[5] = pos_euler(5);

    //     float current_joint[NUMBER_OF_JOINT] = {0,};
    //     memcpy(current_joint, robot_state->actual_joint_position, sizeof(float) * 6);

    //     // 새 motion 시작 시 branch continuity 기준을 현재 joint로 맞춤
    //     if (count_motion == 0) {
    //         memcpy(previous_joint_command,
    //                robot_state->actual_joint_position,
    //                sizeof(float) * 6);
    //     }
    //     //new0410
    //     // ------------------------------------------------------------------
    //     // IK를 하나의 solution space로만 풀지 말고,
    //     // 이전 command와 가장 가까운 해를 선택해서 branch jump를 줄인다.
    //     // ------------------------------------------------------------------
    //     // float best_des[NUMBER_OF_JOINT] = {0,};
    //     // bool found_solution = false;
    //     // float best_cost = 1.0e30f;
    //     // int best_sol_space = sol_space;

    //     // for (int cand_sol = 0; cand_sol < 8; ++cand_sol) {
    //     //     LPINVERSE_KINEMATIC_RESPONSE cand =
    //     //         Drfl_.ikin(x_d, cand_sol, COORDINATE_SYSTEM_WORLD, 1);

    //     //     if (cand == nullptr) {
    //     //         continue;
    //     //     }

    //     //     float cost = 0.0f;
    //     //     for (int i = 0; i < 6; ++i) {
    //     //         float delta = cand->_fTargetPos[i] - previous_joint_command[i];
    //     //         while (delta > 180.0f) delta -= 360.0f;
    //     //         while (delta < -180.0f) delta += 360.0f;
    //     //         cost += delta * delta;
    //     //     }

    //     //     if (cost < best_cost) {
    //     //         best_cost = cost;
    //     //         best_sol_space = cand_sol;
    //     //         for (int i = 0; i < 6; ++i) {
    //     //             best_des[i] = cand->_fTargetPos[i];
    //     //         }
    //     //         found_solution = true;
    //     //     }
    //     // }

    //     // if (!found_solution) {
    //     //     ROS_WARN("MotionGenerator: IK failed for all solution spaces. Holding previous joint command.");

    //     //     for (int i = 0; i < 6; ++i) {
    //     //         des[i] = previous_joint_command[i];
    //     //     }

    //     //     singularity_counter++;
    //     //     if (singularity_counter >= 10) {
    //     //         is_singular = true;
    //     //     }

    //     //     return {des, is_singular};
    //     // }

    //     // sol_space = best_sol_space;
    //     // for (int i = 0; i < 6; ++i) {
    //     //     des[i] = best_des[i];
    //     // }

    //     // // ------------------------------------------------------------------
    //     // // 여기서 보는 것은 "실제 singularity"가 아니라
    //     // // IK branch jump(갑작스러운 해 점프)다.
    //     // // current_joint가 아니라 previous_joint_command와 비교해야 한다.
    //     // // ------------------------------------------------------------------
    //     // bool branch_jump = false;
    //     // int jump_joint = -1;
    //     // float jump_delta = 0.0f;

    //     // for (int i = 0; i < 6; ++i) {
    //     //     float delta = des[i] - previous_joint_command[i];
    //     //     while (delta > 180.0f) delta -= 360.0f;
    //     //     while (delta < -180.0f) delta += 360.0f;

    //     //     // 기존 20 deg는 너무 예민해서 false positive가 잘 난다.
    //     //     if (std::abs(delta) > 35.0f) {
    //     //         branch_jump = true;
    //     //         jump_joint = i;
    //     //         jump_delta = delta;
    //     //         singularity_counter++;
    //     //         break;
    //     //     }
    //     // }

    //     // if (branch_jump) {
    //     //     std::cout << "IK branch jump at joint " << jump_joint
    //     //               << ", prev_cmd : " << previous_joint_command[jump_joint]
    //     //               << ", ik_cmd : " << des[jump_joint]
    //     //               << ", delta : " << jump_delta
    //     //               << ", sol_space : " << best_sol_space << std::endl;
    //     //     ROS_WARN("IK branch jump detected");

    //     //     // 갑자기 다른 branch로 튀는 해는 쓰지 않고 이전 command를 유지
    //     //     for (int i = 0; i < 6; ++i) {
    //     //         des[i] = previous_joint_command[i];
    //     //     }
    //     // } else {
    //     //     singularity_counter = 0;
    //     // }

    //     // if (singularity_counter >= 10) {
    //     //     ROS_WARN("IK branch jump persisted for 10 frames. Exiting motion.");
    //     //     is_singular = true;
    //     // }
    //     // 새 motion 시작 시 branch continuity 기준을 현재 joint로 맞춤
    //     if (count_motion == 0) {
    //         memcpy(previous_joint_command,
    //                robot_state->actual_joint_position,
    //                sizeof(float) * 6);

    //         // 현재 로봇이 실제로 있는 branch를 seed로 들고 간다.
    //         sol_space = static_cast<int>(robot_state->solution_space);
    //     }

    //     auto wrapDeltaDeg = [](float delta) {
    //         while (delta > 180.0f) delta -= 360.0f;
    //         while (delta < -180.0f) delta += 360.0f;
    //         return delta;
    //     };

    //     auto evaluateIkCandidate = [&](int cand_sol,
    //                                    float out_des[NUMBER_OF_JOINT],
    //                                    float& out_cost,
    //                                    float& out_max_delta_deg) -> bool {
    //         LPINVERSE_KINEMATIC_RESPONSE cand =
    //             Drfl_.ikin(x_d, cand_sol, COORDINATE_SYSTEM_WORLD, 1);

    //         if (cand == nullptr) {
    //             return false;
    //         }

    //         out_cost = 0.0f;
    //         out_max_delta_deg = 0.0f;

    //         for (int i = 0; i < 6; ++i) {
    //             float delta_prev = wrapDeltaDeg(cand->_fTargetPos[i] - previous_joint_command[i]);
    //             float delta_curr = wrapDeltaDeg(cand->_fTargetPos[i] - current_joint[i]);

    //             // continuity는 previous command 기준, current joint는 보조 가중치만 줌
    //             out_cost += delta_prev * delta_prev + 0.05f * delta_curr * delta_curr;
    //             out_max_delta_deg = std::max(out_max_delta_deg, std::fabs(delta_prev));

    //             out_des[i] = cand->_fTargetPos[i];
    //         }

    //         return true;
    //     };

    //     constexpr float kFastAcceptMaxDeltaDeg = 12.0f;   // 이 이하면 현재 branch 유지
    //     constexpr float kHardJumpRejectDeg    = 35.0f;    // 이것보다 크면 해를 버림

    //     float best_des[NUMBER_OF_JOINT] = {0,};
    //     bool found_solution = false;
    //     float best_cost = 1.0e30f;
    //     int best_sol_space = sol_space;

    //     bool need_full_scan = (count_motion == 0);

    //     // ------------------------------------------------------------
    //     // 1) 첫 프레임은 무조건 8개 전부 탐색
    //     // 2) 그 다음부터는 현재 sol_space 하나만 먼저 풀어본다
    //     // 3) 현재 branch가 실패하거나 continuity가 나쁘면 그때만 8개 재탐색
    //     // ------------------------------------------------------------
    //     if (!need_full_scan) {
    //         float cand_des[NUMBER_OF_JOINT] = {0,};
    //         float cand_cost = 0.0f;
    //         float cand_max_delta_deg = 0.0f;

    //         if (evaluateIkCandidate(sol_space,
    //                                 cand_des,
    //                                 cand_cost,
    //                                 cand_max_delta_deg) &&
    //             cand_max_delta_deg <= kFastAcceptMaxDeltaDeg) {
    //             found_solution = true;
    //             best_cost = cand_cost;
    //             best_sol_space = sol_space;

    //             for (int i = 0; i < 6; ++i) {
    //                 best_des[i] = cand_des[i];
    //             }
    //         } else {
    //             need_full_scan = true;
    //         }
    //     }

    //     if (need_full_scan) {
    //         found_solution = false;
    //         best_cost = 1.0e30f;

    //         for (int cand_sol = 0; cand_sol < 8; ++cand_sol) {
    //             float cand_des[NUMBER_OF_JOINT] = {0,};
    //             float cand_cost = 0.0f;
    //             float cand_max_delta_deg = 0.0f;

    //             if (!evaluateIkCandidate(cand_sol,
    //                                      cand_des,
    //                                      cand_cost,
    //                                      cand_max_delta_deg)) {
    //                 continue;
    //             }

    //             if (cand_cost < best_cost) {
    //                 best_cost = cand_cost;
    //                 best_sol_space = cand_sol;

    //                 for (int i = 0; i < 6; ++i) {
    //                     best_des[i] = cand_des[i];
    //                 }
    //                 found_solution = true;
    //             }
    //         }
    //     }

    //     if (!found_solution) {
    //         ROS_WARN("MotionGenerator: IK failed for current branch and all 8 solution spaces. Holding previous joint command.");

    //         for (int i = 0; i < 6; ++i) {
    //             des[i] = previous_joint_command[i];
    //         }

    //         singularity_counter++;
    //         if (singularity_counter >= 10) {
    //             is_singular = true;
    //         }

    //         return {des, is_singular};
    //     }

    //     // ------------------------------------------------------------
    //     // 최종 branch jump guard
    //     // full scan 후에도 너무 큰 joint discontinuity면 그 해는 버린다.
    //     // ------------------------------------------------------------
    //     bool branch_jump = false;
    //     int jump_joint = -1;
    //     float jump_delta = 0.0f;

    //     for (int i = 0; i < 6; ++i) {
    //         float delta = wrapDeltaDeg(best_des[i] - previous_joint_command[i]);

    //         if (std::abs(delta) > kHardJumpRejectDeg) {
    //             branch_jump = true;
    //             jump_joint = i;
    //             jump_delta = delta;
    //             singularity_counter++;
    //             break;
    //         }
    //     }

    //     if (branch_jump) {
    //         std::cout << "IK branch jump at joint " << jump_joint
    //                   << ", prev_cmd : " << previous_joint_command[jump_joint]
    //                   << ", ik_cmd : " << best_des[jump_joint]
    //                   << ", delta : " << jump_delta
    //                   << ", candidate sol_space : " << best_sol_space
    //                   << std::endl;

    //         ROS_WARN("IK branch jump detected, holding previous joint command.");

    //         for (int i = 0; i < 6; ++i) {
    //             des[i] = previous_joint_command[i];
    //         }
    //     } else {
    //         singularity_counter = 0;

    //         for (int i = 0; i < 6; ++i) {
    //             des[i] = best_des[i];
    //         }

    //         // 이때만 다음 루프에 branch를 계승한다.
    //         sol_space = best_sol_space;
    //     }

    //     if (singularity_counter >= 10) {
    //         ROS_WARN("IK branch jump persisted for 10 frames. Exiting motion.");
    //         is_singular = true;
    //     }

    //     std::copy(robot_state->actual_flange_position,
    //               robot_state->actual_flange_position + 6,
    //               begin(prev.xPrev));

    //     Eigen::VectorXf::Map(&prev.vPrev[0], 6) = imp.vel_m;
    //     Eigen::VectorXf::Map(&prev.F_extPrev[0], 6) = F_ext;

    //     for (int i = 0; i < 6; ++i) {
    //         previous_joint_command[i] = des[i];
    //     }

    //     count_motion++;

    //     return {des, is_singular};
    // }
    //new0412
    std::pair<std::array<float, 6>, bool> PBIC::MotionGenerator(
        Trajectory &trajectory,
        const LPRT_OUTPUT_DATA_LIST robot_state,
        Prev &prev,
        Impedance &imp,
        int& sol_space,
        bool correction_flag,
        int operator_call_count_,
        TaskPointMode task_point_mode,
        const Eigen::Isometry3f& T_flange_tcp)
    {
        (void)correction_flag;

        static int singularity_counter = 0;
        static bool zyz_ref_initialized = false;
        static float prev_zyz_deg[3] = {0.0f, 0.0f, 0.0f};

        bool is_singular = false;
        std::array<float, 6> des = {0, };

        Eigen::Map<const Eigen::Matrix<float, 6, 1>> tau_ext(robot_state->external_joint_torque);
        Eigen::Map<Eigen::Matrix<float, 6, 1>> F_extPrev(prev.F_extPrev.data());

        // ------------------------------------------------------------
        // Desired task-space trajectory
        // translation: [mm], [mm/s], [mm/s^2]
        // rotation   : quaternion + angular velocity/acceleration
        // ------------------------------------------------------------
        Eigen::Vector3f p_d;
        p_d << trajectory.pos_d[0], trajectory.pos_d[1], trajectory.pos_d[2];

        Eigen::Vector3f v_d;
        v_d << trajectory.vel_d[0], trajectory.vel_d[1], trajectory.vel_d[2];

        Eigen::Vector3f a_d;
        a_d << trajectory.acc_d[0], trajectory.acc_d[1], trajectory.acc_d[2];

        Eigen::Quaternionf q_d = quatFromPose7(trajectory.pos_d);
        alignQuatHemisphere(q_d, imp.q_m);

        Eigen::Vector3f w_d;
        w_d << trajectory.w_d[0], trajectory.w_d[1], trajectory.w_d[2];

        Eigen::Vector3f alpha_d;
        alpha_d << trajectory.alpha_d[0], trajectory.alpha_d[1], trajectory.alpha_d[2];

        // ------------------------------------------------------------
        // external_joint_torque -> task wrench
        // DBIC-style filtering
        // ------------------------------------------------------------
        TaskState s = getTaskState(robot_state, task_point_mode, T_flange_tcp);

        //log
        Eigen::Matrix<float, 6, 1> ref_task_pose_log;
        Eigen::Matrix<float, 6, 1> s_task_pose_log;

        Eigen::Matrix<float, 6, 1> ref_task_vel_log;
        Eigen::Matrix<float, 6, 1> s_task_vel_log;

        Eigen::Matrix<float, 6, 1> ref_task_acc_log;
        Eigen::Matrix<float, 6, 1> s_task_acc_log;

        // PBIC nominal reference
        // p_d, v_d, a_d는 이미 mm, mm/s, mm/s^2
        ref_task_pose_log.head<3>() = p_d;
        ref_task_vel_log.head<3>() = v_d;
        ref_task_acc_log.head<3>() = a_d;

        // 현재 로봇 상태 s는 getTaskState() 기준으로 m, m/s, m/s^2라서 mm로 변환
        s_task_pose_log.head<3>() = s.p * 1000.0f;
        s_task_vel_log.head<3>() = s.v * 1000.0f;
        s_task_acc_log.head<3>() = s.a * 1000.0f;

        // orientation은 rad 기준 3축 rotation vector로 저장
        ref_task_pose_log.tail<3>() = quatLog(q_d);
        s_task_pose_log.tail<3>() = quatLog(s.q);

        // angular velocity / acceleration은 rad/s, rad/s^2
        ref_task_vel_log.tail<3>() = w_d;
        s_task_vel_log.tail<3>() = s.w;

        ref_task_acc_log.tail<3>() = alpha_d;
        s_task_acc_log.tail<3>() = s.alpha;

        for (int i = 0; i < 6; ++i) {
            g_ref_task_pose_log[i].store(ref_task_pose_log(i), std::memory_order_relaxed);
            g_s_task_pose_log[i].store(s_task_pose_log(i), std::memory_order_relaxed);

            g_ref_task_vel_log[i].store(ref_task_vel_log(i), std::memory_order_relaxed);
            g_s_task_vel_log[i].store(s_task_vel_log(i), std::memory_order_relaxed);

            g_ref_task_acc_log[i].store(ref_task_acc_log(i), std::memory_order_relaxed);
            g_s_task_acc_log[i].store(s_task_acc_log(i), std::memory_order_relaxed);
        }

        static int pbic_fext_motion_id = -1;
        static bool pbic_fext_filter_init = false;
        static Eigen::Matrix<float, 6, 1> Fext_prev = Eigen::Matrix<float, 6, 1>::Zero();
        static Eigen::Matrix<float, 6, 1> Fext_filt = Eigen::Matrix<float, 6, 1>::Zero();

        if (pbic_fext_motion_id != operator_call_count_) {
            pbic_fext_motion_id = operator_call_count_;
            pbic_fext_filter_init = false;
            Fext_prev.setZero();
            Fext_filt.setZero();
            singularity_counter = 0;
            zyz_ref_initialized = false;
        }

        auto ft_matched = sensor_data.getMatchedAFTWrench();

        Eigen::Matrix<float, 6, 1> Fft_raw;
        for (int i = 0; i < 6; ++i) {
            Fft_raw(i) = ft_matched[i];
        }

        static int ft_valid_count = 0;
        static bool ft_ready = false;

        const bool ft_sensor_valid = Fft_raw.cwiseAbs().maxCoeff() > 1.0e-6f;
        if (ft_sensor_valid) {
            ++ft_valid_count;
        } else {
            ft_valid_count = 0;
            ft_ready = false;
        }

        if (ft_valid_count >= 5) {
            ft_ready = true;
        }

        Eigen::Matrix<float, 6, 1> Fext_raw =
            Eigen::Matrix<float, 6, 1>::Zero();

        if (ft_ready) {
            Eigen::Matrix<float, 6, 1> Fext_extra_offset;
            Fext_extra_offset << 0.0f, 0.1f, -0.95f, 0.0f, 0.0f, 0.0f;
            Fext_raw = Fft_raw - Fext_extra_offset;
        }
        F_ext = Fext_raw;

//FT센서사용으로 비활성화
        Eigen::Matrix<float, 6, 6> J_inv = dampedPseudoInverse(s.J, 5e-3f);

        Eigen::Matrix<float, 6, 1> Fext_joint_raw =
            1.0f * J_inv.transpose() * tau_ext;

        Eigen::Matrix<float, 6, 1> Fext_joint_log =
            Fext_joint_raw - F_offset;
        
        // Eigen::Matrix<float, 6, 1> Fext_raw = 1.0f * J_inv.transpose() * tau_ext;

        // Eigen::Matrix<float, 6, 1> Fext = Fext_raw;

        // Eigen::Matrix<float, 6, 1> fext_abs_limit;
        // fext_abs_limit << 400.0f, 400.0f, 200.0f, 50.0f, 50.0f, 50.0f;

        // const float force_slew_rate  = 2000.0f;
        // const float torque_slew_rate = 200.0f;

        // Eigen::Matrix<float, 6, 1> fext_delta_limit;
        // fext_delta_limit << force_slew_rate * dt,
        //                     force_slew_rate * dt,
        //                     force_slew_rate * dt,
        //                     torque_slew_rate * dt,
        //                     torque_slew_rate * dt,
        //                     torque_slew_rate * dt;

        // if (!pbic_fext_filter_init) {
        //     Fext_prev = Fext;
        //     Fext_filt = Fext;
        //     pbic_fext_filter_init = true;
        // }

        // for (int i = 0; i < 6; ++i) {
        //     float delta = Fext(i) - Fext_prev(i);

        //     if (delta >  fext_delta_limit(i)) delta =  fext_delta_limit(i);
        //     if (delta < -fext_delta_limit(i)) delta = -fext_delta_limit(i);

        //     Fext(i) = Fext_prev(i) + delta;

        //     if (Fext(i) >  fext_abs_limit(i)) Fext(i) =  fext_abs_limit(i);
        //     if (Fext(i) < -fext_abs_limit(i)) Fext(i) = -fext_abs_limit(i);
        // }

        // const float alpha_fext = 0.80f;
        // Fext_filt = alpha_fext * Fext + (1.0f - alpha_fext) * Fext_filt;
        // Fext_prev = Fext;

        // F_ext = Fext_filt;
//
        // ------------------------------------------------------------
        // motion start initialization
        // ------------------------------------------------------------
        if (count_motion == 0) {
            memcpy(previous_joint_command,
                robot_state->actual_joint_position,
                sizeof(float) * 6);
            //해 전구간 탐색
            sol_space = static_cast<int>(robot_state->solution_space);
            //해 솔루션 2고정
            // sol_space = 2;
            std::cout << "[PBIC IK] initial robot_state solution_space = "
          << sol_space << std::endl;

            prev_zyz_deg[0] = robot_state->actual_flange_position[3];
            prev_zyz_deg[1] = robot_state->actual_flange_position[4];
            prev_zyz_deg[2] = robot_state->actual_flange_position[5];
            zyz_ref_initialized = true;
        }

        // ------------------------------------------------------------
        // Quaternion-based PBIC outer impedance model
        // ------------------------------------------------------------
        rungeKuttaPoseQuaternion(imp,
                                p_d,
                                v_d,
                                a_d,
                                q_d,
                                w_d,
                                alpha_d,
                                F_ext,
                                M,
                                B,
                                K,
                                M_inv,
                                dt,
                                n);
        //log
        Eigen::Matrix<float, 6, 1> imp_task_pose_log;
        Eigen::Matrix<float, 6, 1> imp_task_vel_log;
        Eigen::Matrix<float, 6, 1> imp_task_acc_log;

        // imp.p_m, imp.v_m, imp.a_m은 PBIC에서 mm 기준
        imp_task_pose_log.head<3>() = imp.p_m;
        imp_task_vel_log.head<3>() = imp.v_m;
        imp_task_acc_log.head<3>() = imp.a_m;

        // orientation은 rad 기준 rotation vector
        imp_task_pose_log.tail<3>() = quatLog(imp.q_m);

        // angular velocity / acceleration은 rad/s, rad/s^2
        imp_task_vel_log.tail<3>() = imp.w_m;
        imp_task_acc_log.tail<3>() = imp.alpha_m;

        for (int i = 0; i < 6; ++i) {
            g_imp_task_pose_log[i].store(imp_task_pose_log(i), std::memory_order_relaxed);
            g_imp_task_vel_log[i].store(imp_task_vel_log(i), std::memory_order_relaxed);
            g_imp_task_acc_log[i].store(imp_task_acc_log(i), std::memory_order_relaxed);
        }
        syncImpedanceLegacyMirror(imp,
                                prev_zyz_deg[0],
                                prev_zyz_deg[1],
                                prev_zyz_deg[2]);

        prev_zyz_deg[0] = imp.pos_m(3);
        prev_zyz_deg[1] = imp.pos_m(4);
        prev_zyz_deg[2] = imp.pos_m(5);

        // ------------------------------------------------------------
        // logging force / impedance residual
        // ------------------------------------------------------------
        Eigen::Quaternionf q_d_err = q_d;
        alignQuatHemisphere(q_d_err, imp.q_m);
        const Eigen::Vector3f e_R = quatLogError(q_d_err, imp.q_m);

        Eigen::Vector3f Mlin;
        Mlin << M(0, 0), M(1, 1), M(2, 2);

        Eigen::Vector3f Blin;
        Blin << B(0, 0), B(1, 1), B(2, 2);

        Eigen::Vector3f Klin;
        Klin << K(0, 0), K(1, 1), K(2, 2);

        Eigen::Vector3f Mrot;
        Mrot << M(3, 3),
                M(4, 4),
                M(5, 5);

        Eigen::Vector3f Brot;
        Brot << B(3, 3),
                B(4, 4),
                B(5, 5);

        Eigen::Vector3f Krot;
        Krot << K(3, 3),
                K(4, 4),
                K(5, 5);

        F_imp.setZero();
        F_imp.head<3>() =
            Mlin.cwiseProduct(a_d - imp.a_m)
        + Blin.cwiseProduct(v_d - imp.v_m)
        + Klin.cwiseProduct(p_d - imp.p_m);

        F_imp.tail<3>() =
            Mrot.cwiseProduct(alpha_d - imp.alpha_m)
        + Brot.cwiseProduct(w_d - imp.w_m)
        + Krot.cwiseProduct(e_R);

        //F_imp_val

        Eigen::Matrix<float, 6, 1> e_actual = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> edot_actual = Eigen::Matrix<float, 6, 1>::Zero();
        Eigen::Matrix<float, 6, 1> e2dot_actual = Eigen::Matrix<float, 6, 1>::Zero();

        // translation: p_d, v_d, a_d는 PBIC에서 mm, mm/s, mm/s^2
        // s.p, s.v, s.a는 getTaskState()에서 m, m/s, m/s^2라서 *1000
        e_actual.head<3>() = p_d - s.p * 1000.0f;
        edot_actual.head<3>() = v_d - s.v * 1000.0f;
        e2dot_actual.head<3>() = a_d - s.a * 1000.0f;

        // rotation: quaternion error는 rad, angular velocity/acceleration도 rad 기준
        Eigen::Quaternionf q_d_actual = q_d;
        alignQuatHemisphere(q_d_actual, s.q);

        e_actual.tail<3>() = quatLogError(q_d_actual, s.q);
        edot_actual.tail<3>() = w_d - s.w;
        e2dot_actual.tail<3>() = alpha_d - s.alpha;

        Eigen::Matrix<float, 6, 1> F_imp_val =
            M * e2dot_actual
        + B * edot_actual
        + K * e_actual;

        for (int i = 0; i < 6; ++i) {
            F.Fext[i]   = F_ext(i);
            F.Fimp[i]   = F_imp(i);
            F.F_PBIC[i] = F_imp(i);
            F.F_task[i] = F_imp(i) - F_ext(i);
            F.F_imp_val[i] = F_imp_val(i);
            F.Fext_joint[i] = Fext_joint_log(i);
        }

        // ------------------------------------------------------------
        // IK input:
        // position  = impedance translation state
        // orientation = impedance quaternion state -> continuous ZYZ
        // ------------------------------------------------------------
        float x_d[6] = {0,};

        x_d[0] = imp.p_m(0);
        x_d[1] = imp.p_m(1);
        x_d[2] = imp.p_m(2);
        x_d[3] = imp.pos_m(3);
        x_d[4] = imp.pos_m(4);
        x_d[5] = imp.pos_m(5);

        float current_joint[NUMBER_OF_JOINT] = {0,};
        memcpy(current_joint, robot_state->actual_joint_position, sizeof(float) * 6);

        auto wrapDeltaDeg = [](float delta) {
            while (delta > 180.0f) delta -= 360.0f;
            while (delta < -180.0f) delta += 360.0f;
            return delta;
        };

        auto evaluateIkCandidate = [&](int cand_sol,
                                    float out_des[NUMBER_OF_JOINT],
                                    float& out_cost,
                                    float& out_max_delta_deg) -> bool {
            LPINVERSE_KINEMATIC_RESPONSE cand =
                Drfl_.ikin(x_d, cand_sol, COORDINATE_SYSTEM_WORLD, 1);

            if (cand == nullptr) {
                return false;
            }

            out_cost = 0.0f;
            out_max_delta_deg = 0.0f;

            for (int i = 0; i < 6; ++i) {
                float delta_prev = wrapDeltaDeg(cand->_fTargetPos[i] - previous_joint_command[i]);
                float delta_curr = wrapDeltaDeg(cand->_fTargetPos[i] - current_joint[i]);

                out_cost += delta_prev * delta_prev + 0.05f * delta_curr * delta_curr;
                out_max_delta_deg = std::max(out_max_delta_deg, std::fabs(delta_prev));
                out_des[i] = cand->_fTargetPos[i];
            }

            return true;
        };

        constexpr float kFastAcceptMaxDeltaDeg = 12.0f;
        constexpr float kHardJumpRejectDeg    = 35.0f;

        float best_des[NUMBER_OF_JOINT] = {0,};
        bool found_solution = false;
        float best_cost = 1.0e30f;
        int best_sol_space = sol_space;
        //해 전구간 탐색
        bool need_full_scan = (count_motion == 0);
        // 해 2고정
        // bool need_full_scan = false;

        if (!need_full_scan) {
            float cand_des[NUMBER_OF_JOINT] = {0,};
            float cand_cost = 0.0f;
            float cand_max_delta_deg = 0.0f;

            if (evaluateIkCandidate(sol_space,
                                    cand_des,
                                    cand_cost,
                                    cand_max_delta_deg) &&
                cand_max_delta_deg <= kFastAcceptMaxDeltaDeg) {
                found_solution = true;
                best_cost = cand_cost;
                best_sol_space = sol_space;

                for (int i = 0; i < 6; ++i) {
                    best_des[i] = cand_des[i];
                }
            } else {
                need_full_scan = true;
            }
        }

        if (need_full_scan) {
            found_solution = false;
            best_cost = 1.0e30f;

            for (int cand_sol = 0; cand_sol < 8; ++cand_sol) {
                float cand_des[NUMBER_OF_JOINT] = {0,};
                float cand_cost = 0.0f;
                float cand_max_delta_deg = 0.0f;

                if (!evaluateIkCandidate(cand_sol,
                                        cand_des,
                                        cand_cost,
                                        cand_max_delta_deg)) {
                    continue;
                }

                if (cand_cost < best_cost) {
                    best_cost = cand_cost;
                    best_sol_space = cand_sol;

                    for (int i = 0; i < 6; ++i) {
                        best_des[i] = cand_des[i];
                    }
                    found_solution = true;
                }
            }
        }

        if (!found_solution) {
            ROS_WARN("MotionGenerator: IK failed for current branch and all 8 solution spaces. Holding previous joint command.");

            for (int i = 0; i < 6; ++i) {
                des[i] = previous_joint_command[i];
            }

            singularity_counter++;
            if (singularity_counter >= 10) {
                is_singular = true;
            }

            return {des, is_singular};
        }

        bool branch_jump = false;
        int jump_joint = -1;
        float jump_delta = 0.0f;

        for (int i = 0; i < 6; ++i) {
            float delta = wrapDeltaDeg(best_des[i] - previous_joint_command[i]);

            if (std::abs(delta) > kHardJumpRejectDeg) {
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
                    << ", ik_cmd : " << best_des[jump_joint]
                    << ", delta : " << jump_delta
                    << ", candidate sol_space : " << best_sol_space
                    << std::endl;

            ROS_WARN("IK branch jump detected, holding previous joint command.");

            for (int i = 0; i < 6; ++i) {
                des[i] = previous_joint_command[i];
            }
        } else {
            singularity_counter = 0;

            for (int i = 0; i < 6; ++i) {
                des[i] = best_des[i];
            }

            sol_space = best_sol_space;
            if (count_motion == 0) {
                std::cout << "[PBIC IK] selected initial sol_space = "
                << sol_space << std::endl;
}
        }

        if (singularity_counter >= 10) {
            ROS_WARN("IK branch jump persisted for 10 frames. Exiting motion.");
            is_singular = true;
        }

        std::copy(robot_state->actual_flange_position,
                robot_state->actual_flange_position + 6,
                begin(prev.xPrev));

        prev.vPrev[0] = imp.v_m(0);
        prev.vPrev[1] = imp.v_m(1);
        prev.vPrev[2] = imp.v_m(2);
        prev.vPrev[3] = imp.w_m(0) * RAD2DEG;
        prev.vPrev[4] = imp.w_m(1) * RAD2DEG;
        prev.vPrev[5] = imp.w_m(2) * RAD2DEG;

        Eigen::VectorXf::Map(&prev.F_extPrev[0], 6) = F_ext;

        for (int i = 0; i < 6; ++i) {
            previous_joint_command[i] = des[i];
        }

        count_motion++;
        return {des, is_singular};
    }  //  

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
