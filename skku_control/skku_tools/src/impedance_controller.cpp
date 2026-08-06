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
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <Eigen/Geometry>

// ======================================================================================
// MLP 힘 추정 모델 (main_final_robotiq_model.py 로 학습)
//   출력 6열 : Fx Fy Fz [N], Mx My Mz [Nm]   (sensor_FT_matched 기준)
//
// 두 모델을 나란히 돌려 비교한다. 각 모델은 자기 경로 / 자기 scaler / 자기 입력 차원을
// 따로 갖고, 입력을 만드는 함수도 F_estimate() / F_estimate2() 로 분리돼 있다.
// 모델과 scaler 는 반드시 같은 학습 실행에서 나온 짝이어야 한다.
// scaler 값 자체는 파일에서 읽으므로, 모델을 바꿔도 경로 상수만 고치면 된다.
// ======================================================================================
namespace {
    // ---- 모델 1 : robotiq q + trq_ext (12입력) -------------------------------------
    const char* const MLP1_MODEL_PATH  = "/home/rbl/catkin_ws/model_robotiq_260720_qtrq_tf";
    const char* const MLP1_SCALER_PATH = "/home/rbl/catkin_ws/scaler_info_robotiq_260720_qtrq.txt";
    constexpr int MLP1_INPUT_DIM = 12;

    // ---- 모델 2 : SKKU q + task_p + trq_ext + lag1/3/5 (36입력) ----------------------
    // 학습 스크립트: main_final_FgripperGeneration(1).py
    //   0 ~ 5  q            [deg]        (joint_position.txt)
    //   6 ~11  task position [mm, deg]   (task_position.txt)
    //  12 ~17  external torque [Nm]      (external_torque.txt)
    //  18 ~23  같은 토크 1샘플 전         (external_torque_lag1.txt)
    //  24 ~29  같은 토크 3샘플 전         (external_torque_lag3.txt)
    //  30 ~35  같은 토크 5샘플 전         (external_torque_lag5.txt)
    // 출력은 force_external.txt 기준인데, 이 데이터셋에서 force_external 은
    // sensor_FT_matched 를 1샘플 민 것과 사실상 같은 신호라 모델 1과 비교 가능하다.
    const char* const MLP2_MODEL_PATH  = "/home/rbl/catkin_ws/model_SKKU_260720_lag135_tf";
    const char* const MLP2_SCALER_PATH = "/home/rbl/catkin_ws/scaler_info_SKKU_260720_lag135.txt";
    constexpr int MLP2_INPUT_DIM = 36;

    constexpr int MLP_OUTPUT_DIM = 6;

    // scaler_info_*.txt 에서 읽어들인 스케일러. 학습측 InputScaler / OutputScaler 와 같은 식을 쓴다.
    struct MlpScaler {
        bool valid = false;
        std::vector<float> in_min, in_max, out_min, out_max;
        float in_fr_lo  = 0.0f, in_fr_hi  = 1.0f;
        float out_fr_lo = 0.0f, out_fr_hi = 1.0f;
    };

    // "Min Values: [1.0, 2.0, ...]" 같은 줄에서 대괄호 안 숫자들을 뽑는다.
    std::vector<float> parseFloatList(const std::string& line) {
        std::vector<float> out;
        const std::size_t lb = line.find('[');
        const std::size_t rb = line.rfind(']');
        if (lb == std::string::npos || rb == std::string::npos || rb < lb) {
            return out;
        }
        std::string body = line.substr(lb + 1, rb - lb - 1);
        for (std::size_t i = 0; i < body.size(); ++i) {
            if (body[i] == ',') body[i] = ' ';
        }
        std::istringstream ss(body);
        float v = 0.0f;
        while (ss >> v) {
            out.push_back(v);
        }
        return out;
    }

    // "Feature Range: (0, 1)" 에서 두 값을 뽑는다.
    bool parseRange(const std::string& line, float& lo, float& hi) {
        const std::size_t lb = line.find('(');
        const std::size_t rb = line.rfind(')');
        if (lb == std::string::npos || rb == std::string::npos || rb < lb) {
            return false;
        }
        std::string body = line.substr(lb + 1, rb - lb - 1);
        for (std::size_t i = 0; i < body.size(); ++i) {
            if (body[i] == ',') body[i] = ' ';
        }
        std::istringstream ss(body);
        return static_cast<bool>(ss >> lo >> hi);
    }

    MlpScaler loadMlpScaler(const std::string& path, const char* tag, int input_dim) {
        MlpScaler s;

        std::ifstream f(path.c_str());
        if (!f.is_open()) {
            std::cerr << "[" << tag << "] scaler 파일을 열 수 없습니다: " << path << std::endl;
            return s;
        }

        // 파일 앞부분의 "Validated Envelope" 는 원시 단위 학습 범위라 스케일러가 아니다.
        // 두 스케일러 블록 모두 "Min Values:" 라는 같은 키를 쓰므로 섹션을 추적해야 한다.
        int section = 0;  // 1 = InputScaler, 2 = OutputScaler
        std::string line;
        while (std::getline(f, line)) {
            if (line.find("InputScaler Information") != std::string::npos) {
                section = 1;
                continue;
            }
            if (line.find("OutputScaler Information") != std::string::npos) {
                section = 2;
                continue;
            }
            if (section == 0) {
                continue;
            }

            if (line.rfind("Min Values:", 0) == 0) {
                (section == 1 ? s.in_min : s.out_min) = parseFloatList(line);
            } else if (line.rfind("Max Values:", 0) == 0) {
                (section == 1 ? s.in_max : s.out_max) = parseFloatList(line);
            } else if (line.rfind("Feature Range:", 0) == 0) {
                parseRange(line,
                           section == 1 ? s.in_fr_lo : s.out_fr_lo,
                           section == 1 ? s.in_fr_hi : s.out_fr_hi);
            }
        }

        if (static_cast<int>(s.in_min.size())  != input_dim ||
            static_cast<int>(s.in_max.size())  != input_dim ||
            static_cast<int>(s.out_min.size()) != MLP_OUTPUT_DIM ||
            static_cast<int>(s.out_max.size()) != MLP_OUTPUT_DIM) {
            std::cerr << "[" << tag << "] scaler 차원이 맞지 않습니다 ("
                      << path << "): input " << s.in_min.size() << "/" << s.in_max.size()
                      << " (기대 " << input_dim << "), output "
                      << s.out_min.size() << "/" << s.out_max.size()
                      << " (기대 " << MLP_OUTPUT_DIM << ")" << std::endl;
            return s;
        }
        if (s.in_fr_hi == s.in_fr_lo || s.out_fr_hi == s.out_fr_lo) {
            std::cerr << "[" << tag << "] scaler 의 Feature Range 폭이 0 입니다: " << path << std::endl;
            return s;
        }

        // 학습측 InputScaler 와 동일하게, 폭이 0인 열은 1.0 으로 두어 0 나눗셈을 피한다.
        for (int i = 0; i < input_dim; ++i) {
            if (s.in_max[i] == s.in_min[i]) {
                std::cerr << "[" << tag << "] 입력 " << i
                          << "번 열의 스케일 폭이 0 이라 1.0 으로 대체합니다." << std::endl;
                s.in_max[i] = s.in_min[i] + 1.0f;
            }
        }

        s.valid = true;
        return s;
    }

    // 모델 하나 = TF SavedModel + 짝이 되는 scaler.
    // cppflow::model 은 경로가 잘못되면 생성자에서 예외를 던진다. 전역 객체로 두면 그대로
    // 노드가 죽으므로, 여기서 잡아 "이 모델만 비활성" 으로 처리한다 (경로를 자주 바꾸게 되므로).
    class MlpEngine {
    public:
        MlpEngine(const char* tag, const char* model_path, const char* scaler_path, int input_dim)
            : tag_(tag), input_dim_(input_dim) {
            scaler_ = loadMlpScaler(scaler_path, tag, input_dim);
            if (!scaler_.valid) {
                std::cerr << "[" << tag_ << "] scaler 로드 실패 - 이 모델은 비활성화됩니다." << std::endl;
                return;
            }
            try {
                model_.reset(new cppflow::model(model_path));
            } catch (const std::exception& e) {
                std::cerr << "[" << tag_ << "] 모델 로드 실패 (" << model_path << "): "
                          << e.what() << " - 이 모델은 비활성화됩니다." << std::endl;
                return;
            }
            valid_ = true;
            std::cout << "[" << tag_ << "] model  : " << model_path << std::endl;
            std::cout << "[" << tag_ << "] scaler : " << scaler_path
                      << " (입력 " << input_dim_ << ", 출력 " << MLP_OUTPUT_DIM
                      << ") 로드 완료" << std::endl;
        }

        bool valid() const { return valid_; }
        int inputDim() const { return input_dim_; }

        // raw_input 은 학습 때와 같은 순서/단위의 원시 입력. 스케일링과 역스케일링은 여기서 한다.
        // 실패하면 (그럴듯하지만 틀린 값 대신) 0을 돌려준다. 로그에서 바로 눈에 띈다.
        Eigen::Matrix<float, 6, 1> infer(const std::vector<float>& raw_input) const {
            Eigen::Matrix<float, 6, 1> out = Eigen::Matrix<float, 6, 1>::Zero();
            if (!valid_ || static_cast<int>(raw_input.size()) != input_dim_) {
                return out;
            }

            std::vector<float> scaled(input_dim_, 0.0f);
            for (int i = 0; i < input_dim_; ++i) {
                const float unit = (raw_input[i] - scaler_.in_min[i]) / (scaler_.in_max[i] - scaler_.in_min[i]);
                scaled[i] = unit * (scaler_.in_fr_hi - scaler_.in_fr_lo) + scaler_.in_fr_lo;
            }

            std::vector<int64_t> shape = {1, static_cast<int64_t>(input_dim_)};
            std::vector<float> raw_out;
            try {
                raw_out = (*model_)(cppflow::tensor(scaled, shape)).get_data<float>();
            } catch (const std::exception& e) {
                std::cerr << "[" << tag_ << "] 추론 실패: " << e.what() << std::endl;
                return out;
            }
            if (static_cast<int>(raw_out.size()) < MLP_OUTPUT_DIM) {
                std::cerr << "[" << tag_ << "] 출력 차원이 " << raw_out.size()
                          << " 로 기대(" << MLP_OUTPUT_DIM << ")와 다릅니다." << std::endl;
                return out;
            }

            for (int i = 0; i < MLP_OUTPUT_DIM; ++i) {
                const float unit = (raw_out[i] - scaler_.out_fr_lo) / (scaler_.out_fr_hi - scaler_.out_fr_lo);
                out(i) = unit * (scaler_.out_max[i] - scaler_.out_min[i]) + scaler_.out_min[i];
            }
            return out;
        }

    private:
        const char* tag_;
        int input_dim_;
        MlpScaler scaler_;
        std::unique_ptr<cppflow::model> model_;
        bool valid_ = false;
    };

    // 첫 호출 때 한 번만 로드한다 (전역 초기화 순서 문제를 피하려고 함수 지역 static 사용).
    const MlpEngine& mlpEngine1() {
        static const MlpEngine e("MLP1", MLP1_MODEL_PATH, MLP1_SCALER_PATH, MLP1_INPUT_DIM);
        return e;
    }

    const MlpEngine& mlpEngine2() {
        static const MlpEngine e("MLP2", MLP2_MODEL_PATH, MLP2_SCALER_PATH, MLP2_INPUT_DIM);
        return e;
    }

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


// control_loop.cpp 의 전역. 모션이 바뀔 때 DBIC 쪽 lag 버퍼를 초기화하는 데 쓴다.
extern int operator_call_count_;

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


    
    // 모델 1 : 입력 12열 = q1..q6 [deg] + trq_ext1..trq_ext6 [Nm]
    Eigen::Matrix<float, 6, 1> PBIC::F_estimate(const Eigen::Matrix<float, 1, 6>& q_deg,
                                                const Eigen::Matrix<float, 1, 6>& trq_ext)
    {
        std::vector<float> in;
        in.reserve(MLP1_INPUT_DIM);
        for (int i = 0; i < 6; ++i) in.push_back(q_deg(0, i));
        for (int i = 0; i < 6; ++i) in.push_back(trq_ext(0, i));

        return mlpEngine1().infer(in);
    }

    // 모델 2 : 입력 36열. 순서는 학습 스크립트의 np.concatenate 순서와 같아야 한다
    // (파일 상단 MLP2 주석 참조). 순서가 하나라도 어긋나면 조용히 틀린 힘이 나온다.
    Eigen::Matrix<float, 6, 1> PBIC::F_estimate2(const Eigen::Matrix<float, 1, 6>& q_deg,
                                                 const Eigen::Matrix<float, 1, 6>& task_p,
                                                 const Eigen::Matrix<float, 1, 6>& trq_ext,
                                                 const Eigen::Matrix<float, 1, 6>& trq_ext_lag1,
                                                 const Eigen::Matrix<float, 1, 6>& trq_ext_lag3,
                                                 const Eigen::Matrix<float, 1, 6>& trq_ext_lag5)
    {
        std::vector<float> in;
        in.reserve(MLP2_INPUT_DIM);

        const Eigen::Matrix<float, 1, 6>* blocks[6] = {
            &q_deg, &task_p, &trq_ext, &trq_ext_lag1, &trq_ext_lag3, &trq_ext_lag5
        };
        for (int b = 0; b < 6; ++b) {
            for (int i = 0; i < 6; ++i) {
                in.push_back((*blocks[b])(0, i));
            }
        }

        return mlpEngine2().infer(in);
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
        
        // yaml에 키가 없으면 기존 하드코딩 값을 그대로 쓴다 (구 config 호환)
        auto loadOptFloat = [&config](const char* key, float& dst) {
            if (config[key]) dst = config[key].as<float>();
        };
        auto loadOptArray6 = [&config](const char* key, std::array<float, 6>& dst) {
            if (config[key]) dst = config[key].as<std::array<float, 6>>();
        };

        // Axis order: X, Y, Z, Rx, Ry, Rz.
        // K_gains[i] = imp_k_scale[i] * imp_k
        // M_gains[i] = imp_m_scale[i] * imp_m / 1000
        // B_gains[i] = 2 * zeta[i] * sqrt(K*M)   (zeta = 1이 임계감쇠)
        std::array<float, 6> imp_k_scale =
            {5.0f, 5.0f, 1.0f, 8.0f, 8.0f, 10.0f};
        std::array<float, 6> imp_m_scale =
            {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
        std::array<float, 6> damping_ratios =
            {8.0f, 8.0f, 8.0f, 8.0f, 8.0f, 8.0f};

        loadOptArray6("imp_k_scale", imp_k_scale);
        loadOptArray6("imp_m_scale", imp_m_scale);
        loadOptArray6("imp_damping_ratio", damping_ratios);

        for (int i = 0; i < 6; ++i)
        {
            K_gains[i] = imp_k_scale[i] * imp_k;
            M_gains[i] = imp_m_scale[i] * imp_m / 1000.0f;
            B_gains[i] = 2.0f * damping_ratios[i]
                * std::sqrt(K_gains[i] * M_gains[i]);
        }

        std::cout << "[PBIC Impedance] K=[";
        for (int i = 0; i < 6; ++i) std::cout << K_gains[i] << (i < 5 ? ", " : "]");
        std::cout << " M=[";
        for (int i = 0; i < 6; ++i) std::cout << M_gains[i] << (i < 5 ? ", " : "]");
        std::cout << " zeta=[";
        for (int i = 0; i < 6; ++i) std::cout << damping_ratios[i] << (i < 5 ? ", " : "]");
        std::cout << std::endl;

        // 0804 Fz adaptive z-reference shaping

        if (config["fz_adapt_enable"]) {
            fz_adapt_enable_ = config["fz_adapt_enable"].as<bool>();
        }
        loadOptFloat("fz_target", fz_target_);
        loadOptFloat("fz_kp", fz_kp_);
        loadOptFloat("fz_ki", fz_ki_);
        loadOptFloat("fz_kd", fz_kd_);
        // 구 키 이름 호환: fz_adapt_gain은 적분 게인이었다
        loadOptFloat("fz_adapt_gain", fz_ki_);
        loadOptFloat("fz_adapt_rate", fz_adapt_rate_);
        loadOptFloat("fz_adapt_cutoff_hz", fz_adapt_cutoff_hz_);
        loadOptFloat("fz_d_cutoff_hz", fz_d_cutoff_hz_);
        loadOptFloat("fz_print_hz", fz_print_hz_);
        loadOptFloat("fz_stall_limit", fz_stall_limit_);

        std::cout << "[PBIC Fz-adapt] enable=" << fz_adapt_enable_
                  << " target=" << fz_target_ << " N"
                  << " | Kp=" << fz_kp_ << " mm/N"
                  << " Ki=" << fz_ki_ << " mm/(s*N)"
                  << " Kd=" << fz_kd_ << " mm*s/N"
                  << " | slew=" << fz_adapt_rate_ << " mm/s"
                  << " (dz 제한 없음, 접촉 게이팅 없음)"
                  << std::endl;
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
    bool PBIC::preselectPBICInitialSolution(
        const LPRT_OUTPUT_DATA_LIST robot_state,
        int& sol_space)
    {
        float x_d[6] = {
            imp.p_m(0), imp.p_m(1), imp.p_m(2),
            imp.pos_m(3), imp.pos_m(4), imp.pos_m(5)
        };

        float current_joint[6] = {0,};
        std::memcpy(current_joint, robot_state->actual_joint_position, sizeof(float) * 6);

        auto wrapDeltaDeg = [](float delta) {
            while (delta > 180.0f) delta -= 360.0f;
            while (delta < -180.0f) delta += 360.0f;
            return delta;
        };

        bool found = false;
        int best_sol = sol_space;
        float best_des[6] = {0,};
        float best_cost = 1.0e30f;

        for (int cand_sol = 0; cand_sol < 8; ++cand_sol) {
            LPINVERSE_KINEMATIC_RESPONSE cand =
                Drfl_.ikin(x_d, cand_sol, COORDINATE_SYSTEM_WORLD, 1);

            if (cand == nullptr || cand->_iStatus != 0) {
                continue;
            }

            float cand_cost = 0.0f;
            bool valid = true;

            for (int i = 0; i < 6; ++i) {
                float q = cand->_fTargetPos[i];
                if (!std::isfinite(q)) {
                    valid = false;
                    break;
                }

                float d_prev = wrapDeltaDeg(q - previous_joint_command[i]);
                float d_curr = wrapDeltaDeg(q - current_joint[i]);

                cand_cost += d_prev * d_prev + 0.05f * d_curr * d_curr;
            }

            if (!valid) continue;

            if (cand_cost < best_cost) {
                found = true;
                best_cost = cand_cost;
                best_sol = cand_sol;

                for (int i = 0; i < 6; ++i) {
                    best_des[i] = cand->_fTargetPos[i];
                }
            }
        }

        if (!found) {
            return false;
        }

        sol_space = best_sol;

        for (int i = 0; i < 6; ++i) {
            previous_joint_command[i] = best_des[i];
        }

        pbic_initial_ik_preselected_ = true;

        std::cout << "[PBIC IK] preselected sol_space = "
                << sol_space << std::endl;

        return true;
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

        // 260806 예전에는 ControlGeneratorDBIC 안의 static 이라 여기서 손댈 수
        // 없었고, 그래서 모션이 바뀌어도 직전 값이 남아 첫 샘플에 킥이 생겼다.
        edot_filter_init_dbic_ = false;
        edot_prev_limited_dbic_.setZero();
        edot_filt_state_dbic_.setZero();

        ft_valid_count_dbic_ = 0;
        ft_ready_dbic_ = false;

        fext_filter_init_dbic_ = false;
        Fext_prev_dbic_.setZero();
        Fext_filt_dbic_2_.setZero();

        trq_lag_dbic_.reset();

        dbic_rampup_elapsed_ = 0.0f;

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
        edot_alpha << 0.1f, 0.1f, 0.1f,
                    0.1f, 0.1f, 0.1f;

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

        if (!edot_filter_init_dbic_) {
            edot_prev_limited_dbic_ = edot_raw;
            edot_filt_state_dbic_   = edot_raw;
            edot_filter_init_dbic_  = true;
        }

        for (int i = 0; i < 6; ++i) {
            float delta = edot_raw(i) - edot_prev_limited_dbic_(i);

            if (delta >  edot_delta_limit(i)) delta =  edot_delta_limit(i);
            if (delta < -edot_delta_limit(i)) delta = -edot_delta_limit(i);

            edot_limited(i) = edot_prev_limited_dbic_(i) + delta;

            if (std::fabs(edot_limited(i)) < edot_deadband(i)) {
                edot_limited(i) = 0.0f;
            }

            edot_filt_state_dbic_(i) =
                edot_alpha(i) * edot_limited(i) +
                (1.0f - edot_alpha(i)) * edot_filt_state_dbic_(i);

            edot(i) = edot_filt_state_dbic_(i);
        }

        edot_prev_limited_dbic_ = edot_limited;            

 

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


        const bool ft_sensor_valid = Fft_raw.cwiseAbs().maxCoeff() > 1.0e-6f;
        if (ft_sensor_valid) {
            ++ft_valid_count_dbic_;
        } else {
            ft_valid_count_dbic_ = 0;
            ft_ready_dbic_ = false;
        }

        if (ft_valid_count_dbic_ >= 5) {
            ft_ready_dbic_ = true;
        }

        Eigen::Matrix<float, 6, 1> Fext_raw =
            Eigen::Matrix<float, 6, 1>::Zero();

        // 외력 소스 선택 (impedance_controller.h 의 IMPEDANCE_FORCE_SOURCE).
        // PBIC 의 MotionGenerator 와 같은 규칙을 DBIC 경로에도 적용한다.
        // SENSOR 모드는 FT 센서가 5샘플 이상 유효할 때까지 0 을 유지한다(기존 동작).
        // MLP 모드는 센서를 쓰지 않으므로 그 워밍업이 필요 없다.
        if (IMPEDANCE_FORCE_SOURCE == ImpedanceForceSource::SENSOR) {
            if (ft_ready_dbic_) {
                Fext_raw = Fft_raw;
            }
        } else {
            Eigen::Map<const Eigen::Matrix<float, 6, 1>>
                tau_ext_dbic(robot_state->external_joint_torque);

            Eigen::Matrix<float, 1, 6> q_in;
            Eigen::Matrix<float, 1, 6> trq_in;
            for (int i = 0; i < 6; ++i) {
                q_in(0, i)   = robot_state->actual_joint_position[i];   // [deg]
                trq_in(0, i) = tau_ext_dbic(i);                         // [Nm]
            }

            if (IMPEDANCE_FORCE_SOURCE == ImpedanceForceSource::MLP1) {
                Fext_raw = F_estimate(q_in, trq_in);
            } else {
                // MLP2 는 과거 토크가 필요하다. 이 스레드 전용 이력.
                // 모션마다의 초기화는 resetDBICControllerState() 가 한다.
                trq_lag_dbic_.push(trq_in);

                Eigen::Matrix<float, 1, 6> task_in;
                LPROBOT_POSE fk = Drfl_.fkin(robot_state->actual_joint_position,
                                             COORDINATE_SYSTEM_WORLD);
                for (int i = 0; i < 6; ++i) {
                    task_in(0, i) = fk->_fPosition[i];
                }

                Fext_raw = F_estimate2(q_in,
                                       task_in,
                                       trq_lag_dbic_.get(0),
                                       trq_lag_dbic_.get(1),
                                       trq_lag_dbic_.get(3),
                                       trq_lag_dbic_.get(5));
            }
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

        if (!ft_ready_dbic_) {
            fext_filter_init_dbic_ = false;
            Fext_prev_dbic_.setZero();
            Fext_filt_dbic_2_.setZero();
        }

        Eigen::Matrix<float, 6, 1> Fext = Fext_raw;

        // 축별 절대 제한값 [Fx,Fy,Fz,Mx,My,Mz]
        // 시작은 보수적으로 두고 나중에 조정
        // Eigen::Matrix<float, 6, 1> fext_abs_limit;
        // fext_abs_limit << 400.0f, 400.0f, 200.0f, 50.0f, 50.0f, 50.0f;

        // // 변화율 제한값 [N/s, Nm/s]
        // const float force_slew_rate  = 2000.0f;  // translational
        // const float torque_slew_rate = 200.0f;   // rotational

        // Eigen::Matrix<float, 6, 1> fext_delta_limit;
        // fext_delta_limit << force_slew_rate * dt,
        //                     force_slew_rate * dt,
        //                     force_slew_rate * dt,
        //                     torque_slew_rate * dt,
        //                     torque_slew_rate * dt,
        //                     torque_slew_rate * dt;

        // if (!fext_filter_init_dbic_) {
        //     Fext_prev_dbic_ = Fext;
        //     Fext_filt_dbic_2_ = Fext;
        //     fext_filter_init_dbic_ = true;
        // }

        // // 1) 프레임 간 급격한 점프 제한
        // for (int i = 0; i < 6; ++i) {
        //     float delta = Fext(i) - Fext_prev_dbic_(i);

        //     if (delta >  fext_delta_limit(i)) delta =  fext_delta_limit(i);
        //     if (delta < -fext_delta_limit(i)) delta = -fext_delta_limit(i);

        //     Fext(i) = Fext_prev_dbic_(i) + delta;

        //     // 2) 절대 크기 제한
        //     if (Fext(i) >  fext_abs_limit(i)) Fext(i) =  fext_abs_limit(i);
        //     if (Fext(i) < -fext_abs_limit(i)) Fext(i) = -fext_abs_limit(i);
        // }

        // 3) 저역통과필터
        const float alpha_fext = 1.00f; // 작을수록 더 부드러움
        Fext_filt_dbic_2_ = alpha_fext * Fext + (1.0f - alpha_fext) * Fext_filt_dbic_2_;

        Fext_prev_dbic_ = Fext;

        //
        Eigen::Matrix<float, 6, 1> Fmass = Md_ * e2dot;
        Eigen::Matrix<float, 6, 1> Fspring = Kd_ * e;
        Eigen::Matrix<float, 6, 1> Fdamp   = Bd_ * edot;
        Eigen::Matrix<float, 6, 1> Fdbic   = Fspring + Fdamp - Fe_paper;
        Eigen::Matrix<float, 6, 1> Fimp   = Fspring + Fdamp + Fmass;

        // ------------------------------------------------------------------
        // 260806 기동 램프업
        //
        // 모션 시작 직후 남아 있는 과도(속도 추정 초기값, 필터 워밍업 등)가
        // 그대로 큰 토크가 되는 것을 막는다. 임피던스 힘에만 걸고 중력/코리올리
        // 보상에는 걸지 않는다 (거기까지 줄이면 로봇이 주저앉는다).
        //
        // 260806/1219 에서는 첫 샘플의 err_dot 39.8 mm/s 가 곧바로 132 N 을
        // 만들었고 4ms 뒤 559 N 이 됐다. 램프 구간에서는 그 힘이 서서히 실린다.
        // ------------------------------------------------------------------
        {
            constexpr float kDbicRampupSec = 0.3f;

            if (dbic_rampup_elapsed_ < kDbicRampupSec) {
                dbic_rampup_elapsed_ += dt;

                float ramp = dbic_rampup_elapsed_ / kDbicRampupSec;
                ramp = std::min(1.0f, std::max(0.0f, ramp));

                // 시작/끝에서 기울기가 0인 smoothstep. 계단이 생기지 않는다.
                const float ramp_s = ramp * ramp * (3.0f - 2.0f * ramp);

                Fimp *= ramp_s;
            }
        }
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
        s.J.transpose()*(Fimp+Fext_filt_dbic_2_-F_offset)
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
            F.Fext[i] = Fext_filt_dbic_2_(i);
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
        // 현재 filtered derivative를 다음 제어주기의 이전 값으로 저장
        prev.derrPrev = derr;
        
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
        pbic_ik_jump_log = 0.0f;

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

        // 0804 Fz adaptive z-reference shaping state
        static float fz_ref_offset = 0.0f;       // dz [mm]
        static float fz_ref_offset_rate = 0.0f;  // dz_dot 실측 [mm/s], 로그용
        static float fz_ref_offset_ff = 0.0f;    // v_d로 나가는 feedforward [mm/s]
        static float fz_adapt_filt = 0.0f;       // 힘 오차 판정용 |Fz| LPF [N]
        static bool  fz_adapt_filt_init = false;
        static float fz_integ = 0.0f;            // 적분항 [N*s]
        static float fz_err_prev = 0.0f;         // D항용 이전 오차 [N]
        static bool  fz_err_prev_valid = false;
        static float fz_error_dot = 0.0f;        // 필터링된 오차 변화율 [N/s]

        if (pbic_fext_motion_id != operator_call_count_) {
            pbic_fext_motion_id = operator_call_count_;
            pbic_fext_filter_init = false;
            Fext_prev.setZero();
            Fext_filt.setZero();
            singularity_counter = 0;
            zyz_ref_initialized = false;

            // 새 모션은 항상 nominal reference에서 시작해야 bumpless
            fz_ref_offset = 0.0f;
            fz_ref_offset_rate = 0.0f;
            fz_ref_offset_ff = 0.0f;
            fz_adapt_filt = 0.0f;
            fz_adapt_filt_init = false;
            fz_integ = 0.0f;
            fz_err_prev = 0.0f;
            fz_err_prev_valid = false;
            fz_error_dot = 0.0f;
        }

        auto ft_matched = sensor_data.getMatchedAFTWrench();

        Eigen::Matrix<float, 6, 1> Fft_raw;
        for (int i = 0; i < 6; ++i) {
            Fft_raw(i) = ft_matched[i];
        }

        Eigen::Matrix<float, 6, 1> Fext_raw =
            Eigen::Matrix<float, 6, 1>::Zero();

        // 외력 소스 선택 (impedance_controller.h 의 IMPEDANCE_FORCE_SOURCE).
        // SENSOR 모드에서는 아래 MLP 분기가 아예 실행되지 않으므로 RT 루프에 추론 부담이 없다.
        // 어느 모드든 F_mlp / F_mlp2 로그는 dataSaving() 스레드에서 따로 계속 기록된다.
        if (IMPEDANCE_FORCE_SOURCE == ImpedanceForceSource::SENSOR) {
            Fext_raw = Fft_raw;
        } else {
            Eigen::Matrix<float, 1, 6> q_in;
            Eigen::Matrix<float, 1, 6> trq_in;
            for (int i = 0; i < 6; ++i) {
                q_in(0, i)   = robot_state->actual_joint_position[i];   // [deg]
                trq_in(0, i) = tau_ext(i);                              // [Nm]
            }

            if (IMPEDANCE_FORCE_SOURCE == ImpedanceForceSource::MLP1) {
                Fext_raw = F_estimate(q_in, trq_in);
            } else {
                // MLP2 는 과거 토크가 필요하다. 이 스레드 전용 이력을 따로 들고 간다
                // (로깅 스레드의 버퍼와 섞이면 안 된다). 새 모션마다 초기화한다.
                static TorqueLagBuffer trq_lag_rt;
                static int trq_lag_rt_motion_id = -1;
                if (trq_lag_rt_motion_id != operator_call_count_) {
                    trq_lag_rt_motion_id = operator_call_count_;
                    trq_lag_rt.reset();
                }
                trq_lag_rt.push(trq_in);

                // task position 은 로깅 스레드의 task_position.txt 와 같은 방식으로 만든다
                // (PBIC 경로 = fkin(actual_joint_position, WORLD), mm + deg).
                Eigen::Matrix<float, 1, 6> task_in;
                LPROBOT_POSE fk = Drfl_.fkin(robot_state->actual_joint_position,
                                             COORDINATE_SYSTEM_WORLD);
                for (int i = 0; i < 6; ++i) {
                    task_in(0, i) = fk->_fPosition[i];
                }

                Fext_raw = F_estimate2(q_in,
                                       task_in,
                                       trq_lag_rt.get(0),
                                       trq_lag_rt.get(1),
                                       trq_lag_rt.get(3),
                                       trq_lag_rt.get(5));
            }
        }

// 0729 PBIC 외력 1차 LPF
        // 0729 matched Ty 부호 반전
        Fext_raw(4) = Fext_raw(4);

        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kFextCutoffHz = 5.0f;

        const float alpha_fext =
            1.0f - std::exp(-2.0f * kPi * kFextCutoffHz * dt);

        if (!pbic_fext_filter_init) {
            // 첫 샘플을 그대로 초기값으로 사용해서 시작 충격 방지
            Fext_filt = Fext_raw;
            pbic_fext_filter_init = true;
        } else {
            Fext_filt += alpha_fext * (Fext_raw - Fext_filt);
        }

        F_ext = Fext_filt;
        // F_ext[3] = 0.0;
        // F_ext[4] = 0.0;
        // F_ext[5] = 0.0;
        // // F_ext = Fext_raw;
//
        // ------------------------------------------------------------
        // 0804 Fz adaptive z-reference shaping
        //
        // 들어온 z reference가 무엇이든, |Fz|를 fz_target_에 유지하는
        // 것만 목표로 한다. 목표에서 벗어난 양과 방향에 비례한 속도로
        // z reference를 움직이고, 상한 속도로 saturate.
        //
        //   |Fz| > target -> dz를 + 로 (z 올림, 접촉력 약해짐)
        //   |Fz| < target -> dz를 - 로 (z 내림, 접촉력 세짐)
        //
        // 밴드(deadband)를 쓰지 않는 이유: 밴드 안에서는 보정이 멈춰서
        // 루프가 열린다. 표면이 움직여도 밴드 끝에 닿을 때까지 방치하다
        // 뒤늦게 반응하게 되고, 260804/1601에서 밴드 진입 직후 dz가
        // 멈추면서 힘이 5.9 -> 1.2 N으로 흘러나간 게 그 현상이다.
        // setpoint 하나면 항상 닫힌 루프다.
        //
        // dz 크기 제한도, 접촉 게이팅도 두지 않는다. 비접촉이면
        // |Fz| < target 이므로 접촉을 찾을 때까지 fz_adapt_rate_ 속도로
        // 계속 내려간다 (force-seeking approach).
        //
        // 비례 대상이 dz(위치)가 아니라 dz_dot(속도)인 이유:
        // dz = k*e 로 하면 유효 강성이 K_z/(1+K_z*k)로 바뀔 뿐이라
        // K를 낮춘 것과 같아지고, P droop 때문에 밴드 안으로 못 들어온다.
        // 속도에 비례시켜야 누적이 생겨서 표면 위치와 무관하게
        // 힘을 밴드에 넣을 수 있다.
        //
        // saturate가 필요한 이유: 목표 근처에서는 속도가 작아져서
        // 표면이 움직이면 v_surface/gain 만큼 뒤처진다. gain을 키우고
        // 상한으로 자르면 목표 근처에서만 부드럽고 멀리서는 최대 속도가 된다.
        // ------------------------------------------------------------
        {
            const float fz_mag = std::fabs(F_ext(2));

            if (!fz_adapt_filt_init) {
                fz_adapt_filt = fz_mag;
                fz_adapt_filt_init = true;
            } else {
                const float alpha_fz =
                    1.0f - std::exp(-2.0f * kPi * fz_adapt_cutoff_hz_ * dt);
                fz_adapt_filt += alpha_fz * (fz_mag - fz_adapt_filt);
            }

            const float fz_error = fz_adapt_filt - fz_target_;

            // D항 입력: 힘 오차의 변화율. 미분은 노이즈를 키우므로
            // fz_d_cutoff_hz_로 한 번 더 눌러서 쓴다.
            float fz_error_dot_raw = 0.0f;
            if (fz_err_prev_valid) {
                fz_error_dot_raw = (fz_error - fz_err_prev) / dt;
            }
            fz_err_prev = fz_error;
            fz_err_prev_valid = true;

            const float alpha_d =
                1.0f - std::exp(-2.0f * kPi * fz_d_cutoff_hz_ * dt);
            fz_error_dot += alpha_d * (fz_error_dot_raw - fz_error_dot);

            // 로봇이 명령을 실행하지 못하는 상태(보호정지/서보오프)에서는
            // 힘이 변하지 않으므로 적분기가 무한정 감긴다. 1732에서 imp_z가
            // 실제 TCP z보다 421 mm 앞서 나갔고, 그 상태로 정지를 풀면
            // 로봇이 그만큼 튄다. 모델과 실제가 벌어지면 적분을 멈춘다.
            const float fz_model_gap =
                std::fabs(imp.p_m(2) - s.p(2) * 1000.0f);
            const bool fz_stalled =
                (fz_stall_limit_ > 0.0f) && (fz_model_gap > fz_stall_limit_);

            if (fz_stalled) {
                static int fz_stall_warn = 0;
                if ((fz_stall_warn++ % 200) == 0) {
                    ROS_WARN("[Fz-adapt] STALLED: imp_z - actual_z = %.1f mm > %.1f mm. "
                             "Robot is not following the command; freezing adaptation.",
                             fz_model_gap, fz_stall_limit_);
                }
            }

            // 위치정렬 구간에서는 dz를 "물러나는 방향으로만" 움직인다.
            //
            //   힘이 약함 -> 정지. 접촉을 찾겠다고 내려가지 않는다.
            //                (260804/1732: 자유공간에서 40mm/s로 내리꽂아 충돌)
            //   힘이 셈   -> 올라간다. 눌리면 무조건 물러날 수 있어야 한다.
            //                (260804/1839: TRAVEL에서 보정을 완전히 껐더니
            //                 nominal이 표면을 파고들어 110N까지 갔다)
            //
            // dz는 0으로 리셋하지 않고 유지해서, 다시 내려올 때
            // 직전에 학습한 표면 오프셋에서 재개하도록 한다.
            const bool fz_draw_segment = trajectory.draw_mode;

            if (fz_adapt_enable_ && !fz_stalled) {
                fz_integ += fz_error * dt;

                // PID 출력이 곧 z reference offset [mm]
                float dz_cmd = fz_kp_ * fz_error
                             + fz_ki_ * fz_integ
                             + fz_kd_ * fz_error_dot;

                // 위치정렬 구간: 물러나는 방향(dz 증가)만 통과시킨다.
                if (!fz_draw_segment && dz_cmd < fz_ref_offset) {
                    dz_cmd = fz_ref_offset;
                }

                // 안전용 slew limit. dz 크기 제한은 두지 않는다.
                const float dz_step_max = fz_adapt_rate_ * dt;
                if (dz_cmd > fz_ref_offset + dz_step_max) {
                    dz_cmd = fz_ref_offset + dz_step_max;
                } else if (dz_cmd < fz_ref_offset - dz_step_max) {
                    dz_cmd = fz_ref_offset - dz_step_max;
                }

                // anti-windup: 실제로 나간 dz에 맞춰 적분항을 역산해두면
                // slew에 걸린 동안 적분기가 부풀지 않는다.
                if (fz_ki_ > 1.0e-6f) {
                    fz_integ = (dz_cmd
                                - fz_kp_ * fz_error
                                - fz_kd_ * fz_error_dot) / fz_ki_;
                }

                fz_ref_offset_rate = (dz_cmd - fz_ref_offset) / dt;
                fz_ref_offset = dz_cmd;

                // v_d로 내보낼 feedforward 속도.
                // dz의 실제 미분(fz_ref_offset_rate)을 쓰면 P/D항이 힘 신호를
                // 미분한 값이라 노이즈가 그대로 실린다. 260804/1726에서
                // dz_dot std가 14.4 mm/s였고 그 중 P항 기여가 지배적이었다.
                // 적분항의 속도(Ki*e)만 매끄러우므로 이것만 feedforward한다.
                fz_ref_offset_ff = fz_ki_ * fz_error;
            } else {
                fz_ref_offset_rate = 0.0f;
                fz_ref_offset_ff = 0.0f;
            }

            // offset을 z reference에 반영.
            // v_d까지 같이 밀어줘야 임피던스 모델이 B(v_d - v) 때문에
            // offset 이동을 외란으로 되받지 않는다.
            const float z_ref_nominal = p_d(2);
            p_d(2) += fz_ref_offset;
            v_d(2) += fz_ref_offset_ff;

            pbic_fz_adapt_log_[0] = fz_ref_offset;
            pbic_fz_adapt_log_[1] = fz_ref_offset_rate;
            pbic_fz_adapt_log_[2] = fz_adapt_filt;
            pbic_fz_adapt_log_[3] = fz_error;       // |Fz| - target [N]
            pbic_fz_adapt_log_[4] = z_ref_nominal;  // 원래 들어온 z reference
            pbic_fz_adapt_log_[5] = p_d(2);         // 실제로 제어기에 들어간 z reference

            // ------------------------------------------------------------
            // 터미널 실시간 출력. fz_print_hz_ = 0 이면 끈다.
            // RT loop에서 매 주기 찍으면 밀리므로 주기를 낮춰서 찍는다.
            // ------------------------------------------------------------
            if (fz_print_hz_ > 0.0f) {
                static float fz_print_acc = 0.0f;
                fz_print_acc += dt;

                if (fz_print_acc >= 1.0f / fz_print_hz_) {
                    fz_print_acc = 0.0f;

                    const char* state =
                        fz_stalled ? "STALL (동결)"
                      : (fz_ref_offset_rate > 0.01f)
                            ? (fz_draw_segment ? "UP   (힘 줄임)"
                                               : "UP   (TRAVEL 후퇴)")
                      : (fz_ref_offset_rate < -0.01f) ? "DOWN (힘 늘림)"
                      : (fz_draw_segment ? "HOLD" : "TRAVEL(후퇴만 허용)");

                    std::printf(
                        "[Fz-adapt] |Fz|=%6.2f N (target %.1f, err %+6.2f) | "
                        "P=%+7.3f I=%+8.3f D=%+7.3f -> dz=%+8.3f mm (%+7.2f mm/s) | "
                        "z_ref %8.3f -> %8.3f | %s\n",
                        fz_adapt_filt, fz_target_, fz_error,
                        fz_kp_ * fz_error,
                        fz_ki_ * fz_integ,
                        fz_kd_ * fz_error_dot,
                        fz_ref_offset, fz_ref_offset_rate,
                        z_ref_nominal, p_d(2), state);
                    std::fflush(stdout);
                }
            }

            // ref_task_*_log는 위에서 nominal 기준으로 이미 채워졌으므로
            // 실제로 임피던스에 들어간 z reference로 갱신한다.
            // nominal은 ref_z - pbic_fz_adapt[0]으로 복원 가능하다.
            g_ref_task_pose_log[2].store(p_d(2), std::memory_order_relaxed);
            g_ref_task_vel_log[2].store(v_d(2), std::memory_order_relaxed);
        }

        //FT센서사용으로 비활성화
        Eigen::Matrix<float, 6, 6> J_inv = dampedPseudoInverse(s.J, 5e-3f);

        Eigen::Matrix<float, 6, 1> Fext_joint_raw =
            1.0f * J_inv.transpose() * tau_ext;

        Eigen::Matrix<float, 6, 1> Fext_joint_log =
            Fext_joint_raw - F_offset;
        

        //joint torque sensor 를 입력값으로 쓰고싶을떄

        // F_ext = Fext_joint_log;


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
//0609
        // if (count_motion == 0) {
        //     memcpy(previous_joint_command,
        //         robot_state->actual_joint_position,
        //         sizeof(float) * 6);
        //     //해 전구간 탐색
        //     sol_space = static_cast<int>(robot_state->solution_space);
        //     //해 솔루션 2고정
        //     // sol_space = 2;
        //     std::cout << "[PBIC IK] initial robot_state solution_space = "
        //   << sol_space << std::endl;

        //     prev_zyz_deg[0] = robot_state->actual_flange_position[3];
        //     prev_zyz_deg[1] = robot_state->actual_flange_position[4];
        //     prev_zyz_deg[2] = robot_state->actual_flange_position[5];
        //     zyz_ref_initialized = true;
        // }
        if (count_motion == 0) {
            if (!pbic_initial_ik_preselected_) {
                std::memcpy(previous_joint_command,
                            robot_state->actual_joint_position,
                            sizeof(float) * 6);

                sol_space = static_cast<int>(robot_state->solution_space);

                std::cout << "[PBIC IK] initial robot_state solution_space = "
                        << sol_space << std::endl;
            }

            // actual이 아니라 현재 imp 기준으로 ZYZ unwrap 시작
            prev_zyz_deg[0] = imp.pos_m(3);
            prev_zyz_deg[1] = imp.pos_m(4);
            prev_zyz_deg[2] = imp.pos_m(5);
            zyz_ref_initialized = true;
        }
        //

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
//0609
        // bool need_full_scan = (count_motion == 0);
//
        // 해 2고정
        // bool need_full_scan = false;
        bool need_full_scan = (count_motion == 0) && !pbic_initial_ik_preselected_;

        if (!need_full_scan) {
            float cand_des[NUMBER_OF_JOINT] = {0,};
            float cand_cost = 0.0f;
            float cand_max_delta_deg = 0.0f;
//0609
            // if (evaluateIkCandidate(sol_space,
            //                         cand_des,
            //                         cand_cost,
            //                         cand_max_delta_deg) &&
            //     cand_max_delta_deg <= kFastAcceptMaxDeltaDeg) {
            //     found_solution = true;
            //     best_cost = cand_cost;
            //     best_sol_space = sol_space;

            //     for (int i = 0; i < 6; ++i) {
            //         best_des[i] = cand_des[i];
            //     }
            // } else {
            //     need_full_scan = true;
            // }
    
            if (evaluateIkCandidate(sol_space,
                                    cand_des,
                                    cand_cost,
                                    cand_max_delta_deg)) {
                found_solution = true;
                best_cost = cand_cost;
                best_sol_space = sol_space;

                for (int i = 0; i < 6; ++i) {
                    best_des[i] = cand_des[i];
                }

                if (cand_max_delta_deg > kFastAcceptMaxDeltaDeg) {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "PBIC IK current sol_space delta %.3f deg exceeds fast threshold %.3f deg.",
                        cand_max_delta_deg,
                        kFastAcceptMaxDeltaDeg
                    );
                }
            } else {
                found_solution = false;
            }
//
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
        pbic_ik_jump_log = branch_jump ? 1.0f : 0.0f;

//0609
        // if (branch_jump) {
        //     std::cout << "IK branch jump at joint " << jump_joint
        //             << ", prev_cmd : " << previous_joint_command[jump_joint]
        //             << ", ik_cmd : " << best_des[jump_joint]
        //             << ", delta : " << jump_delta
        //             << ", candidate sol_space : " << best_sol_space
        //             << std::endl;

        //     ROS_WARN("IK branch jump detected, holding previous joint command.");

        //     for (int i = 0; i < 6; ++i) {
        //         des[i] = previous_joint_command[i];
        //     }

        if (branch_jump) {
            ROS_WARN_THROTTLE(
                1.0,
                "PBIC IK branch jump: joint=%d prev=%.3f ik=%.3f delta=%.3f sol=%d. Using differential IK fallback.",
                jump_joint,
                previous_joint_command[jump_joint],
                best_des[jump_joint],
                jump_delta,
                best_sol_space
            );

            Eigen::Matrix<float, 6, 1> xdot_imp =
                Eigen::Matrix<float, 6, 1>::Zero();

            // s.J는 qdot[rad/s] -> twist[m/s, rad/s] 기준이라고 보고 맞춤
            xdot_imp(0) = imp.v_m(0) * 1e-3f;  // mm/s -> m/s
            xdot_imp(1) = imp.v_m(1) * 1e-3f;
            xdot_imp(2) = imp.v_m(2) * 1e-3f;
            xdot_imp(3) = imp.w_m(0);          // rad/s
            xdot_imp(4) = imp.w_m(1);
            xdot_imp(5) = imp.w_m(2);

            Eigen::Matrix<float, 6, 6> J_inv_for_fallback =
                dampedPseudoInverse(s.J, 5e-3f);

            Eigen::Matrix<float, 6, 1> qdot_fallback_rad =
                J_inv_for_fallback * xdot_imp;

            const float max_step_deg = 0.5f;  // 4ms 기준 125 deg/s 제한. 필요하면 1.0으로 완화
            bool fallback_valid = true;

            for (int i = 0; i < 6; ++i) {
                float step_deg = qdot_fallback_rad(i) * RAD2DEG * dt;

                if (!std::isfinite(step_deg)) {
                    fallback_valid = false;
                    break;
                }

                if (step_deg >  max_step_deg) step_deg =  max_step_deg;
                if (step_deg < -max_step_deg) step_deg = -max_step_deg;

                des[i] = previous_joint_command[i] + step_deg;
            }

            if (!fallback_valid) {
                for (int i = 0; i < 6; ++i) {
                    des[i] = previous_joint_command[i];
                }
                singularity_counter++;
            } else {
                // fallback은 실패가 아니라 연속 적분으로 처리
                singularity_counter = 0;
            }
//
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
//0609
        pbic_initial_ik_preselected_ = false;
//
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
