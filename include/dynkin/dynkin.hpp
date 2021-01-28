
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <unsupported/Eigen/EulerAngles>

namespace Eigen {
    typedef Eigen::Matrix<double,6,1> Vector6d;
    typedef Eigen::Matrix<double,6,6> Matrix6d;
}

namespace dynkin {

    struct _Frame;
    typedef std::shared_ptr<_Frame> Frame;
    struct Transform;
    Transform transform(Frame, Frame);



    inline Eigen::Vector3d rotation_to_euler(const Eigen::Matrix3d& rotation){
        return rotation.eulerAngles(2,1,0).reverse();
    }

    inline Eigen::Matrix3d euler_to_rotation(const Eigen::Vector3d& attitude){
        return (
            Eigen::AngleAxisd(attitude(2), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(attitude(1), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(attitude(0), Eigen::Vector3d::UnitX())
            ).toRotationMatrix();
    }

    inline Eigen::Matrix3d eulerian(const Eigen::Vector3d& attitude){
        double fi = attitude(0);
        double theta = attitude(1);
        Eigen::Matrix3d out;
        out <<  1,      sin(fi)*cos(theta),     cos(fi)*tan(theta),
                0,      cos(fi),                -sin(fi),
                0,      sin(fi)/cos(theta),     cos(fi)/cos(theta);
        return out;
    }

    inline Eigen::Vector3d angular_velocity_to_deuler(
        const Eigen::Vector3d& attitude,
        const Eigen::Vector3d& angular_velocity
    ){
        return eulerian(attitude)*angular_velocity;
    }

    struct Transform{
        Eigen::Isometry3d HTM;

        Transform(const Eigen::Isometry3d& HTM):HTM(HTM){}

        Eigen::Vector3d apply_vector(const Eigen::Vector3d& vector){
            return this->HTM.linear()*vector;
        }

        Eigen::Vector3d apply_position(const Eigen::Vector3d& position){
            return this->HTM*position;
        }

        Eigen::Vector6d apply_wrench(const Eigen::Vector6d& wrench){
            Eigen::Vector6d out;
            out.head(3) = this->apply_vector(wrench.head(3));
            out.tail(3) = this->apply_vector(wrench.tail(3)) + this->HTM.translation().cross(out.head<3>());
            return out;
        }

        Transform inverse(){
            return Transform(this->HTM.inverse());
        }

    };

    Frame create_frame(Frame parent = nullptr){
      return std::make_shared<_Frame>(parent);
    }

    struct _Frame: public std::enable_shared_from_this<_Frame>{
        Eigen::Isometry3d HTM = Eigen::Isometry3d::Identity();
        Eigen::Vector3d _linear_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d _angular_velocity = Eigen::Vector3d::Zero();
        Frame parent;

        _Frame(Frame parent):parent(parent){};

        Frame create_child(){
          return create_frame(this->shared_from_this());
        }

        auto position(){
            return this->HTM.translation();
        }

        auto rotation(){
            return this->HTM.linear();
        }

        Eigen::Vector3d& linear_velocity(){
            return this->_linear_velocity;
        }

        Eigen::Vector3d& angular_velocity(){
            return this->_angular_velocity;
        }

        Eigen::Vector3d get_attitude(){
            return rotation_to_euler(this->rotation());
        }

        void set_attitude(const Eigen::Vector3d& attitude){
            this->rotation() = euler_to_rotation(attitude);
        }

        Eigen::Vector6d get_pose(){
            Eigen::Vector6d out;
            Transform t = transform(nullptr, this->shared_from_this());
            out.head(3) = t.HTM.translation();
            out.tail(3) = rotation_to_euler(t.HTM.linear());
            return out;
        }

        Eigen::Vector6d get_twist(){
            Eigen::Vector6d out;

            Eigen::Vector3d v = this->linear_velocity();
            Eigen::Vector3d w = this->angular_velocity();

            if (this->parent != nullptr){
                Eigen::Vector6d twist_p = this->parent->get_twist();
                Transform t = transform(this->shared_from_this(), this->parent);
                v += t.apply_vector(twist_p.head(3)) + twist_p.tail<3>().cross(this->position());
                w += t.apply_vector(twist_p.tail(3));
            }
            out.head(3) = v;
            out.tail(3) = w;
            return out;
        }

    };

    inline Transform transform(Frame zeroth, Frame end){

        Eigen::Isometry3d HTM = Eigen::Isometry3d::Identity();
        Frame f = end;

        while (f != nullptr){
            if (f == zeroth){
                return Transform(HTM);
            }

            HTM = f->HTM*HTM;
            f = f->parent;

        }

        // If we get to here, the least common base frame is the inertial frame (nullptr)
        Transform T = Transform(HTM);

        if (zeroth == nullptr){
            return T;
        }

        Transform T_ = transform(nullptr, zeroth).inverse();

        return Transform(T_.HTM*T.HTM);
    }

    namespace rigidbody{

        inline Eigen::Matrix3d skew(const Eigen::Vector3d& v){
            Eigen::Matrix3d out;
            out <<  0,  -v(2),  v(1),
                    v(2),   0,  -v(0),
                    -v(1),  v(0),   0;
            return out;
        }

        inline Eigen::Matrix6d motion_transformation_matrix(const Eigen::Vector3d& position){
            Eigen::Matrix6d out = Eigen::Matrix6d::Identity();
            out.block<3,3>(0,3) = skew(position).transpose();
            return out;
        }

        Eigen::Matrix6d generalized_inertia_matrix(const double& mass, const Eigen::Vector3d& gyradii){
            if (mass <= 0.0){
                throw std::invalid_argument("mass must be greater than zero!");
            }
            if ((gyradii.array() <= 0.0).any()){
                throw std::invalid_argument("All gyradii must be greater than zero!");
            }

            Eigen::Matrix6d H = Eigen::Matrix6d::Zero();
            H.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * mass;
            H.block<3,3>(3,3) = (mass*gyradii.array().square()).matrix().asDiagonal();
            return H;
        }

        struct RigidBody {
            Eigen::Matrix6d inertia = Eigen::Matrix6d::Identity();
            Frame origin = create_frame();
            Frame CoG = create_frame(origin);

            RigidBody(
                const Eigen::Matrix6d inertia,
                const Eigen::Vector3d& cog = Eigen::Vector3d::Zero()
                ): inertia(inertia)
                {
                    this->CoG->position() = cog;
                };

            double mass(){
                return this->inertia(0,0);
            }

            Eigen::Matrix6d coriolis_centripetal_matrix(
                const Eigen::Matrix6d inertia,
                const Eigen::Vector6d twist
            ){
                Eigen::Matrix6d C = Eigen::Matrix6d::Zero();
                C.block<3,3>(0,0) = this->mass()*skew(twist.tail(3));
                C.block<3,3>(3,3) = -skew(inertia.block<3,3>(3,3)*twist.tail(3));
                return C;
            }

            Eigen::Vector6d generalized_coordinates(){
                return this->origin->get_pose();
            }

            Eigen::Vector6d generalized_velocities(){
                Eigen::Vector6d twist = this->origin->get_twist();
                Transform t = transform(nullptr, this->origin);

                Eigen::Vector6d out = Eigen::Vector6d::Zero();
                out.head(3) = t.apply_vector(twist.head(3));
                out.tail(3) = angular_velocity_to_deuler(
                    this->origin->get_attitude(), twist.tail(3)
                );

                return out;
            }

            Eigen::Vector6d acceleration(
                const Eigen::Vector6d& wrench,
                const Eigen::Matrix6d& additional_inertia = Eigen::Matrix6d::Zero()
            ){
                Eigen::Vector6d twist, f_cc, f;
                Eigen::Matrix6d H, I_cg, C_cg, I_b, C_b;

                f = wrench;
                twist = this->origin->get_twist();

                H = motion_transformation_matrix(this->CoG->position());
                I_cg = (this->inertia.array() + additional_inertia.array()).matrix();
                C_cg = this->coriolis_centripetal_matrix(I_cg, twist);
                I_b = H.transpose() * I_cg * H;
                C_b = H.transpose() * C_cg * H;

                f_cc = C_b * twist;
                f -= f_cc;

                return I_b.lu().solve(f);
            }

            Eigen::Vector6d wrench(
                const Eigen::Vector6d& acceleration,
                const Eigen::Matrix6d& additional_inertia = Eigen::Matrix6d::Zero()
            ){
                Eigen::Vector6d twist, f_cc, f;
                Eigen::Matrix6d H, I_cg, C_cg, I_b, C_b;

                twist = this->origin->get_twist();

                H = motion_transformation_matrix(this->CoG->position());
                I_cg = (this->inertia.array() + additional_inertia.array()).matrix();
                C_cg = this->coriolis_centripetal_matrix(I_cg, twist);
                I_b = H.transpose() * I_cg * H;
                C_b = H.transpose() * C_cg * H;

                f = I_b * acceleration;
                f_cc = C_b * twist;
                f += f_cc;

                return f;
            }

        };


    }


}
