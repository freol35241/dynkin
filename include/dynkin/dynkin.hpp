
#include <memory>
#include <experimental/memory>
#include <Eigen/Dense>
#include <unsupported/Eigen/EulerAngles>

namespace Eigen {
    typedef Eigen::Matrix<double,6,1> Vector6d;
}

namespace dynkin {

    struct Frame;
    struct Transform;
    Transform transform(Frame*, Frame*);

    inline Eigen::Vector3d rotation_to_euler(const Eigen::Matrix3d rotation){
        double sy = sqrt(
            rotation(0,0)*rotation(0,0) + rotation(1,0)*rotation(1,0)
            );
        bool singular = sy < 1e-6;
        Eigen::Vector3d out;

        if (singular){
            out(0) = atan2(rotation(2,1), rotation(2,2));
            out(1) = atan2(-rotation(2,0), sy);
            out(2) = atan2(rotation(1,0), rotation(0,0));
        } 
        else {
            out(0) = atan2(-rotation(1,2), rotation(1,1));
            out(1) = atan2(-rotation(2,0), sy);
            out(2) = 0;
        }

        return out;
    }

    inline Eigen::Matrix3d euler_to_rotation(const Eigen::Vector3d attitude){
        return (
            Eigen::AngleAxisd(attitude(2), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(attitude(1), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(attitude(0), Eigen::Vector3d::UnitX())
            ).toRotationMatrix();
    }

    struct Transform{
        Eigen::Isometry3d HTM;

        Transform(const Eigen::Isometry3d HTM):HTM(HTM){}

        Eigen::Vector3d apply_vector(const Eigen::Vector3d vector){
            return this->HTM.linear()*vector;
        }

        Eigen::Vector3d apply_position(const Eigen::Vector3d position){
            return this->HTM*position;
        }

        Eigen::Vector6d apply_wrench(const Eigen::Vector6d wrench){
            Eigen::Vector6d out;
            out.head(3) = this->apply_vector(wrench.head(3));
            out.tail(3) = this->apply_vector(wrench.tail(3)) + this->HTM.translation().cross(out.head<3>());
            return out;
        }

        Transform inverse(){
            return Transform(this->HTM.inverse());
        }

    };

    struct Frame{
        Eigen::Isometry3d HTM = Eigen::Isometry3d::Identity();
        Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        std::experimental::observer_ptr<Frame> parent = nullptr;

        Frame& set_parent(Frame* parent){
            this->parent.reset(parent);
            return *this;
        }

        Eigen::Vector3d position(){
            return this->HTM.translation();
        }

        Frame& set_position(const Eigen::Vector3d position){
            this->HTM.translation() = position;
            return *this;
        }

        Eigen::Matrix3d rotation(){
            return this->HTM.linear();
        }

        Frame& set_rotation(const Eigen::Matrix3d rotation){
            this->HTM.linear() = rotation;
            return *this;
        }

        Eigen::Vector3d attitude(){
            return rotation_to_euler(this->rotation());
        }

        Frame& set_attitude(const Eigen::Vector3d attitude){
            this->set_rotation(
                euler_to_rotation(attitude)
            );
            return *this;
        }

        Eigen::Vector6d get_pose(){
            Eigen::Vector6d out;
            Transform t = transform(nullptr, this);
            out.head(3) = t.HTM.translation();
            out.tail(3) = rotation_to_euler(t.HTM.linear());
            return out;
        }

        Eigen::Vector6d get_twist(){
            Eigen::Vector6d out;

            Eigen::Vector3d v = this->linear_velocity;
            Eigen::Vector3d w = this->angular_velocity;

            if (this->parent != nullptr){
                Eigen::Vector6d twist_p = this->parent->get_twist();
                Transform t = transform(this, this->parent.get());
                v += t.apply_vector(twist_p.head(3)) + twist_p.tail<3>().cross(this->position());
                w += t.apply_vector(twist_p.tail(3));
            }
            out.head(3) = v;
            out.tail(3) = w;
            return out;
        }

    };

    inline Transform transform(Frame* zeroth, Frame* end){

        Eigen::Isometry3d HTM = Eigen::Isometry3d::Identity();
        Frame* f = end;
        
        while (f != nullptr){
            if (f == zeroth){
                return Transform(HTM);
            }

            HTM = f->HTM*HTM;
            f = f->parent.get();
            
        }

        // If we get to here, the least common base frame is the inertial frame (nullptr)
        Transform T = Transform(HTM);

        if (zeroth == nullptr){
            return T;
        }

        Transform T_ = transform(nullptr, zeroth).inverse();

        return Transform(T_.HTM*T.HTM);
    }


}
