#ifndef OMNI_DART_OMNIPOINTER_HPP_
#define OMNI_DART_OMNIPOINTER_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <streambuf>

#include <Eigen/Core>

#include <dart/dart.h>
#include <dart/utils/urdf/urdf.h>

namespace omni_dart{
class Omnipointer {
public:
    Omnipointer(std::string model_file, std::string path_to_youbot_description) : _model_file(model_file), _path_to_youbot_description(path_to_youbot_description)
    {
        _load_urdf();
    }

    dart::dynamics::SkeletonPtr skeleton()
    {
        return _skeleton;
    }

    Eigen::Vector4d get_arm_joint_positions() const
    {
        return Eigen::Vector4d(_arm_joints[0]->getPosition(0), _arm_joints[1]->getPosition(0), _arm_joints[2]->getPosition(0), _arm_joints[3]->getPosition(0));
    }

    Eigen::Vector4d get_arm_joint_velocities() const
    {
        return Eigen::Vector4d(_arm_joints[0]->getVelocity(0), _arm_joints[1]->getVelocity(0), _arm_joints[2]->getVelocity(0), _arm_joints[3]->getVelocity(0));
    }

    Eigen::Vector3d get_end_effector_position() const
    {
        return _arm_links.back()->getTransform(_base_link).translation();
    }

    Eigen::Vector6d pose() const
    {
        auto pos_and_rot = _skeleton->getPositions();
        Eigen::Vector6d tmp;
        tmp << pos_and_rot(3), pos_and_rot(4), pos_and_rot(5), pos_and_rot(0), pos_and_rot(1), pos_and_rot(2);
        return tmp;
    }

    void arm_joints_step(const Eigen::Vector4d& target)
    {
        // A stable PD controller taken from the Domioes example

        const double KpPD = 2.0;
        const double KdPD = 1.5;

        Eigen::Vector4d q = get_arm_joint_positions();
        Eigen::Vector4d dq = get_arm_joint_velocities();

        q += dq * _skeleton->getTimeStep();

        Eigen::Vector4d q_err = target - q;
        Eigen::Vector4d dq_err = -dq;

        // Compute the desired joint forces
        Eigen::Vector4d forces = _skeleton->getMassMatrix().bottomRightCorner<4, 4>() * (KpPD * q_err + KdPD * dq_err) + _skeleton->getCoriolisAndGravityForces().tail<4>();

        size_t i = 0;
        for (auto joint : _arm_joints) {
            joint->setForce(0, forces(i));
            i++;
        }
    }

    void enable_self_collisions()
    {
        _skeleton->enableSelfCollision();
    }

    void disable_self_collisions()
    {
        _skeleton->disableSelfCollision();
    }

    bool check_collision() const
    {
        for (auto link : _arm_links) {
            if (link->isColliding())
                return true;
        }

        return false;
    }

protected:
    void _load_urdf()
    {
        // Load file into string
        std::ifstream t(_model_file);
        std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

        // Load the Skeleton from a file
        dart::utils::DartLoader loader;
        loader.addPackageDirectory("youbot_description", _path_to_youbot_description);
        _skeleton = loader.parseSkeletonString(str, "");

        if (_skeleton == nullptr)
            return;
        _skeleton->setName("omnipointer");

        for (size_t i = 1; i <= 4; ++i) {
            _arm_joints.push_back(_skeleton->getJoint("arm_joint_" + std::to_string(i)));
            _arm_joints.back()->setPositionLimitEnforced(true);
            _arm_joints.back()->setActuatorType(dart::dynamics::Joint::FORCE);
            _arm_joints.back()->getDof(0)->setPosition(M_PI);
        }

        for (size_t i = 0; i <= 5; ++i){
            _arm_links.push_back(_skeleton->getBodyNode("arm_link_" + std::to_string(i)));
        }

          _base_link = _skeleton->getBodyNode("base_link");
    }

    std::string _model_file;
    std::string _path_to_youbot_description;
    dart::dynamics::SkeletonPtr _skeleton;
    std::vector<dart::dynamics::JointPtr> _arm_joints;
    std::vector<dart::dynamics::BodyNodePtr> _arm_links;
    dart::dynamics::BodyNodePtr _base_link;
};
}

#endif
