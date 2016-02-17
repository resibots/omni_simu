#ifndef OMNI_DART_OMNIPOINTER_HPP_
#define OMNI_DART_OMNIPOINTER_HPP_

#include <cassert>
#include <vector>
#include <string>
#include <fstream>
#include <streambuf>

#include <Eigen/Core>

#include <dart/dart.h>

namespace omni_dart{
class Omnipointer {
public:
    Omnipointer(std::string model_file)
    {
        _load_urdf(model_file);
    }

    dart::dynamics::SkeletonPtr skeleton()
    {
        return _skeleton;
    }

    double get_arm_joint_position(int joint)
    {
        assert(joint >= 0 && joint < 4);
        return _arm_joints[joint]->getPosition(0);
    }

    double get_arm_joint_command(int joint)
    {
        assert(joint >= 0 && joint < 4);
        return _arm_joints[joint]->getCommand(0);
    }

    void set_arm_joint_command(int joint, double value)
    {
        assert(joint >= 0 && joint < 4);
        _arm_joints[joint]->setCommand(0, value);
    }

    Eigen::Vector3d get_end_effector_position()
    {
        return _arm_links.back()->getCOM();
    }

    Eigen::Vector6d pose()
    {
        auto pos_and_rot = _skeleton->getPositions();
        Eigen::Vector6d tmp;
        tmp << pos_and_rot(3), pos_and_rot(4), pos_and_rot(5), pos_and_rot(0), pos_and_rot(1), pos_and_rot(2);
        return tmp;
    }

    void arm_joint_step(int joint, double target)
    {
        assert(joint >= 0 && joint < 4);
        double q = _arm_joints[joint]->getPosition(0);
        double q_err = target - q;
        double gain = DART_PI / 3;

        _arm_joints[joint]->setCommand(0, q_err * gain);
    }

    void enable_self_collisions()
    {
        _skeleton->enableSelfCollision();
    }

    void disable_self_collisions()
    {
        _skeleton->disableSelfCollision();
    }

    bool check_collision()
    {
        for (auto link : _arm_links){            
            if (link->isColliding()){
                std::cout << link->getName() << " Colliding" << std::endl;
                return true;
            }
        }

        return false;
    }

protected:
    void _load_urdf(std::string urdf_file)
    {
        // Load file into string
        std::ifstream t(urdf_file);
        std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

        // Load the Skeleton from a file
        dart::utils::DartLoader loader;
        loader.addPackageDirectory("youbot_description", "/home/fedeallocati/Workspaces/catkin_ws/src/youbot_description");
        _skeleton = loader.parseSkeletonString(str, "");

        if (_skeleton == nullptr)
            return;
        _skeleton->setName("omnipointer");

        for (size_t i = 1; i <= 4; ++i) {
            _arm_joints.push_back(_skeleton->getJoint("arm_joint_" + std::to_string(i)));
            _arm_joints.back()->setPositionLimitEnforced(true);
            _arm_joints.back()->setActuatorType(dart::dynamics::Joint::VELOCITY);
            _arm_joints.back()->getDof(0)->setPosition(M_PI);
        }

        for (size_t i = 0; i <= 5; ++i){
            _arm_links.push_back(_skeleton->getBodyNode("arm_link_" + std::to_string(i)));
        }
    }

    dart::dynamics::SkeletonPtr _skeleton;
    std::vector<dart::dynamics::JointPtr> _arm_joints;
    std::vector<dart::dynamics::BodyNodePtr> _arm_links;
};
}

#endif