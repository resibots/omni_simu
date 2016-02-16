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

    void arm_joint_step(int joint, double target)
    {
        assert(joint >= 0 && joint < 4);
        double q = _arm_joints[joint]->getPosition(0);
        double q_err = target - q;
        double gain = 1.0 / (DART_PI * _skeleton->getTimeStep());

        _arm_joints[joint]->setCommand(0, q_err * gain);
    }

protected:
    void _load_urdf(std::string urdf_file)
    {
        std::cout << "Loading URDF..." << std::endl;
        // Load file into string
        std::ifstream t(urdf_file);
        std::string str((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());

        // Load the Skeleton from a file
        dart::utils::DartLoader loader;
        loader.addPackageDirectory("youbot_description", "/home/fedeallocati/Workspaces/catkin_ws/src/youbot_description");
        _skeleton = loader.parseSkeletonString(str, "");

        std::cout << "Loaded URDF" << std::endl;

        if (_skeleton == nullptr)
            return;
        _skeleton->setName("omnipointer");

        _skeleton->getBodyNode(0)->changeParentJointType<WeldJoint>();

        std::cout << "Joints: ";

        for (size_t i = 0; i < _skeleton->getNumJoints() - 1; ++i)
            std::cout << _skeleton->getJoint(i)->getType() << " - ";
        std::cout << _skeleton->getJoint(_skeleton->getNumJoints() - 1)->getType() << std::endl;

        std::cout << "Bodies: ";

        for (size_t i = 0; i < _skeleton->getNumBodyNodes() - 1; ++i)
            std::cout << _skeleton->getBodyNode(i)->getName() << " - ";
        std::cout << _skeleton->getBodyNode(_skeleton->getNumBodyNodes() - 1)->getName() << std::endl;

        for (size_t i = 1; i <= 4; ++i) {
            _arm_joints.push_back(_skeleton->getJoint("arm_joint_" + std::to_string(i)));
            _arm_joints.back()->setPositionLimitEnforced(true);
            //std::cout << "[" << _arm_joints.back()->getPositionLowerLimit(0) << ", " << _arm_joints.back()->getPositionUpperLimit(0) << "]" << std::endl;
            _arm_joints.back()->setActuatorType(dart::dynamics::Joint::VELOCITY);
            _arm_joints.back()->getDof(0)->setPosition(M_PI);
        }

        _skeleton->enableSelfCollision();
        std::cin.get();
    }

    dart::dynamics::SkeletonPtr _skeleton;
    std::vector<dart::dynamics::JointPtr> _arm_joints;
};
}

#endif