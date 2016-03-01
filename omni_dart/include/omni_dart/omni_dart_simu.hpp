#ifndef OMNI_DART_SIMU_HPP
#define OMNI_DART_SIMU_HPP

#include <string>
#include <chrono>
#include <memory>

#include <Eigen/Core>

#include <dart/dart.h>
#include <dart/collision/bullet/BulletCollisionDetector.h>

#ifdef USE_DART_GRAPHIC
#include <osgDart/osgDart.h>
#endif

namespace omni_dart {
template <typename Robot>
class OmniDARTSimu {
public:
    OmniDARTSimu(std::shared_ptr<Robot> robot, int max_ms_to_wait = 3000, double position_eps = 1e-3)
    {
        _max_ms_to_wait = max_ms_to_wait;
        _position_eps = position_eps;

        _world = std::make_shared<dart::simulation::World>();
        _robot = robot;
        _floor = _make_floor();
        _broken = false;

        _world->setTimeStep(0.01);
        _world->getConstraintSolver()->setCollisionDetector(std::unique_ptr<dart::collision::BulletCollisionDetector>(new dart::collision::BulletCollisionDetector()));

        _robot->skeleton()->setPosition(2, M_PI);
        _robot->skeleton()->setPosition(5, 0.123793);
        _world->addSkeleton(_robot->skeleton());

        _world->addSkeleton(_floor);

        _stabilize_robot();

        _world->setTime(0.0);
        _world->checkCollision(true);
        _robot->enable_self_collisions();

#ifdef USE_DART_GRAPHIC
        _osg_world_node = new osgDart::WorldNode(_world);
        _osg_world_node->simulate(true);
        _osg_viewer.addWorldNode(_osg_world_node);
         _osg_viewer.setUpViewInWindow(0, 0, 640, 480);
        // full-screen
        //_osg_viewer.setUpViewOnSingleScreen();
#endif
    }

    bool set_joint_positions(const Eigen::VectorXd& joints)
    {
        bool reached = false;
        auto t1 = std::chrono::steady_clock::now();
        auto time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t1).count();

        int count = 0;
        Eigen::Vector4d last_pos = Eigen::Vector4d::Zero();

#ifdef USE_DART_GRAPHIC
        while (!reached && !_osg_viewer.done())
#else
        while (!reached)
#endif
        {
            Eigen::Vector4d q = _robot->get_arm_joint_positions();
            double q_err = (joints - q).norm();

            if (std::abs(q_err) < _position_eps) {
                reached = true;
                continue;
            }

            time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
            if (time_waiting > _max_ms_to_wait) {
                reached = true;
                continue;
            }

            if ((last_pos - q).norm() < 1e-3) {
                count++;
            }
            else
                count = 0;

            last_pos = q;

            if (count == 50) {
                reached = true;
                continue;
            }

            _robot->arm_joints_step(joints);
            _world->step();

            if(_robot->check_collision())
            {
              _broken = true;
              return false;
            }
            else
            {
                _broken = false;
            }

#ifdef USE_DART_GRAPHIC
            auto COM = _robot->skeleton()->getCOM();
            // set camera to follow robot
            _osg_viewer.getCameraManipulator()->setHomePosition(osg::Vec3d(-0.5, 3, 1), osg::Vec3d(COM(0), COM(1), COM(2)), osg::Vec3d(0, 0, 1));
            _osg_viewer.home();
            // process next frame
            _osg_viewer.frame();
#endif
        }

        return true;
    }

    Eigen::Vector3d get_end_effector_position() const
    {
        return _robot->get_end_effector_position();
    }

    bool broken() const
    {
        return _broken;
    }

protected:
    dart::dynamics::SkeletonPtr _make_floor()
    {
        auto floor = dart::dynamics::Skeleton::create("floor");

        // Give the floor a body
        dart::dynamics::BodyNodePtr body = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
        body->setName("floor_link");

        // Give the body a shape
        double floor_width = 10.0;
        double floor_height = 0.1;
        std::shared_ptr<dart::dynamics::BoxShape> box(new dart::dynamics::BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
        box->setColor(dart::Color::Gray());

        body->addVisualizationShape(box);
        body->addCollisionShape(box);

        // Put the body into position
        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        tf.translation() = Eigen::Vector3d(0.0, 0.0, 0);
        body->getParentJoint()->setTransformFromParentBodyNode(tf);

        return floor;
    }

    bool _stabilize_robot()
    {
        bool stabilized = false;
        int stab = 0;

        for (size_t s = 0; s < 1000 && !stabilized; ++s) {
            Eigen::Vector6d prev_pose = _robot->pose();

            _world->step();

            if ((_robot->pose() - prev_pose).norm() < 1e-4)
                stab++;
            else
                stab = 0;
            if (stab > 30)
                stabilized = true;
        }

        return stabilized;
    }

    dart::simulation::WorldPtr _world;
    std::shared_ptr<Robot> _robot;
    dart::dynamics::SkeletonPtr _floor;
    bool _broken;

    int _max_ms_to_wait;
    double _position_eps;

#ifdef USE_DART_GRAPHIC
    osg::ref_ptr<osgDart::WorldNode> _osg_world_node;
    osgDart::Viewer _osg_viewer;
#endif
};
}

#endif
