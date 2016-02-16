#ifndef OMNI_DART_SIMU_HPP
#define OMNI_DART_SIMU_HPP

#include <string>
#include <chrono>
#include <memory>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <dart/dart.h>
#include <dart/collision/dart/DARTCollisionDetector.h>

#ifdef USE_DART_GRAPHIC
#include <osgDart/osgDart.h>
#endif

namespace omni_dart {
template <typename Robot>
class OmniDARTSimu {
public:
    using robot_t = std::shared_ptr<Robot>;

    OmniDARTSimu(std::string model_file, int max_ms_to_wait = 3000, double position_eps = 1e-3)
    {
        _model_file = boost::filesystem::complete(boost::filesystem::path(model_file)).native();
        _max_ms_to_wait = max_ms_to_wait;
        _position_eps = position_eps;

        _world = std::make_shared<dart::simulation::World>();
        _robot = std::make_shared<Robot>(_model_file);

        _world->getConstraintSolver()->setCollisionDetector(new dart::collision::DARTCollisionDetector());
        _robot->skeleton()->setPosition(5, 0.2);
        
        //_add_floor();
        _world->addSkeleton(_robot->skeleton());
        _world->setTimeStep(0.01);
        _world->setTime(0.0);
        _detector = _world->getConstraintSolver()->getCollisionDetector();
        _detector->detectCollision(true, true);

#ifdef USE_DART_GRAPHIC
        _osg_world_node = new osgDart::WorldNode(_world);
        _osg_world_node->simulate(true);
        _osg_viewer.addWorldNode(_osg_world_node);
         _osg_viewer.setUpViewInWindow(0, 0, 640, 480);
        // full-screen
        //_osg_viewer.setUpViewOnSingleScreen();
#endif
    }

    void base_displace(double x, double y, bool = false)
    {
    }

    void base_rotate(double theta, bool = false)
    {
    }

    void set_joints_positions(const Eigen::VectorXd& joints, bool = false)
    {
        _break = false;
        int current = 0;        

        auto t1 = std::chrono::steady_clock::now();
        auto time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t1).count();

#ifdef USE_DART_GRAPHIC
        while (current < joints.size() && !_osg_viewer.done())
#else
        while (current < joints.size())
#endif
        {
            double q = _robot->get_arm_joint_position(current);
            std::cout << "Joint: " << current << " - Position: " << q << " rads" << std::endl;
            double q_err = joints(current) - q;

            if (q_err < _position_eps){
                _robot->set_arm_joint_command(current, 0);
                current++;
                t1 = std::chrono::steady_clock::now();
                //std::cin.get();
                continue;
            }

            time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
            if (time_waiting > _max_ms_to_wait){
                _robot->set_arm_joint_command(current, 0);
                current++;
                t1 = std::chrono::steady_clock::now();
                //std::cin.get();
                continue;
            }

            _robot->arm_joint_step(current, joints(current));
            _world->step(false);

            bool collision = false;
            for(size_t i = 0; i < _detector->getNumContacts(); ++i)
            {
                // If neither of the colliding BodyNodes belongs to the floor, then we
                // know the robot is in contact with something it shouldn't be
                const dart::collision::Contact& contact = _detector->getContact(i);
                if (contact.bodyNode1.lock()->getSkeleton() == _robot->skeleton() && contact.bodyNode2.lock()->getSkeleton() == _robot->skeleton())
                    collision = false;
                else if(contact.bodyNode1.lock()->getSkeleton() != _floor && contact.bodyNode2.lock()->getSkeleton() != _floor)
                    collision = true;
                else if (contact.bodyNode1.lock()->getSkeleton() == _floor && contact.bodyNode2.lock()->getSkeleton()->getName().substr(0, 3) == "arm")
                    collision = true;                
                else if (contact.bodyNode2.lock()->getSkeleton() == _floor && contact.bodyNode1.lock()->getSkeleton()->getName().substr(0, 3) == "arm")
                    collision = true;

                if (collision){
                    std::cout << "COLLISION: " << contact.bodyNode1.lock()->getSkeleton()->getName() << " - " << contact.bodyNode2.lock()->getSkeleton()->getName() << std::endl;
                    collision = false;
                    //break;
                }
            }

            if(collision)
            {
              _break = true;
              return;
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
    }

    void run()
    {
#ifdef USE_DART_GRAPHIC
        while (!_osg_viewer.done())
#else
        while (true)
#endif
        {
            _world->step(false);

            bool collision = false;
            for(size_t i = 0; i < _detector->getNumContacts(); ++i)
            {
              // If neither of the colliding BodyNodes belongs to the floor, then we
              // know the robot is in contact with something it shouldn't be
              const dart::collision::Contact& contact = _detector->getContact(i);
              if(contact.bodyNode1.lock()->getSkeleton() != _floor && contact.bodyNode2.lock()->getSkeleton() != _floor)
              {
                collision = true;
                break;
              }
            }

            if(collision)
            {
              _break = true;
              return;
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
    }

    /*Eigen::Vector3d get_end_effector_position()
    {
        return _robot->;
    }*/

    /*bool break() const
    {
        return _break;
    }*/

protected:
    void _add_floor()
    {
        // We do not want 2 floors!
        if (_world->getSkeleton("floor") != nullptr)
            return;

        _floor = dart::dynamics::Skeleton::create("floor");

        // Give the floor a body
        dart::dynamics::BodyNodePtr body = _floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
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

        _world->addSkeleton(_floor);
    }

    robot_t _robot;
    dart::simulation::WorldPtr _world; 
    dart::collision::CollisionDetector* _detector;
    dart::dynamics::SkeletonPtr _floor;
    bool _break;

    std::string _model_file;
    int _max_ms_to_wait;
    double _position_eps;

#ifdef USE_DART_GRAPHIC
    osg::ref_ptr<osgDart::WorldNode> _osg_world_node;
    osgDart::Viewer _osg_viewer;
#endif
};
}

#endif