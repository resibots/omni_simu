#ifndef OMNI_VREP_OMNIPOINTER_HPP_
#define OMNI_VREP_OMNIPOINTER_HPP_

#include <vector>
#include <string>
#include <chrono>
#include <math.h>

#include <boost/filesystem.hpp>

#include <Eigen/Core>

#include <ros/ros.h>

#include <vrep_common/simRosLoadScene.h>
#include <vrep_common/simRosStartSimulation.h>
#include <vrep_common/simRosPauseSimulation.h>
#include <vrep_common/simRosStopSimulation.h>
#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosSetObjectPosition.h>
#include <vrep_common/simRosSetObjectQuaternion.h>
#include <vrep_common/simRosSetJointPosition.h>
#include <vrep_common/simRosSetJointTargetPosition.h>
#include <vrep_common/simRosGetObjectPose.h>
#include <vrep_common/simRosGetJointState.h>

namespace omni_vrep{
class Omnipointer {
public:
    Omnipointer(std::string model_file, bool dynamic = true, int sim_retries = 5, int max_ms_to_wait = 3000, double position_eps = 1e-3)
    {
        _model_file = boost::filesystem::complete(boost::filesystem::path(model_file)).native();
        _dynamic = dynamic;
        _sim_retries = sim_retries;
        _max_ms_to_wait = max_ms_to_wait;
        _position_eps = position_eps;

        _vrep_start_client = _n.serviceClient<vrep_start_t>("vrep/simRosStartSimulation");
        _vrep_pause_client = _n.serviceClient<vrep_pause_t>("vrep/simRosPauseSimulation");
        _vrep_stop_client = _n.serviceClient<vrep_stop_t>("vrep/simRosStopSimulation");
        _vrep_set_obj_pos_client = _n.serviceClient<vrep_set_obj_pos_t>("vrep/simRosSetObjectPosition");
        _vrep_set_obj_quat_client = _n.serviceClient<vrep_set_obj_quat_t>("vrep/simRosSetObjectQuaternion");
        _vrep_set_joint_pos_client = _n.serviceClient<vrep_set_joint_pos_t>("vrep/simRosSetJointPosition");
        _vrep_set_joint_target_pos_client = _n.serviceClient<vrep_set_joint_target_pos_t>("vrep/simRosSetJointTargetPosition");
        _vrep_get_obj_pose_client = _n.serviceClient<vrep_get_obj_pose_t>("vrep/simRosGetObjectPose");
        _vrep_get_joint_state_client = _n.serviceClient<vrep_get_joint_state_t>("vrep/simRosGetJointState");

        _initialize();
    }

    void init()
    {
        if (!_dynamic)
            return;

        vrep_start_t srv;
        bool started = false;
        int i = 0;

        while (!started && i < _sim_retries) {
            if (_vrep_start_client.call(srv)) {
                started = srv.response.result == 1;
            } else {
                ROS_ERROR("Falled to call service simRosStartSimulation");
            }
            i++;
        }

        if (!started) {
            ROS_ERROR("Simulation not started");
            throw "Unable to start simulator";
        }
    }

    void pause()
    {
        if (!_dynamic)
            return;

        vrep_pause_t srv;
        bool paused = false;
        int i = 0;

        while (!paused && i < _sim_retries) {
            if (_vrep_pause_client.call(srv)) {
                paused = srv.response.result == 1;
            } else {
                ROS_ERROR("Falled to call service simRosPauseSimulation");
            }
            i++;
        }

        if (!paused) {
            ROS_ERROR("Simulation not paused");
            throw "Unable to pause simulator";
        }
    }

    void stop()
    {
        if (!_dynamic)
            return;

        vrep_stop_t srv;
        bool stoped = false;
        int i = 0;

        while (!stoped && i < _sim_retries) {
            if (_vrep_stop_client.call(srv)) {
                stoped = srv.response.result == 1;
            } else {
                ROS_ERROR("Falled to call service simRosStopSimulation");
            }
            i++;
        }

        if (stoped) {
            ROS_DEBUG("Simulation stoped");
        }
        else {
            ROS_ERROR("Simulation not stoped");
            throw "Unable to stop simulator";
        }
    }

    void restart()
    {
        _initialize();
    }

    void base_displace(double x, double y, bool wait_to_reach = false)
    {
        vrep_set_obj_pos_t srv;

        if (_dynamic){
            srv.request.handle = _vrep_base_target_pos_handle;
            srv.request.relativeToObjectHandle = _vrep_base_target_pos_handle;
            srv.request.position.x = x;
            srv.request.position.y = y;
            srv.request.position.z = 0;
        } else {
            srv.request.handle = _vrep_youBot_handle;
            srv.request.relativeToObjectHandle = _vrep_youBot_handle;
            srv.request.position.x = 0;
            srv.request.position.y = -y;
            srv.request.position.z = x;
        }

        if (!_vrep_set_obj_pos_client.call(srv)) {
            ROS_ERROR("Falled to call service simRosSetObjectPosition");
        }

        if (!_dynamic || !wait_to_reach) {
            return;
        }

        auto t1 = std::chrono::steady_clock::now();
        auto time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t1).count();
        while (time_waiting < _max_ms_to_wait + (5000 * sqrt(x * x + y * y))) {
            vrep_get_obj_pose_t srv;
            srv.request.handle = _vrep_base_reference_handle;
            srv.request.relativeToObjectHandle = _vrep_base_target_pos_handle;

            if (!_vrep_get_obj_pose_client.call(srv)) {
                ROS_ERROR("Falled to call service simRosGetObjectPose");
            } else {
                double a = srv.response.pose.pose.position.x;
                double b = srv.response.pose.pose.position.y;

                if (sqrt(a * a + b * b) < _position_eps)
                    break;
            }

            time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
        }
    }

    void base_rotate(double theta, bool wait_to_reach = false)
    {
        vrep_set_obj_quat_t srv;

        if (_dynamic){
            srv.request.handle = _vrep_base_target_pos_handle;
            srv.request.relativeToObjectHandle = _vrep_base_target_pos_handle;
            srv.request.quaternion.x = 0;
            srv.request.quaternion.y = 0;
            srv.request.quaternion.z = sin(theta / 2);
            srv.request.quaternion.w = cos(theta / 2);
        } else {
            srv.request.handle = _vrep_youBot_handle;
            srv.request.relativeToObjectHandle = _vrep_youBot_handle;
            srv.request.quaternion.x = sin(theta / 2);
            srv.request.quaternion.y = 0;
            srv.request.quaternion.z = 0;
            srv.request.quaternion.w = cos(theta / 2);
        }

        if (!_vrep_set_obj_quat_client.call(srv)) {
            ROS_ERROR("Falled to call service simRosSetObjectQuaternion");
        }

        if (!_dynamic || !wait_to_reach) {
            return;
        }

        auto t1 = std::chrono::steady_clock::now();
        auto time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t1).count();
        while (time_waiting < _max_ms_to_wait + (5000 * theta)) {
            vrep_get_obj_pose_t srv;
            srv.request.handle = _vrep_base_reference_handle;
            srv.request.relativeToObjectHandle = _vrep_base_target_pos_handle;

            if (!_vrep_get_obj_pose_client.call(srv)) {
                ROS_ERROR("Falled to call service simRosGetObjectPose");
            } else {
                double a = srv.response.pose.pose.orientation.w;

                if (fabs(fabs(a) - 1) < _position_eps)
                    break;
            }

            time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
        }
    }

    void set_joints_positions(const Eigen::Vector4d& joints, bool wait_to_reach = false)
    {
        for (auto i = 0; i < 4; i++) {
            if (_dynamic) {
                vrep_set_joint_target_pos_t srv;
                srv.request.handle = _vrep_joint_handle[i];
                srv.request.targetPosition = joints(i);

                if (!_vrep_set_joint_target_pos_client.call(srv)) {
                    ROS_ERROR("Falled to call service simRosSetJointTargetPosition");
                }
            } else {
                vrep_set_joint_pos_t srv;
                srv.request.handle = _vrep_joint_handle[i];
                srv.request.position = joints(i);

                if (!_vrep_set_joint_pos_client.call(srv)) {
                    ROS_ERROR("Falled to call service simRosSetJointPosition");
                }
            }

            if (!_dynamic || !wait_to_reach) {
                continue;
            }

            auto t1 = std::chrono::steady_clock::now();
            auto time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t1).count();
            while (time_waiting < _max_ms_to_wait) {
                vrep_get_joint_state_t srv;
                srv.request.handle = _vrep_joint_handle[i];

                if (!_vrep_get_joint_state_client.call(srv)) {
                    ROS_ERROR("Falled to call service simRosGetJoinState");
                }

                if (fabs(joints(i) - srv.response.state.position[0]) < _position_eps) {
                    break;
                }

                time_waiting = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
            }
        }
    }

    Eigen::VectorXd get_end_effector_position()
    {
        vrep_get_obj_pose_t srv;
        srv.request.handle = _vrep_end_effector_handle;
        srv.request.relativeToObjectHandle = -1;

        if (!_vrep_get_obj_pose_client.call(srv)) {
            ROS_ERROR("Falled to call service simRosGetObjectPose");
            return Eigen::Vector3d(0, 0, 0);
        }
        else {
            return Eigen::Vector3d(srv.response.pose.pose.position.x, srv.response.pose.pose.position.y, srv.response.pose.pose.position.z);
        }
    }

private:
    void _initialize()
    {
        const std::vector<std::string> joint_names = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4"};
        const std::string youBot_name = "youBot";
        const std::string base_reference_name = "youBot_vehicleReference";
        const std::string base_target_pos_name = "youBot_vehicleTargetPosition";
        const std::string end_effector_name = "Omnipointer_EndEffector";

        ros::service::waitForService("vrep/simRosLoadScene");

        ros::ServiceClient vrep_load_model = _n.serviceClient<vrep_common::simRosLoadScene>("vrep/simRosLoadScene");
        vrep_common::simRosLoadScene load_model_srv;

        load_model_srv.request.fileName = _model_file;
        if (vrep_load_model.call(load_model_srv)) {
            if (load_model_srv.response.result != 1) {
                ROS_ERROR("The model file could not be loaded %d", load_model_srv.response.result);
                throw "The model file could not be loaded";
            }
        } else {
            ROS_ERROR("Falled to call service simRosLoadScene");
            throw "Falled to call service simRosLoadScene";
        }

        ros::ServiceClient vrep_get_obj_handle_client = _n.serviceClient<vrep_common::simRosGetObjectHandle>("vrep/simRosGetObjectHandle");
        vrep_common::simRosGetObjectHandle get_obj_handle_srv;

        get_obj_handle_srv.request.objectName = youBot_name;
        if (vrep_get_obj_handle_client.call(get_obj_handle_srv)) {
            _vrep_youBot_handle = get_obj_handle_srv.response.handle;
        } else {
            ROS_ERROR("Falled to call service simRosGetObjectHandle");
        }

        get_obj_handle_srv.request.objectName = base_reference_name;
        if (vrep_get_obj_handle_client.call(get_obj_handle_srv)) {
            _vrep_base_reference_handle = get_obj_handle_srv.response.handle;
        } else {
            ROS_ERROR("Falled to call service simRosGetObjectHandle");
        }

        get_obj_handle_srv.request.objectName = base_target_pos_name;
        if (vrep_get_obj_handle_client.call(get_obj_handle_srv)) {
            _vrep_base_target_pos_handle = get_obj_handle_srv.response.handle;
        } else {
            ROS_ERROR("Falled to call service simRosGetObjectHandle");
        }

        for (auto i = 0; i < 4; i++) {
            get_obj_handle_srv.request.objectName = joint_names[i];
            if (vrep_get_obj_handle_client.call(get_obj_handle_srv)) {
                _vrep_joint_handle[i] = get_obj_handle_srv.response.handle;
            } else {
                ROS_ERROR("Falled to call service simRosGetObjectHandle");
            }
        }

        get_obj_handle_srv.request.objectName = end_effector_name;
        if (vrep_get_obj_handle_client.call(get_obj_handle_srv)) {
            _vrep_end_effector_handle = get_obj_handle_srv.response.handle;
        } else {
            ROS_ERROR("Falled to call service simRosGetObjectHandle");
        }
    }

    typedef vrep_common::simRosStartSimulation vrep_start_t;
    typedef vrep_common::simRosPauseSimulation vrep_pause_t;
    typedef vrep_common::simRosStopSimulation vrep_stop_t;
    typedef vrep_common::simRosSetObjectPosition vrep_set_obj_pos_t;
    typedef vrep_common::simRosSetObjectQuaternion vrep_set_obj_quat_t;
    typedef vrep_common::simRosSetJointPosition vrep_set_joint_pos_t;
    typedef vrep_common::simRosSetJointTargetPosition vrep_set_joint_target_pos_t;
    typedef vrep_common::simRosGetObjectPose vrep_get_obj_pose_t;
    typedef vrep_common::simRosGetJointState vrep_get_joint_state_t;

    ros::NodeHandle _n;
    ros::ServiceClient _vrep_start_client;
    ros::ServiceClient _vrep_pause_client;
    ros::ServiceClient _vrep_stop_client;
    ros::ServiceClient _vrep_set_obj_pos_client;
    ros::ServiceClient _vrep_set_obj_quat_client;
    ros::ServiceClient _vrep_set_joint_pos_client;
    ros::ServiceClient _vrep_set_joint_target_pos_client;
    ros::ServiceClient _vrep_get_obj_pose_client;
    ros::ServiceClient _vrep_get_joint_state_client;

    int _vrep_youBot_handle;
    int _vrep_base_reference_handle;
    int _vrep_base_target_pos_handle;
    int _vrep_joint_handle[4];
    int _vrep_end_effector_handle;

    std::string _model_file;
    bool _dynamic;
    int _sim_retries;
    int _max_ms_to_wait;
    double _position_eps;
};
}

#endif