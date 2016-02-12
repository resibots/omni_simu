#ifndef OMNI_VREP_UTILS_HPP_
#define OMNI_VREP_UTILS_HPP_

#include <string>

#include <Eigen/Core>

#include <ros/ros.h>

#include <vrep_common/simRosGetObjectHandle.h>
#include <vrep_common/simRosSetObjectPosition.h>

namespace omni_vrep{
class Utils {
public:
    Utils()
    {
        _vrep_get_obj_handle_client = n.serviceClient<vrep_get_obj_handle_t>("vrep/simRosGetObjectHandle");
        _vrep_set_obj_pos_client = n.serviceClient<vrep_set_obj_pos_t>("vrep/simRosSetObjectPosition");
    }

    int get_object_handle(std::string name)
    {
        vrep_get_obj_handle_t get_obj_handle_srv;
        get_obj_handle_srv.request.objectName = name;

        if (_vrep_get_obj_handle_client.call(get_obj_handle_srv)) {
            return get_obj_handle_srv.response.handle;
        }
        else {
            ROS_ERROR("Falled to call service simRosGetObjectHandle");
            return -1;
        }
    }

    bool set_object_position(int handle, const Eigen::Vector3d& pos, int relative_to_handle = -1)
    {
        vrep_set_obj_pos_t srv;
        srv.request.handle = handle;
        srv.request.relativeToObjectHandle = relative_to_handle;
        srv.request.position.x = pos(0);
        srv.request.position.y = pos(1);
        srv.request.position.z = pos(2);

        if (!_vrep_set_obj_pos_client.call(srv)) {
            ROS_ERROR("Falled to call service simRosSetObjectPosition");
            return false;
        }

        return true;
    }

    bool set_object_position(std::string name, const Eigen::Vector3d& pos, std::string relative_to_name = "")
    {
        int handle = get_object_handle(name);
        if (handle == -1)
            return false;

        int relative_to_handle = -1;

        if (relative_to_name != "") {
            relative_to_handle = get_object_handle(relative_to_name);
            if (relative_to_handle == -1)
                return false;
        }

        return set_object_position(handle, pos, relative_to_handle);
    }

private:
    typedef vrep_common::simRosGetObjectHandle vrep_get_obj_handle_t;
    typedef vrep_common::simRosSetObjectPosition vrep_set_obj_pos_t;

    ros::NodeHandle n;

    ros::ServiceClient _vrep_get_obj_handle_client;
    ros::ServiceClient _vrep_set_obj_pos_client;
};
}

#endif