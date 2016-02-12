#include <iostream>

#include <ros/ros.h>

#include <omni_vrep/omnigrasper.hpp>

void run(char* model_file, bool dynamic)
{
    double x = 0.3;
    double y = 1;
    double theta = M_PI / 2;

    Eigen::Vector4d joints;
    joints(0) = 0;
    joints(1) = M_PI;
    joints(2) = - M_PI / 4;
    joints(3) = M_PI / 2;

    omni_vrep::Omnigrasper robot(model_file, dynamic);
    robot.init();

    std::cout << "Displacing" << std::endl;
    robot.base_displace(x, y);
    std::cin.get();

    std::cout << "Rotating" << std::endl;
    robot.base_rotate(theta);
    std::cin.get();

    std::cout << "Moving arm" << std::endl;
    robot.set_joints_positions(joints);

    std::cout << robot.get_end_effector_position().transpose() << std::endl;
    std::cin.get();

    std::cout << "Stop" << std::endl;
    robot.stop();
}

int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "omni_vrep_test");

        if (argc != 2) {
            std::cout << "Invalid arguments. Usage: " << argv[0] << " v-rep_scene_file.ttt" << std::endl;
            return -1;
        }

        std::cout << "Static: " << std::endl;
        run(argv[1], false);

        std::cout << std::endl;

        std::cout << "Dynamic: " << std::endl;
        run(argv[1], true);
    }
    catch (char const* e)
    {
        std::cout << "Error: " << e << std::endl;
    }

    return 0;
}