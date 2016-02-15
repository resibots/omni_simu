#include <iostream>

#include <dart/dart.h>

#ifdef USE_DART_GRAPHIC
#include <osgDart/osgDart.h>
#endif

using namespace dart::dynamics;
using namespace dart::simulation;

#include <omni_dart/omni_dart_simu.hpp>
#include <omni_dart/omnipointer.hpp>

void run(char* model_file)
{
    double x = 0.3;
    double y = 1;
    double theta = M_PI / 2;

    omni_dart::OmniDARTSimu<omni_dart::Omnipointer> robot(model_file);
    //robot.init();

    /*std::cout << "Displacing" << std::endl;
    robot.base_displace(x, y);
    std::cin.get();

    std::cout << "Rotating" << std::endl;
    robot.base_rotate(theta);*/
    //std::cin.get();

    std::cout << "Moving arm" << std::endl;

    //while (true){
        Eigen::Vector4d joints = (Eigen::Vector4d::Random() * M_PI).array() + M_PI;
        joints(0) = M_PI;
        joints(1) = 3 * M_PI / 2;
        joints(2) = (3 * M_PI / 2) + M_PI / 4;
        joints(3) = 3 * M_PI / 2  + M_PI / 4;
        robot.set_joints_positions(joints);
    //}
    robot.run();
    
    //std::cout << robot.get_end_effector_position().transpose() << std::endl;
    std::cin.get();

    std::cout << "Stop" << std::endl;
    //robot.stop();
}

int main(int argc, char** argv)
{
    try
    {
        if (argc != 2) {
            std::cout << "Invalid arguments. Usage: " << argv[0] << " robot_model.urdf" << std::endl;
            return -1;
        }

        run(argv[1]);
    }
    catch (char const* e)
    {
        std::cout << "Error: " << e << std::endl;
    }

    return 0;
}