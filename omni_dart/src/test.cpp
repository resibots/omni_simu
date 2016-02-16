#include <iostream>

#include <omni_dart/omni_dart_simu.hpp>
#include <omni_dart/omnipointer.hpp>

void run(char* model_file, Eigen::Vector4d joints)
{
    omni_dart::OmniDARTSimu<omni_dart::Omnipointer> robot(model_file);

    robot.set_joints_positions(joints);

    if (robot.broken())
        std::cout << "Finished because of a collision detected" << std::endl;
    else
        std::cout << "Finished" << std::endl;

    std::cin.get();
}

int main(int argc, char** argv)
{
    try
    {
        if (argc != 6) {
            std::cout << "Invalid arguments. Usage: " << argv[0] << " robot_model.urdf Joint1 Joint2 Joint3 Joint4" << std::endl;
            return -1;
        }

        Eigen::Vector4d joints;
        joints(0) = atof(argv[2]);
        joints(1) = atof(argv[3]);
        joints(2) = atof(argv[4]);
        joints(3) = atof(argv[5]);

        run(argv[1], joints);
    }
    catch (char const* e)
    {
        std::cout << "Error: " << e << std::endl;
    }

    return 0;
}