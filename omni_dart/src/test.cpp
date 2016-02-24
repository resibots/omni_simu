#include <iostream>

#include <memory>
#include <boost/filesystem.hpp>

#include <omni_dart/omni_dart_simu.hpp>
#include <omni_dart/omnipointer.hpp>

void run(char* model_file, char* path_to_youbot_description, const Eigen::Vector4d& joints)
{
    auto robot = std::make_shared<omni_dart::Omnipointer>(boost::filesystem::complete(boost::filesystem::path(model_file)).native(), boost::filesystem::complete(boost::filesystem::path(path_to_youbot_description)).native());
    omni_dart::OmniDARTSimu<omni_dart::Omnipointer> simu(robot);

    simu.set_joints_positions(joints);

    if (simu.broken())
        std::cout << "Finished because of a collision detected" << std::endl;
    else
        std::cout << "Finished" << std::endl;

    std::cin.get();
}

int main(int argc, char** argv)
{
    try
    {
        if (argc != 7) {
            std::cout << "Invalid arguments. Usage: " << argv[0] << " robot_model.urdf /path/to/youbot_description Joint1 Joint2 Joint3 Joint4" << std::endl;
            return -1;
        }

        Eigen::Vector4d joints;
        joints(0) = atof(argv[3]);
        joints(1) = atof(argv[4]);
        joints(2) = atof(argv[5]);
        joints(3) = atof(argv[6]);

        run(argv[1], argv[2], joints);
    }
    catch (char const* e)
    {
        std::cout << "Error: " << e << std::endl;
    }

    return 0;
}
