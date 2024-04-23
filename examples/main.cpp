#include <iostream>
#include "coppeliasim_driver.hpp"
#include <memory>


int main()
{
    //std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2", "Franka_joint3", "Franka_joint4",
    //                                       "Franka_joint5", "Franka_joint6", "Franka_joint7"};

    std::vector<std::string> jointnames = {"/Franka/joint","/Franka/link2_resp/joint","/Franka/link3_resp/joint",
    "/Franka/link4_resp/joint","/Franka/link5_resp/joint","/Franka/link6_resp/joint",
        "/Franka/link7_resp/joint"};
    auto q_min = (Eigen::VectorXd(7) << -2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895).finished();
    auto q_max = (Eigen::VectorXd(7) <<  2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895).finished();
    auto q_dot_min = (Eigen::VectorXd(7) <<  -2.62, -2.62, -2.62, -2.62, -5.26, -4.18, -5.26).finished();
    auto q_dot_max = (Eigen::VectorXd(7) <<  2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26).finished();

    auto coppeliasim_driver = std::make_unique<CoppeliaSimDriver>();
    //coppeliasim_driver->set_parameters("127.0.0.1", 19997, jointnames, {q_min, q_max}, {q_dot_min, q_dot_max});
    coppeliasim_driver->set_parameters("localhost", 23000, jointnames, {q_min, q_max}, {q_dot_min, q_dot_max});
    coppeliasim_driver->connect();
    std::cout<<coppeliasim_driver->get_status_message()<<std::endl;

    coppeliasim_driver->initialize();
    std::cout<<coppeliasim_driver->get_status_message()<<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    coppeliasim_driver->deinitialize();
    std::cout<<coppeliasim_driver->get_status_message()<<std::endl;
    coppeliasim_driver->disconnect();
    std::cout<<coppeliasim_driver->get_status_message()<<std::endl;
    return 0;
}
