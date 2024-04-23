#pragma once
#include <string>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
//#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterface.h>
#include <memory>
#include <thread>
#include <atomic>

class CoppeliaSimDriver
{
protected:
    std::string ip_;
    std::vector<std::string> jointnames_;
    VectorXd q_min_;
    VectorXd q_max_;
    VectorXd q_dot_min_;
    VectorXd q_dot_max_;
    VectorXd q_;
    VectorXd q_dot_;
    VectorXd torques_;
    int n_joints_;
    int port_;
    std::string status_msg_;

    std::unique_ptr<DQ_CoppeliaSimInterface> vi_;
    bool parameters_ready_;

    void _start_echo_robot_state_mode();
    std::thread echo_robot_state_mode_thread_;
    void _start_echo_robot_state_mode_thread();
    std::atomic<bool> finish_echo_robot_state_;
    void _finish_echo_robot_state();
    void _connect_coppeliasim();
    void _check_parameter_sizes();

public:
    CoppeliaSimDriver();
    CoppeliaSimDriver(const CoppeliaSimDriver&) = delete;
    CoppeliaSimDriver& operator= (const CoppeliaSimDriver&) = delete;

    void set_parameters(const std::string& ip,
                        const int& port,
                        const std::vector<std::string>& jointnames,
                        const std::tuple<VectorXd, VectorXd>& joint_limits,
                        const std::tuple<VectorXd, VectorXd>& joint_velocity_limits
                        );

    std::string get_ip();
    std::vector<std::string> get_jointnames();
    std::tuple<VectorXd, VectorXd> get_joint_limits();
    std::tuple<VectorXd, VectorXd> get_joint_velocity_limits();
    int get_port();

    void connect();
    void initialize();
    void deinitialize();
    void disconnect();

    VectorXd get_configuration_space_positions();
    VectorXd get_configuration_space_velocities();
    VectorXd get_configuration_space_torques();
    std::string get_status_message();
};

