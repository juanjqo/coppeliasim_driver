#include "coppeliasim_driver.hpp"


CoppeliaSimDriver::CoppeliaSimDriver()
    :parameters_ready_(false)
{
   vi_ = std::make_unique<DQ_VrepInterface>();
}

void CoppeliaSimDriver::set_parameters(const std::string& ip,
                                       const int& port,
                                       const std::vector<std::string>& jointnames,
                                       const std::tuple<VectorXd, VectorXd>& joint_limits,
                                       const std::tuple<VectorXd, VectorXd>& joint_velocity_limits)
{
    ip_ = ip;
    port_ = port;
    jointnames_ = jointnames;
    std::tie(q_min_, q_max_) = joint_limits;
    std::tie(q_dot_min_, q_dot_max_) = joint_velocity_limits;
    _check_parameter_sizes();
    parameters_ready_ = true;
}

void CoppeliaSimDriver::_check_parameter_sizes()
{
    if (static_cast<std::size_t>(q_min_.size()) != static_cast<std::size_t>(q_max_.size())
        or static_cast<std::size_t>(q_min_.size()) != jointnames_.size())
    {
        throw std::runtime_error("Incorrect size of q_min and q_max or jointnames. ");
    }
    if (static_cast<std::size_t>(q_dot_min_.size()) != static_cast<std::size_t>(q_dot_max_.size())
        or static_cast<std::size_t>(q_dot_min_.size()) != jointnames_.size())
    {
        throw std::runtime_error("Incorrect size of q_dot_min and q_dot_max or jointnames. ");
    }

    n_joints_ = jointnames_.size();
    q_ = Eigen::VectorXd::Zero(n_joints_);
    q_dot_= Eigen::VectorXd::Zero(n_joints_);
}

std::vector<std::string> CoppeliaSimDriver::get_jointnames()
{
    return jointnames_;
}


std::tuple<VectorXd, VectorXd> CoppeliaSimDriver::get_joint_limits()
{
    return {q_min_, q_max_};
}

std::tuple<VectorXd, VectorXd> CoppeliaSimDriver::get_joint_velocity_limits()
{
    return {q_dot_min_, q_dot_max_};
}

int CoppeliaSimDriver::get_port()
{
    return port_;
}


/**
 * @brief CoppeliaSimDriver::connect_coppeliasim
 */
void CoppeliaSimDriver::connect_coppeliasim()
{
    if (!parameters_ready_)
    {
        throw std::runtime_error("Parameters unitialized!");
    }else{
        _connect_coppeliasim();
    }
}

void CoppeliaSimDriver::_connect_coppeliasim()
{
    try
    {
        if (!vi_->connect(ip_, port_,100,10))
            throw std::runtime_error("Unable to connect to CoppeliaSim.");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        status_msg_ = "connected!";
    }
    catch (std::exception& e)
    {
        std::cout<<e.what()<<std::endl;
        vi_->stop_simulation();
        vi_->disconnect();
    }
}

void CoppeliaSimDriver::initialize_coppeliasim()
{
    vi_->start_simulation();
    status_msg_ = "Initialized!";
    _start_echo_robot_state_mode_thread();
}

void CoppeliaSimDriver::deinitialize_coppeliasim()
{
    vi_->stop_simulation();
    status_msg_ = "Deinitialized!";
    _finish_echo_robot_state();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }

}

void CoppeliaSimDriver::disconnect_coppeliasim()
{
    vi_->disconnect();
    status_msg_ = "Disconnected!";
}

VectorXd CoppeliaSimDriver::get_configuration_space_positions()
{
    return q_;
}

VectorXd CoppeliaSimDriver::get_configuration_space_velocities()
{
    return q_dot_;
}

VectorXd CoppeliaSimDriver::get_configuration_space_torques()
{
    return torques_;
}

std::string CoppeliaSimDriver::get_status_message()
{
    return status_msg_;
}

void CoppeliaSimDriver::_start_echo_robot_state_mode()
{
    while(!finish_echo_robot_state_)
    {
        try
        {
            q_ = vi_->get_joint_positions(jointnames_);
            q_dot_ = vi_->get_joint_velocities(jointnames_);
            torques_ = vi_->get_joint_torques(jointnames_);
        }
        catch (std::exception& e)
        {
            //set_message_window_parameters(true, e.what());
            std::cout<<e.what()<<std::endl;
        }

    }
    //show_message_window();
}



void  CoppeliaSimDriver::_start_echo_robot_state_mode_thread()
{
    finish_echo_robot_state_ = false;
    status_msg_ = "Checking echo robot state thread";
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }
    status_msg_ ="Starting echo robot state thread";
    echo_robot_state_mode_thread_ = std::thread(&CoppeliaSimDriver::_start_echo_robot_state_mode, this);
}


void  CoppeliaSimDriver::_finish_echo_robot_state()
{
    status_msg_ = "Finishing echo robot state.";
    finish_echo_robot_state_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


