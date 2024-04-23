#include "coppeliasim_driver.hpp"


/**
 * @brief CoppeliaSimDriver::CoppeliaSimDriver
 * @param operation_mode Define the operation mode {STEALTH, MASTER}. STEALTH
 *        means the driver won't be allowed to start or stop a CoppeliaSim simulation.
 *        Otherwise, use MASTER (default).
 *        STEALTH is useful when you have another client controlling a scene and the driver
 *        is used only to monitor the scene.
 */
CoppeliaSimDriver::CoppeliaSimDriver(const OPERATION_MODE &operation_mode)
    :parameters_ready_(false),master_mode_(false)
{
    vi_ = std::make_unique<DQ_CoppeliaSimInterface>();
    if (operation_mode == MASTER)
        master_mode_ = true;
}


/**
 * @brief CoppeliaSimDriver::set_parameters
 * @param ip
 * @param port
 * @param jointnames
 * @param joint_limits
 * @param joint_velocity_limits
 */
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

/**
 * @brief CoppeliaSimDriver::get_ip
 * @return
 */
std::string CoppeliaSimDriver::get_ip() const
{
    return ip_;
}


/**
 * @brief CoppeliaSimDriver::_check_parameter_sizes
 */
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


/**
 * @brief CoppeliaSimDriver::get_jointnames
 * @return
 */
std::vector<std::string> CoppeliaSimDriver::get_jointnames() const
{
    return jointnames_;
}


/**
 * @brief CoppeliaSimDriver::get_joint_limits
 * @return
 */
std::tuple<VectorXd, VectorXd> CoppeliaSimDriver::get_joint_limits() const
{
    return {q_min_, q_max_};
}


/**
 * @brief CoppeliaSimDriver::get_joint_velocity_limits
 * @return
 */
std::tuple<VectorXd, VectorXd> CoppeliaSimDriver::get_joint_velocity_limits() const
{
    return {q_dot_min_, q_dot_max_};
}


/**
 * @brief CoppeliaSimDriver::get_port
 * @return
 */
int CoppeliaSimDriver::get_port() const
{
    return port_;
}


/**
 * @brief CoppeliaSimDriver::connect_coppeliasim
 */
void CoppeliaSimDriver::connect()
{
    if (!parameters_ready_)
    {
        throw std::runtime_error("Error in CoppeliaSimDriver::connect(). Parameters unitialized!");
    }else{
        _connect_coppeliasim();
    }
}

/**
 * @brief CoppeliaSimDriver::_connect_coppeliasim
 */
void CoppeliaSimDriver::_connect_coppeliasim()
{
    try
    {
        if (!vi_->connect(ip_, port_))
            throw std::runtime_error("Unable to connect to CoppeliaSim.");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        status_msg_ = "connected!";
    }
    catch (std::exception& e)
    {
        std::cout<<e.what()<<std::endl;
        vi_->stop_simulation();
        //vi_->disconnect();
    }
}

/**
 * @brief CoppeliaSimDriver::initialize
 */
void CoppeliaSimDriver::initialize()
{
    if (master_mode_)
        vi_->start_simulation();
    status_msg_ = "Initialized!";
    _start_echo_robot_state_mode_thread();
}

void CoppeliaSimDriver::deinitialize()
{
    status_msg_ = "Deinitialized!";
    _finish_echo_robot_state();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (echo_robot_state_mode_thread_.joinable())
    {
        echo_robot_state_mode_thread_.join();
    }
    if (master_mode_)
        vi_->stop_simulation();
}

/**
 * @brief CoppeliaSimDriver::disconnect
 */
void CoppeliaSimDriver::disconnect()
{

    //vi_->disconnect();
    status_msg_ = "Disconnected!";
}

/**
 * @brief CoppeliaSimDriver::get_configuration_space_positions
 * @return
 */
VectorXd CoppeliaSimDriver::get_configuration_space_positions() const
{
    return q_;
}

/**
 * @brief CoppeliaSimDriver::get_configuration_space_velocities
 * @return
 */
VectorXd CoppeliaSimDriver::get_configuration_space_velocities() const
{
    return q_dot_;
}

/**
 * @brief CoppeliaSimDriver::get_configuration_space_torques
 * @return
 */
VectorXd CoppeliaSimDriver::get_configuration_space_torques() const
{
    return torques_;
}

double CoppeliaSimDriver::get_simulation_time() const
{
    return simulation_time_;
}

/**
 * @brief CoppeliaSimDriver::get_status_message
 * @return
 */
std::string CoppeliaSimDriver::get_status_message()
{
    return status_msg_;
}

/**
 * @brief CoppeliaSimDriver::_start_echo_robot_state_mode
 */
void CoppeliaSimDriver::_start_echo_robot_state_mode()
{
    while(!finish_echo_robot_state_)
    {
        try
        {
            q_ = vi_->get_joint_positions(jointnames_);
            q_dot_ = vi_->get_joint_velocities(jointnames_);
            torques_ = vi_->get_joint_torques(jointnames_);
            simulation_time_ = vi_->get_simulation_time();
            std::cout<<"q: "<<q_.transpose()<<std::endl;
        }
        catch (std::exception& e)
        {
            //set_message_window_parameters(true, e.what());
            std::cout<<e.what()<<std::endl;
        }
    }
}


/**
 * @brief CoppeliaSimDriver::_start_echo_robot_state_mode_thread
 */
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

/**
 * @brief CoppeliaSimDriver::_finish_echo_robot_state
 */
void  CoppeliaSimDriver::_finish_echo_robot_state()
{
    status_msg_ = "Finishing echo robot state.";
    finish_echo_robot_state_ = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}


