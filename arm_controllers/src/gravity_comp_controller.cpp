#include "utils/gravity_comp_controller.h"

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI

namespace arm_controllers {

GravityCompController::GravityCompController() 
    : loop_count_(0)
{
    // Constructor implementation (if needed)
}

GravityCompController::~GravityCompController() 
{
    command_sub_.shutdown();
}

bool GravityCompController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n){   
    // List of controlled joints
    if (!n.getParam("joints", joint_names_))
    {
        ROS_ERROR("Could not find joint name");
        return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0)
    {
        ROS_ERROR("List of joint names is empty.");
        return false;
    }

    // URDF
    urdf::Model urdf;
    if (!urdf.initParam("robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    // Joint handle
    for(int i = 0; i < n_joints_; i++)
    {
        try
        {
            joints_.push_back(hw->getHandle(joint_names_[i]));
        }
        catch (const hardware_interface::HardwareInterfaceException& e)
        {
            ROS_ERROR_STREAM("Exception thrown: " << e.what());
            return false;
        }

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
        if (!joint_urdf)
        {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
            return false;
        }
        joint_urdfs_.push_back(joint_urdf); 
    }

    // KDL parser
    if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)){
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    // KDL chain
    std::string root_name, tip_name;
    if (!n.getParam("root_link", root_name))
    {
        ROS_ERROR("Could not find root link name");
        return false;
    }
    if (!n.getParam("tip_link", tip_name))
    {
        ROS_ERROR("Could not find tip link name");
        return false;
    }
    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for(it = segment_map.begin(); it != segment_map.end(); it++)
            ROS_ERROR_STREAM("    "<<(*it).first);

        return false;
    }

    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;
    G_.resize(n_joints_);    

    // Inverse dynamics solver
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

    // Command and state
    tau_cmd_ = Eigen::VectorXd::Zero(n_joints_);
    tau_fric_ = Eigen::VectorXd::Zero(n_joints_);
    q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
    qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
    
    q_.data = Eigen::VectorXd::Zero(n_joints_);
    qdot_.data = Eigen::VectorXd::Zero(n_joints_);

    q_error_ = Eigen::VectorXd::Zero(n_joints_);
    q_error_dot_ = Eigen::VectorXd::Zero(n_joints_);

    // PIDs
    pids_.resize(n_joints_);
    for (size_t i = 0; i < n_joints_; i++)
    {
        // Load PID Controller using gains set on parameter server
        if (!pids_[i].init(ros::NodeHandle(n, "gains/" + joint_names_[i] + "/pid")))
        {
            ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
            return false;
        }
    }

    // Subscriber for motion commands
    motion_command_sub_ = n.subscribe("/motion_command", 1, &GravityCompController::motionCommandCB, this);
    
    // Initialize command buffer with positions and velocities only
    commands_buffer_.initRT(std::vector<double>(2 * n_joints_, 0.0));  // Initialize with zeros

    // Additional command subscriber (if needed)
    // This part seems redundant as you already have motion_command_sub_
    // If 'command' is a different topic, handle accordingly
    // commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    // command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &GravityCompController::commandCB, this);

    // Start realtime state publisher
    controller_state_pub_.reset(
        new realtime_tools::RealtimePublisher<arm_controllers::ControllerJointState>(n, "state", 1));

    controller_state_pub_->msg_.header.stamp = ros::Time::now();
    for (size_t i = 0; i < n_joints_; i++)
    {
        controller_state_pub_->msg_.name.push_back(joint_names_[i]);
        controller_state_pub_->msg_.command.push_back(0.0);
        controller_state_pub_->msg_.command_dot.push_back(0.0);
        controller_state_pub_->msg_.state.push_back(0.0);
        controller_state_pub_->msg_.state_dot.push_back(0.0);
        controller_state_pub_->msg_.error.push_back(0.0);
        controller_state_pub_->msg_.error_dot.push_back(0.0);
        controller_state_pub_->msg_.effort_command.push_back(0.0);
        controller_state_pub_->msg_.effort_feedforward.push_back(0.0);
        controller_state_pub_->msg_.effort_feedback.push_back(0.0);
    }
    
    return true;
}


void GravityCompController::motionCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg) {
    // Check if the message contains both positions and velocities for all joints
    if (msg->data.size() < 2 * n_joints_) {
        ROS_ERROR("Expected at least %u positions and velocities, but got %u!", 2 * n_joints_, (unsigned int)msg->data.size());
        return;
    }

    // Create a temporary vector to store the new commands (positions and velocities)
    std::vector<double> new_commands(2 * n_joints_);

    // Extract positions and velocities
    for (size_t i = 0; i < n_joints_; i++) {
        new_commands[i] = msg->data[i];                       // Position
        new_commands[i + n_joints_] = msg->data[i + n_joints_]; // Velocity
    }

    // Write the new commands into the real-time buffer
    commands_buffer_.writeFromNonRT(new_commands);  // Use the buffer for thread-safe communication

   
}

void GravityCompController::starting(const ros::Time& time) {
    // Set desired initial positions
    std::vector<double> initial_positions = {0.1, -1, -0.5, 0.0, +1, -0.1}; // Define your fixed starting positions

    for (size_t i = 0; i < n_joints_; i++) {
        q_cmd_(i) = initial_positions[i];  // Set fixed position command
        q_(i) = joints_[i].getPosition();
        qdot_(i) = joints_[i].getVelocity();

        ROS_INFO("Starting position for joint %lu: %f", i, q_cmd_(i));
    }

    ROS_INFO("Starting Gravity Compensation Controller in fixed position mode");
}



void GravityCompController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if(msg->data.size() != n_joints_)
    { 
        ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
        return; 
    }
    commands_buffer_.writeFromNonRT(msg->data);
}


void GravityCompController::update(const ros::Time& time, const ros::Duration& period) {
    // Read the latest commands from the real-time buffer filled by motionCommandCB
    std::vector<double> *commands_ptr = commands_buffer_.readFromRT();

    if (!commands_ptr) {
        ROS_WARN_THROTTLE(1.0, "No commands received yet.");
        return;
    }

    std::vector<double>& commands = *commands_ptr;  // Dereference the real-time buffer pointer
    double dt = period.toSec();  // Time step in seconds
    double q_cmd_old;

    static double t = 0;  // Time variable to track the simulation time for sinusoidal motions

    for (size_t i = 0; i < n_joints_; i++) {
        // Store old joint command for possible future use
        q_cmd_old = q_cmd_(i);

        // Set the command for each joint from the latest received command
        q_cmd_(i) = commands[i];  // Position command for the i-th joint
        qdot_cmd_(i) = commands[i + n_joints_];  // Velocity command for the i-th joint

        ROS_DEBUG("Command for joint %lu: Position %f, Velocity %f", i, q_cmd_(i), qdot_cmd_(i));

        // Enforce joint limits to make sure the command doesn't exceed joint capabilities
        enforceJointLimits(q_cmd_(i), i);

        // Get the current joint position and velocity from the hardware
        q_(i) = joints_[i].getPosition();  // Actual joint position for the i-th joint
        qdot_(i) = joints_[i].getVelocity();  // Actual joint velocity for the i-th joint

        // Compute position error between the command and the actual position
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE) {
            // For revolute joints, compute the shortest angular distance considering limits
            angles::shortest_angular_distance_with_limits(
                q_(i), q_cmd_(i), joint_urdfs_[i]->limits->lower, joint_urdfs_[i]->limits->upper, q_error_(i));
        } else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS) {
            // For continuous joints, compute the shortest angular distance
            q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
        } else {  // For prismatic joints
            q_error_(i) = q_cmd_(i) - q_(i);  // Linear distance error
        }

        // Compute velocity error between the command and the actual velocity
        q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

        // Friction compensation (you can adjust parameters for real-world testing)
        tau_fric_(i) = 1.0 * qdot_(i) + 1.0 * KDL::sign(qdot_(i));

        // Gravity compensation (calculates gravity-induced torque based on current joint positions)
        id_solver_->JntToGravity(q_, G_);

        // Compute total torque command:
        tau_cmd_(i) = G_(i) + tau_fric_(i);  // Start with gravity compensation and add friction compensation


        // Add the PID control effort based on the position and velocity errors
        tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), period);

        // Saturate the torque command to avoid exceeding joint effort limits
        tau_cmd_(i) = std::max(-joint_urdfs_[i]->limits->effort, std::min(tau_cmd_(i), joint_urdfs_[i]->limits->effort));

        // Apply the computed torque command to the joint
        joints_[i].setCommand(tau_cmd_(i));
    }

    // Publish the current state every 10 iterations to reduce communication overhead
    if (loop_count_ % 10 == 0) {
        if (controller_state_pub_->trylock()) {
            controller_state_pub_->msg_.header.stamp = time;
            for (size_t i = 0; i < n_joints_; i++) {
                // Populate the message with the current state and commands for the i-th joint
                controller_state_pub_->msg_.command[i] = R2D * q_cmd_(i);  // Commanded position (radians to degrees)
                controller_state_pub_->msg_.command_dot[i] = R2D * qdot_cmd_(i);  // Commanded velocity (radians to degrees)
                controller_state_pub_->msg_.state[i] = R2D * q_(i);  // Actual position (radians to degrees)
                controller_state_pub_->msg_.state_dot[i] = R2D * qdot_(i);  // Actual velocity (radians to degrees)
                controller_state_pub_->msg_.error[i] = R2D * q_error_(i);  // Position error
                controller_state_pub_->msg_.error_dot[i] = R2D * q_error_dot_(i);  // Velocity error
                controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);  // Total torque command
                controller_state_pub_->msg_.effort_feedback[i] = tau_cmd_(i) - G_(i);  // PID effort (command - gravity)
                controller_state_pub_->msg_.effort_feedforward[i] = G_(i);  // Gravity compensation torque
            }
            // Unlock and publish the state message
            controller_state_pub_->unlockAndPublish();
        }
    }

    // Increment time for periodic functions like sinusoidal motion
    t += dt;
    loop_count_++;
}



void GravityCompController::stopping(const ros::Time& time)
{
    // Stop the controller and cleanup if necessary
    ROS_INFO("Stopping Gravity Compensation Controller");
}

void GravityCompController::enforceJointLimits(double &command, unsigned int index)
{
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE)
    {
        command = std::max(joint_urdfs_[index]->limits->lower, 
                            std::min(joint_urdfs_[index]->limits->upper, command));
    }
}

} // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityCompController, controller_interface::ControllerBase)
