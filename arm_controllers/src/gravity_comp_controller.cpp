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

	//subscriber from the key command
	motion_command_sub_ = n.subscribe("/motion_command", 1, &GravityCompController::motionCommandCB, this);
	

    // Initialize mode flag
    use_circular_motion_ = false;

    // Command subscriber
    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &GravityCompController::commandCB, this);

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

//set the flag fot the chaning in behavior
void GravityCompController::motionCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    if (msg->data.size() != 1)
    {
        ROS_ERROR("Expected a single float value for motion command!");
        return;
    }

    double command = msg->data[0];
    if (command == 1.0)
    {
        use_circular_motion_ = true;
    }
    else if (command == 0.0)
    {
        use_circular_motion_ = false;
    }
    else
    {
        ROS_WARN("Unknown command value received: %f", command);
    }
}



void GravityCompController::starting(const ros::Time& time)
{
    // Get joint positions
    for(size_t i = 0; i < n_joints_; i++) 
    {
        q_(i) = joints_[i].getPosition();
        qdot_(i) = joints_[i].getVelocity();
    }

    ROS_INFO("Starting Gravity Compensation Controller");
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

void GravityCompController::update(const ros::Time& time, const ros::Duration& period)
{
    std::vector<double>& commands = *commands_buffer_.readFromRT();
    double dt = period.toSec();
    static double t = 0;

    // Define the parameters for the ellipse
    double a = 10.0; // semi-major axis
    double b = 5.0;  // semi-minor axis
    double omega = 0.5; // angular frequency

    for (size_t i = 0; i < n_joints_; i++)
    {
        if (use_circular_motion_)
        {
            // Elliptical motion parameters
            double x = a * cos(omega * t);
            double y = b * sin(omega * t);
            double z = 0; // Keep this zero or adjust as needed

            // Map x, y, z to joint commands (assuming a 6-joint robot with 3 DoF)
            q_cmd_(i) = x; // This is just an example, adjust as needed for your robot
            qdot_cmd_(i) = y; // Same here, adjust based on desired motion

            // Adjust the other parameters to create a smoother transition
            qdot_cmd_(i) = -omega * a * sin(omega * t); // Derivative of x
        }
        else
        {
            // Sinusoidal motion
            q_cmd_(i) = 45 * D2R * sin(PI / 2 * t);
            qdot_cmd_(i) = 45 * D2R * PI / 2 * cos(PI / 2 * t);
        }

        enforceJointLimits(q_cmd_(i), i);
        q_(i) = joints_[i].getPosition();
        qdot_(i) = joints_[i].getVelocity();

        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
        {
            angles::shortest_angular_distance_with_limits(
                q_(i),
                q_cmd_(i),
                joint_urdfs_[i]->limits->lower,
                joint_urdfs_[i]->limits->upper,
                q_error_(i));
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
            q_error_(i) = angles::shortest_angular_distance(q_(i), q_cmd_(i));
        }
        else
        {
            q_error_(i) = q_cmd_(i) - q_(i);
        }
        q_error_dot_(i) = qdot_cmd_(i) - qdot_(i);

        tau_fric_(i) = 1 * qdot_(i) + 1 * KDL::sign(qdot_(i));
    }

    t += dt;
    
    id_solver_->JntToGravity(q_, G_);

    for (int i = 0; i < n_joints_; i++)
    {
        tau_cmd_(i) = G_(i) + pids_[i].computeCommand(q_error_(i), q_error_dot_(i), ros::Duration(dt));
        controller_state_pub_->msg_.effort_feedforward[i] = G_(i);
        tau_cmd_(i) += pids_[i].computeCommand(q_error_(i), q_error_dot_(i), ros::Duration(dt));

        if (tau_cmd_(i) >= joint_urdfs_[i]->limits->effort)
            tau_cmd_(i) = joint_urdfs_[i]->limits->effort;

        if (tau_cmd_(i) <= -joint_urdfs_[i]->limits->effort)
            tau_cmd_(i) = -joint_urdfs_[i]->limits->effort;

        joints_[i].setCommand(tau_cmd_(i));

        controller_state_pub_->msg_.command[i] = q_cmd_(i);
        controller_state_pub_->msg_.command_dot[i] = qdot_cmd_(i);
        controller_state_pub_->msg_.state[i] = q_(i);
        controller_state_pub_->msg_.state_dot[i] = qdot_(i);
        controller_state_pub_->msg_.error[i] = q_error_(i);
        controller_state_pub_->msg_.error_dot[i] = q_error_dot_(i);
        controller_state_pub_->msg_.effort_command[i] = tau_cmd_(i);
        controller_state_pub_->msg_.effort_feedforward[i] = G_(i);
        controller_state_pub_->msg_.effort_feedback[i] = tau_fric_(i);
    }

    controller_state_pub_->msg_.header.stamp = time;
    controller_state_pub_->unlockAndPublish();
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
