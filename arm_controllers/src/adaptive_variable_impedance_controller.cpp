#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <boost/scoped_ptr.hpp>

#include <Eigen/Dense>

#define PI 3.14159265358979323846

struct Commands
{
    KDL::JntArray qd;
    KDL::JntArray qd_dot;
    KDL::JntArray qd_ddot;
    Eigen::VectorXd xd;     // Desired task-space position (6 elements)
    Eigen::VectorXd xd_dot; // Desired task-space velocity (6 elements)
    Commands() {}
};

namespace arm_controllers {

    class AdaptiveImpedanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        ~AdaptiveImpedanceController() 
        {
            sub_q_cmd_.shutdown(); 
            sub_forcetorque_sensor_.shutdown();
            sub_command_.shutdown();
        }

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {	
            ROS_INFO("Initializing Adaptive Impedance Controller");

            // List of controlled joints
            if (!n.getParam("joints", joint_names_))
            {
                ROS_ERROR("Could not find joint names");
                return false;
            }
            n_joints_ = joint_names_.size();

            if(n_joints_ == 0)
            {
                ROS_ERROR("List of joint names is empty.");
                return false;
            }

            // Parse URDF
            urdf::Model urdf_model;			
            if (!urdf_model.initParam("elfin/robot_description"))
            {
                ROS_ERROR("Failed to parse URDF file");
                return false;
            }

            // Get joint handles and URDF information
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

                urdf::JointConstSharedPtr joint_urdf = urdf_model.getJoint(joint_names_[i]);
                if (!joint_urdf)
                {
                    ROS_ERROR("Could not find joint '%s' in URDF", joint_names_[i].c_str());
                    return false;
                }
                joint_urdfs_.push_back(joint_urdf); 
            }

            // Construct KDL tree and chain
            if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_)){
                ROS_ERROR("Failed to construct KDL tree");
                return false;
            }

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
                ROS_ERROR_STREAM("Failed to get KDL chain from tree: " << root_name << " --> " << tip_name);
                return false;
            }

            // Initialize gravity
            gravity_ = KDL::Vector::Zero();
            gravity_(2) = -9.81;
            G_.resize(n_joints_);

            // Initialize Jacobian and other KDL structures
            J_.resize(n_joints_);
            qdot_.resize(n_joints_);
            tau_d_.resize(n_joints_);
            C_.resize(n_joints_);

            // Initialize KDL solvers
            id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
            fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
            ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
            jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

            // Initialize command and state vectors
            tau_d_.data = Eigen::VectorXd::Zero(n_joints_);
            C_.data = Eigen::VectorXd::Zero(n_joints_);
            q_.data = Eigen::VectorXd::Zero(n_joints_);
            qdot_.data = Eigen::VectorXd::Zero(n_joints_);
            q_cmd_sp_ = KDL::JntArray(n_joints_); // Initialize q_cmd_sp_
            q_cmd_sp_.resize(n_joints_);

            // Initialize command buffer
            Commands initial_commands;
            initial_commands.qd = KDL::JntArray(n_joints_);
            initial_commands.qd_dot = KDL::JntArray(n_joints_);
            initial_commands.qd_ddot = KDL::JntArray(n_joints_);
            initial_commands.xd = Eigen::VectorXd::Zero(6);
            initial_commands.xd_dot = Eigen::VectorXd::Zero(6);
            command_buffer_.initRT(initial_commands);

            // Initialize desired position
            desired_position_A_ = Eigen::VectorXd::Zero(3);
            desired_position_A_ << 0.38, 0.0, 0.05;
            
           

            
                

            // Subscribe to topics
            sub_q_cmd_ = n.subscribe("command", 1, &AdaptiveImpedanceController::commandCB, this);
            sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/elfin/elfin/ft_sensor_topic", 1, &AdaptiveImpedanceController::updateFTsensor, this);
            sub_command_ = n.subscribe("/motion_command", 1000, &AdaptiveImpedanceController::commandPosition, this);

            // Initialize control stage
            control_stage_ = 1;

            return true;
        }

        void starting(const ros::Time& /*time*/)
        {
            time_ = 0.0;
            total_time_ = 0.0;
            control_stage_ = 1;

            // Initialize previous desired positions with current position
            KDL::Frame end_effector_pose;
            fk_solver_->JntToCart(q_, end_effector_pose);
            x_d_x_prev_ = end_effector_pose.p.x();
            x_d_y_prev_ = end_effector_pose.p.y();
            x_d_z_prev_ = end_effector_pose.p.z();

            ROS_INFO("Adaptive Impedance Controller Started");
        }

        void update(const ros::Time& /*time*/, const ros::Duration& period)
        {
            dt_ = period.toSec();

            // Read joint states
            for (int i = 0; i < n_joints_; i++)
            {
                q_(i) = joints_[i].getPosition();
                qdot_(i) = joints_[i].getVelocity();
            }

            // Update dynamics
            id_solver_->JntToGravity(q_, G_);
            id_solver_->JntToCoriolis(q_, qdot_, C_);

            // Forward kinematics to get current end-effector pose
            KDL::Frame end_effector_pose;
            fk_solver_->JntToCart(q_, end_effector_pose);

            // Compute current end-effector position
            Eigen::VectorXd current_position(3);
            current_position << end_effector_pose.p.x(), end_effector_pose.p.y(), end_effector_pose.p.z();

            // Compute Jacobian and end-effector velocity
            jac_solver_->JntToJac(q_, J_);
            Eigen::VectorXd xdot = J_.data * qdot_.data;

            if (control_stage_ == 1)
            {
                // Stage 1: Move to Position A

                // Compute position error
                Eigen::VectorXd ex(3);
                ex = desired_position_A_.head<3>() - current_position;

                // Compute velocity error (desired velocity is zero)
                Eigen::VectorXd xdot_error = -xdot.head<3>();

                // Define task-space PID gains
                Eigen::VectorXd Kp(3), Kd(3);
                Kp << 1000, 1000, 1000; // Position gains
                Kd << 100, 100, 100;    // Velocity gains

                // Compute desired task-space force
                Eigen::VectorXd F_desired = Kp.cwiseProduct(ex) + Kd.cwiseProduct(xdot_error);

                // Extend F_desired to 6D (force and torque)
                Eigen::VectorXd F_ext = Eigen::VectorXd::Zero(6);
                F_ext.head<3>() = F_desired;

                // Compute joint torques
                tau_d_.data = J_.data.transpose() * F_ext + C_.data + G_.data;
            }

            // Apply torque commands
            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(tau_d_.data(i));
            }

            // Update time
            time_ += dt_;
            total_time_ += dt_;
        }

        void stopping(const ros::Time& /*time*/) { }

    private:
        void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
        {
            if(msg->data.size() != n_joints_)
            { 
                ROS_ERROR_STREAM("Command size (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")!");
                return; 
            }

            for (unsigned int i = 0; i < n_joints_; i++)
                q_cmd_sp_(i) = msg->data[i]; // Corrected syntax
        }

        void commandPosition(const std_msgs::Float64MultiArrayConstPtr &msg)
        {
            size_t expected_size = 30; // 3*6 (qd, qd_dot, qd_ddot) + 6*2 (xd, xd_dot)
            if (msg->data.size() != expected_size)
            {
                ROS_ERROR_STREAM("Command size (" << msg->data.size() << ") does not match expected size (" << expected_size << ")!");
                return;
            }

            Commands cmd;
            cmd.qd = KDL::JntArray(n_joints_);
            cmd.qd_dot = KDL::JntArray(n_joints_);
            cmd.qd_ddot = KDL::JntArray(n_joints_);
            cmd.xd = Eigen::VectorXd::Zero(6);
            cmd.xd_dot = Eigen::VectorXd::Zero(6);

            // Read joint positions, velocities, and accelerations
            for (size_t i = 0; i < n_joints_; i++)
            {
                cmd.qd(i) = msg->data[i];
                cmd.qd_dot(i) = msg->data[n_joints_ + i];
                cmd.qd_ddot(i) = msg->data[2 * n_joints_ + i];
            }

            // Read task-space coordinates and velocities
            size_t offset = 3 * n_joints_;
            for (size_t i = 0; i < 6; i++)
            {
                cmd.xd(i) = msg->data[offset + i];
                cmd.xd_dot(i) = msg->data[offset + 6 + i];
            }

            command_buffer_.writeFromNonRT(cmd);
        }

        void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
        {
            // Store the current force-torque measurements
            f_cur_[0] = msg->wrench.force.x;
            f_cur_[1] = msg->wrench.force.y;
            f_cur_[2] = first_order_lowpass_filter_z(msg->wrench.force.z);
            f_cur_[3] = msg->wrench.torque.x;
            f_cur_[4] = msg->wrench.torque.y;
            f_cur_[5] = msg->wrench.torque.z;
        }

        double first_order_lowpass_filter_z(double input)
        {
            double tau_z = 1.0 / (2 * PI * 10.0); // Cutoff frequency at 10 Hz
            double filt_z = (tau_z * filt_z_old_ + dt_ * input) / (tau_z + dt_);
            filt_z_old_ = filt_z;
            return filt_z;
        }

        // Joint handles
        unsigned int n_joints_;
        std::vector<std::string> joint_names_;
        std::vector<hardware_interface::JointHandle> joints_;
        std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

        // KDL structures
        KDL::Tree kdl_tree_;
        KDL::Chain kdl_chain_;
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
        KDL::JntArray G_; 
        KDL::Vector gravity_;

        // Dynamics
        KDL::JntArray C_;

        // Command and state
        KDL::JntArray q_;
        KDL::JntArray qdot_;
        KDL::JntArray tau_d_;
        KDL::JntArray q_cmd_sp_; // Declared q_cmd_sp_

        // Command buffer
        realtime_tools::RealtimeBuffer<Commands> command_buffer_;

        // Subscribers
        ros::Subscriber sub_q_cmd_;
        ros::Subscriber sub_forcetorque_sensor_;
        ros::Subscriber sub_command_;

        // Control parameters
        int control_stage_;
        double dt_;
        double time_;
        double total_time_;

        // Desired positions
        Eigen::VectorXd desired_position_A_;
        double x_d_x_prev_, x_d_y_prev_, x_d_z_prev_;

        // Force-torque data
        KDL::Wrench f_cur_;
        double filt_z_old_;

        // Jacobian and task-space variables
        KDL::Jacobian J_;
    };

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdaptiveImpedanceController, controller_interface::ControllerBase)