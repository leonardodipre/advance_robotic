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
#include "std_msgs/Int32.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

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
            desired_position_A_ << 0.4, 0.0, 0.3;
            

            // Subscribe to topics
            sub_q_cmd_ = n.subscribe("command", 1, &AdaptiveImpedanceController::commandCB, this);
            sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/elfin/elfin/ft_sensor_topic", 1, &AdaptiveImpedanceController::updateFTsensor, this);
            sub_command_ = n.subscribe("/motion_command", 1000, &AdaptiveImpedanceController::commandPosition, this);

            // Yolo pose
            sub_transformed_centers_ = n.subscribe("/transformed_centers", 1, &AdaptiveImpedanceController::transformedCentersCB, this);
            //command 
            sub_controller_command_ = n.subscribe("/controller_command", 1, &AdaptiveImpedanceController::controllerCommandCB, this);
            control_stage_buffer_.writeFromNonRT(0);

            //error
            pub_end_effector_error_ = n.advertise<geometry_msgs::Vector3Stamped>("/end_effector_error", 10);

            error_traj_q =  n.advertise<std_msgs::Float64MultiArray>("/erro_traj_pose", 1000);

            error_traj_qd =  n.advertise<std_msgs::Float64MultiArray>("/erro_traj_velocity", 1000);
            
            // Initialize control stage
            control_stage_ = 1;

            //equilibrium
            contact_detected_ = false;
            contact_force_threshold_ = 3.0;  // Adjust as needed

            // Initialize admittance control parameters
            M_adm_ = 1.0;    // Mass (kg)
            B_adm_ = 50.0;   // Damping (N·s/m)
            K_adm_ = 500.0;  // Stiffness (N/m)

            x_desired_ = Eigen::Vector3d::Zero();
            x_dot_desired_ = Eigen::Vector3d::Zero();
            x_eq_ = Eigen::Vector3d::Zero();

            // Initialize the desired force value in the x-direction
            desired_force_x_ = 5.0;
            push_force_x = 150.0;
            // Load PID gains for x-direction force control
            Kp_x_ =  100.0;
            Kd_x_= 10.0;
            // Load PID gains for z-direction position control
            n.param("Kp_z", Kp_z_, 1000.0);
            n.param("Kd_z", Kd_z_, 100.0);

            // Initialize desired position in z-direction (set to current position or a specific value)
            desired_position_z_ = 0.3; // Example value, adjust as needed

            // Initialize previous force error
            F_error_x_prev_ = 0.0;

            // Initialize publishers for telemetry data
            pub_control_stage_ = n.advertise<std_msgs::Int32>("/control_stage", 10);
            pub_force_error_ = n.advertise<std_msgs::Float64>("/force_error", 10);
            pub_end_effector_position_ = n.advertise<geometry_msgs::Vector3Stamped>("/end_effector_position", 10);
            pub_desired_end_effector_position_ = n.advertise<geometry_msgs::Vector3Stamped>("/desired_end_effector_position", 10);
            pub_joint_positions_ = n.advertise<std_msgs::Float64MultiArray>("/joint_positions", 10);
            pub_desired_joint_positions_ = n.advertise<std_msgs::Float64MultiArray>("/desired_joint_positions", 10);
            pub_force_sensor_readings_ = n.advertise<geometry_msgs::WrenchStamped>("/force_sensor_readings", 10);
            pub_computed_torques_ = n.advertise<std_msgs::Float64MultiArray>("/computed_torques", 10);
            pub_joint_velocities_ = n.advertise<std_msgs::Float64MultiArray>("/joint_velocities", 10);
            pub_desired_joint_velocities_ = n.advertise<std_msgs::Float64MultiArray>("/desired_joint_velocities", 10);
            pub_end_effector_velocity_ = n.advertise<geometry_msgs::Vector3Stamped>("/end_effector_velocity", 10);
            pub_desired_end_effector_velocity_ = n.advertise<geometry_msgs::Vector3Stamped>("/desired_end_effector_velocity", 10);

            // Initialize additional publishers for force command and error
            pub_force_command_ = n.advertise<geometry_msgs::WrenchStamped>("/force_command", 10);
            pub_force_control_error_ = n.advertise<geometry_msgs::Vector3Stamped>("/force_control_error", 10);

            // Initialize variables for publishing
            q_desired_.resize(n_joints_);
            qd_desired_.resize(n_joints_);
            xdot_.resize(6);
            x_dot_desired_vec6_.resize(6); // Renamed variable to avoid conflict

            return true;
        }

        double trajectory_generator_pos(double dStart, double dEnd, double dDuration, double t)
        {
            double dA0 = dStart;
            double dA3 = (20.0 * dEnd - 20.0 * dStart) / (2.0 * dDuration * dDuration * dDuration);
            double dA4 = (30.0 * dStart - 30.0 * dEnd) / (2.0 * dDuration * dDuration * dDuration * dDuration);
            double dA5 = (12.0 * dEnd - 12.0 * dStart) / (2.0 * dDuration * dDuration * dDuration * dDuration * dDuration);

            return dA0 + dA3 * t * t * t + dA4 * t * t * t * t + dA5 * t * t * t * t * t;
        }

        double trajectory_generator_vel(double dStart, double dEnd, double dDuration, double t)
        {
            double dA3 = (20.0 * dEnd - 20.0 * dStart) / (2.0 * dDuration * dDuration * dDuration);
            double dA4 = (30.0 * dStart - 30.0 * dEnd) / (2.0 * dDuration * dDuration * dDuration * dDuration);
            double dA5 = (12.0 * dEnd - 12.0 * dStart) / (2.0 * dDuration * dDuration * dDuration * dDuration * dDuration);

            return 3.0 * dA3 * t * t + 4.0 * dA4 * t * t * t + 5.0 * dA5 * t * t * t * t;
        }

        double trajectory_generator_acc(double dStart, double dEnd, double dDuration, double t)
        {
            double dA3 = (20.0 * dEnd - 20.0 * dStart) / (2.0 * dDuration * dDuration * dDuration);
            double dA4 = (30.0 * dStart - 30.0 * dEnd) / (2.0 * dDuration * dDuration * dDuration * dDuration);
            double dA5 = (12.0 * dEnd - 12.0 * dStart) / (2.0 * dDuration * dDuration * dDuration * dDuration * dDuration);

            return 6.0 * dA3 * t + 12.0 * dA4 * t * t + 20.0 * dA5 * t * t * t;
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

            control_stage_buffer_.initRT(0);

            desired_position_set_ = false;
            previous_control_mode_ = -1;

            admittance_initialized_ = false;

            trajectory_initialized_ = false; // Initialize trajectory flag

            // Initialize q_desired_ and qd_desired_
            q_desired_.resize(n_joints_);
            qd_desired_.resize(n_joints_);

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

            // Store end-effector velocity
            xdot_ = xdot; // xdot_ is Eigen::VectorXd of size n_joints_

            int control_mode = *(control_stage_buffer_.readFromRT());
            //ROS_INFO("Control mode %d", control_mode);

            // Error
            Eigen::VectorXd ex(3);

            // Variables for publishing
            Eigen::Vector3d desired_position_to_publish = current_position; // Initialize with current position
            q_desired_.data = q_.data; // Initialize desired joint positions
            qd_desired_.data = qdot_.data; // Initialize desired joint velocities
            x_dot_desired_vec6_.setZero(); // Initialize desired end-effector velocity for publishing
            F_error_x_ = 0.0; // Initialize force error

            // Initialize force command to publish
            Eigen::Vector3d force_command_to_publish = Eigen::Vector3d::Zero();

            KDL::Frame stored_end_effector_pose_;

            if (control_mode != previous_control_mode_)
            {
                desired_position_set_ = false;
                previous_control_mode_ = control_mode;
            }

            switch (control_mode)
            {
                case 0:
                {
                    // Compute position error
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

                    desired_position_to_publish = desired_position_A_.head<3>(); // Update desired position for publishing

                    // Assign force command to publish
                    force_command_to_publish = F_desired.head<3>();

                    break;
                }
                case 1:
                {
                    // Only set the desired position once
                    if (!desired_position_set_)
                    {
                        desired_position_stored_ = *desired_position_buffer_.readFromRT();
                        desired_position_set_ = true;
                        ROS_INFO_STREAM("Stored desired position: [" 
                                        << desired_position_stored_.transpose() << "]");
                    }

                    // Use the stored desired position
                    Eigen::Vector3d desired_position = desired_position_stored_;

                    // Compute position error
                    ex = desired_position - current_position;

                    // Compute velocity error (desired velocity is zero)
                    Eigen::VectorXd xdot_error = -xdot.head<3>();

                    // Define task-space PID gains
                    Eigen::VectorXd Kp(3), Kd(3);
                    Kp << 1000, 1000, 1000; // Position gains
                    Kd << 100, 100, 100;    // Velocity gains

                    // Compute desired task-space force
                    Eigen::Vector3d F_desired = Kp.cwiseProduct(ex) + Kd.cwiseProduct(xdot_error);

                    // Extend F_desired to 6D (force and torque)
                    Eigen::VectorXd F_ext = Eigen::VectorXd::Zero(6);
                    F_ext.head<3>() = F_desired;

                    // Compute joint torques
                    tau_d_.data = J_.data.transpose() * F_ext + C_.data + G_.data;

                    desired_position_to_publish = desired_position; // Update desired position for publishing

                    // Assign force command to publish
                    force_command_to_publish = F_desired;

                    // Contact detection (optional)
                    double force_magnitude = f_cur_.force.Norm();
                    if (force_magnitude > contact_force_threshold_)
                    {
                        ROS_INFO("Contact detected, switching to admittance control.");
                        contact_detected_ = true;

                        // Set the equilibrium position to the current position
                        x_eq_ = current_position;

                        // Initialize desired position and velocity for admittance controller
                        x_desired_ = current_position;
                        x_dot_desired_ = Eigen::Vector3d::Zero();

                        // Store the end-effector pose
                        fk_solver_->JntToCart(q_, stored_end_effector_pose_);
                        ROS_INFO_STREAM("Pose: [x: " << stored_end_effector_pose_.p.x()
                                        << ", y: " << stored_end_effector_pose_.p.y()
                                        << ", z: " << stored_end_effector_pose_.p.z() << "]");

                        // Optionally switch to control mode 2
                        control_stage_buffer_.writeFromNonRT(2);
                    }

                    break;
                }
                case 2:
                {
                    // Define impedance parameters
                    Eigen::Vector3d K_impedance(500.0, 500.0, 500.0); // Stiffness in N/m
                    Eigen::Vector3d B_impedance(50.0, 50.0, 50.0);    // Damping in N·s/m

                    // Compute position and velocity errors
                    Eigen::Vector3d x_error = current_position - x_eq_; // x_eq_ is the equilibrium position
                    Eigen::Vector3d xdot_error = xdot.head(3);          // Desired velocity is zero

                    // Convert f_cur_.force (KDL::Vector) to Eigen::Vector3d
                    Eigen::Vector3d f_cur_force;
                    f_cur_force << f_cur_.force.x(), f_cur_.force.y(), f_cur_.force.z();

                    // Compute commanded force using the impedance control law
                    Eigen::Vector3d F_cmd = -K_impedance.cwiseProduct(x_error)
                                            - B_impedance.cwiseProduct(xdot_error)
                                            - f_cur_force;

                    // Extend F_cmd to 6D
                    Eigen::VectorXd F_ext = Eigen::VectorXd::Zero(6);
                    F_ext.head(3) = F_cmd;

                    // Compute joint torques
                    tau_d_.data = J_.data.transpose() * F_ext + C_.data + G_.data;

                    // Update variables for publishing
                    desired_position_to_publish = x_eq_;
                    x_dot_desired_vec6_.head(3) = Eigen::Vector3d::Zero();

                    // Assign force command to publish
                    force_command_to_publish = F_cmd;

                    break;
                }
                case 3:
                {
                    // Initialize admittance variables once
                    if (!admittance_initialized_)
                    {
                        x_desired_ = x_eq_; // Start from equilibrium position
                        x_dot_desired_ = Eigen::Vector3d::Zero();
                        admittance_initialized_ = true;
                        ROS_INFO("Admittance control for pushing initialized.");
                    }

                    // Set the desired pushing force
                    desired_force_x_ = push_force_x;

                    // Compute force error in x-direction
                    F_error_x_ = desired_force_x_ - f_cur_.force.x();

                    // Compute desired acceleration using admittance model in x-direction
                    double x_ddot_desired_x = (1.0 / M_adm_) * (F_error_x_ - B_adm_ * x_dot_desired_(0) - K_adm_ * (x_desired_(0) - x_eq_(0)));

                    // Update desired velocity and position in x-direction
                    x_dot_desired_(0) += x_ddot_desired_x * dt_;
                    x_desired_(0) += x_dot_desired_(0) * dt_;

                    // Keep y and z positions constant at equilibrium or adjust as needed
                    x_desired_(1) = x_eq_(1);
                    x_dot_desired_(1) = 0.0;
                    x_desired_(2) = x_eq_(2);
                    x_dot_desired_(2) = 0.0;

                    // Compute position and velocity errors
                    ex = x_desired_ - current_position;
                    Eigen::Vector3d xdot_error = x_dot_desired_ - xdot.head<3>();

                    // Define task-space PID gains
                    Eigen::Vector3d Kp(1000.0, 1000.0, 1000.0);
                    Eigen::Vector3d Kd(100.0, 100.0, 100.0);

                    // Compute desired task-space force
                    Eigen::Vector3d F_desired = Kp.cwiseProduct(ex) + Kd.cwiseProduct(xdot_error);

                    // Extend F_desired to 6D (force and torque)
                    Eigen::VectorXd F_ext = Eigen::VectorXd::Zero(6);
                    F_ext.head<3>() = F_desired;

                    // Compute joint torques
                    tau_d_.data = J_.data.transpose() * F_ext + C_.data + G_.data;

                    desired_position_to_publish = x_desired_; // Update desired position for publishing

                    // Assign force command to publish
                    force_command_to_publish = F_desired;

                    // Prepare desired end-effector velocity for publishing
                    x_dot_desired_vec6_.head<3>() = x_dot_desired_;
                    x_dot_desired_vec6_.tail<3>().setZero(); // Assuming zero angular velocity

                    break;
                }
                case 4:
                {
                    // Initialize trajectory once
                    if (!trajectory_initialized_)
                    {
                        trajectory_start_time_ = total_time_;
                        trajectory_duration_ = 5.0; // Duration in seconds

                        // Store starting joint positions
                        q_start_.resize(n_joints_);
                        for (int i = 0; i < n_joints_; i++)
                        {
                            q_start_(i) = q_(i);
                        }
                     
                        // Set goal joint positions (convert degrees to radians)
                        q_goal_.resize(n_joints_);
                        q_goal_.data.setZero(); // Ensure the data vector is initialized
                        q_goal_(0) = 0.0 * PI / 180.0;
                        q_goal_(1) = -15.0 * PI / 180.0;
                        q_goal_(2) = 60.0 * PI / 180.0;
                        q_goal_(3) = 0.0 * PI / 180.0;
                        q_goal_(4) = 80.0 * PI / 180.0;
                        q_goal_(5) = 0.0 * PI / 180.0;
                        q_goal_(6) = 0.0 * PI / 180.0;
                       

                        trajectory_initialized_ = true;
                        ROS_INFO("Trajectory initialized for control mode 4.");
                    }

                    // Compute elapsed time
                    double t = total_time_ - trajectory_start_time_;

                    // Ensure time doesn't exceed duration
                    if (t > trajectory_duration_)
                    {
                        t = trajectory_duration_;
                        // Optionally reset trajectory_initialized_ or switch control mode
                        trajectory_initialized_ = false;
                        control_stage_buffer_.writeFromNonRT(0); // Switch back to mode 0
                    }

                    // Prepare desired joint positions, velocities, and accelerations
                    KDL::JntArray q_desired(n_joints_);
                    KDL::JntArray qd_desired(n_joints_);
                    KDL::JntArray qdd_desired(n_joints_);

                    for (int i = 0; i < n_joints_; i++)
                    {
                        q_desired(i) = trajectory_generator_pos(q_start_(i), q_goal_(i), trajectory_duration_, t);
                        qd_desired(i) = trajectory_generator_vel(q_start_(i), q_goal_(i), trajectory_duration_, t);
                        qdd_desired(i) = trajectory_generator_acc(q_start_(i), q_goal_(i), trajectory_duration_, t);
                    }

                    // Compute errors
                    KDL::JntArray q_error(n_joints_);
                    KDL::JntArray qd_error(n_joints_);

                    for (int i = 0; i < n_joints_; i++)
                    {
                        q_error(i) = q_desired(i) - q_(i);
                        qd_error(i) = qd_desired(i) - qdot_(i);
                    }

                    //publishing error in position
                    std_msgs::Float64MultiArray q_error_msg;
                    q_error_msg.data.resize(n_joints_);
                    for (int i = 0; i < n_joints_; i++)
                    {
                        q_error_msg.data[i] = q_error(i);
                    }

                    // Publish position errors
                    error_traj_q.publish(q_error_msg);

                    //publishing error in velocity
                    std_msgs::Float64MultiArray qd_error_msg;
                    qd_error_msg.data.resize(n_joints_);
                    for (int i = 0; i < n_joints_; i++)
                    {
                        qd_error_msg.data[i] = qd_error(i);
                    }
                    error_traj_qd.publish(qd_error_msg);

                    // Get Mass matrix
                    KDL::JntSpaceInertiaMatrix M(n_joints_);
                    id_solver_->JntToMass(q_, M);

                    // Define PD gains
                    Eigen::VectorXd Kp(n_joints_), Kd(n_joints_);
                    for (int i = 0; i < n_joints_; i++)
                    {
                        Kp(i) = 500.0; // Adjust as needed
                        Kd(i) = 100.0;  // Adjust as needed
                    }

                    // Compute control torques
                    tau_d_.data = M.data * qdd_desired.data + Kp.cwiseProduct(q_error.data) + Kd.cwiseProduct(qd_error.data) + C_.data + G_.data;

                    // Update desired joint positions and velocities for publishing
                    q_desired_.data = q_desired.data;
                    qd_desired_.data = qd_desired.data;

                    // No force command to publish in this mode
                    force_command_to_publish = Eigen::Vector3d::Zero();

                    break;
                }
                default:
                {
                    ROS_WARN("Invalid control mode selected: %d", control_mode);
                    break;
                }
            }

            // Apply torque commands
            for (int i = 0; i < n_joints_; i++)
            {
                joints_[i].setCommand(tau_d_.data(i));
            }

            // Compute force error
            Eigen::Vector3d f_cur_force;
            f_cur_force << f_cur_.force.x(), f_cur_.force.y(), f_cur_.force.z();
            Eigen::Vector3d force_error = force_command_to_publish - f_cur_force;

            // Publish force command
            geometry_msgs::WrenchStamped force_command_msg;
            force_command_msg.header.stamp = ros::Time::now();
            force_command_msg.wrench.force.x = force_command_to_publish(0);
            force_command_msg.wrench.force.y = force_command_to_publish(1);
            force_command_msg.wrench.force.z = force_command_to_publish(2);
            force_command_msg.wrench.torque.x = 0.0;
            force_command_msg.wrench.torque.y = 0.0;
            force_command_msg.wrench.torque.z = 0.0;
            pub_force_command_.publish(force_command_msg);

            // Publish force control error
            geometry_msgs::Vector3Stamped force_control_error_msg;
            force_control_error_msg.header.stamp = ros::Time::now();
            force_control_error_msg.vector.x = force_error(0);
            force_control_error_msg.vector.y = force_error(1);
            force_control_error_msg.vector.z = force_error(2);
            pub_force_control_error_.publish(force_control_error_msg);

            // Publish error in task space
            geometry_msgs::Vector3Stamped error_msg;
            error_msg.header.stamp = ros::Time::now();
            error_msg.vector.x = ex(0);
            error_msg.vector.y = ex(1);
            error_msg.vector.z = ex(2);
            pub_end_effector_error_.publish(error_msg);

            // Publish force error
            std_msgs::Float64 force_error_msg;
            force_error_msg.data = F_error_x_;
            pub_force_error_.publish(force_error_msg);

            // Publish current end-effector position
            geometry_msgs::Vector3Stamped ee_position_msg;
            ee_position_msg.header.stamp = ros::Time::now();
            ee_position_msg.vector.x = current_position(0);
            ee_position_msg.vector.y = current_position(1);
            ee_position_msg.vector.z = current_position(2);
            pub_end_effector_position_.publish(ee_position_msg);

            // Publish desired end-effector position
            geometry_msgs::Vector3Stamped desired_ee_position_msg;
            desired_ee_position_msg.header.stamp = ros::Time::now();
            desired_ee_position_msg.vector.x = desired_position_to_publish(0);
            desired_ee_position_msg.vector.y = desired_position_to_publish(1);
            desired_ee_position_msg.vector.z = desired_position_to_publish(2);
            pub_desired_end_effector_position_.publish(desired_ee_position_msg);

            // Publish joint positions
            std_msgs::Float64MultiArray joint_positions_msg;
            joint_positions_msg.data.resize(n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                joint_positions_msg.data[i] = q_(i);
            }
            pub_joint_positions_.publish(joint_positions_msg);

            // Publish desired joint positions
            std_msgs::Float64MultiArray desired_joint_positions_msg;
            desired_joint_positions_msg.data.resize(n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                desired_joint_positions_msg.data[i] = q_desired_(i);
            }
            pub_desired_joint_positions_.publish(desired_joint_positions_msg);

            // Publish force sensor readings
            geometry_msgs::WrenchStamped force_sensor_msg;
            force_sensor_msg.header.stamp = ros::Time::now();
            force_sensor_msg.wrench.force.x = f_cur_.force(0);
            force_sensor_msg.wrench.force.y = f_cur_.force(1);
            force_sensor_msg.wrench.force.z = f_cur_.force(2);
            force_sensor_msg.wrench.torque.x = f_cur_.torque(0);
            force_sensor_msg.wrench.torque.y = f_cur_.torque(1);
            force_sensor_msg.wrench.torque.z = f_cur_.torque(2);
            pub_force_sensor_readings_.publish(force_sensor_msg);

            // Publish computed torques
            std_msgs::Float64MultiArray computed_torques_msg;
            computed_torques_msg.data.resize(n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                computed_torques_msg.data[i] = tau_d_(i);
            }
            pub_computed_torques_.publish(computed_torques_msg);

            // Publish joint velocities
            std_msgs::Float64MultiArray joint_velocities_msg;
            joint_velocities_msg.data.resize(n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                joint_velocities_msg.data[i] = qdot_(i);
            }
            pub_joint_velocities_.publish(joint_velocities_msg);

            // Publish desired joint velocities
            std_msgs::Float64MultiArray desired_joint_velocities_msg;
            desired_joint_velocities_msg.data.resize(n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                desired_joint_velocities_msg.data[i] = qd_desired_(i);
            }
            pub_desired_joint_velocities_.publish(desired_joint_velocities_msg);

            // Publish end-effector velocity
            geometry_msgs::Vector3Stamped ee_velocity_msg;
            ee_velocity_msg.header.stamp = ros::Time::now();
            ee_velocity_msg.vector.x = xdot(0);
            ee_velocity_msg.vector.y = xdot(1);
            ee_velocity_msg.vector.z = xdot(2);
            pub_end_effector_velocity_.publish(ee_velocity_msg);

            // Publish desired end-effector velocity
            geometry_msgs::Vector3Stamped desired_ee_velocity_msg;
            desired_ee_velocity_msg.header.stamp = ros::Time::now();
            desired_ee_velocity_msg.vector.x = x_dot_desired_(0);
            desired_ee_velocity_msg.vector.y = x_dot_desired_(1);
            desired_ee_velocity_msg.vector.z = x_dot_desired_(2);
            pub_desired_end_effector_velocity_.publish(desired_ee_velocity_msg);

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
            // Apply low-pass filter to the x-component of the force
            double filt_force_x = first_order_lowpass_filter_z(msg->wrench.force.x);

            // Update the force-torque measurements
            f_cur_.force = KDL::Vector(filt_force_x,
                                    msg->wrench.force.y,
                                    msg->wrench.force.z);
            f_cur_.torque = KDL::Vector(msg->wrench.torque.x,
                                        msg->wrench.torque.y,
                                        msg->wrench.torque.z);
        }

        void controllerCommandCB(const std_msgs::Int32::ConstPtr &msg)
        {
            int new_stage = msg->data;
            control_stage_buffer_.writeFromNonRT(new_stage);
            ROS_INFO_STREAM("Received control stage command: " << new_stage);
        }

        // Callback for /transformed_centers
        void transformedCentersCB(const geometry_msgs::PointStamped::ConstPtr &msg)
        {
            Eigen::Vector3d new_desired_position;
            new_desired_position << msg->point.x, msg->point.y, msg->point.z;
            desired_position_buffer_.writeFromNonRT(new_desired_position);
            ROS_INFO_STREAM("Received new desired position: [" 
                            << new_desired_position.transpose() << "]");
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

        //Yolo center in robot frame
        ros::Subscriber sub_transformed_centers_;
        Eigen::Vector3d desired_position_;

        //command plan
        ros::Subscriber sub_controller_command_;

        realtime_tools::RealtimeBuffer<int> control_stage_buffer_;
        realtime_tools::RealtimeBuffer<Eigen::Vector3d> desired_position_buffer_;

        // Equilibrium controller 
        // Variables for contact detection and admittance control
        bool contact_detected_;
        double contact_force_threshold_;
        Eigen::Vector3d x_eq_;              // Equilibrium position (desired position)
        Eigen::Vector3d x_desired_;         // Desired position
        Eigen::Vector3d x_dot_desired_;     // Desired velocity

        // Admittance control parameters
        double M_adm_, B_adm_, K_adm_;      // Admittance parameters: Mass, Damping, Stiffness

        // Error
        ros::Publisher pub_end_effector_error_;

        // Force reading from sensor

        //
        double desired_force_x_;    // Desired force in the x-direction
        double Kp_x_, Kd_x_;        // PID gains for x-force control
        double desired_position_z_; // Desired position in z-direction
        double Kp_z_, Kd_z_;        // PID gains for z-position control
        double F_error_x_prev_;     // Previous force error in x-direction

        // Save tracked
        bool desired_position_set_;
        Eigen::Vector3d desired_position_stored_;
        int previous_control_mode_;  

        // Admittance 2.0
        bool admittance_initialized_; 
        double desired_force_z_; 

        // Push force
        double push_force_x; 

        // Case 4 trajectory planning
        bool trajectory_initialized_;
        double trajectory_start_time_;
        double trajectory_duration_;
        KDL::JntArray q_start_;
        KDL::JntArray q_goal_;

        // Error trajectory
        ros::Publisher error_traj_q;
        ros::Publisher error_traj_qd;

        // Additional publishers for telemetry data
        ros::Publisher pub_control_stage_;
        ros::Publisher pub_force_error_;
        ros::Publisher pub_end_effector_position_;
        ros::Publisher pub_desired_end_effector_position_;
        ros::Publisher pub_joint_positions_;
        ros::Publisher pub_desired_joint_positions_;
        ros::Publisher pub_force_sensor_readings_;
        ros::Publisher pub_computed_torques_;
        ros::Publisher pub_joint_velocities_;
        ros::Publisher pub_desired_joint_velocities_;
        ros::Publisher pub_end_effector_velocity_;
        ros::Publisher pub_desired_end_effector_velocity_;

        // Additional publishers for force command and error
        ros::Publisher pub_force_command_;
        ros::Publisher pub_force_control_error_;

        // Variables for publishing
        KDL::JntArray q_desired_; // Desired joint positions
        KDL::JntArray qd_desired_; // Desired joint velocities
        Eigen::VectorXd xdot_; // End-effector velocity
        Eigen::VectorXd x_dot_desired_vec6_; // Desired end-effector velocity for publishing
        double F_error_x_; // Force error in x-direction

    };

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdaptiveImpedanceController, controller_interface::ControllerBase)
