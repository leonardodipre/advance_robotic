#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <boost/scoped_ptr.hpp>

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI
#define JointMax 6
#define SaveDataMax 7

struct Commands
{
    KDL::JntArray qd;
    KDL::JntArray qd_dot;
    KDL::JntArray qd_ddot;
    Eigen::VectorXd xd;     // Desired task-space position (6 elements)
    Eigen::VectorXd xd_dot; // Desired task-space velocity (6 elements)
    Commands() {}
};


namespace arm_controllers{

	class AdaptiveImpedanceController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		~AdaptiveImpedanceController() {sub_q_cmd_.shutdown(); sub_forcetorque_sensor_.shutdown();}

		bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  		{	
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

			// urdf
			urdf::Model urdf;			
			if (!urdf.initParam("elfin/robot_description"))
			{
				ROS_ERROR("Failed to parse urdf file");
            	return false;
			}

			// joint handle
			for(int i=0; i<n_joints_; i++)
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

			// kdl parser
			if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)){
				ROS_ERROR("Failed to construct kdl tree");
				return false;
			}

			// kdl chain
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

            	for( it=segment_map.begin(); it != segment_map.end(); it++ )
              		ROS_ERROR_STREAM( "    "<<(*it).first);

            	return false;
			}

			gravity_ = KDL::Vector::Zero();
			gravity_(2) = -9.81;
			G_.resize(n_joints_);

			J_.resize(n_joints_); // Correctly resizes Jacobian for task-space control

			      // 6 rows for task space, n_joints_ columns for joint space
			qdot_.resize(n_joints_);         // 7-dimensional vector for joint velocities
			tau_d_.resize(n_joints_);        // 7-dimensional vector for joint torques
			C_.resize(n_joints_);
						
			// inverse dynamics solver
			id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );
			fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
			ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));

			jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

			//ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, fk_solver_, ik_vel_solver_));
			
			id_solver_->JntToCoriolis(q_, qdot_, C_);

			// command and state
			tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			tau_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_sp_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_cmd_old_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
			q_cmd_end_.data = Eigen::VectorXd::Zero(n_joints_);
			
			q_.data = Eigen::VectorXd::Zero(n_joints_);
			q_init_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_.data = Eigen::VectorXd::Zero(n_joints_);
			qdot_old_.data = Eigen::VectorXd::Zero(n_joints_);
			qddot_.data = Eigen::VectorXd::Zero(n_joints_);


			qd_.data = Eigen::VectorXd::Zero(n_joints_);
			qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
			qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
			
			xd = Eigen::VectorXd::Zero(6);
        	xd_dot = Eigen::VectorXd::Zero(6);

			
			

			for (size_t i = 0; i < 6; i++)
			{
				Xc_dot_(i) = 0.0;
				Xc_dot_old_(i) = 0.0;
				Xc_ddot_(i) = 0.0;
			}

			// gains
			Mbar_.resize(n_joints_);
			Mbar_dot_.resize(n_joints_);
			Ramda_.resize(n_joints_);
			Alpha_.resize(n_joints_);
			Omega_.resize(n_joints_);

			

			Xr_dot_ = 0.0;
			Xe_dot_ = 0.0;
			Xe_ddot_ = 0.0;
			Fd_ = 0.0;
			Fd_temp_ = 0.0;
			Fd_old_ = 0.0;
			Fe_ = 0.0;
			Fe_old_ = 0.0;
			M_ = 0.0;
			B_ = 0.0;
			del_B_ = 0.0;
			B_buffer_ = 0.0;
			PI_ = 0.0;
			PI_old_ = 0.0;

			filt_old_ = 0.0;
			filt_ = 0.0;
			tau_ = 1.0/(2*PI*9.0);

			f_cur_buffer_ = 0.0;

			experiment_mode_ = 0;

			std::vector<double> Mbar(n_joints_), Ramda(n_joints_), Alpha(n_joints_), Omega(n_joints_);
			for (size_t i=0; i<n_joints_; i++)
			{
				std::string si = boost::lexical_cast<std::string>(i+1);
				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/mbar", Mbar[i]) )
				{
					Mbar_(i) = Mbar[i];
				}
				else
				{
					std::cout << "/elfin/adaptive_impedance_controller/joint" + si + "/tdc/mbar" << std::endl;
					ROS_ERROR("Cannot find tdc/mbar gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/r", Ramda[i]) )
				{
					Ramda_(i) = Ramda[i];
				}
				else
				{
					ROS_ERROR("Cannot find tdc/r gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/a", Alpha[i]) )
				{
					Alpha_(i) = Alpha[i];
				}
				else
				{
					ROS_ERROR("Cannot find tdc/a gain");
					return false;
				}

				if ( n.getParam("/elfin/adaptive_impedance_controller/joint" + si + "/tdc/w", Omega[i]) )
				{
					Omega_(i) = Omega[i];
				}
				else
				{
					ROS_ERROR("Cannot find tdc/w gain");
					return false;
				}
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/fd", Fd_temp_))
			{
				ROS_ERROR("Cannot find aci/fd");
				return false;
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/m", M_))
			{
				ROS_ERROR("Cannot find aci/m");
				return false;
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/b", B_))
			{
				ROS_ERROR("Cannot find aci/b");
				return false;
			}

			if (!n.getParam("/elfin/adaptive_impedance_controller/mode", experiment_mode_))
			{
				ROS_ERROR("Cannot find mode");
				return false;
			}

			// Initialize control stage
			control_stage_ = 1;
			position_tolerance_ = 0.05; // e.g., 1 cm tolerance

			// Initialize desired positions
			desired_position_A_ = Eigen::VectorXd::Zero(6);
			desired_position_B_ = Eigen::VectorXd::Zero(6);

			// Set desired position A
			desired_position_A_(0) = 0.5; // x-coordinate
			desired_position_A_(1) = 0.0; // y-coordinate
			desired_position_A_(2) = 0.09; // z-coordinate

			// Set desired position B
			desired_position_B_(0) = 0.5; // x-coordinate
			desired_position_B_(1) = 0.5; // y-coordinate
			desired_position_B_(2) = 0.05; // z-coordinate

			// Initialize force control parameters
			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/fd_z", Fd_z_))
			{
				Fd_z_ = -50.0; // Desired force in z-direction (e.g., pressing down with 10 N)
			}
			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/kf_z", Kf_z_))
			{
				Kf_z_ = 1.0; // Force control gain for the 3
				//Kf_z_ = 0.001; //fr 2
			}
			if (!n.getParam("/elfin/adaptive_impedance_controller/aic/k", K_))
			{
				K_ = 0.5; // If not found, set stiffness to zero
			}

			// Initialize desired velocities
			x_dot_desired_x = 0.1; // m/s (adjust as needed)
			x_dot_desired_y = 0.0; // m/s

			ex_ = Eigen::VectorXd::Zero(6); 

			// command
			sub_q_cmd_ = n.subscribe("command", 1, &AdaptiveImpedanceController::commandCB, this);
			sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/elfin/elfin/ft_sensor_topic", 1, &AdaptiveImpedanceController::updateFTsensor, this);

			pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);

			sub_command_ = n.subscribe("/motion_command", 1000, &AdaptiveImpedanceController::commandPosition, this);

			// Initialize the command buffer with zeros
			Commands initial_commands;
			initial_commands.qd.data = Eigen::VectorXd::Zero(6);
			initial_commands.qd_dot.data = Eigen::VectorXd::Zero(6);
			initial_commands.qd_ddot.data = Eigen::VectorXd::Zero(6);
			initial_commands.xd = Eigen::VectorXd::Zero(6);
			initial_commands.xd_dot = Eigen::VectorXd::Zero(6);

			command_buffer_.initRT(initial_commands);

			//velcoity case 3
			ex_ = Eigen::VectorXd::Zero(6);
			xdot_error = Eigen::VectorXd::Zero(6);
			max_Fz = 50.0; // Set maximum force limit as needed


			// In your init function
			double mass_value = 1.0;      // Adjust as needed
			double damping_value = 10.0;  // Adjust as needed
			double stiffness_value = 100.0; // Adjust as needed

			//impednace controller
			Md = Eigen::MatrixXd::Identity(6, 6) * mass_value;
			Kd = Eigen::MatrixXd::Identity(6, 6) * damping_value;
			Kp = Eigen::MatrixXd::Identity(6, 6) * stiffness_value;

						return true;
  		}

		void starting(const ros::Time& time)
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


			x_eq_z_ = x_d_z_prev_;     // Equilibrium position in z-direction
			x_desired_z_ = x_eq_z_;    // Desired z-position starts at equilibrium
			x_dot_desired_z_ = 0.0;    // Initial desired z-velocity is zero


    		ROS_INFO("Starting Adaptive Impedance Controller");

			
		}

		void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
		{
			if(msg->data.size()!=n_joints_)
			{ 
				ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
				return; 
			}

    		for (unsigned int i = 0; i<n_joints_; i++)
    			q_cmd_sp_(i) = msg->data[i];
		}

		void commandPosition(const std_msgs::Float64MultiArrayConstPtr &msg)
		{
			 size_t expected_size = 3 * 6 + 12; // 18 + 12 = 30
			if (msg->data.size() != expected_size)
			{
				ROS_ERROR_STREAM("Dimension of command (" << msg->data.size()
								<< ") does not match expected size (" << expected_size << ")! Not executing!");
				return;
			}

			Commands cmd;
			cmd.qd.data = Eigen::VectorXd::Zero(6);
			cmd.qd_dot.data = Eigen::VectorXd::Zero(6);
			cmd.qd_ddot.data = Eigen::VectorXd::Zero(6);
			cmd.xd = Eigen::VectorXd::Zero(6);
			cmd.xd_dot = Eigen::VectorXd::Zero(6);

			// Read joint positions, velocities, and accelerations
			for (size_t i = 0; i < 6; i++)
			{
				cmd.qd(i) = msg->data[i];
				cmd.qd_dot(i) = msg->data[6 + i];
				cmd.qd_ddot(i) = msg->data[2 * 6 + i];
			}

			// Read task-space coordinates and velocities
			size_t offset = 3 * 6;
			for (size_t i = 0; i < 6; i++)
			{
				cmd.xd(i) = msg->data[offset + i];
				cmd.xd_dot(i) = msg->data[offset + 6 + i];
			}

			command_buffer_.writeFromNonRT(cmd);
		
		}



		void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
		{
			// Convert Wrench msg to KDL wrench
			geometry_msgs::Wrench f_meas = msg->wrench;

			f_cur_[0] = f_meas.force.x;
			f_cur_buffer_ = f_meas.force.y;
			f_cur_[2] = f_meas.force.z;
			f_cur_[3] = f_meas.torque.x;
			f_cur_[4] = f_meas.torque.y;
			f_cur_[5] = f_meas.torque.z;

			f_cur_[2] = first_order_lowpass_filter_z(f_cur_[2]);
		}

		// load gain is not permitted during controller loading?
		void loadGainCB()
		{
			
		}

  		void update(const ros::Time& time, const ros::Duration& period)
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

			Eigen::VectorXd xdot_desired(6);
			xdot_desired.setZero();

			Eigen::VectorXd xdot_error(6);
			xdot_error.setZero();

			// Forward kinematics to get current end-effector pose
			KDL::Frame end_effector_pose;
			fk_solver_->JntToCart(q_, end_effector_pose);

			// Extract current end-effector position
			Eigen::VectorXd current_position(3);
			current_position << end_effector_pose.p.x(), end_effector_pose.p.y(), end_effector_pose.p.z();

			// Compute current end-effector velocity
			jac_solver_->JntToJac(q_, J_);
			xdot_ = J_.data * qdot_.data;

			if (control_stage_ == 1)
			{
				// --- Stage 1: Move to Position A ---

				// Compute position error
				Eigen::VectorXd ex_(6);
				ex_.setZero();
				ex_.head(3) = desired_position_A_.head(3) - current_position;

				// Compute velocity error (desired velocity is zero)
				Eigen::VectorXd xdot_error(6);
				xdot_error.setZero();
				xdot_error.head(3) = -xdot_.head(3);

				// Define task-space PID gains
				Eigen::VectorXd Kp(6), Kd(6);
				Kp << 1000, 1000, 1000, 50, 50, 50; // Adjust as needed
				Kd << 100, 100, 100, 10, 10, 10;    // Adjust as needed

				// Compute desired task-space force
				Eigen::VectorXd F_desired(6);
				F_desired = Kp.cwiseProduct(ex_) + Kd.cwiseProduct(xdot_error);

				// Compute joint torques
				tau_d_.data = J_.data.transpose() * F_desired + C_.data + G_.data;

				// Check if position error is within tolerance
				if (ex_.head(3).norm() < position_tolerance_)
				{
					// Switch to Stage 2
					control_stage_ = 4;

					// Initialize previous desired positions for Stage 2
					x_d_x_prev_ = current_position(0);
					x_d_y_prev_ = current_position(1);
					x_d_z_prev_ = current_position(2);

					// Set initial desired positions
					xd(0) = x_d_x_prev_;
					xd(1) = x_d_y_prev_;
					xd(2) = x_d_z_prev_;

					// Initialize admittance control variables
					x_eq_z_ = x_d_z_prev_;     // Equilibrium position in z-direction
					x_desired_z_ = x_eq_z_;    // Desired z-position starts at equilibrium
					x_dot_desired_z_ = 0.0;    // Initial desired z-velocity is zero
				}
			}
			if (control_stage_ == 2)
			{
				// --- Stage 2: Admittance Control in Z-Direction ---

				// Circular motion in x and y (position control)
				double omega = 1.0;     // Angular velocity in rad/s
				double radius = 0.1;    // Radius in meters
				double theta = omega * total_time_;

				double x_center = x_d_x_prev_;
				double y_center = x_d_y_prev_;

				// Desired positions in x and y
				xd(0) = x_center + radius * cos(theta);
				xd(1) = y_center + radius * sin(theta);

				// Desired velocities in x and y
				xdot_desired(0) = -radius * omega * sin(theta);
				xdot_desired(1) = radius * omega * cos(theta);

				// --- Admittance Control in Z-Direction ---

				// Measured force in z-direction
				double Fm_z = f_cur_[2]; // Force sensor reading

				// Desired force in z-direction (e.g., pressing down with Fd_z_ Newtons)
				double force_error_z = Fd_z_ - Fm_z;

				// Displacement from equilibrium in z
				double x_z_displacement = x_desired_z_ - x_eq_z_;

				// **Admittance Equation** (solving for acceleration):
				// x_ddot_desired_z = (1 / M_) * (force_error_z - B_ * x_dot_desired_z_ - K_ * x_z_displacement)
				double x_ddot_desired_z = (1.0 / M_) * (force_error_z - B_ * x_dot_desired_z_ - K_ * x_z_displacement);

				// Integrate acceleration to get velocity
				x_dot_desired_z_ += x_ddot_desired_z * dt_;

				// Integrate velocity to get position
				x_desired_z_ += x_dot_desired_z_ * dt_;

				// Update desired position and velocity in z-direction
				xd(2) = x_desired_z_;
				xdot_desired(2) = x_dot_desired_z_;

				// Compute position and velocity errors
				ex_.head(3) = xd.head(3) - current_position;
				xdot_error.head(3) = xdot_desired.head(3) - xdot_.head(3);

				// Define task-space PD gains
				Eigen::VectorXd Kp(6), Kd(6);
				Kp << 1000, 1000, 1000, 50, 50, 50; // Proportional gains
				Kd << 100, 100, 100, 10, 10, 10;    // Derivative gains

				// **Control Law**:
				// F_desired = Kp * ex_ + Kd * xdot_error
				Eigen::VectorXd F_desired(6);
				F_desired = Kp.cwiseProduct(ex_) + Kd.cwiseProduct(xdot_error);

				// **Compute Joint Torques**:
				// tau_d_ = J^T * F_desired + Coriolis + Gravity
				tau_d_.data = J_.data.transpose() * F_desired + C_.data + G_.data;
			}

			else if (control_stage_ == 3)
			{
				// --- Stage 3: Direct Force Control in Z-Direction ---

				// Circular motion in x and y (position control)
				double omega = 1.0;     // Angular velocity in rad/s
				double radius = 0.1;    // Radius in meters
				double theta = omega * total_time_;

				double x_center = x_d_x_prev_;
				double y_center = x_d_y_prev_;

				// Desired positions in x and y
				xd(0) = x_center + radius * cos(theta);
				xd(1) = y_center + radius * sin(theta);

				// Desired velocities in x and y
				xdot_desired(0) = -radius * omega * sin(theta);
				xdot_desired(1) = radius * omega * cos(theta);

				// Compute position and velocity errors in x and y
				ex_.setZero();
				ex_.head(2) = xd.head(2) - current_position.head(2);
				xdot_error.setZero();
				xdot_error.head(2) = xdot_desired.head(2) - xdot_.head(2);

				// Define task-space PD gains for x and y
				Eigen::VectorXd Kp(6), Kd(6);
				Kp.setZero();
				Kd.setZero();
				Kp(0) = 1000; Kp(1) = 1000; // Proportional gains
				Kd(0) = 100;  Kd(1) = 100;  // Derivative gains

				// Compute desired task-space force for x and y
				Eigen::VectorXd F_desired(6);
				F_desired.setZero();
				F_desired.head(2) = Kp.head(2).cwiseProduct(ex_.head(2)) + Kd.head(2).cwiseProduct(xdot_error.head(2));

				// --- Direct Force Control in Z-Direction ---

				// Measured force in z-direction
				double Fm_z = f_cur_[2]; // Force sensor reading

				// **Force Error**:
				// force_error_z = Fd_z_ - Fm_z
				double force_error_z = Fd_z_ - Fm_z;

				// **Compute Desired Force Command**:
				// F_desired(2) = Kf_z_ * force_error_z
				F_desired(2) = Kf_z_ * force_error_z; // Kf_z_ is the force control gain

				// **Compute Joint Torques**:
				// tau_d_ = J^T * F_desired + Coriolis + Gravity
				tau_d_.data = J_.data.transpose() * F_desired + C_.data + G_.data;
			}

			else if (control_stage_ == 4)
			{
				// Time variable
				double t = total_time_;

				// Circular trajectory parameters
				double radius = 0.1;    // Adjust as needed
				double omega = 0.5;     // Adjust as needed
				double x_center = x_d_x_prev_;  // Center of the circle in x
				double y_center = x_d_y_prev_;  // Center of the circle in y

				// Define desired z position
				double desired_z = 0.1; // Set this to the desired height

				// **Desired Position (x_d)**
				// x_d = [x_d_x, x_d_y, x_d_z, roll_d, pitch_d, yaw_d]^T
				xd(0) = x_center + radius * cos(omega * t);  // x_d_x
				xd(1) = y_center + radius * sin(omega * t);  // x_d_y
				xd(2) = desired_z;                           // x_d_z
				xd(3) = 0.0;  // Desired roll (assuming zero)
				xd(4) = 0.0;  // Desired pitch (assuming zero)
				xd(5) = 0.0;  // Desired yaw (assuming zero)

				// **Desired Velocity (\dot{x}_d)**
				xd_dot(0) = -radius * omega * sin(omega * t);  // \dot{x}_d_x
				xd_dot(1) =  radius * omega * cos(omega * t);  // \dot{x}_d_y
				xd_dot(2) = 0.0;  // \dot{x}_d_z
				xd_dot(3) = 0.0;  // \dot{roll}_d
				xd_dot(4) = 0.0;  // \dot{pitch}_d
				xd_dot(5) = 0.0;  // \dot{yaw}_d

				// **Desired Acceleration (\ddot{x}_d)**
				Eigen::VectorXd xddot_desired(6);
				xddot_desired.setZero();
				xddot_desired(0) = -radius * omega * omega * cos(omega * t);  // \ddot{x}_d_x
				xddot_desired(1) = -radius * omega * omega * sin(omega * t);  // \ddot{x}_d_y
				// \ddot{x}_d_z = 0.0 (already zero)

				// **Compute Position Error (\tilde{x})**
				// \tilde{x} = x_d - x
				ex_.setZero();
				ex_.head(3) = xd.head(3) - current_position.head(3);

				// **Compute Velocity Error (\dot{\tilde{x}})**
				// \dot{\tilde{x}} = \dot{x}_d - \dot{x}
				xdot_error.setZero();
				xdot_error.head(3) = xd_dot.head(3) - xdot_.head(3);

				// **External Force Measurement (h_e)**
				// Assuming zero torque measurements
				Eigen::VectorXd he(6);
				he.setZero();
				he(0) = f_cur_.force.x();  // Measured force in x
				he(1) = f_cur_.force.y();  // Measured force in y
				he(2) = f_cur_.force.z();  // Measured force in z
				// he(3), he(4), he(5) are assumed zero (torque measurements)

				// **Compute Desired Operational Space Force (F_imp)**
				// F_imp = M_d * \ddot{x}_d + K_D * \dot{\tilde{x}} + K_P * \tilde{x} - h_e
				Eigen::VectorXd F_imp = Md * xddot_desired + Kd * xdot_error + Kp * ex_ - he;

				// **Compute Joint Torques (\tau_d)**
				// \tau_d = J^T * F_imp + C + G
				tau_d_.data = J_.data.transpose() * F_imp + C_.data + G_.data;
			}



			// Apply torque commands
			for (int i = 0; i < n_joints_; i++)
			{
				joints_[i].setCommand(tau_d_(i));
			}

			// Update time
			time_ += dt_;
			total_time_ += dt_;
		}



  		void stopping(const ros::Time& time) { }

	

		double first_order_lowpass_filter()
		{
			filt_ = (tau_ * filt_old_ + dt_*f_cur_buffer_)/(tau_ + dt_);
			filt_old_ = filt_;

			return filt_;
		}

		double first_order_lowpass_filter_z(double input)
		{
			double tau_z = 1.0 / (2 * PI * 10.0); // Adjust the cutoff frequency as needed
			double filt_z = (tau_z * filt_z_old_ + dt_ * input) / (tau_z + dt_);
			filt_z_old_ = filt_z;
			return filt_z;
		}

	private:
		// joint handles
		unsigned int n_joints_;
		std::vector<std::string> joint_names_;
  		std::vector<hardware_interface::JointHandle> joints_;
		std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

		// kdl
		KDL::Tree 	kdl_tree_;
		KDL::Chain	kdl_chain_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;	
		boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;
		KDL::JntArray G_; 
		KDL::Vector gravity_;

		// tdc gain
		KDL::JntArray Mbar_, Mbar_dot_, Ramda_, Alpha_, Omega_;					

		// cmd, state
		KDL::JntArray q_init_;
		KDL::JntArray tau_cmd_, tau_cmd_old_;
		KDL::JntArray q_cmd_, q_cmd_old_, qdot_cmd_, qdot_cmd_old_, qddot_cmd_;
		KDL::JntArray q_cmd_end_;
		KDL::JntArray q_cmd_sp_;
		KDL::JntArray q_, qdot_, qdot_old_, qddot_;
		KDL::Wrench f_cur_;
		double f_cur_buffer_;

		KDL::Twist Xc_dot_, Xc_dot_old_, Xc_ddot_;

		double Xr_dot_, Xe_dot_;
		double Xe_ddot_;

		double Fd_, Fd_temp_, Fd_old_, Fe_, Fe_old_;
		double M_, B_, del_B_, B_buffer_;
		double PI_, PI_old_;

		double dt_;
		double time_;
		double total_time_;

		double filt_old_;
		double filt_;
		double tau_;

		int experiment_mode_;

		// topic
		ros::Subscriber sub_q_cmd_;
		ros::Subscriber sub_forcetorque_sensor_;

		double SaveData_[SaveDataMax];

		ros::Publisher pub_SaveData_;

		std_msgs::Float64MultiArray msg_SaveData_;	

		// ROS subscriber to /motion_command
		ros::Subscriber sub_command_;
		realtime_tools::RealtimeBuffer<Commands> command_buffer_;

		KDL::JntArray qd_, qd_dot_, qd_ddot_, q_cmd_dot;	
		//task space 
    	Eigen::VectorXd xd, xd_dot;     // Desired task-space position (6 elements)
		KDL::Jacobian J_;            // Jacobian at current q_
		Eigen::VectorXd xdot_;       // Current end-effector velocity
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
		KDL::JntArray C_;
		
    	Eigen::MatrixXd M_joint_; // Inertia matrix in joint space

		KDL::JntArray tau_d_;
		Eigen::VectorXd ex_; 
		KDL::Twist ex_temp_; 


		//Admitance
		int control_stage_;
		double position_tolerance_;
		Eigen::VectorXd desired_position_A_;
		Eigen::VectorXd desired_position_B_;
		double x_d_x_prev_, x_d_y_prev_, x_d_z_prev_;
		double x_dot_desired_x, x_dot_desired_y;
		double Fd_z_, Kf_z_;

		double x_desired_z_;       // Desired position in z-direction
		double x_dot_desired_z_;   // Desired velocity in z-direction
		double K_;                 // Virtual stiffness in z-direction
		double x_eq_z_; // Equilibrium position in z-direction
		double filt_z_old_;

		// Variables for Case 3
		
		
		Eigen::VectorXd xdot_error; // Velocity error vector
		double max_Fz;     // Maximum allowable force in z-direction

		//impendace controller
		Eigen::MatrixXd Md; // Desired mass matrix
		Eigen::MatrixXd Kd; // Damping matrix
		Eigen::MatrixXd Kp; // Stiffness matrix
	};
}

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdaptiveImpedanceController, controller_interface::ControllerBase)