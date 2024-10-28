// Include necessary headers
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h> // For control mode messages

#include <urdf/model.h>

// KDL packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // Inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // Jacobian solver
#include <kdl/chainfksolverpos_recursive.hpp> // Forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>

#include <realtime_tools/realtime_buffer.h>

#include "arm_controllers/ArucoTracker.h"

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <limits>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49

struct ArucoData
{
    Eigen::Vector3d aruco_x;
    Eigen::Vector3d aruco_x_dot;
    Eigen::Quaterniond aruco_q;      // Orientation as a quaternion
    // Eigen::Vector3d aruco_q_dot;  // Angular velocity (optional)

    ArucoData() : aruco_x(Eigen::Vector3d::Zero()), 
                 aruco_x_dot(Eigen::Vector3d::Zero()),
                 aruco_q(Eigen::Quaterniond::Identity()) 
                 /*, aruco_q_dot(Eigen::Vector3d::Zero())*/ {}
};

struct Commands
{
    KDL::JntArray qd;
    KDL::JntArray qd_dot;
    KDL::JntArray qd_ddot;
    Eigen::VectorXd xd;     // Desired task-space position (6 elements)
    Eigen::VectorXd xd_dot; // Desired task-space velocity (6 elements)
    Commands() {}
};




namespace arm_controllers
{
class Computed_Torque_Controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/computed_torque_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/computed_torque_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/computed_torque_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/computed_torque_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 2. ********* urdf *********
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
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

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
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
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // 4.3 Inverse dynamics solver initialization
        gravity_ = KDL::Vector::Zero();
        gravity_(2) = -9.81;

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

        // 4.4 Forward kinematics solver initialization
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        // 4.5 Jacobian solver initialization
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

        // 5. ********* Initialize variables *********
        // 5.1 Vector initialization
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

        // Resize joint arrays for velocity control
        q_cmd_dot.resize(n_joints_);
        e_cmd.resize(n_joints_);
        q_cmd_dot.data = Eigen::VectorXd::Zero(n_joints_);
        e_cmd.data = Eigen::VectorXd::Zero(n_joints_);

        // 5.2 Matrix initialization
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());

        // Inizializzazione dei Jacobiani
        J_.resize(kdl_chain_.getNrOfJoints());
        Jd_.resize(kdl_chain_.getNrOfJoints());

        // Resize vectors for task-space variables
        xdot_.resize(6);
        xd_dot_.resize(6);
        xd_dot_desired.resize(6);
        ex_.resize(6);
        Kp_task_.resize(6);
        qd_dot_task.resize(n_joints_);

        aruco_x.resize(3);
        aruco_x_dot.resize(3);

        //Aruco camera KDL::Frame T_ee_camera;
        // Initialize the fixed transformation from end-effector to camera frame
        // Set the translation and rotation according to your setup
        // Example: Camera translated along the z-axis of the end-effector by 0.1 meters
        
        // Initialize task-space proportional gains (tune these values as needed)
        //Kp_task_ << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;
        // Increase the proportional gains to respond more aggressively
        Kp_task_ << 2.0, 2.0, 2.0, 0.0, 0.0, 0.0;


        // ********* 6. ROS commands *********
        // 6.1 Publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);

        // 6.2 Subscriber
        sub_command_ = n.subscribe("/motion_command", 1000, &Computed_Torque_Controller::commandCB, this);

        sub_aurco = n.subscribe("/aruco_tracker", 1000, &Computed_Torque_Controller::commandAruco, this);


        // Initialize the command buffer with zeros
        Commands initial_commands;
        initial_commands.qd.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.qd_dot.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.qd_ddot.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.xd = Eigen::VectorXd::Zero(6);
        initial_commands.xd_dot = Eigen::VectorXd::Zero(6);

        command_buffer_.initRT(initial_commands);
        
        ArucoData initial_aruco_data;
        aruco_buffer_.initRT(initial_aruco_data);

        // Initialize control mode to 1
        control_mode_buffer_.writeFromNonRT(1);

        // Subscribe to the control mode topic
        sub_control_mode_ = n.subscribe("/control_mode", 10, &Computed_Torque_Controller::controlModeCB, this);

        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        size_t expected_size = 3 * n_joints_ + 12; // 18 + 12 = 30
        if (msg->data.size() != expected_size)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size()
                             << ") does not match expected size (" << expected_size << ")! Not executing!");
            return;
        }

        Commands cmd;
        cmd.qd.data = Eigen::VectorXd::Zero(n_joints_);
        cmd.qd_dot.data = Eigen::VectorXd::Zero(n_joints_);
        cmd.qd_ddot.data = Eigen::VectorXd::Zero(n_joints_);
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


    void commandAruco(const arm_controllers::ArucoTrackerConstPtr &msg)
    {
        // Create a new ArucoData instance
        ArucoData aruco_data;
        aruco_data.aruco_x = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
        aruco_data.aruco_x_dot = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        aruco_data.aruco_q = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        // If angular velocity is available:
        // aruco_data.aruco_q_dot = Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        // Write to the aruco_buffer_
        aruco_buffer_.writeFromNonRT(aruco_data);

        ROS_DEBUG_STREAM("Aruco Tracker Callback - Position: " << aruco_data.aruco_x.transpose()
                        << ", Velocity: " << aruco_data.aruco_x_dot.transpose()
                        << ", Orientation: [" << aruco_data.aruco_q.w() << ", "
                        << aruco_data.aruco_q.x() << ", "
                        << aruco_data.aruco_q.y() << ", "
                        << aruco_data.aruco_q.z() << "]");
    }

    // Callback function for control mode
    void controlModeCB(const std_msgs::Int32::ConstPtr &msg)
    {
        int mode = msg->data;
        if (mode >= 1 && mode <= 7)
        {
            control_mode_buffer_.writeFromNonRT(mode);
        }
        else
        {
            ROS_WARN("Received invalid control mode: %d", mode);
        }
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Computed Torque Controller");
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 Sampling time
        double dt = period.toSec();
        t = t + dt;

        // 0.2 Joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        // ********* 1. Desired Trajectory in Joint Space *********
        // Retrieve the latest commands
        Commands cmd = *(command_buffer_.readFromRT());
        qd_.data = cmd.qd.data;
        qd_dot_.data = cmd.qd_dot.data;
        qd_ddot_.data = cmd.qd_ddot.data;

        // ********* 2. Motion Controller *********
        // Get the control mode
        int control_mode = *(control_mode_buffer_.readFromRT());

        // Compute model (M, G)
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToGravity(q_, G_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);

        switch (control_mode)
        {
            case 1:
                // *** Position PD Control ***
                e_.data = qd_.data - q_.data;
                e_dot_.data = qd_dot_.data - qdot_.data;
                tau_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data)) + G_.data + C_.data;
                break;
            case 2:
                // *** Velocity PD Control ***

                e_dot_.data = qd_dot_.data - qdot_.data;
                tau_d_.data = M_.data * (qd_ddot_.data + Kd_.data.cwiseProduct(e_dot_.data)) + G_.data + C_.data;
                break;
            case 3:
                // Implement other control modes if needed
                break;
            case 4:
                // *** Velocity Controller in Joint Space with kinematic ***
                e_.data = qd_.data - q_.data;
                q_cmd_dot.data = qd_dot_.data + Kp_.data.cwiseProduct(e_.data);

                e_cmd.data = q_cmd_dot.data - qdot_.data;
                tau_d_.data = M_.data * (Kd_.data.cwiseProduct(e_cmd.data)) + G_.data + C_.data;

                break;
            case 5:
            {
                // *** Kinematic Controller in Task Space using Velocity Controller ***
                // Compute current end-effector pose x_
                fk_pos_solver_->JntToCart(q_, x_);

                // Compute Jacobian at current q_
                jnt_to_jac_solver_->JntToJac(q_, J_);

                // Compute current end-effector velocity xdot_
                xdot_ = J_.data * qdot_.data;

                // Compute desired end-effector pose xd_frame_
                fk_pos_solver_->JntToCart(qd_, xd_frame_);

                // Compute Jacobian at desired qd_
                jnt_to_jac_solver_->JntToJac(qd_, Jd_);

                // Compute desired end-effector velocity xd_dot_
                xd_dot_ = Jd_.data * qd_dot_.data;

                // Compute task-space position error ex_ = xd_ - x_
                ex_temp_ = KDL::diff(x_, xd_frame_);
                ex_(0) = ex_temp_.vel(0);
                ex_(1) = ex_temp_.vel(1);
                ex_(2) = ex_temp_.vel(2);
                ex_(3) = ex_temp_.rot(0);
                ex_(4) = ex_temp_.rot(1);
                ex_(5) = ex_temp_.rot(2);

                // Compute desired task-space velocity xd_dot_desired
                xd_dot_desired = xd_dot_ + Kp_task_.cwiseProduct(ex_);

                // Compute pseudoinverse of current Jacobian J_
                pseudo_inverse(J_.data, J_pinv);

                // Compute desired joint velocities from task-space velocities
                qd_dot_task = J_pinv * xd_dot_desired;

                // Update qd_dot_.data
                qd_dot_.data = qd_dot_task;

                // Compute joint position error e_.data = qd_.data - q_.data
                e_.data = qd_.data - q_.data;

                // Compute desired joint velocities including position error
                q_cmd_dot.data = qd_dot_.data + Kp_.data.cwiseProduct(e_.data);

                // Compute velocity error e_cmd.data = q_cmd_dot.data - qdot_.data
                e_cmd.data = q_cmd_dot.data - qdot_.data;

                // Compute torque command
                tau_d_.data = M_.data * (qd_ddot_.data + Kd_.data.cwiseProduct(e_cmd.data)) + G_.data + C_.data;

                break;
            }
            case 6:
            {
                // *** Control in Task Space using Cartesian coordinates from the command ***
                // Compute current end-effector pose x_
                fk_pos_solver_->JntToCart(q_, x_);

                // Compute Jacobian at current q_
                jnt_to_jac_solver_->JntToJac(q_, J_);

                // Compute current end-effector velocity xdot_
                xdot_ = J_.data * qdot_.data;

                // Construct desired end-effector pose xd_frame_ from cmd.xd
                xd_frame_.p = KDL::Vector(cmd.xd(0), cmd.xd(1), cmd.xd(2));
                xd_frame_.M = KDL::Rotation::RPY(cmd.xd(3), cmd.xd(4), cmd.xd(5));

                // Compute task-space position error ex_ = xd_ - x_
                ex_temp_ = KDL::diff(x_, xd_frame_);
                ex_(0) = ex_temp_.vel(0);
                ex_(1) = ex_temp_.vel(1);
                ex_(2) = ex_temp_.vel(2);
                ex_(3) = ex_temp_.rot(0);
                ex_(4) = ex_temp_.rot(1);
                ex_(5) = ex_temp_.rot(2);

                // Compute desired task-space velocity
                xd_dot_desired = cmd.xd_dot + Kp_task_.cwiseProduct(ex_);

                // Compute pseudoinverse of current Jacobian J_
                pseudo_inverse(J_.data, J_pinv);

                // Compute desired joint velocities from task-space velocities
                qd_dot_task = J_pinv * xd_dot_desired;

                // Compute velocity error
                e_cmd.data = qd_dot_task - qdot_.data;

                // Compute torque command
                tau_d_.data = M_.data * (Kd_.data.cwiseProduct(e_cmd.data)) + G_.data + C_.data;

                break;
            }
            case 7:
            {
                // *** Visual Servoing Control using Aruco Tracker Data ***
                ROS_INFO("------------- inter ------------------------");

                // 1. Read Aruco data from buffer
                ArucoData aruco_data = *(aruco_buffer_.readFromRT());

                // 2. Compute current end-effector pose x_
                fk_pos_solver_->JntToCart(q_, x_); // x_ contains the transformation from base to EE frame

                // 3. Compute Jacobian at current q_
                jnt_to_jac_solver_->JntToJac(q_, J_);

                // 4. Obtain the Aruco marker position in the camera frame
                KDL::Vector p_marker_in_camera(aruco_data.aruco_x(0), aruco_data.aruco_x(1), aruco_data.aruco_x(2));
                KDL::Vector v_marker_in_camera(aruco_data.aruco_x_dot(0), aruco_data.aruco_x_dot(1), aruco_data.aruco_x_dot(2));

                KDL::Rotation R_marker_in_camera = KDL::Rotation::Quaternion(
                    aruco_data.aruco_q.x(),
                    aruco_data.aruco_q.y(),
                    aruco_data.aruco_q.z(),
                    aruco_data.aruco_q.w()
                );

                KDL::Frame marker_pose_in_camera(R_marker_in_camera, p_marker_in_camera);

                // 5. CAMERA
                // Example values for the translation and rotation from end-effector to camera
                double trans_x = 0.0;  // Translation along X in meters
                double trans_y = 0.0;  // Translation along Y in meters
                double trans_z = 0.1;  // Translation along Z in meters (example: 10 cm forward)

                double rot_roll = 0.0;   // Rotation around X-axis in radians
                double rot_pitch = 0.0;  // Rotation around Y-axis in radians
                double rot_yaw = -M_PI;   // Rotation around Z-axis in radians (180 degrees)

                KDL::Rotation camera_rotation = KDL::Rotation::RPY(rot_roll, rot_pitch, rot_yaw);
                KDL::Vector camera_translation(trans_x, trans_y, trans_z);
                KDL::Frame T_ee_camera(camera_rotation, camera_translation);

                // Invert T_ee_camera to get T_camera_ee
                KDL::Frame T_camera_ee = T_ee_camera.Inverse();

                // Transform marker pose to EE frame
                KDL::Frame marker_pose_in_ee = T_camera_ee * marker_pose_in_camera;

                // 6. Compute the transformation from base frame to marker frame
                KDL::Frame marker_pose_in_base = x_ * marker_pose_in_ee;

                // Transform marker velocity to base frame
                KDL::Vector v_marker_in_base = x_.M * T_camera_ee.M * v_marker_in_camera;

                // Extract the quaternion for marker orientation in base frame
                KDL::Rotation R_marker_in_base = x_.M * T_camera_ee.M * R_marker_in_camera;
                double qx_base, qy_base, qz_base, qw_base;
                R_marker_in_base.GetQuaternion(qx_base, qy_base, qz_base, qw_base);

                // Print marker data in base frame
                ROS_INFO_STREAM("Aruco Data In BASE Frame - Position: ("
                                << marker_pose_in_base.p.x() << ", "
                                << marker_pose_in_base.p.y() << ", "
                                << marker_pose_in_base.p.z() << "), Velocity: ("
                                << v_marker_in_base.x() << ", "
                                << v_marker_in_base.y() << ", "
                                << v_marker_in_base.z() << "), Orientation (quaternion): ["
                                << qw_base << ", " << qx_base << ", " << qy_base << ", " << qz_base << "]");

                // 7. Compute current end-effector velocity in base frame
                xdot_ = J_.data * qdot_.data; // xdot_ is a 6x1 vector [linear; angular]
                KDL::Vector v_ee_base(xdot_(0), xdot_(1), xdot_(2));

                // Print end-effector position and velocity
                ROS_INFO_STREAM("End-effector in BASE Frame - Position: ("
                                << x_.p.x() << ", "
                                << x_.p.y() << ", "
                                << x_.p.z() << "), Velocities: "
                                << "X: " << xdot_(0) << ", "
                                << "Y: " << xdot_(1) << ", "
                                << "Z: " << xdot_(2) << "; "
                                << "Angular Velocities: "
                                << "Roll: " << xdot_(3) << ", "
                                << "Pitch: " << xdot_(4) << ", "
                                << "Yaw: " << xdot_(5));

                // 8. Calculate positional error
                KDL::Vector position_error = marker_pose_in_base.p - x_.p;

                // Optionally, add an offset in z-direction
                double z_offset_aruco = 0.4;  // Adjust this value as needed
                position_error.z(position_error.z() + z_offset_aruco);

                // Print positional error
                ROS_INFO_STREAM("Positional Error - "
                                << "X: " << position_error.x() << ", "
                                << "Y: " << position_error.y() << ", "
                                << "Z: " << position_error.z());


                // Definisci una costante di tempo del filtro e la frequenza di aggiornamento
                double tau = 0.5; // Costante di tempo del filtro
                double dt = 0.01; // Intervallo di tempo tra gli aggiornamenti (100 Hz)

                // Inizializza la velocità filtrata (se non è già definita)
                static KDL::Vector v_marker_in_base_filtered(0.0, 0.0, 0.0);

                // Applica il filtro passa basso
                v_marker_in_base_filtered = v_marker_in_base_filtered + (dt / (tau + dt)) * (v_marker_in_base - v_marker_in_base_filtered);

                // Usa v_marker_in_base_filtered invece di v_marker_in_base per il calcolo degli errori di velocità
                KDL::Vector velocity_error = v_marker_in_base_filtered - v_ee_base;

                // 9. Calculate velocity error
                //KDL::Vector velocity_error = v_marker_in_base - v_ee_base;

                // Print velocity error
                ROS_INFO_STREAM("Velocity Error - "
                                << "VX: " << velocity_error.x() << ", "
                                << "VY: " << velocity_error.y() << ", "
                                << "VZ: " << velocity_error.z());

                // 10. Convert errors to Eigen vectors
                Eigen::Vector3d position_error_vector(position_error.x(), position_error.y(), position_error.z());
                Eigen::Vector3d velocity_error_vector(velocity_error.x(), velocity_error.y(), velocity_error.z());

                // 11. Define Proportional and Derivative Gains
                Eigen::Vector3d Kp_task(2.0, 2.0, 2.0);  // Adjust gains as needed
                Eigen::Vector3d Kd_task(0.1, 0.1, 0.1);  // Adjust gains as needed

                // 12. Compute desired acceleration in task space
                Eigen::Vector3d x_ddot_desired = Kp_task.cwiseProduct(position_error_vector) + Kd_task.cwiseProduct(velocity_error_vector);

                // Print desired acceleration
                ROS_INFO_STREAM("Desired Task-Space Acceleration: " << x_ddot_desired.transpose());

                // 13. Compute the Damped Least Squares Pseudoinverse of the Jacobian
                Eigen::MatrixXd J_linear = J_.data.topRows(3);  // 3 x N matrix
                double lambda = 0.1;  // Damping factor, adjust as needed
                Eigen::MatrixXd J_linear_pinv = (J_linear.transpose() * J_linear + lambda * Eigen::MatrixXd::Identity(n_joints_, n_joints_)).inverse() * J_linear.transpose();

                // 14. Map Task-Space Accelerations to Joint-Space Accelerations
                Eigen::VectorXd q_ddot_desired = J_linear_pinv * x_ddot_desired;  // N x 1 vector

                // Print desired joint accelerations
                ROS_INFO_STREAM("Desired Joint Accelerations: " << q_ddot_desired.transpose());

                // 15. Convert M_, C_, G_ to Eigen matrices/vectors
                Eigen::MatrixXd M_eigen(n_joints_, n_joints_);
                for (int i = 0; i < n_joints_; ++i)
                    for (int j = 0; j < n_joints_; ++j)
                        M_eigen(i, j) = M_(i, j);

                // 16. Compute torque command
                tau_d_.data = M_eigen * q_ddot_desired + C_.data + G_.data;

                // Print torque commands
                ROS_INFO_STREAM("Torque Commands: " << tau_d_.data.transpose());

                break;
            }


                
            default:
                ROS_WARN("Invalid control mode selected: %d", control_mode);
                break;
        }

        // Apply torque commands
        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
        }

        // ********* 3. Save data *********
        save_data();

        // ********* 4. Print state *********
        print_state();
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(2);
        SaveData_[40] = e_dot_(3);
        SaveData_[41] = e_dot_(4);
        SaveData_[42] = e_dot_(5);

        // Error integral value in joint space (unit: rad*sec)
        SaveData_[43] = e_int_(0);
        SaveData_[44] = e_int_(1);
        SaveData_[45] = e_int_(2);
        SaveData_[46] = e_int_(3);
        SaveData_[47] = e_int_(4);
        SaveData_[48] = e_int_(5);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Desired State in Joint Space (unit: deg) ***\n");
            printf("qd_(0): %f, ", qd_(0) * R2D);
            printf("qd_(1): %f, ", qd_(1) * R2D);
            printf("qd_(2): %f, ", qd_(2) * R2D);
            printf("qd_(3): %f, ", qd_(3) * R2D);
            printf("qd_(4): %f, ", qd_(4) * R2D);
            printf("qd_(5): %f\n", qd_(5) * R2D);
            printf("\n");

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");

            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("%f, ", R2D * e_(0));
            printf("%f, ", R2D * e_(1));
            printf("%f, ", R2D * e_(2));
            printf("%f, ", R2D * e_(3));
            printf("%f, ", R2D * e_(4));
            printf("%f\n", R2D * e_(5));
            printf("\n");

            count = 0;
        }
        count++;
    }

  private:
    // Others
    double t;

    // Joint handles
    unsigned int n_joints_;
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    // KDL
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    // KDL M, C, G
    KDL::JntSpaceInertiaMatrix M_;
    KDL::JntArray C_;
    KDL::JntArray G_;
    KDL::Vector gravity_;

    // KDL solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_, q_cmd_dot;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_, e_cmd;

    // Task Space State
    KDL::Frame x_;               // Current end-effector pose
    KDL::Frame xd_frame_;        // Desired end-effector pose
    KDL::Twist ex_temp_;         // Task-space position error as KDL::Twist
    KDL::Jacobian J_;            // Jacobian at current q_
    KDL::Jacobian Jd_;           // Jacobian at desired qd_
    Eigen::VectorXd xdot_;       // Current end-effector velocity
    Eigen::VectorXd xd_dot_;     // Desired end-effector velocity from command
    Eigen::VectorXd xd_dot_desired; // Desired task-space velocity with feedback
    Eigen::VectorXd ex_;         // Task-space position error as Eigen vector
    Eigen::VectorXd Kp_task_;    // Task-space proportional gains
    Eigen::MatrixXd J_pinv;      // Pseudoinverse of the Jacobian
    Eigen::VectorXd qd_dot_task; // Desired joint velocities from task-space mapping

    //aruco
    Eigen::VectorXd aruco_x; 
    Eigen::VectorXd aruco_x_dot; 

    // Define the fixed transformation from end-effector to camera frame
    KDL::Frame T_ee_camera;
    
    // Set the translation and rotation according to your setup
    // For example, if the camera is translated along the z-axis of the end-effector by 0.1 meters:


    // Input
    KDL::JntArray tau_d_;

    // Gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // Save the data
    double SaveData_[SaveDataMax];

    // ROS publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;

    // ROS message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;

    // ROS subscriber to /motion_command
    ros::Subscriber sub_command_;
    realtime_tools::RealtimeBuffer<Commands> command_buffer_;
    realtime_tools::RealtimeBuffer<ArucoData> aruco_buffer_;

    //Ros subscriber aruco
    ros::Subscriber sub_aurco;
    // Control mode
    realtime_tools::RealtimeBuffer<int> control_mode_buffer_;
    ros::Subscriber sub_control_mode_;

    // Pseudoinverse function
    void pseudo_inverse(const Eigen::MatrixXd& a, Eigen::MatrixXd& result, double epsilon = std::numeric_limits<double>::epsilon())
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();
        Eigen::VectorXd singularValues_inv = svd.singularValues();
        for (long i = 0; i < singularValues_inv.size(); ++i)
        {
            if (singularValues_inv(i) > tolerance)
            {
                singularValues_inv(i) = 1.0 / singularValues_inv(i);
            }
            else
            {
                singularValues_inv(i) = 0.0;
            }
        }
        result = svd.matrixV() * singularValues_inv.asDiagonal() * svd.matrixU().transpose();
    }

    #include <Eigen/Dense>



    Eigen::MatrixXd pseudo_inverse_DLS(const Eigen::MatrixXd& J, double lambda)
{
    Eigen::MatrixXd JJt = J * J.transpose();
    Eigen::MatrixXd damping = lambda * lambda * Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
    Eigen::MatrixXd JJt_damped = JJt + damping;
    Eigen::MatrixXd inv_JJt_damped = JJt_damped.inverse();
    Eigen::MatrixXd J_pinv = J.transpose() * inv_JJt_damped;
    return J_pinv;
}

};
}; // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(arm_controllers::Computed_Torque_Controller, controller_interface::ControllerBase)