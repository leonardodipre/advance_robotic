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
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>

#include "arm_controllers/ArucoTracker.h"

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <limits>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49


struct Commands
{
    KDL::JntArray qd;
    KDL::JntArray qd_dot;
    KDL::JntArray qd_ddot;
    Eigen::VectorXd xd;     // Desired task-space position (6 elements)
    Eigen::VectorXd xd_dot; // Desired task-space velocity (6 elements)
    Commands() {}
};


struct ArucoData
{
    Eigen::Vector3d aruco_x;
    Eigen::Vector3d aruco_x_dot;
    Eigen::Quaterniond aruco_q;      // Orientation as a quaternion
    // Eigen::Vector3d aruco_q_dot;  // Angular velocity (optional)

    ArucoData() : aruco_x(Eigen::Vector3d::Zero()), 
                 aruco_x_dot(Eigen::Vector3d::Zero()),
                 aruco_q(Eigen::Quaterniond::Identity()) 
                 {}
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
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_ ))
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

        ex_ = Eigen::VectorXd::Zero(n_joints_); // Initialize as a 6D vector with all elements zero
        er_i = Eigen::VectorXd::Zero(3);

        //aruco
        aruco_x.resize(3);
        aruco_x_dot.resize(3);


        // 5.2 Matrix initialization
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());


        
     

        // ********* 6. ROS commands *********
        // 6.1 Publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000);

        // 6.2 Subscriber
        sub_command_ = n.subscribe("/motion_command", 1000, &Computed_Torque_Controller::commandCB, this);

        sub_aurco = n.subscribe("/transformed_aruco_pose", 1000, &Computed_Torque_Controller::commandAruco, this);
 

        sub_control_mode_ = n.subscribe("/control_mode", 10, &Computed_Torque_Controller::controlModeCB, this);

        pub_ex_ = n.advertise<std_msgs::Float64MultiArray>("task_space_error", 1000);
        pub_force_ = n.advertise<std_msgs::Float64MultiArray>("task_space_force", 1000);


        control_mode_buffer_.writeFromNonRT(1);


        // Initialize the command buffer with zeros
        Commands initial_commands;
        initial_commands.qd.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.qd_dot.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.qd_ddot.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.xd = Eigen::VectorXd::Zero(6);
        initial_commands.xd_dot = Eigen::VectorXd::Zero(6);

        command_buffer_.initRT(initial_commands);

        //Buffer Aruco
        ArucoData initial_aruco_data;
        aruco_buffer_.initRT(initial_aruco_data);
        
        //Smooth trasnition case 8:

        traj_initialized_ = false;
        traj_start_time_ = 0.0;
        traj_duration_ = 5.0; // Adjust the duration as needed
        x_init_ = KDL::Vector::Zero();
        xd_target_ = KDL::Vector::Zero();

        traj_start_position_ = KDL::Vector::Zero();
        traj_goal_position_ = KDL::Vector::Zero();


        waypoints_.clear();
        segment_durations_.clear();
        current_segment_index_ = 0;
        segment_start_time_ = 0.0;


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

    void commandAruco(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {

        // Create a new ArucoData instance
        ArucoData aruco_data;
        //aruco_data.aruco_x = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
        //aruco_data.aruco_x_dot = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        //aruco_data.aruco_q = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        aruco_data.aruco_x = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        aruco_data.aruco_q = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    
        // Write to the aruco_buffer_
        aruco_buffer_.writeFromNonRT(aruco_data);

      
    }

     // Callback function for control mode
    void controlModeCB(const std_msgs::Int32::ConstPtr &msg)
    {
        int mode = msg->data;
        if (mode >= 1 && mode <= 8)
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
        
        
      
        traj_initialized_ = false;

        // Print the initial pose
        ROS_INFO("Starting Computed Torque Controller");
      
        
            
    }


    void update(const ros::Time &time, const ros::Duration &period){
        // Get current joint positions and velocities
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        //////////////////// READ DATA FROM TOPIC /////////////////////////////

        Commands cmd = *(command_buffer_.readFromRT());
        qd_.data = cmd.qd.data;
        qd_dot_.data = cmd.qd_dot.data;
        qd_ddot_.data = cmd.qd_ddot.data;

        Eigen::Vector3d cmd_xd_position = cmd.xd.segment(0,3);
        //ROS_INFO("cmd_xd_position: x = %f, y = %f, z = %f", cmd_xd_position.x(), cmd_xd_position.y(), cmd_xd_position.z());
        
        int control_mode = *(control_mode_buffer_.readFromRT());


        // Retrieve the latest commands (if any)
        
        ArucoData aruco_data = *(aruco_buffer_.readFromRT());
        KDL::Vector p_marker_in_camera(
            aruco_data.aruco_x(0),
            aruco_data.aruco_x(1),
            aruco_data.aruco_x(2)
        );

        double roll, pitch, yaw;
        KDL::Rotation R_marker_in_camera = KDL::Rotation::Quaternion(
            aruco_data.aruco_q.z(),
            aruco_data.aruco_q.y(),
            aruco_data.aruco_q.x(),
            aruco_data.aruco_q.w()
        );

        
      
        KDL::Frame marker_pose_in_robot_frame(R_marker_in_camera, p_marker_in_camera);

        

        ///////////////////////////////////////////////////////////////////////////////

        // Compute model (M, C, G)
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToGravity(q_, G_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);


        switch (control_mode)
        {
            case 1:
               {

                e_.data = qd_.data - q_.data;
                e_dot_.data = qd_dot_.data - qdot_.data;
                tau_d_.data = M_.data * (qd_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data)) + G_.data + C_.data;

                // Apply torque commands
                for (int i = 0; i < n_joints_; i++)
                {
                    joints_[i].setCommand(tau_d_(i));
                }

                break;

               }

            case 7 :
            {   
                ROS_INFO("---------------------Inside Aruco Traker------------------------------");

                ////////////// END EFFECTOR POSITION TO BASE //////////

                // Compute End-effector position and Jacobian
                KDL::Frame end_effector_frame;
                fk_pos_solver_->JntToCart(q_, end_effector_frame);
                jnt_to_jac_solver_->JntToJac(q_, J_);

                // Extract position for debugging
                double x = end_effector_frame.p.x();
                double y = end_effector_frame.p.y();
                double z = end_effector_frame.p.z();

                ROS_INFO("End-effector position: [x: %f, y: %f, z: %f]", x, y, z);

                ROS_INFO("Marker Position in Camera Frame: [x: %f, y: %f, z: %f]", 
                        p_marker_in_camera.x(), 
                        p_marker_in_camera.y(), 
                        p_marker_in_camera.z());

                ROS_INFO("---------------------------------------------------");
                
                // End-effector velocity
                xdot_ = J_.data * qdot_.data;
                
                //ROS_INFO("End-effector velocity: [vx: %f, vy: %f, vz: %f, wx: %f, wy: %f, wz: %f]", xdot_(0), xdot_(1), xdot_(2), xdot_(3), xdot_(4), xdot_(5));


                double x_desire = marker_pose_in_robot_frame.p.x() ;
                double y_desire = marker_pose_in_robot_frame.p.y() ;
                double z_desire  =  marker_pose_in_robot_frame.p.z() ;
                // Output the marker position and orientation in base frame
                ROS_INFO("Marker Position in Base Frame: [x: %f, y: %f, z: %f]", x_desire , y_desire, z_desire);

               
                /////////////////////////////////////
                
                
              
                // Desired end-effector pose
                double z_offset = 0.45; // Adjust this value as needed
                KDL::Vector desired_position(x_desire, y_desire, z_desire + z_offset);
                                
               
                //KDL::Rotation desired_orientation = KDL::Rotation::RotX(- 1.5* M_PI);
                KDL::Rotation desired_orientation = KDL::Rotation::RotX(-1.5 * M_PI) * KDL::Rotation::RotY(M_PI)* KDL::Rotation::RotZ(0);
                                    
                KDL::Frame desired_frame(desired_orientation, desired_position);

                ex_temp_ = KDL::diff(end_effector_frame, desired_frame);
                for (int i = 0; i < 6; ++i) {
                    ex_(i) = ex_temp_(i);
                }

                ROS_INFO("Task-space position error (ex_): [vx: %f, vy: %f, vz: %f, wx: %f, wy: %f, wz: %f]",
                        ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5));

                ROS_INFO_STREAM("Jacobian:\n" << J_.data);

                

                // Task-space PID gains
                KDL::Vector Kp_trans(600, 600, 600); // Increase proportional gains
                KDL::Vector Kd_trans(80, 80, 80); 

                KDL::Vector Kp_rot( 30.0,  30.0, 0.0);
                KDL::Vector Kd_rot( 20.0,  20.0,  0.0);

                KDL::Vector ki_trans(0.13, 0.13 , 0.13);

                

                // Compute the task-space control effort
                KDL::Wrench F_desired;
                for (int i = 0; i < 3; i++) {
                    er_i(i) = er_i(i) + ex_(i);
                    F_desired.force(i) = Kp_trans(i) * ex_(i) + Kd_trans(i) * (-xdot_(i)) + ki_trans(i)*er_i(i);
                    F_desired.torque(i) = Kp_rot(i) * ex_(i+3) + Kd_rot(i) * (-xdot_(i+3));
                }

                ROS_INFO("er_i: [Fx: %f, Fy: %f, Fz: %f]", 
                        er_i(0), er_i(1), er_i(2));

                ROS_INFO("Task-space force: [Fx: %f, Fy: %f, Fz: %f]", 
                        F_desired.force(0), F_desired.force(1), F_desired.force(2));
                ROS_INFO("Task-space torque: [Tx: %f, Ty: %f, Tz: %f]", 
                        F_desired.torque(0), F_desired.torque(1), F_desired.torque(2));

                // Map task-space forces to joint torques (Corrected)
                Eigen::VectorXd F_desired_vec(6);
                F_desired_vec << F_desired.force.x(), F_desired.force.y(), F_desired.force.z(),
                                F_desired.torque.x(), F_desired.torque.y(), F_desired.torque.z();
                
                tau_d_.data = J_.data.transpose() * F_desired_vec;

                // Add Coriolis and Gravity compensation (Removed M * qddot_)--> inertial forces already taken into account inside F_desired end-effector : F = M*qddot;
                tau_d_.data += C_.data + G_.data;

                // Debugging output
                ROS_INFO("Computed joint torques (tau_):");
                for (int i = 0; i < n_joints_; i++) {
                    ROS_INFO("tau_[%d]: %f", i, tau_d_(i));
                }

                //plotting stuff
                std_msgs::Float64MultiArray msg_ex;
                msg_ex.data.resize(ex_.size());
                for (size_t i = 0; i < ex_.size(); ++i)
                {
                    msg_ex.data[i] = ex_(i);
                }
                pub_ex_.publish(msg_ex);
                                
                std_msgs::Float64MultiArray msg_force;
            msg_force.data.resize(6); // Task-space forces are 6D (3 linear + 3 angular)
            msg_force.data[0] = F_desired.force(0);
            msg_force.data[1] = F_desired.force(1);
            msg_force.data[2] = F_desired.force(2);
            msg_force.data[3] = F_desired.torque(0);
            msg_force.data[4] = F_desired.torque(1);
            msg_force.data[5] = F_desired.torque(2);
            pub_force_.publish(msg_force);

                ROS_INFO("-----------------------------------------------------------------------");
                ROS_INFO("- ");

                // Apply torque commands
                for (int i = 0; i < n_joints_; i++)
                {
                    joints_[i].setCommand(tau_d_(i));
                }
                
                
                
                break;
            } 
            case 8:
            {
                KDL::Frame current_ee_frame;
                fk_pos_solver_->JntToCart(q_, current_ee_frame);
                KDL::Vector current_position = current_ee_frame.p;

                KDL::Vector goal_position(cmd.xd(0), cmd.xd(1), cmd.xd(2));

                // Check if the goal position has changed
                if ((goal_position - traj_goal_position_).Norm() > 1e-6)
                {
                    // Goal has changed, reinitialize trajectory
                    traj_start_time_ = ros::Time::now().toSec();

                    // Definisci i waypoint
                    waypoints_.clear();
                    waypoints_.push_back(current_position);

                    // Definisci waypoint intermediario per evitare punti problematici
                    KDL::Vector waypoint1(0.2, 0.3, current_position.z()); // Regola secondo necessità
                    waypoints_.push_back(waypoint1);

                    

                    waypoints_.push_back(goal_position);

                    // Definisci le durate per ogni segmento
                    segment_durations_.clear();
                    double duration1 = 5.0; // Durata da current_position a waypoint1
                    double duration2 = 5.0; // Durata da waypoint1 a goal_position
                    segment_durations_.push_back(duration1);
                    segment_durations_.push_back(duration2);

                    // Reimposta le variabili di tracciamento della traiettoria
                    current_segment_index_ = 0;
                    segment_start_time_ = traj_start_time_;

                    // Aggiorna traj_goal_position_
                    traj_goal_position_ = goal_position;
                }

                // Calcola il tempo trascorso dall'inizio del segmento corrente
                double elapsed_time = ros::Time::now().toSec() - segment_start_time_;

                // Assicurati che elapsed_time non superi la durata del segmento corrente
                if (elapsed_time > segment_durations_[current_segment_index_])
                {
                    elapsed_time = segment_durations_[current_segment_index_];
                }

                // Ottieni le posizioni di inizio e fine per il segmento corrente
                KDL::Vector p0 = waypoints_[current_segment_index_];
                KDL::Vector pf = waypoints_[current_segment_index_ + 1];
                double T = segment_durations_[current_segment_index_];
                double t = elapsed_time;

                // Pianificazione della traiettoria quintica per ogni asse
                KDL::Vector desired_position, desired_velocity, desired_acceleration;

                for (int idx = 0; idx < 3; ++idx) // Per x, y, z
                {
                    double p0_i = p0(idx);
                    double pf_i = pf(idx);

                    // Coefficienti del polinomio quintico
                    double a0 = p0_i;
                    double a1 = 0;
                    double a2 = 0;
                    double a3 = (10 * (pf_i - p0_i)) / pow(T, 3);
                    double a4 = (-15 * (pf_i - p0_i)) / pow(T, 4);
                    double a5 = (6 * (pf_i - p0_i)) / pow(T, 5);

                    // Posizione desiderata
                    desired_position(idx) = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5);

                    // Velocità desiderata
                    desired_velocity(idx) = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4);

                    // Accelerazione desiderata
                    desired_acceleration(idx) = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3);
                }

                // Calcola la velocità corrente dell'end-effector
                jnt_to_jac_solver_->JntToJac(q_, J_);
                xdot_ = J_.data * qdot_.data;

                // Estrai la velocità lineare (primi tre elementi)
                Eigen::Vector3d current_velocity = xdot_.segment(0, 3);

                // Errori di posizione e velocità
                Eigen::Vector3d position_error;
                position_error << current_position.x() - desired_position.x(),
                                current_position.y() - desired_position.y(),
                                current_position.z() - desired_position.z();

                Eigen::Vector3d desired_velocity_eigen(desired_velocity.x(), desired_velocity.y(), desired_velocity.z());
                Eigen::Vector3d velocity_error = current_velocity - desired_velocity_eigen;

                // Guadagni PD nello spazio delle task
                double Kp = 600.0; // Guadagno proporzionale
                double Kd = 80.0;  // Guadagno derivativo

                // Accelerazione desiderata nello spazio delle task con feedback
                Eigen::Vector3d desired_acceleration_eigen(desired_acceleration.x(), desired_acceleration.y(), desired_acceleration.z());
                Eigen::Vector3d desired_task_acceleration = desired_acceleration_eigen - Kp * position_error - Kd * velocity_error;

                // Assembla il vettore di accelerazione desiderata nello spazio delle task (6D)
                Eigen::VectorXd xddot_task(6);
                xddot_task.head(3) = desired_task_acceleration;
                xddot_task.tail(3) = Eigen::Vector3d::Zero(); // Supponendo nessun controllo dell'orientamento

                // Calcola l'inverso della matrice di inerzia nello spazio delle giunte
                Eigen::MatrixXd M_inv = M_.data.inverse();

                // Calcola la matrice di inerzia dello spazio operativo Lambda
                Eigen::MatrixXd Lambda = (J_.data * M_inv * J_.data.transpose()).inverse();

                // Calcola le forze desiderate nello spazio delle task
                Eigen::VectorXd F_desired = Lambda * xddot_task;

                // Mappa le forze dello spazio delle task ai torques delle giunte
                tau_d_.data = J_.data.transpose() * F_desired;

                // Aggiungi compensazione di Coriolis e Gravità
                tau_d_.data += C_.data + G_.data;

                // Applica i comandi di torque alle giunte
                for (int i = 0; i < n_joints_; i++)
                {
                    joints_[i].setCommand(tau_d_(i));
                }

                // Verifica se il segmento corrente è completato e passa al successivo
                if (elapsed_time >= segment_durations_[current_segment_index_])
                {
                    if (current_segment_index_ < waypoints_.size() - 2)
                    {
                        current_segment_index_++;
                        segment_start_time_ += segment_durations_[current_segment_index_ - 1];
                    }
                }

                break;
            }


            default:
                ROS_WARN("Invalid control mode selected: %d", control_mode);
                
                break;
        }

    
        
        
        
        
    }



    void stopping(const ros::Time &time)
    {
    }

    void computePseudoInverse(const Eigen::MatrixXd &J, Eigen::MatrixXd &J_pinv, double tolerance = 1e-5) {
        // Perform SVD decomposition
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        
        // Get singular values
        Eigen::VectorXd singularValues = svd.singularValues();
        
        // Create a vector to hold the inverted singular values
        Eigen::VectorXd singularValuesInv(singularValues.size());
        
        for (int i = 0; i < singularValues.size(); ++i) {
            if (singularValues(i) > tolerance) {
                singularValuesInv(i) = 1.0 / singularValues(i);  // Invert if above tolerance
            } else {
                singularValuesInv(i) = 0.0;  // Set to zero if below tolerance
            }
        }
        
        // Construct the pseudo-inverse
        J_pinv = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose();
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
    boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
    
    boost::scoped_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_; // 

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

    Eigen::VectorXd qd_dot_task; // Desired joint velocities from task-space mapping

    //errori untegrativi
    Eigen::VectorXd er_i;  
  
 

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
    
    //Aruco part
    ros::Subscriber sub_aurco;
    realtime_tools::RealtimeBuffer<ArucoData> aruco_buffer_;
    Eigen::VectorXd aruco_x; 
    Eigen::VectorXd aruco_x_dot;

    // Control mode
    realtime_tools::RealtimeBuffer<int> control_mode_buffer_;
    ros::Subscriber sub_control_mode_;

    //Smooth task space controller case8

    // Trajectory planning variables
    bool traj_initialized_;
    double traj_start_time_;
    double traj_duration_;
    KDL::Vector x_init_;
    KDL::Vector xd_target_;

    KDL::Vector traj_start_position_; // Start position of the trajectory
    KDL::Vector traj_goal_position_;  // Goal position of the trajectory

    // Variabili per la pianificazione della traiettoria con waypoint
    std::vector<KDL::Vector> waypoints_;
    std::vector<double> segment_durations_;
    int current_segment_index_ = 0;
    double segment_start_time_ = 0.0;

    //erorri
    // ROS publishers for task-space error and force
    ros::Publisher pub_ex_;
    ros::Publisher pub_force_;



};



}; // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(arm_controllers::Computed_Torque_Controller, controller_interface::ControllerBase)