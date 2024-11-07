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
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>


#include <visualization_msgs/Marker.h> // Ensure this is present
#include "arm_controllers/ExtendedVector3.h"


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


// Define a structure for cylindrical obstacles
struct CylindricalObstacle {
    KDL::Vector position;  // Base center position (x, y, z)
    double radius;
    double height;
    double influence_radius; // Radius within which the obstacle repels the robot
};

//Ve define the rectnale base on its center so height = 1.0 the z pose shoud be 0.5
// center ( 0.5 , 0.0 , 0.5)
    //  0.5 costant over x
    //  0.5 on z as in the middel of the height
struct RectangularObstacle {
    KDL::Vector center;   // Center position of the rectangle (x, y, z)
    double width;         // Size along the x-axis
    double depth;         // Size along the y-axis
    double height;        // Size along the z-axis
    double safety_margin; // Optional safety margin
};



namespace arm_controllers
{
class GravityCompController : public controller_interface::Controller<hardware_interface::EffortJointInterface>    
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
            if (n.getParam("/elfin/gravity_comp_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/gravity_comp_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/gravity_comp_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/gravity_comp_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
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

        xd = Eigen::VectorXd::Zero(6);
        xd_dot = Eigen::VectorXd::Zero(6);

        ex_ = Eigen::VectorXd::Zero(n_joints_); // Initialize as a 6D vector with all elements zero



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
        sub_command_ = n.subscribe("/motion_command", 1000, &GravityCompController::commandCB, this);

        sub_control_mode_ = n.subscribe("/control_mode", 10, &GravityCompController::controlModeCB, this);

        //publisher of end effecot
        pub_end_effector_pos_ = n.advertise<std_msgs::Float64MultiArray>("/end_effector_pos", 1000);

      

        pub_influence_field_viz_ = n.advertise<visualization_msgs::Marker>("/influence_field_marker", 10);

        distance_from_target = n.advertise<arm_controllers::ExtendedVector3>("/distance_obj", 1000);
        
        //publisher force and distance for each joint
        pub_distance_per_joint_.resize(n_joints_);
        pub_force_per_joint_.resize(n_joints_);

        for (int i = 0; i < n_joints_; ++i)
        {
            std::string distance_topic_name = "/joint_" + std::to_string(i + 1) + "/distance";
            pub_distance_per_joint_[i] = n.advertise<std_msgs::Float64MultiArray>(distance_topic_name, 1000);

            std::string force_topic_name = "/joint_" + std::to_string(i + 1) + "/force";
            pub_force_per_joint_[i] = n.advertise<std_msgs::Float64MultiArray>(force_topic_name, 1000);
        }

        
        control_mode_buffer_.writeFromNonRT(1);


        // Initialize the command buffer with zeros
        Commands initial_commands;
        initial_commands.qd.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.qd_dot.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.qd_ddot.data = Eigen::VectorXd::Zero(n_joints_);
        initial_commands.xd = Eigen::VectorXd::Zero(6);
        initial_commands.xd_dot = Eigen::VectorXd::Zero(6);

        command_buffer_.initRT(initial_commands);

        
        //Obstacle// Initialize obstacles
        CylindricalObstacle cylinder;
        cylinder.position = KDL::Vector(0.0, 0.5, 0.15); // Position of the base of the cylinder
        cylinder.radius = 0.25;  // Cylinder radius
        cylinder.height = 0.3;   // Cylinder height
        cylinder.influence_radius = 0.30; // Influence radius for APF
        cylindrical_obstacles_.push_back(cylinder);
        
    

        // Define the wall as a rectangular obstacle
        RectangularObstacle wall;
        wall.center = KDL::Vector(0.85, 0.0, 0.3); // Center position
        wall.width = 0.7;                         // Thickness along x-axis (e.g., 0.1 meters)
        wall.depth = 1.0;                         // Depth along y-axis (from -0.5 to 0.5)
        wall.height = 1.0;                        // Height along z-axis
        wall.safety_margin = 0.05;                // Optional safety margin (adjust as needed)
        rectangular_obstacles_.push_back(wall);
        
        //Retrive joint for ob avoidance
        joint_segment_indices_.resize(n_joints_);
        for (size_t j = 0; j < n_joints_; ++j) {
            joint_segment_indices_[j] = -1; // Initialize with invalid index
        }

        const std::vector<KDL::Segment>& segments = kdl_chain_.segments;

        for (size_t i = 0; i < segments.size(); ++i) {
            const KDL::Segment& seg = segments[i];
            const std::string& joint_name = seg.getJoint().getName();

            for (size_t j = 0; j < joint_names_.size(); ++j) {
                if (joint_names_[j] == joint_name) {
                    joint_segment_indices_[j] = i;
                    ROS_INFO("Joint %s corresponds to segment index %zu", joint_name.c_str(), i);
                    break;
                }
            }
        }

        // Check that all joints have valid segment indices
        for (size_t j = 0; j < n_joints_; ++j) {
            if (joint_segment_indices_[j] == -1) {
                ROS_ERROR("Could not find segment index for joint %s", joint_names_[j].c_str());
                return false;
            }
        }

         // Stampa la mappatura finale Joint -> Segmento
        ROS_INFO("Mappatura finale Joint -> Segmento:");
        for (size_t j = 0; j < n_joints_; ++j) {
            ROS_INFO("Joint %zu (%s) -> Segmento %d", j, joint_names_[j].c_str(), joint_segment_indices_[j]);
        }

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

        xd = cmd.xd;          // xd is a 6-element vector
        xd_dot = cmd.xd_dot;  // xd_dot is a 6-element vector

        int control_mode = *(control_mode_buffer_.readFromRT());


       
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

            case 7:
            {
                ROS_INFO("---------------------Inside Task-Space Control with Obstacle Avoidance------------------------------");

                // ------------------------ Task-Space Control ------------------------

                // Compute End-effector position and Jacobian
                KDL::Frame end_effector_frame;  
                int fk_status = fk_pos_solver_->JntToCart(q_, end_effector_frame);
               

                // Compute Jacobian for end-effector
                KDL::Jacobian J_ee(n_joints_);
                J_ee.resize(n_joints_);
                int jac_status = jnt_to_jac_solver_->JntToJac(q_, J_ee);
               

                // Extract current end-effector position
                double x = end_effector_frame.p.x();
                double y = end_effector_frame.p.y();
                double z = end_effector_frame.p.z();

                ROS_INFO("End-effector position: [x: %f, y: %f, z: %f]", x, y, z);

                // Compute end-effector velocity
                xdot_ = J_ee.data * qdot_.data;

                // Desired Task-Space Position and Orientation
                KDL::Vector desired_position(xd(0), xd(1), xd(2));
                KDL::Rotation desired_orientation = KDL::Rotation::RotX(-1.5 * M_PI); // Example rotation
                KDL::Frame desired_frame(desired_orientation, desired_position);

                // Compute Task-Space Error
                KDL::Twist ex_temp_ = KDL::diff(end_effector_frame, desired_frame);
                for (int i = 0; i < 6; ++i) {
                    ex_(i) = ex_temp_(i);
                }

                // ROS_INFO for debugging
                // ROS_INFO("Task-space position error (ex_): [vx: %f, vy: %f, vz: %f, wx: %f, wy: %f, wz: %f]",
                //          ex_(0), ex_(1), ex_(2), ex_(3), ex_(4), ex_(5));

                // Task-Space PID Gains
                KDL::Vector Kp_trans(500, 500, 500); // Proportional gains for translation
                KDL::Vector Kd_trans(80, 80, 80);    // Derivative gains for translation

                KDL::Vector Kp_rot(15.0, 15.0, 15.0); // Proportional gains for rotation
                KDL::Vector Kd_rot(15.0, 15.0, 15.0); // Derivative gains for rotation

                // Compute Desired Task-Space Forces and Torques
                KDL::Wrench F_desired;
                for (int i = 0; i < 3; i++) {
                    F_desired.force(i) = Kp_trans(i) * ex_(i) + Kd_trans(i) * (-xdot_(i));
                    F_desired.torque(i) = Kp_rot(i) * ex_(i + 3) + Kd_rot(i) * (-xdot_(i + 3));
                }

                // ROS_INFO("Task-space force: [Fx: %f, Fy: %f, Fz: %f]", 
                //          F_desired.force(0), F_desired.force(1), F_desired.force(2));

                // ------------------------ Obstacle Avoidance for All Joints ------------------------

                // Initialize the total obstacle avoidance torque
                Eigen::VectorXd tau_obstacle_total = Eigen::VectorXd::Zero(n_joints_);

                // Loop through each joint to compute its position and corresponding repulsive force
                for (int j = 0; j < n_joints_; ++j)
                {
                    int segment_idx = joint_segment_indices_[j];
                    

                    // Compute the frame (position and orientation) of the current joint
                    KDL::Frame joint_frame;
                    fk_pos_solver_->JntToCart(q_, joint_frame, segment_idx);
                   

                    // Compute the Jacobian for the current joint
                    KDL::Jacobian J_link(n_joints_);
                    J_link.resize(n_joints_);
                    jnt_to_jac_solver_->JntToJac(q_, J_link, segment_idx);
                    

                    // Get the position of the current joint
                    KDL::Vector joint_position = joint_frame.p;

                    // Initialize the repulsive force for this joint
                    KDL::Vector F_repulsive_joint(0.0, 0.0, 0.0);

                    // -------------------- Obstacle Avoidance for Cylindrical Obstacles --------------------
                    for (const auto& obs : cylindrical_obstacles_)
                    {
                        // Compute the vector from obstacle to joint
                        KDL::Vector delta = joint_position - obs.position;
                        double distance = delta.Norm();

                        // Avoid division by zero
                        double epsilon = 1e-6;
                        if (distance < epsilon)
                            distance = epsilon;

                        // Check if within influence radius
                        if (distance <= obs.influence_radius)
                        {
                            // Compute repulsive force magnitude
                            double eta = 1000.0; // Repulsive gain (adjust as needed)
                            double repulsive_magnitude = eta * (1.0 / distance - 1.0 / obs.influence_radius) / (distance * distance);

                            // Compute direction of repulsion
                            KDL::Vector direction = delta / distance;

                            // Compute repulsive force vector
                            KDL::Vector F_rep = repulsive_magnitude * direction;

                            // Accumulate the repulsive force
                            F_repulsive_joint += F_rep;
                        }
                    }

                    // -------------------- Obstacle Avoidance for Rectangular Obstacles --------------------
                    for (const auto& obs : rectangular_obstacles_)
                    {
                        // Compute the closest point on the rectangular obstacle to the joint
                        double x_min = obs.center.x() - (obs.width / 2.0) - obs.safety_margin;
                        double x_max = obs.center.x() + (obs.width / 2.0) + obs.safety_margin;
                        double y_min = obs.center.y() - (obs.depth / 2.0) - obs.safety_margin;
                        double y_max = obs.center.y() + (obs.depth / 2.0) + obs.safety_margin;
                        double z_min = obs.center.z() - (obs.height / 2.0) - obs.safety_margin;
                        double z_max = obs.center.z() + (obs.height / 2.0) + obs.safety_margin;

                        double x_closest = std::max(x_min, std::min(joint_position.x(), x_max));
                        double y_closest = std::max(y_min, std::min(joint_position.y(), y_max));
                        double z_closest = std::max(z_min, std::min(joint_position.z(), z_max));

                        KDL::Vector closest_point(x_closest, y_closest, z_closest);

                        // Compute the vector from the closest point to the joint
                        KDL::Vector delta = joint_position - closest_point;
                        double distance = delta.Norm();

                        // Avoid division by zero
                        double epsilon = 1e-6;
                        if (distance < epsilon)
                            distance = epsilon;

                        // Check if within safety margin
                        if (distance <= obs.safety_margin)
                        {
                            // Compute repulsive force magnitude
                            double eta = 100.0; // Repulsive gain (adjust as needed)
                            double repulsive_magnitude = eta * (1.0 / distance - 1.0 / obs.safety_margin) / (distance * distance);

                            // Compute direction of repulsion
                            KDL::Vector direction = delta / distance;

                            // Compute repulsive force vector
                            KDL::Vector F_rep = repulsive_magnitude * direction;

                            // Optionally, add a tangential force to prevent the robot from getting stuck
                            // Define a reference vector (e.g., upward direction)
                            KDL::Vector ref_vector(0.0, 0.0, 1.0);

                            // Compute the tangential direction as the cross product of direction and reference vector
                            KDL::Vector tangential_direction = direction * ref_vector; // Cross product

                            // Check if 'direction' is parallel to 'ref_vector' to avoid zero tangential force
                            if (fabs(KDL::dot(direction, ref_vector)) >= 0.99) // Adjust threshold as needed
                            {
                                ref_vector = KDL::Vector(1.0, 0.0, 0.0); // Use x-axis if parallel
                                tangential_direction = direction * ref_vector;
                            }

                            // Normalize the tangential direction
                            double tangential_norm = tangential_direction.Norm();
                            if (tangential_norm > epsilon)
                            {
                                tangential_direction = tangential_direction / tangential_norm;

                                // Define tangential gain
                                double tangential_gain = 1000.0; // Adjust this gain as needed

                                // Compute tangential force
                                KDL::Vector F_tangential = tangential_gain * tangential_direction;

                                // Accumulate the tangential force
                                F_rep += F_tangential;

                             
                            }

                            // Accumulate the repulsive force
                            F_repulsive_joint += F_rep;
                        }
                    }

                    //mesage of the joint
                    std_msgs::Float64MultiArray distance_msg;
                    distance_msg.data.resize(3);
                    distance_msg.data[0] = joint_position.x();
                    distance_msg.data[1] = joint_position.y();
                    distance_msg.data[2] = joint_position.z();
                    pub_distance_per_joint_[j].publish(distance_msg);

                    std_msgs::Float64MultiArray force_msg;
                    force_msg.data.resize(3);
                    force_msg.data[0] = F_repulsive_joint.x();
                    force_msg.data[1] = F_repulsive_joint.y();
                    force_msg.data[2] = F_repulsive_joint.z();
                    pub_force_per_joint_[j].publish(force_msg);

                    // -------------------- Map Repulsive Forces to Joint Torques --------------------

                    // Convert KDL::Vector to Eigen::VectorXd (only force components, ignoring torque for simplicity)
                    Eigen::VectorXd F_repulsive_vec(6);
                    F_repulsive_vec << F_repulsive_joint.x(), F_repulsive_joint.y(), F_repulsive_joint.z(),
                                    0.0, 0.0, 0.0; // Assuming no torque from obstacles

                    // Map the spatial force to joint torques using the Jacobian transpose
                    Eigen::VectorXd tau_obstacle_joint = J_link.data.transpose() * F_repulsive_vec;

                    // Accumulate the torques
                    tau_obstacle_total += tau_obstacle_joint;

                   
                }

                // ------------------------ Combine Task-Space Control and Obstacle Avoidance ------------------------

                // 1. Map Task-Space Forces to Joint Torques
                Eigen::VectorXd F_desired_vec(6);
                F_desired_vec << F_desired.force.x(), F_desired.force.y(), F_desired.force.z(),
                                F_desired.torque.x(), F_desired.torque.y(), F_desired.torque.z();

                // Compute joint torques from task-space forces using end-effector Jacobian
                Eigen::VectorXd tau_task = J_ee.data.transpose() * F_desired_vec;

                // 2. Compute Inverse Dynamics for Compensation (Coriolis and Gravity)
                // tau_task already includes Coriolis and Gravity in your original code, but ensure it's correctly applied
                tau_task += C_.data + G_.data;

                // 3. Combine Task-Space Torques with Obstacle Avoidance Torques
                tau_d_.data = tau_task + tau_obstacle_total;

                // ------------------------ Apply Torque Commands ------------------------
                for (int i = 0; i < n_joints_; i++)
                {
                    joints_[i].setCommand(tau_d_(i));
                }

                // ------------------------ Publish End-Effector Position and Velocity ------------------------
                std_msgs::Float64MultiArray end_effector_msg;
                end_effector_msg.data.resize(6);
                end_effector_msg.data[0] = x;         // x position
                end_effector_msg.data[1] = y;         // y position
                end_effector_msg.data[2] = z;         // z position
                end_effector_msg.data[3] = xdot_(0);  // vx
                end_effector_msg.data[4] = xdot_(1);  // vy
                end_effector_msg.data[5] = xdot_(2);  // vz

                pub_end_effector_pos_.publish(end_effector_msg);

               

                ROS_INFO("-----------------------------------------------------------------------");
                ROS_INFO("-");

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

    //task space 
    Eigen::VectorXd xd, xd_dot;     // Desired task-space position (6 elements)



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
    
  
    // Control mode
    realtime_tools::RealtimeBuffer<int> control_mode_buffer_;
    ros::Subscriber sub_control_mode_;


    //End effector postion sub
    ros::Publisher pub_end_effector_pos_;

    // ROS message for end-effector position and velocity
    std_msgs::Float64MultiArray msg_end_effector_pos_;


    //Obstacole
    std::vector<CylindricalObstacle> obstacles_;

    std::vector<CylindricalObstacle> cylindrical_obstacles_;
    std::vector<RectangularObstacle> rectangular_obstacles_;

  
    ros::Publisher distance_from_target;

    ros::Publisher pub_influence_field_viz_;
    
    //Position of joint
    std::vector<int> joint_segment_indices_;

    //Bublishing for each joint
    std::vector<ros::Publisher> pub_distance_per_joint_;
    std::vector<ros::Publisher> pub_force_per_joint_;



};



}; // namespace arm_controllers


PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityCompController, controller_interface::ControllerBase)
