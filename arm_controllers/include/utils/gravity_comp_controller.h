#ifndef GRAVITY_COMP_CONTROLLER_H
#define GRAVITY_COMP_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>
#include <vector>
#include <string>

#include "arm_controllers/ControllerJointState.h"

namespace arm_controllers {

class GravityCompController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    GravityCompController();
    ~GravityCompController();

    bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void stopping(const ros::Time& time) override;
    void motionCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

private:
    void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
    void enforceJointLimits(double &command, unsigned int index);

    int loop_count_;
    unsigned int n_joints_;
    std::vector<std::string> joint_names_;
    std::vector<hardware_interface::JointHandle> joints_;
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
    KDL::JntArray G_;
    KDL::Vector gravity_;

    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
    KDL::JntArray q_cmd_, qdot_cmd_, qddot_cmd_;
    KDL::JntArray q_, qdot_;

    Eigen::VectorXd tau_cmd_, tau_fric_;
    Eigen::VectorXd q_error_, q_error_dot_;

    std::vector<control_toolbox::Pid> pids_;

    ros::Subscriber command_sub_;
    boost::scoped_ptr<
        realtime_tools::RealtimePublisher<
            arm_controllers::ControllerJointState> > controller_state_pub_;

    //publisher for the key from the changing in behavior
    ros::Subscriber motion_command_sub_;
    bool use_circular_motion_;
};

} // namespace arm_controllers

#endif // GRAVITY_COMP_CONTROLLER_H
