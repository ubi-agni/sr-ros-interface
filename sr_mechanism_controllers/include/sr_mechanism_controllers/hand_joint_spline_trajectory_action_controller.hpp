/**
 * @file   hand_joint_spline_trajectory_action_controller.hpp
 * @author Guillaume Walck (UPMC) & Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Dec 2012
 * 
 * @brief  Implement an actionlib server to execute a 
 * control_msgs/FollowJointTrajectoryAction. Follows the 
 * given trajectory with the hand.
 * 
 * 
 */

#ifndef _SR_HAND_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_
#define _SR_HAND_JOINT_TRAJECTORY_ACTION_CONTROLLER_H_
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <std_msgs/String.h>

namespace shadowrobot 
{
  class HandJointTrajectoryActionController
  {
    typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> JTAS;
  public:
    HandJointTrajectoryActionController();
    ~HandJointTrajectoryActionController();
    void init();


  private:
    ros::NodeHandle nh, nh_tilde;
    ros::Subscriber command_sub;
    ros::Subscriber reload_sub;
    ros::Publisher sr_hand_target_pub;
    ros::Publisher desired_joint_state_pusblisher;
    std::vector<std::string> joint_names_;
    std::vector<std::string> j0Labels;          //!< stores J0 joint names for easy access
    ros::ServiceClient joint_state_client;
    std::map<std::string,double> joint_state_map;
    std::map<std::string,double> joint_velocity_map;
    std::vector<std::string> jointLabels;
    void reload_controllers_callback(const std_msgs::String &command);
    ros::ServiceClient ctrlListClient;
    
    std::map<std::string, ros::Publisher > pubTargetMap;  //!< stores a map of publishers to each joint controller command.
    std::map<std::string, ros::Publisher > pubMaxForceMap;  //!< stores a map of publishers to each joint controller force setting.
    std::map<std::string,std::string> jointControllerMap ; //!< stores a map of controller name and associated joints
    std::map<std::string,unsigned int> joint_state_idx_map; //! store internal order of joints
    std::vector<int> coupled_joints; //! stores the coupled joints in internal order
    bool use_sendupdate;
    
    ros::Time last_time_;
    boost::shared_ptr<JTAS> action_server;
    
   /* bool queryStateService(pr2_controllers_msgs::QueryTrajectoryState::Request &req,
                         pr2_controllers_msgs::QueryTrajectoryState::Response &resp);
  ros::ServiceServer serve_query_state_;*/

    
    // coef[0] + coef[1]*t + ... + coef[5]*t^5
    struct Spline
    {
      std::vector<double> coef;

      Spline() : coef(6, 0.0) {}
    };

    struct Segment
    {
      double start_time;
      double duration;
      std::vector<Spline> splines;
    };
    typedef std::vector<Segment> SpecifiedTrajectory;
    
    std::vector<double> q, qd, qdd;  // Preallocated in init

  // Samples, but handling time bounds.  When the time is past the end
  // of the spline duration, the position is the last valid position,
  // and the derivatives are all 0.
  static void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
                                         double& position, double& velocity, double& acceleration);

    void getParam();
		void publishAll(const std::vector<std::string> &jointNames);
    void execute_trajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
    void commandCB(const trajectory_msgs::JointTrajectoryConstPtr &msg);
    void updateJointState();
    bool getPosition(std::string joint_name, double &position);
    bool getVelocity(std::string joint_name, double &velocity);

  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
