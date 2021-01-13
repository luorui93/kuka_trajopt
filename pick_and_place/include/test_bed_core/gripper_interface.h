#ifndef GRIPPER_INTERFACE_H
#define GRIPPER_INTERFACE_H

#endif // GRIPPER_INTERFACE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ipa325_wsg50/WSG50HomingAction.h>
#include <ipa325_wsg50/WSG50GraspPartAction.h>
#include <ipa325_wsg50/WSG50ReleasePartAction.h>
#include <ipa325_wsg50/WSG50PrePositionFingersAction.h>
#include <ipa325_wsg50/ackFastStop.h>
#include <ipa325_wsg50/setForceLimit.h>
#include <ipa325_wsg50/setSoftLimits.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>

class GripperInterface
{
private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50HomingAction> home_ac_;
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50GraspPartAction> grasp_ac_;
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50PrePositionFingersAction> position_ac_;
    actionlib::SimpleActionClient<ipa325_wsg50::WSG50ReleasePartAction> release_ac_;
    ros::ServiceClient fast_stop_srv_;
    ros::ServiceClient force_srv_;
    ros::ServiceClient soft_limits_srv_;

    float force_limit;
public:
    GripperInterface (ros::NodeHandle& nh):
        nh_(nh),
        home_ac_("WSG50Gripper_Homing", true),
        grasp_ac_("WSG50Gripper_GraspPartAction", true),
        release_ac_("WSG50Gripper_ReleasePartAction", true),
        position_ac_("WSG50Gripper_PrePositionFingers", true)
    {
        fast_stop_srv_ = nh_.serviceClient<ipa325_wsg50::ackFastStop>("/AcknowledgeFastStop");
        force_srv_ = nh.serviceClient<ipa325_wsg50::setForceLimit>("/SetForceLimit");
        soft_limits_srv_ = nh.serviceClient<ipa325_wsg50::setSoftLimits>("/SetSoftLimits");
        nh.param<float>("test_gripper/force_limit", force_limit, 80.0);

        ROS_INFO("Waiting for action servers to start.");
        home_ac_.waitForServer(); //will wait for infinite time
        grasp_ac_.waitForServer();
        release_ac_.waitForServer();
        ipa325_wsg50::ackFastStop ack;
        ipa325_wsg50::setForceLimit set_force;
        ipa325_wsg50::setSoftLimits set_soft_limit;

        set_force.request.force = force_limit;
//        set_soft_limit.request.limit_plus = 0;
//        set_soft_limit.request.limit_minus = 50;

        fast_stop_srv_.call(ack);
        force_srv_.call(set_force);
//        soft_limits_srv_.call(set_soft_limit);

    }

    void homeGripper()
    {
        ROS_INFO("Homing gripper...");
        ipa325_wsg50::WSG50HomingGoal home_goal;
        home_goal.direction = true;   // True is in the out direction
        home_ac_.sendGoal(home_goal);
        bool finished_before_timeout = home_ac_.waitForResult(ros::Duration(10.0));

        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = home_ac_.getState();
          ROS_INFO("Homing finished: %s",state.toString().c_str());
        }
        else
          ROS_INFO("Homing did not finish before the time out.");
    }

    void releaseGripper(float openwidth, float speed)
    {
        ROS_INFO("Releasing Part...");
        ipa325_wsg50::WSG50ReleasePartGoal release_goal;
        release_goal.openwidth = openwidth;
        release_goal.speed = speed;
        release_ac_.sendGoal(release_goal);
        bool finished_before_timeout = release_ac_.waitForResult(ros::Duration(5.0));
        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = release_ac_.getState();
          ROS_INFO("Release finished: %s",state.toString().c_str());
        }
    }

    void closeGripper(float closewidth, float speed)
    {
        ROS_INFO("Grasping Part...");
//        ipa325_wsg50::WSG50GraspPartGoal grasp_goal;
//        grasp_goal.width = closewidth;  // Part width in mm
//        grasp_goal.speed = speed;  // Speed in mm/s
//        grasp_ac_.sendGoal(grasp_goal);
        ipa325_wsg50::WSG50PrePositionFingersGoal grasp_goal;
        grasp_goal.width = closewidth;
        grasp_goal.speed = speed;
        grasp_goal.stopOnBlock = false;
        position_ac_.sendGoal(grasp_goal);
        bool finished_before_timeout = position_ac_.waitForResult(ros::Duration(5.0));

        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = position_ac_.getState();
          ROS_INFO("Grasp finished: %s",state.toString().c_str());
        }
        else
          ROS_INFO("Grasp did not finish before the time out.");
    }

    void setForceLimit(float limit)
    {
        ipa325_wsg50::setForceLimit set_force;
        set_force.request.force = limit;
        ROS_INFO("Setting force limit to %.2f", limit);
        force_srv_.call(set_force);
    }
};
