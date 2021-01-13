#ifndef KUKA_MANIPULATION_H
#define KUKA_MANIPULATION_H

#endif // KUKA_MANIPULATION_H

#include <tesseract_core/basic_types.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>

#include <urdf_parser/urdf_parser.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <test_bed_core/trajopt_pick_and_place_constructor.h>
#include <test_bed_core/trajopt_utils.h>
#include <pick_and_place_perception/GetTargetPose.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <iiwa_msgs/JointPosition.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Time.h>

#include <trajopt/file_write_callback.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>

#include <test_bed_core/gripper_interface.h>
#include <pick_and_place/PickTargetPose.h>
#include <pick_and_place/PickTargetID.h>

class KukaManipulation
{
private:
    ros::NodeHandle nh_;
    int steps_per_phase;
    std::string world_frame, pick_frame;
    bool sim_robot, actuate_gripper, plotting_cb, file_write_cb;

    tf::TransformListener listener;
    ros::Publisher test_pub, goal_pub, joint_pub;
    ros::Publisher cube_pub;
    ros::ServiceServer pick_pose_server, pick_id_server;
    std::string urdf_xml_string, srdf_xml_string;
    urdf::ModelInterfaceSharedPtr urdf_model;
    srdf::ModelSharedPtr srdf_model;
    tesseract::tesseract_ros::KDLEnvPtr env;
    std::unordered_map<std::string, double> joint_states;

    tesseract::tesseract_planning::TrajOptPlanner planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;
    tesseract::tesseract_planning::PlannerResponse retreat_response;
    tesseract::tesseract_planning::PlannerResponse planning_response_place;

public:
    KukaManipulation(ros::NodeHandle& nh):
        nh_(nh),
        env(new tesseract::tesseract_ros::KDLEnv)
    {
        nh_.param<int>("steps_per_phase", steps_per_phase, 5);
        nh_.param<std::string>("world_frame", world_frame, "world");
        nh_.param<std::string>("pick_frame", pick_frame, "part");
        nh_.param<bool>("sim_robot", sim_robot, false);
        nh_.param<bool>("actuate_gripper", actuate_gripper, true);
        nh_.param<bool>("plotting", plotting_cb, false);
        nh_.param<bool>("file_write_cb", file_write_cb, false);

        test_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_traj", 10);
        goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/command/CartesianPose", 10);
        joint_pub = nh_.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 10);
        cube_pub = nh_.advertise<geometry_msgs::Pose>("/target_cube_pose",10);
        pick_id_server = nh_.advertiseService("pick_by_id", &KukaManipulation::pick_by_id, this);
        pick_pose_server = nh_.advertiseService("pick_by_pose", &KukaManipulation::pick_by_pose, this);

        nh.getParam("robot_description", urdf_xml_string);
        nh.getParam("robot_description_semantic", srdf_xml_string);
        urdf_model = urdf::parseURDF(urdf_xml_string);
        srdf_model = srdf::ModelSharedPtr(new srdf::Model);
        srdf_model->initString(*urdf_model, srdf_xml_string);

        assert(urdf_model != nullptr);
        assert(env != nullptr);
        bool success = env->init(urdf_model, srdf_model);
        assert(success);

        if (sim_robot)
        {
          joint_states["iiwa_joint_1"] = 0.0;
          joint_states["iiwa_joint_2"] = 0.0;
          joint_states["iiwa_joint_3"] = 0.0;
          joint_states["iiwa_joint_4"] = -1.57;
          joint_states["iiwa_joint_5"] = 0.0;
          joint_states["iiwa_joint_6"] = 0.0;
          joint_states["iiwa_joint_7"] = 0.0;
        }
        else
        {
          boost::shared_ptr<const iiwa_msgs::JointPosition> joint_pos =
              ros::topic::waitForMessage<iiwa_msgs::JointPosition>("iiwa/state/JointPosition", nh);

          joint_states["iiwa_joint_1"] = joint_pos.get()->position.a1;
          joint_states["iiwa_joint_2"] = joint_pos.get()->position.a2;
          joint_states["iiwa_joint_3"] = joint_pos.get()->position.a3;
          joint_states["iiwa_joint_4"] = joint_pos.get()->position.a4;
          joint_states["iiwa_joint_5"] = joint_pos.get()->position.a5;
          joint_states["iiwa_joint_6"] = joint_pos.get()->position.a6;
          joint_states["iiwa_joint_7"] = joint_pos.get()->position.a7;
        }
        env->setState(joint_states);

    }
    bool generate_pick_place_trajectory(const Eigen::Isometry3d &pick_pose,
                                        const Eigen::Isometry3d &place_pose)
    {
        ////////////
        /// PICK ///
        ////////////

        // Begin the path planning
        if (true)
        {
          ROS_ERROR("Press enter to start planning pick");
          std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

          Eigen::Quaterniond orientation(0.0, 0.0, 1.0, 0.0);

          std::string manip = "Manipulator";
          std::string end_effector = "iiwa_link_ee";
          TrajoptPickAndPlaceConstructor prob_constructor(env, manip, end_effector, "box");

          // Define the final pose
          Eigen::Isometry3d final_pose;
          final_pose.linear() = pick_pose.rotation();
          final_pose.translation() = pick_pose.translation();

          double gripper_offset = 0.10;
          final_pose.translation() += Eigen::Vector3d(0.0, 0.0, gripper_offset);  // We add an offset for a gripper since it's not in the URDF

          // Define the approach pose
          Eigen::Isometry3d approach_pose = final_pose;
          approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

          // Create and solve pick problem
          trajopt::TrajOptProbPtr pick_prob =
              prob_constructor.generatePickProblem(approach_pose, final_pose, steps_per_phase);

          // Set the optimization parameters (Most are being left as defaults)
          tesseract::tesseract_planning::TrajOptPlannerConfig config(pick_prob);
          config.params.max_iter = 500;
          // Solve problem
          planner.solve(planning_response, config);


          // Publish the trajectory for debugging
          trajectory_msgs::JointTrajectory traj_msg3;
          ros::Duration t1(0.25);
          traj_msg3 = trajArrayToJointTrajectoryMsg(planning_response.joint_names, planning_response.trajectory, false, t1);
          test_pub.publish(traj_msg3);

  //        ROS_ERROR("Press enter to start planning place");
  //        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

          /////////////
          /// PLACE ///
          /////////////

          // Set the current state to the last state of the trajectory
          env->setState(env->getJointNames(), planning_response.trajectory.bottomRows(1).transpose());

          // Pick up box
          Eigen::Isometry3d retreat_pose = approach_pose;

          // Define some place locations.
          Eigen::Isometry3d table_spot_1, random_place;

          // Set the target pose to middle_right_shelf
          final_pose.linear() = place_pose.rotation();
          final_pose.translation() = place_pose.translation();

          // Setup approach for place
          approach_pose = final_pose;
          approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

          // generate and solve the problem

          trajopt::TrajOptProbPtr place_prob =
              prob_constructor.generatePlaceProblem(retreat_pose, approach_pose, final_pose, steps_per_phase);

          // Set the parameters
          tesseract::tesseract_planning::TrajOptPlannerConfig config_place(place_prob);
          config_place.params.max_iter = 500;

          // Solve problem
          planner.solve(planning_response_place, config_place);

          // Publish the trajectory for debugging
          trajectory_msgs::JointTrajectory traj_msg4;
          ros::Duration t2(0.25);
          traj_msg4 = trajArrayToJointTrajectoryMsg(
              planning_response_place.joint_names, planning_response_place.trajectory, false, t2);
          test_pub.publish(traj_msg4);


          ////////////////////
          /// LinearMotion ///
          ////////////////////

          // Set the current state to the last state of the trajectory
          env->setState(env->getJointNames(), planning_response_place.trajectory.bottomRows(1).transpose());

          Eigen::Isometry3d ready_pose = final_pose;
          ready_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.1);
          trajopt::TrajOptProbPtr retreat_prob =
                  prob_constructor.generateLinearMotion(final_pose, ready_pose, steps_per_phase);


          tesseract::tesseract_planning::TrajOptPlannerConfig config_retreat(retreat_prob);
          config_retreat.params.max_iter = 500;
          planner.solve(retreat_response, config_retreat);
        }
        std::cout << "Execute Trajectory on hardware? y/n \n";
        char input = 'n';
        std::cin >> input;
        if (input == 'y')
            return(execute());
        return false;
    }
    bool execute() {
        GripperInterface gi(nh_);
        if (actuate_gripper)
        {
            gi.homeGripper();
        }  // End gripper setup here
        std::cout << "Executing... \n";

        // Create action client to send trajectories
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> execution_client("iiwa/"
                                                                                                  "PositionJointInterfa"
                                                                                                  "ce_trajectory_"
                                                                                                  "controller/"
                                                                                                  "follow_joint_"
                                                                                                  "trajectory",
                                                                                                  true);
        execution_client.waitForServer();

        // Convert TrajArray (Eigen Matrix of joint values) to ROS message
        trajectory_msgs::JointTrajectory traj_msg;
        ros::Duration t(0.25);
        traj_msg = trajArrayToJointTrajectoryMsg(planning_response.joint_names, planning_response.trajectory, false, t);

        // Create action message
        control_msgs::FollowJointTrajectoryGoal trajectory_action;
        trajectory_action.trajectory = traj_msg;
        // May need to update other tolerances as well.

        // Send to hardware
        execution_client.sendGoal(trajectory_action);
        execution_client.waitForResult(ros::Duration(20.0));

        if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          std::cout << "Pick action succeeded! \n";
          // Put gripper pick code here (See Demo 3.10)
          if (actuate_gripper)
          {
             gi.closeGripper(40,20);
          }  // End gripper pick code
        }
        else
        {
          std::cout << "Pick action failed \n";
          return false;
        }

        // Convert TrajArray (Eigen Matrix of joint values) to ROS message
        trajectory_msgs::JointTrajectory traj_msg2;
        ros::Duration t2(0.25);

        traj_msg2 = trajArrayToJointTrajectoryMsg(
            planning_response_place.joint_names, planning_response_place.trajectory, false, t2);
        //test_pub.publish(traj_msg2);

        // Create action message
        control_msgs::FollowJointTrajectoryGoal trajectory_action_place;
        trajectory_action_place.trajectory = traj_msg2;
        // May need to update tolerances as well.

        // Send to hardware
        execution_client.sendGoal(trajectory_action_place);
        execution_client.waitForResult(ros::Duration(20.0));
        if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          std::cout << "Place action succeeded! \n";
          // Put gripper release code here (See Demo 3.10)
          if (actuate_gripper)
          {
              gi.homeGripper();
          }  // End gripper release code
        }
        else
        {
          std::cout << "Place action failed \n";
          return false;
        }

        trajectory_msgs::JointTrajectory retreat_traj_msg;
        ros::Duration t3(0.25);

        retreat_traj_msg = trajArrayToJointTrajectoryMsg(
                    retreat_response.joint_names, retreat_response.trajectory, false, t3);
        control_msgs::FollowJointTrajectoryGoal trajectory_action_retreat;
        trajectory_action_retreat.trajectory = retreat_traj_msg;

        execution_client.sendGoal(trajectory_action_retreat);
        execution_client.waitForResult(ros::Duration(20.0));
        if (execution_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ros::Duration(0.5).sleep();
            iiwa_msgs::JointPosition joint_msg;
            joint_msg.position.a1 = 0;
            joint_msg.position.a2 = 0.17;
            joint_msg.position.a3 = 0;
            joint_msg.position.a4 = -1.41;
            joint_msg.position.a5 = 0;
            joint_msg.position.a6 = 1.56;
            joint_msg.position.a7 = 0;
            joint_pub.publish(joint_msg);

            //boost::shared_ptr<std_msgs::Time const> arrived;
            auto arrived_joint = ros::topic::waitForMessage<std_msgs::Time>("/iiwa/state/DestinationReached");
            std::cout << "Arm in home";
        }

        ROS_INFO("Pick & place completed");
        return true;

    }
    bool pick_by_pose(pick_and_place::PickTargetPose::Request &req,
                         pick_and_place::PickTargetPose::Response &res){

        Eigen::Isometry3d pick_pose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d place_pose = Eigen::Isometry3d::Identity();

        tf::poseMsgToEigen(req.target_pose, pick_pose);
        res.succeeded = generate_pick_place_trajectory(pick_pose, place_pose);
    }

    // This function is designed for April tag ID: cube_xxx
    bool pick_by_id(pick_and_place::PickTargetID::Request &req,
                               pick_and_place::PickTargetID::Response &res){
        tf::TransformListener listener;
        tf::StampedTransform transform;
        geometry_msgs::Pose target_pose;
        std::string cube_id = "/cube_" + std::to_string(req.id);
        try{
            listener.lookupTransform(cube_id, "/world", ros::Time(0), transform);
        }
        catch(tf::TransformException e){
            ROS_ERROR("%s",e.what());
        }
        target_pose.position.x = transform.getOrigin().x();
        target_pose.position.y = transform.getOrigin().y();
        target_pose.position.z = transform.getOrigin().z();

        tf::Quaternion quat = tf::Quaternion(tf::getYaw(transform.getRotation()),0,0);
        target_pose.orientation.w = quat.w();
        target_pose.orientation.x = quat.x();
        target_pose.orientation.y = quat.y();
        target_pose.orientation.z = quat.z();

        cube_pub.publish(target_pose);

    }
};
