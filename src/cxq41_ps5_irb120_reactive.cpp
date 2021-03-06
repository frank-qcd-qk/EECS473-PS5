// irb120_reactive_task_commander.cpp
// Originally written by Dr. Wyatt Neuman
// Modified by Chude Qian CXQ41@case.edu
// this version is a variation on irb120_task_commander, but adds an action
// client of the magic_object_finder to make the robot move in response to
// perceived objects
using namespace std;  // avoids having to say: std::string, std::cout, etc
// Mandatory for all ROS projects
#include <ros/ros.h>
// Following are Eigen Math Classes
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
// The following libraries can be found under the dependencies folder.
#include <cartesian_interpolator/cartesian_interpolator.h>
#include <fk_ik_virtual/fk_ik_virtual.h>  //defines the base class with virtual fncs
#include <generic_cartesian_planner/generic_cartesian_planner.h>
#include <irb120_fk_ik/irb120_kinematics.h>  //access to forward and inverse kinematics
// this is useful to keep the motion planner generic
// Files can be find within the src folder
#include "robot_specific_fk_ik_mappings.h"  //these two files are needed to provide robot-specific info to generic planner
#include "robot_specific_names.h"
// Msgs definition
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
// add these to use the "magic" object finder action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <magic_object_finder/magicObjectFinderAction.h>
// the following will be useful when need tool transforms
// #include <tf/transform_listener.h>
// #include <xform_utils/xform_utils.h>
// XformUtils xformUtils; //handy conversion utilities--but don't need these yet

//! The following variables are magical and global.
// TUNE: specify weights to use for planner optimization
// some magic numbers--place at the top of the program
std::vector<double> g_planner_joint_weights{
    3, 3, 2, 1, 1, 0.5};  // specify weights to use for planner optimization
string g_object_name("gear_part_ariac");  // hard-coded object of interest name;
                                          // edit this for different objects
// Tune: The following value is used for specify target:
float targetX = 0.3;
float targetY = 0.2;

//! The following variables are made global in order to make the function for
//! move robot to runnable.
ros::Publisher traj_publisher;     // Publisher is included in the function
                                   // therefore initiated on main, and then
ros::Publisher* g_pose_publisher;  // make this global so callback can access
                                   // it--for displaying object frames in rviz

CartTrajPlanner* pCartTrajPlanner;  // does  not  have to be global, unless
                                    // needed by other functions

Eigen::Affine3d
    goal_flange_affine;  // specify start and goal in Cartesian coords

Eigen::VectorXd
    g_q_vec_arm_Xd;  // arm pose in joint space; the only reason this is global
                     // is that it will be useful, in the future, for it to be
                     // updated by a subscriber to joint_states

std::vector<Eigen::VectorXd>
    optimal_path;  // a path in joint space is a sequence of 6-DOF joint-angle
                   // specifications
//! The following are global variables for control stop/kill switch
int g_found_object_code;  // global to communicate between callback and main:
                          // true if named object was found
int killSwitch = 0;  // since now changed to function, we still need a mechanism
                     // to kill node instead of coredump, here is a work around

geometry_msgs::PoseStamped
    g_perceived_object_pose;  // global to communicate between callback and
                              // main: pose  of found object

//! a utility for debugging: displays affines (origins only) from an std::vector
// of affines
void print_affines(std::vector<Eigen::Affine3d> affine_path) {
    int npts = affine_path.size();
    ROS_INFO("affine path, origins only: ");
    for (int i = 0; i < npts; i++) {
        cout << affine_path[i].translation().transpose() << endl;
    }
}

//! a utility for debugging: displays a trajectory message
void print_traj(trajectory_msgs::JointTrajectory des_trajectory) {
    int npts = des_trajectory.points.size();
    int njnts = des_trajectory.points[0].positions.size();
    // Eigen::VectorXd jspace_pt;
    // jspace_pt.resize(njnts);
    ROS_INFO("traj points: ");
    for (int i = 0; i < npts; i++) {
        // jspace_pt=optimal_path[i];
        for (int j = 0; j < njnts; j++) {
            cout << (des_trajectory.points[i]).positions[j] << ", ";
        }
        cout << "; t = " << des_trajectory.points[i].time_from_start.toSec()
             << endl;
    }
}

//! this callback function receives a result from the magic object finder action
//! server it sets g_found_object_code to true or false, depending on whether
//! the object was found if the object was found, then components of
//! g_perceived_object_pose are filled in
void objectFinderDoneCb(
    const actionlib::SimpleClientGoalState& state,
    const magic_object_finder::magicObjectFinderResultConstPtr& result) {
    ROS_INFO(" objectFinderDoneCb: server responded with state [%s]",
             state.toString().c_str());
    g_found_object_code = result->found_object_code;
    ROS_INFO("got object code response = %d; ", g_found_object_code);
    if (g_found_object_code ==
        magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND) {
        ROS_WARN("object-finder responded: object not found");
    } else if (g_found_object_code ==
               magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
        g_perceived_object_pose = result->object_pose;
        ROS_INFO("got pose x,y,z = %f, %f, %f",
                 g_perceived_object_pose.pose.position.x,
                 g_perceived_object_pose.pose.position.y,
                 g_perceived_object_pose.pose.position.z);

        ROS_INFO("got quaternion x,y,z, w = %f, %f, %f, %f",
                 g_perceived_object_pose.pose.orientation.x,
                 g_perceived_object_pose.pose.orientation.y,
                 g_perceived_object_pose.pose.orientation.z,
                 g_perceived_object_pose.pose.orientation.w);
        g_pose_publisher->publish(
            g_perceived_object_pose);  // this is to enable display of pose of
                                       // found object in rviz
    } else {
        ROS_WARN("object not found!");
    }
}

/**
 *  Summarized and extracted by CXQ41
    Obtain x,y,z coordinate, resolution and movetime and send command to robot.
    @param x The x coordinate of the desired location
    @param y The y coordinate of the desired location
    @param z The z coordinate of the desired location
    @param resolution The step amount of the movement
    @param motionTime The time given to the robot for traveling
    @return return nothing, but send action to node handler and move the robot
 */
void moveRobotTo(float x, float y, float z, int resolution, int motionTime) {
    ROS_INFO("[FunctionCalled] moveRobotTo...");
    // * Declaration
    trajectory_msgs::JointTrajectory
        new_trajectory;  // will package trajectory messages here
    Eigen::Vector3d flange_origin;

    // * Action
    flange_origin << x, y, z;
    goal_flange_affine.translation() = flange_origin;
    ROS_INFO_STREAM("move to flange origin: "
                    << goal_flange_affine.translation().transpose() << endl);
    ROS_INFO_STREAM("with orientation: " << endl
                                         << goal_flange_affine.linear()
                                         << endl);
    g_q_vec_arm_Xd =
        optimal_path.back();  // extract the last joint-space pose from the
                              // plan, so can use it for start of next plan
    // better would be to get resulting joint-space values from joint_states
    // compute an optimal Cartesian motion in joint space from current
    // joint-space pose to desired Cartesian pose
    optimal_path.clear();
    // compute the plan, to be returned in optimal_path
    if (!pCartTrajPlanner->plan_cartesian_path_w_rot_interp(
            g_q_vec_arm_Xd, goal_flange_affine, resolution, optimal_path)) {
        ROS_ERROR(
            "no feasible IK path for specified Cartesian motion; quitting");
        killSwitch = 1;  // There should be an observer inside the loop
    } else {
        ROS_INFO("Path Found! Moving now....");
    }
    // convert the path to a trajectory (adds joint-space names,  arrival times,
    // etc)
    pCartTrajPlanner->path_to_traj(optimal_path, motionTime, new_trajectory);
    // print_traj(new_trajectory);  // Enable only if you want to get flushed by
    // data.
    traj_publisher.publish(new_trajectory);  // publish the trajectory
    ros::Duration(motionTime).sleep();       // wait for the motion
    ROS_INFO("Movement complete!....");
}

void updatePosition() {
    ROS_INFO("[FunctionCalled] updatePosition...");
    ros::NodeHandle n;
    //! The following part of the code is the action server for obtaining the
    // set up an action client to query object poses using the magic object
    // finder
    actionlib::SimpleActionClient<magic_object_finder::magicObjectFinderAction>
        object_finder_ac("object_finder_action_service", true);
    bool finished_before_timeout = false;
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists) && (ros::ok())) {
        server_exists = object_finder_ac.waitForServer(ros::Duration(0.5));  //
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to object_finder action server");  // if here, then we
                                                           // connected to the
                                                           // server;
    ros::Publisher pose_publisher =
        n.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    g_pose_publisher = &pose_publisher;
    magic_object_finder::magicObjectFinderGoal
        object_finder_goal;  // instantiate goal message to communicate with
                             // magic_object_finder
    // xxxxxxxxxxxxxx  the following makes an inquiry for the pose of the part
    // of interest specify the part name, send it in the goal message, wait for
    // and interpret the result
    object_finder_goal.object_name =
        g_object_name
            .c_str();  // convert string object to old C-style string data
    object_finder_ac.sendGoal(
        object_finder_goal,
        &objectFinderDoneCb);  // request object finding via action server

    finished_before_timeout = object_finder_ac.waitForResult(
        ros::Duration(10.0));  // wait for a max time for response
    // NOTE: could do something else here (if useful) while waiting for response
    // from action server
    if (!finished_before_timeout) {
        ROS_ERROR("giving up waiting on result ");  // this should not happen;
                                                    // should get result of
                                                    // found or not-found
        killSwitch = 1;  // There should be an observer inside the loop
    }
    // check the result code to see if object was found or not
    if (g_found_object_code ==
        magic_object_finder::magicObjectFinderResult::OBJECT_FOUND) {
        ROS_INFO("found object!");
    } else {
        ROS_ERROR("object not found! Quitting");
        killSwitch = 1;  // There should be an observer inside the loop
    }
    //! End magic object finder initiator
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "reactive_commander");  // name this node
    ros::NodeHandle nh;                           // standard ros node handle
    Eigen::Affine3d
        start_flange_affine;  // specify start and goal in Cartesian coords
    trajectory_msgs::JointTrajectory
        new_trajectory;  // will package trajectory messages here

    updatePosition();
    if (killSwitch == 1) {
        ROS_ERROR("NO object found, throwing error now....");
        return 1;
    }
    //! Initiation Stage based on updated position. Hard Coded.
    // the following is an std::vector of affines.  It describes a path in
    // Cartesian coords, including orientations not needed yet; is constructed
    // inside the generic planner by interpolation std::vector<Eigen::Affine3d>
    // affine_path;
    Eigen::Matrix3d R_down;  // define an orientation corresponding to
                             // toolflange pointing down
    Eigen::Vector3d x_axis, y_axis, z_axis, flange_origin;
    z_axis << 0, 0, -1;             // points flange down
    x_axis << -1, 0, 0;             // arbitrary
    y_axis = z_axis.cross(x_axis);  // construct y-axis consistent with
                                    // right-hand coordinate frame
    R_down.col(0) = x_axis;
    R_down.col(1) = y_axis;
    R_down.col(2) = z_axis;
    flange_origin << 0.2, 0, 0.01;  // SHOULD GET FIXED: hard-coded pose can
                                    // result in ugly/dangerous motion
    int nsteps = 5;  // will need to specify how many interpolation points in
                     // Cartesian path; this is pretty coarse
    double arrival_time =
        3.0;  // will  need to specify arrival time for a Cartesian path

    // for this next line, I apparently did something wrong.  I should not have
    // to  instantiate a cartesianInterpolator, since the generic planner
    // instantiates one.  But I get a compiler error.  Hmm...  Workaround.
    CartesianInterpolator cartesianInterpolator;

    g_q_vec_arm_Xd.resize(
        NJNTS);  // generic vector resized to actual robot number of joints
    g_q_vec_arm_Xd << 0, 0, 0, 0, 0,
        0;  // assumes arm starts in this pose; better would be  to subscribe to
            // joint_states to get actual angles

    // our irb120 control  interface uses this topic to receive trajectories
    traj_publisher =
        nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);

    // somewhat odd construction: a pointer to an object of type
    // CartTrajPlanner, with arguments provided that are pointers to forward and
    // inverse kinematic functions.  This is to keep the planner generic, and
    // defer WHICH robot FK and IK to use until run time; Uses virtual functions
    // for this.
    pCartTrajPlanner = new CartTrajPlanner(pIKSolver, pFwdSolver, njnts);
    // the planner needs to define penalty weights to optimize a path
    pCartTrajPlanner->set_jspace_planner_weights(g_planner_joint_weights);
    // to fill out a trajectory, need to provide the joint names; these are
    // contained in a robot-specific header file
    pCartTrajPlanner->set_joint_names(g_jnt_names);

    optimal_path.clear();  // reset this std::vector before  each use, else will
                           // have old values persisting
    optimal_path.push_back(g_q_vec_arm_Xd);  // start from current pose
    optimal_path.push_back(
        g_q_vec_arm_Xd);  // go from current pose to current pose--not very
                          // useful; but can "warm up" control
    // publish/subscribe interface
    arrival_time =
        1;  // move should require zero time, but provide something small

    // function call from library (Class) CartTrajPlanner: converts a
    // joint-space path to a joint-space trajectory
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);  // display for  debug

    traj_publisher.publish(new_trajectory);  // publish the trajectory;
    ros::Duration(1).sleep();

    // example to show how to use forward kinematics from the class pointers
    // provided
    start_flange_affine = pFwdSolver->fwd_kin_solve(g_q_vec_arm_Xd);
    // display the flange affine corresponding to the specfied arm angles
    ROS_INFO_STREAM("fwd soln: origin = "
                    << start_flange_affine.translation().transpose() << endl);
    ROS_INFO_STREAM("fwd soln: orientation: " << endl
                                              << start_flange_affine.linear()
                                              << endl);
    goal_flange_affine.linear() =
        R_down;  // set the  goal orientation for flange to point down; will not
                 // need to change this for now
    moveRobotTo(g_perceived_object_pose.pose.position.x,
                g_perceived_object_pose.pose.position.y, 0.3, 50,
                2);  // Hover above the robot to show identification sucess.
                       // Abort if not
    if (killSwitch == 1) {
        ROS_ERROR("NO path found, throwing error now....");
        return 1;
    }
    ROS_INFO(
        "[StructureDebug] INITIATION DONE!!! HAND OVER to path finding...");

    // Gear height is 0.005
    // Gear diameter is 0.04
    bool arrivedTargetX =
        false;  // Switch for defining if target X position has been aquired
    bool arrivedTargetY =
        false;  // Switch for defining if target Y position has been aquired
    float desiredX =
        0.00;  // Local variable for storing calculated target X position
    float desiredY =
        0.00;  // Local variable for storing calculated target Y position
    while (ros::ok && (!arrivedTargetX || !arrivedTargetY)) {
        updatePosition();  // Always first update position
        //Check so that no redundant operation are operated...
        if (abs(targetX - g_perceived_object_pose.pose.position.x) <= 0.01) {
            arrivedTargetX = true;
            ROS_INFO(
                "[DecisionDebug] Current Detection X axis motion complete!");
        }
        if (abs(targetY - g_perceived_object_pose.pose.position.y) <= 0.01) {
            arrivedTargetY = true;
            ROS_INFO(
                "[DecisionDebug] Current Detection Y axis motion complete!");
        }
        if (arrivedTargetX && arrivedTargetY) {
            ROS_INFO(
                "[DecisionDebug] Current Detection both axis completed. "
                "Breaking loop...");
        }        
        if (!arrivedTargetX) {
            // If statement decide which side of the gear the robot should be
            ROS_INFO("[PathDebug] Current State: Moving along X axis...");
            if ((targetX - g_perceived_object_pose.pose.position.x) < 0) {
                ROS_INFO(
                    "[PathDebug] Current choice is go positive side on X "
                    "axis...");
                //* Calculate and gets to the correct side
                desiredX = g_perceived_object_pose.pose.position.x + 0.08;
                desiredY = g_perceived_object_pose.pose.position.y;
                ROS_INFO(
                    "[PathDebug] Desired inital targetX for toolflange is: %f",
                    desiredX);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 2);
                ROS_INFO("[PathDebug] Arm Ready to push along X...");
                ros::Duration(2)
                    .sleep();  // For debug stop the screen print to see...
                //* Push along one axis only
                desiredX = targetX + 0.07;
                ROS_INFO(
                    "[PathDebug] Desired final targetX for toolflange is: %f",
                    desiredX);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 10);
                ROS_INFO("[PathDebug] X Axis push motion complete....");
                ros::Duration(2)
                    .sleep();  // for debug stop the screen print to see...
                desiredX = targetX + 0.1;
                ROS_INFO(
                    "[PathDebug] Desired retrieve targetX for toolflange is: %f",
                    desiredX);                
                moveRobotTo(desiredX, desiredY, 0.005, 50, 1);
                ROS_INFO("[PathDebug] X Axis back out motion complete....");

            } else {
                ROS_INFO(
                    "[PathDebug] Current choice is go negative side on X "
                    "axis...");
                desiredX = g_perceived_object_pose.pose.position.x - 0.08;
                desiredY = g_perceived_object_pose.pose.position.y;
                ROS_INFO(
                    "[PathDebug] Desired inital targetX for toolflange is: %f",
                    desiredX);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 2);
                ROS_INFO("[PathDebug] Arm Ready to push along X...");
                ros::Duration(2)
                    .sleep();  // For debug stop the screen print to see...
                //* Push along one axis only
                desiredX = targetX - 0.05;
                ROS_INFO(
                    "[PathDebug] Desired final targetX for toolflange is: %f",
                    desiredX);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 10);
                ROS_INFO("[PathDebug] X Axis push motion complete....");
                ros::Duration(2)
                    .sleep();  // for debug stop the screen print to see...
                desiredX = targetX - 0.1;
                ROS_INFO(
                    "[PathDebug] Desired retrieve targetX for toolflange is: %f",
                    desiredX);                
                moveRobotTo(desiredX, desiredY, 0.005, 50, 1);
                ROS_INFO("[PathDebug] X Axis back out motion complete....");
            }
            updatePosition();  // Reset above the object once finished operation
            moveRobotTo(g_perceived_object_pose.pose.position.x,
                        g_perceived_object_pose.pose.position.y, 0.2, 50,
                        2);  // Hover above the robot to show identification
                               // sucess. Abort if not
            ROS_INFO(
                "[PathDebug] After pushing across X axis, current X,Y position is: %f,%f.",
                g_perceived_object_pose.pose.position.x,
                g_perceived_object_pose.pose.position.y);
            ROS_INFO(
                "[PathDebug] Tool Flange Reset from X motion ready to "
                "hover...");
        }
        if (!arrivedTargetY) {
            ROS_INFO("[PathDebug] Current State: Moving along Y axis...");
            if ((targetY - g_perceived_object_pose.pose.position.y) < 0) {
                ROS_INFO(
                    "[PathDebug] Current Case is go positive side on Y "
                    "axis...");
                //* Calculate and gets to the correct side
                desiredX = g_perceived_object_pose.pose.position.x;
                desiredY = g_perceived_object_pose.pose.position.y + 0.08;
                ROS_INFO(
                    "[PathDebug] Desired inital targetY for toolflange is: %f",
                    desiredY);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 2);
                ROS_INFO("[PathDebug] Arm Ready to push along Y...");
                ros::Duration(2)
                    .sleep();  // For debug stop the screen print to see...
                //* Push along one axis only
                desiredY = targetY + 0.07;
                ROS_INFO(
                    "[PathDebug] Desired final targetY for toolflange is: %f",
                    desiredY);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 10);
                ROS_INFO("[PathDebug] Y Axis motion complete....");
                ros::Duration(2)
                    .sleep();  // for debug stop the screen print to see...
                desiredY = targetY + 0.1;
                ROS_INFO(
                    "[PathDebug] Desired retrieve targetY for toolflange is: %f",
                    desiredX);                
                moveRobotTo(desiredX, desiredY, 0.005, 50, 1);
                ROS_INFO("[PathDebug] Y Axis back out motion complete....");

            } else {
                ROS_INFO("Current Case is go smaller side on Y axis...");
                desiredX = g_perceived_object_pose.pose.position.x;
                desiredY = g_perceived_object_pose.pose.position.y - 0.08;
                ROS_INFO("Desired inital targetY for toolflange is: %f",
                         desiredY);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 2);
                ROS_INFO("[PathDebug] Arm Ready to push along Y...");
                ros::Duration(2)
                    .sleep();  // For debug stop the screen print to see...
                //* Push along one axis only
                desiredY = targetY - 0.05;
                ROS_INFO("Desired final targetY for toolflange is: %f",
                         desiredY);
                moveRobotTo(desiredX, desiredY, 0.005, 50, 10);
                ROS_INFO("Y Axis motion complete....");
                ros::Duration(2)
                    .sleep();  // for debug stop the screen print to see...
                desiredY = targetY - 0.1;
                ROS_INFO(
                    "[PathDebug] Desired retrieve targetY for toolflange is: %f",
                    desiredX);                    
                moveRobotTo(desiredX, desiredY, 0.005, 50, 1);
                ROS_INFO("[PathDebug] Y Axis back out motion complete....");
            }
            updatePosition();  // Reset above the object once finished operation
            moveRobotTo(g_perceived_object_pose.pose.position.x,
                        g_perceived_object_pose.pose.position.y, 0.2, 50,
                        2);  // Hover above the robot to show identification
                               // sucess. Abort if not
            ROS_INFO(
                "[PathDebug] After pushing across X axis, current X,Y position is: %f,%f.",
                g_perceived_object_pose.pose.position.x,
                g_perceived_object_pose.pose.position.y);
            ROS_INFO("[PathDebug] Tool Flange Reset to hover...");
        }
        updatePosition();  // Need to update position in case X axis got
                           // moved during the process
        // Tune: Controll for deciding if target has been reached or not.
        // Including Error.
        if (abs(targetX - g_perceived_object_pose.pose.position.x) <= 0.01) {
            arrivedTargetX = true;
            ROS_INFO(
                "[DecisionDebug] Current Detection X axis motion complete!");
        }
        if (abs(targetY - g_perceived_object_pose.pose.position.y) <= 0.01) {
            arrivedTargetY = true;
            ROS_INFO(
                "[DecisionDebug] Current Detection Y axis motion complete!");
        }
        if (arrivedTargetX && arrivedTargetY) {
            ROS_INFO(
                "[DecisionDebug] Current Detection both axis completed. "
                "Breaking loop...");
        }
    }
    ROS_INFO(
        "[DecisionDebug] System consider target has been aquired....END.....");
}
