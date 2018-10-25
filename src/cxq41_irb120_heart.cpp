
//! The following variables are made global in order to make the function
//! runnable.
CartTrajPlanner* pCartTrajPlanner;
Eigen::Affine3d
    goal_flange_affine;  // specify start and goal in Cartesian coords
// arm pose in joint space; the only reason this is global is that it will be
// useful, in the future, for it to be updated by a subscriber to joint_states
Eigen::VectorXd g_q_vec_arm_Xd;
std::vector<Eigen::VectorXd>
    optimal_path;  // a path in joint space is a sequence of 6-DOF joint-angle
                   // specifications
int killSwitch = 0;  // since now changed to function, we still need a mechanism to
                 // kill node instead of coredump, here is a work around

// a utility for debugging: displays affines (origins only) from an std::vector
// of affines
void print_affines(std::vector<Eigen::Affine3d> affine_path) {
    int npts = affine_path.size();
    ROS_INFO("affine path, origins only: ");
    for (int i = 0; i < npts; i++) {
        cout << affine_path[i].translation().transpose() << endl;
    }
}

// a utility for debugging: displays a trajectory message
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
    // * Declaration
    trajectory_msgs::JointTrajectory new_trajectory;  // will package trajectory messages here
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
        killSwitch = 1; //There should be an observer inside the loop
    } else {
        ROS_INFO("Path Found! Moving now....");
    }
    // convert the path to a trajectory (adds joint-space names,  arrival times,
    // etc)
    pCartTrajPlanner->path_to_traj(optimal_path, motionTime, new_trajectory);
    print_traj(new_trajectory); //Enable only if you want to get flushed by data.
    traj_publisher.publish(new_trajectory);  // publish the trajectory
    ros::Duration(motionTime).sleep();       // wait for the motion
    ROS_INFO("Movement complete!....");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_commander");  // name this node
    ros::NodeHandle nh;                       // standard ros node handle
    Eigen::Affine3d
        start_flange_affine;  // specify start and goal in Cartesian coords
    trajectory_msgs::JointTrajectory
        new_trajectory;  // will package trajectory messages here

    // the following is an std::vector of affines.  It describes a path in
    // Cartesian coords, including orientations not needed yet; is constructed
    // inside the generic planner by interpolation std::vector<Eigen::Affine3d>
    // affine_path;
    Eigen::Matrix3d R_FaceAudience;  // define an orientataion corresponding to
                             // toolflange facing audience
    Eigen::Vector3d x_axis, y_axis, z_axis, flange_origin;
    z_axis << 1, 0, 0;             // ! points facing audience
    x_axis << 0, 1, 0;             // arbitrary
    y_axis = z_axis.cross(x_axis);  // construct y-axis consistent with
                                    // right-hand coordinate frame
    R_FaceAudience.col(0) = x_axis;
    R_FaceAudience.col(1) = y_axis;
    R_FaceAudience.col(2) = z_axis;
    flange_origin << 0.2, 0, 0.01;
    int nsteps = 5;  // will need to specify how many interpolation points in
                     // Cartesian path
    double arrival_time =
        5.0;  // will  need to specify arrival time for a Cartesian path

    // for this next line, I apparently did something wrong.  I should not have
    // to instantiate a cartesianInterpolator, since the generic planner
    // instantiates one.  But I get a compiler error.  Hmm...  Workaround.
    CartesianInterpolator cartesianInterpolator;

    g_q_vec_arm_Xd.resize(
        NJNTS);  // generic vector resized to actual robot number of joints
    g_q_vec_arm_Xd << 0, 0, 0, 0, 0,
        0;  // assumes arm starts in this pose; better would be  to subscribe to
            // joint_states to get actual angles

    // our irb120 control interface uses this topic to receive trajectories
    //Previously declared as global, no need to re-declare
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
        0.1;  // move should require zero time, but provide something small

    // function call from library (Class) CartTrajPlanner: converts a
    // joint-space path to a joint-space trajectory
    pCartTrajPlanner->path_to_traj(optimal_path, arrival_time, new_trajectory);
    print_traj(new_trajectory);  // display for  debug

    traj_publisher.publish(new_trajectory);  // publish the trajectory;
    ros::Duration(0.2).sleep();

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
        R_FaceAudience;  // set the  goal orientation for flange to point down; will not
                 // need to change this for now
    ROS_INFO(
        "Initialization stage passed, hand over to my own creation now....");

    // ! Star your code below
    moveRobotTo(0.5, 0, 0.35, 100, 1); //Smooth transition to the initialization position
    ROS_INFO("System ready....Please toggle TF path in RVIZ");
    ros::Duration(10).sleep();

    // ! Draw a heart to show the love for irb120........
    double t = 0.00;
    double z = 0.00;
    while (t < 6.28) {
        if (killSwitch== 1) {
            ROS_ERROR("NO path found, throwing error now....");
            return 1;
        }
        float y = (16 * (sin(t)) * (sin(t)) * (sin(t))) / 100;
        float x =
            (13 * cos(t) - 5 * cos(2 * t) - 2 * cos(3 * t) - cos(4 * t)) / 100 +
            0.35;
        // TUNE: this value can and should be tuned to observe result.
        t = t + 0.1;  
        moveRobotTo(0.5, y, x, 10, 1);
    }
    ROS_WARN("Drawing complete!");
}