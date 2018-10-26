
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