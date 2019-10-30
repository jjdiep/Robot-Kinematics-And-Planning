
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.timer = 0;

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) {
        song.pause()
        return; 
    }
    // STENCIL: implement FSM to cycle through dance pose setpoints
    if (song.paused) { // not working, no song?!
        song.load();
        song.play();
    }

    kineval.params.dance_sequence_index = [0,1,0,1,0,1,0,1,0,1];

    kineval.setpoints = [{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0},
    {"torso_lift_joint":1.2000000000000008,"shoulder_pan_joint":0.3300000000000001,"shoulder_lift_joint":-0.5800000000000003,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0,"r_wheel_joint":0,"l_wheel_joint":0,"r_gripper_finger_joint":0,"l_gripper_finger_joint":0,"bellows_joint":0,"bellows_joint2":0,"estop_joint":0,"laser_joint":0}]
    




    setTimeout(function() {
        if (kineval.params.dance_pose_index >= kineval.params.dance_sequence_index.length) {
            kineval.params.dance_pose_index = 0;
        }
        var dance_pose_index = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];
        var j;
        for (j in robot.joints) {
            kineval.params.setpoint_target[j] = kineval.setpoints[dance_pose_index][j];
        }
        ++kineval.params.dance_pose_index;
        kineval.timer += 1000;
    }, kineval.timer);
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    var j;

    for (j in robot.joints) {
        var current_joint = robot.joints[j];
        current_joint.servo.p_desired = kineval.params.setpoint_target[j];
        var error = current_joint.servo.p_desired - current_joint.angle;
        current_joint.control = current_joint.servo.p_gain * error;
    }
}


