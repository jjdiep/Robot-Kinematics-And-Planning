
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
        kineval.timer += 2000;
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


