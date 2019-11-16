
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    // Provided over Slack Channel via @OCJ

     // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);
    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
        + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
        + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );
    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // Find joints from base to end effector
    kineval.kinChain(endeffector_joint);
    var test = kinChain;
    // Find position of endeffector in the world frame
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,endeffector_position_local);

    // Already obtained via Chad's code
    // Calculate Geometric Joint Jacobian
    var j1 = kineval.generate_jacobian(endeffector_world);

    // Calculate Inverse Jacobian
    var j1_inv = kineval.matrix_jacobian_inverse(j1);
    
    var end_rotation = [[robot.joints[robot.endeffector.frame].xform[0][0],robot.joints[robot.endeffector.frame].xform[0][1],robot.joints[robot.endeffector.frame].xform[0][2]],
                        [robot.joints[robot.endeffector.frame].xform[1][0],robot.joints[robot.endeffector.frame].xform[1][1],robot.joints[robot.endeffector.frame].xform[1][2]],
                        [robot.joints[robot.endeffector.frame].xform[2][0],robot.joints[robot.endeffector.frame].xform[2][1],robot.joints[robot.endeffector.frame].xform[2][2]]];

    var rpy = extract_euler_angles(end_rotation);

    // if (last_rpy.length == 0) { // this seems to make the orientation IK way too unstable
    //     var rpy = [rpy_set[0],rpy_set[1],rpy_set[2]];
    // } else {
    //     var rpy_diff1 = Math.abs(rpy_set[0]-last_rpy[0]) + Math.abs(rpy_set[1]-last_rpy[1]) + Math.abs(rpy_set[2]-last_rpy[2]);
    //     var rpy_diff2 = Math.abs(rpy_set[3]-last_rpy[0]) + Math.abs(rpy_set[4]-last_rpy[1]) + Math.abs(rpy_set[5]-last_rpy[2]);
    //     if (rpy_diff1 > rpy_diff2) {
    //         var rpy = [rpy_set[3],rpy_set[4],rpy_set[5]];
    //     } else {
    //         var rpy = [rpy_set[3],rpy_set[4],rpy_set[5]];
    //     }
    // }

    // last_rpy = [rpy[0],rpy[1],rpy[2]];

    // remember to follow order specified by cube
    // Calculate dx_n or endpoint error
    var x_endeffector = [endeffector_world[0],endeffector_world[1],endeffector_world[2],rpy[0],rpy[1],rpy[2]]; // temp, need to add orientation
    var end_angle = endeffector_target_world.orientation;
    var end_pos = endeffector_target_world.position;
    var end_target_world_vec = [end_pos[0],end_pos[1],end_pos[2],end_angle[0],end_angle[1],end_angle[2]];
    var dx_n_vec = vector_subtract(end_target_world_vec,x_endeffector);
    
    if (kineval.params.ik_orientation_included == false) {
        dx_n_vec[3] = 0; //temp remove for orientation!!!
        dx_n_vec[4] = 0;
        dx_n_vec[5] = 0;
    }
    var dx_n = matrix_transpose(dx_n_vec);

    // Compute dq_n step direction
    var dq_n = matrix_multiply(j1_inv,dx_n); 

    // Perform and return q_n step direction
    var dq_n_vec = matrix_transpose(dq_n);
    var dq_step_vec = scalar_multiply(kineval.params.ik_steplength,dq_n_vec);
    
    for (i = 0; i < kinChain.length; i++) {
        robot.joints[kinChain[i]].control = dq_step_vec[i];
    } 
}

kineval.kinChain = function calculate_kinematic_chain(endeffector_joint) {
    kinChain = [];
    var curr_joint = endeffector_joint;
    while (robot.joints[curr_joint].parent != robot.base) {
        kinChain.unshift(curr_joint);
        var parent_link = robot.joints[curr_joint].parent;
        var parent_joint = robot.links[parent_link].parent;
        curr_joint = parent_joint;
    }
    kinChain.unshift(curr_joint);
}

kineval.generate_jacobian = function generate_jacobian(endeffector_world) {
    // var joint_J = matrix_zeroes(6,1);
    var jacobian = [];

    for (var i = 0; i < kinChain.length; i++) {
        var curr_joint = robot.joints[kinChain[i]];
        var joint_type = curr_joint.type;

        // Calculate joint axis in local frame
        var local_axis = vector_normalize(curr_joint.axis);
        local_axis.push(1); // to homogenize vector
        local_axis_T = matrix_transpose(local_axis);
        // Calculate joint axis in world frame
        var world_axis = matrix_multiply(curr_joint.xform,local_axis_T);
        world_axis_vec = matrix_transpose(world_axis);
        world_axis_vec.pop(); // to revert to nonhomogeneous vector

        // Calculate O_n - O_i-1 where O_n is the end effector, O_i-1 is current joint
        // init_z_T = matrix_transpose(init_z);
        var world_origin = [[0],[0],[0],[1]];
        var joint_world_origin = matrix_multiply(curr_joint.xform,world_origin); // test this!
        var joint_world_origin_vec = matrix_transpose(joint_world_origin); // for vector operations
        joint_world_origin_vec.pop(); // to revert to nonhomogeneous vector

        // console.log(joint_world_origin_vec)

        var endeffector_world_copy = matrix_copy(endeffector_world);
        var endeffector_world_vec = matrix_transpose(endeffector_world_copy);
        endeffector_world_vec.pop();

        var subtract_O_n = vector_subtract(endeffector_world_vec,joint_world_origin_vec);
        var cross_O_n = vector_cross(world_axis_vec,subtract_O_n);
        if (joint_type === 'prismatic') {
            var joint_J = [[world_axis_vec[0]],
                           [world_axis_vec[1]],
                           [world_axis_vec[2]],
                           [0],
                           [0],
                           [0]]; 

        } else { // assume joint_type === 'rotational'...how does it handle joint limits?
            var joint_J = [[cross_O_n[0]],
                           [cross_O_n[1]],
                           [cross_O_n[2]],
                           [world_axis_vec[0]],
                           [world_axis_vec[1]],
                           [world_axis_vec[2]]]; 
        }

        jacobian = matrix_append_vector(jacobian,joint_J);
    }

    return jacobian;
}

kineval.matrix_jacobian_inverse = function matrix_jacobian_inverse(m1) {
    // Depending on param, either pseudoinverse or transpose
    if (kineval.params.ik_pseudoinverse === true) {
        var inv_mat = matrix_pseudoinverse(m1);
    } else {
        var inv_mat = matrix_transpose(m1);
    }
    return inv_mat;
}