
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | collision detection

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

// KE: merge collision test into FK
// KE: make FK for a configuration and independent of current robot state

kineval.robotIsCollision = function robot_iscollision() {
    // test whether geometry of current configuration of robot is in collision with planning world 

    // form configuration from base location and joint angles
    var q_robot_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    for (x in robot.joints) {
        q_names[x] = q_robot_config.length;
        q_robot_config = q_robot_config.concat(robot.joints[x].angle);
    }
    // console.log(q_names);
    // test for collision and change base color based on the result
    collision_result = kineval.poseIsCollision(q_robot_config);

    robot.collision = collision_result;
}


kineval.poseIsCollision = function robot_collision_test(q) {
    // perform collision test of robot geometry against planning world 

    // test base origin (not extents) against world boundary extents
    if ((q[0]<robot_boundary[0][0])||(q[0]>robot_boundary[1][0])||(q[2]<robot_boundary[0][2])||(q[2]>robot_boundary[1][2]))
        return robot.base;

    // traverse robot kinematics to test each body for collision
    // STENCIL: implement forward kinematics for collision detection
    //return robot_collision_forward_kinematics(q);
        // Compute transform matrix for base link from the configuration q

    var xyz = [q[0],q[1],q[2]];
    var rpy = [q[3],q[4],q[5]];

    var ms = kineval.get_transformation_matrix(xyz,rpy);

    var mat_stack = [];

    // mat_stack.push(generate_identity(4));
    mat_stack.push(ms);
    mat_stack.push(FKBaseTransform);


    // Invariant: ms is the xform matrix associated with the link
    // that we pass to traverse_link

    var base_link = robot.links[robot.base];
    return traverse_collision_forward_kinematics_link(base_link,ms,q,mat_stack);
}

// Joint is a joint object e.g. robot.joints[""]
function traverse_collision_forward_kinematics_joint(joint,mstack,q,mat_stack) {
    // STENCIL: To be implemented. Uses recursion

    //Set ms to be the xform of this joint. Need to recompute based on given q
    //index of q that has this joint's angle
    var index = q_names[joint.name];

    // Compute xform from the angle given in q[index] as well as from the
    // joint's forward kinematics

    //console.log("Joint name: ");
    //console.log(joint);

    // Generate xform matrix through multiplication with top of matrix stack
    var xyz = joint.origin.xyz;
    var rpy = joint.origin.rpy;

    mstack = kineval.get_transformation_matrix(xyz,rpy);

    // Add in rotation due to quaternion and multiply transform_mat by
    // the quaternion matrix.
    if (joint.type == 'prismatic') {
        var trans = [joint.axis[0]*q[index], joint.axis[1]*q[index], joint.axis[2]*q[index]];
        var rot = [0,0,0];
        var q_matrix = kineval.get_transformation_matrix(trans,rot);
    } else if (joint.type == 'fixed') {
        var q_matrix = kineval.get_transformation_matrix([0,0,0],[0,0,0]);
    } else {
        var q_r = quaternion_from_axisangle(joint.axis,q[index]);
        var q_r_norm = quaternion_normalize(q_r);
        // var q_matrix = quaternion_to_rotation_matrix(q_r);
        var q_matrix = quaternion_to_rotation_matrix(q_r_norm);
    }

    var final_transform = matrix_multiply(mstack, q_matrix);

    mstack = final_transform;

    // --------------------------------------------------------------

    // Set xform by multiplying with stack top
    var ms_top = mat_stack[mat_stack.length - 1];
    //console.log("Top of ms:");
    //console.log(JSON.stringify(ms));
    var composed_mat = matrix_multiply(ms_top, mstack);

    mstack = composed_mat;

    // Push xform onto stack
    mat_stack.push(mstack);

    // Pass in child link to traverse link
    var next_link = robot.links[joint.child];

    var ret_val = traverse_collision_forward_kinematics_link(next_link,mstack,q,mat_stack);
    
    mat_stack.pop();

    return ret_val;
}
function traverse_collision_forward_kinematics_link(link,mstack,q,mat_stack) {

    /* test collision FK
    console.log(link);
    */
    if (typeof link.visual !== 'undefined') {
        var local_link_xform = matrix_multiply(mstack,generate_translation_matrix(link.visual.origin.xyz[0],link.visual.origin.xyz[1],link.visual.origin.xyz[2]));
    }
    else {
        var local_link_xform = matrix_multiply(mstack,generate_identity(4));
    }

    // test collision by transforming obstacles in world to link space


    // mstack_inv1 = numeric.inv(mstack);
    mstack_inv = matrix_invert_affine(mstack);

    var i;
    var j;

    // test each obstacle against link bbox geometry by transforming obstacle into link frame and testing against axis aligned bounding box
    //for (j=0;j<robot_obstacles.length;j++) { 
    for (j in robot_obstacles) { 

        var obstacle_local = matrix_multiply(mstack_inv,robot_obstacles[j].location);

        // assume link is in collision as default
        var in_collision = true; 

        // if obstacle lies outside the link extents along any dimension, no collision is detected
        if (
            (obstacle_local[0][0]<(link.bbox.min.x-robot_obstacles[j].radius))
            ||
            (obstacle_local[0][0]>(link.bbox.max.x+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[1][0]<(link.bbox.min.y-robot_obstacles[j].radius))
            ||
            (obstacle_local[1][0]>(link.bbox.max.y+robot_obstacles[j].radius))
        )
                in_collision = false;
        if (
            (obstacle_local[2][0]<(link.bbox.min.z-robot_obstacles[j].radius)) 
            ||
            (obstacle_local[2][0]>(link.bbox.max.z+robot_obstacles[j].radius))
        )
                in_collision = false;

        // if obstacle lies within link extents along all dimensions, a collision is detected and return true
        if (in_collision)
            return link.name;
    }

    // recurse child joints for collisions, returning true if child returns collision
    if (typeof link.children !== 'undefined') { // return if there are no children
        var local_collision;
        //for (i=0;i<link.children.length;i++) {
        for (i in link.children) {
            local_collision = traverse_collision_forward_kinematics_joint(robot.joints[link.children[i]],mstack,q,mat_stack)
            if (local_collision)
                return local_collision;
        }
    }

    // return false, when no collision detected for this link and children 
    return false;
}