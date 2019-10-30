
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();
    kineval.buildFKTransforms()
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

kineval.buildFKTransforms = function buildFKTransforms () {
    var init_mtx = generate_identity(4);
    // var base_link = robot.links[robot.base];
    var FKBase_mtx = kineval.traverseFKBase(init_mtx);
    var FKLink_mtx = kineval.traverseFKLink(robot.base,FKBase_mtx);
}

kineval.traverseFKBase = function traverseFKBase (mat) { 
    var init_z = matrix_transpose([0,0,1,1]);
    var init_x = matrix_transpose([1,0,0,1]);
    var t_rpy = robot.origin.rpy;
    var t_xyz = robot.origin.xyz;
    var base_transform = kineval.get_transformation_matrix(t_xyz,t_rpy);

    robot_heading = matrix_copy(init_z);
    robot_lateral = matrix_copy(init_x);

    // need to set only for ROS robots for transform to threejs coordinate frame
    if (robot.links_geom_imported === true) {

        var ros_transform = matrix_multiply(generate_rotation_matrix_X(-Math.PI/2),generate_rotation_matrix_Z(-Math.PI/2));
        init_z = matrix_transpose([1,0,0,1]);
        init_x = matrix_transpose([0,1,0,1]);
        base_transform = matrix_multiply(base_transform,ros_transform);

    }

    robot_heading = matrix_multiply(base_transform,init_z);
    robot_lateral = matrix_multiply(base_transform,init_x);
    FKBaseTransform = matrix_multiply(mat, base_transform);
    robot.origin.xform = base_transform;
    return FKBaseTransform;
}

kineval.traverseFKLink = function traverseFKLink (link,mat) {
    var link_object = robot.links[link];
    var link_children = link_object.children;

    if(link === robot.base) {
        link_object.xform = robot.origin.xform;
    }
    // Originates from a joint. xform is the same as parent's xform
    else {
        var parent_joint = link_object.parent;
        var parent_xform = robot.joints[parent_joint].xform;

        // Link shares xform matrix with parent joint
        link_object.xform = parent_xform;
    }

    var FKMtx = matrix_copy(mat) 
    for(var i = 0; i < link_children.length; ++i) {
        var FK_undone = kineval.traverseFKJoint(link_children[i],FKMtx);
        FKMtx = FK_undone;
    } 
}

kineval.traverseFKJoint = function traverseFKJoint (joint,mat) {
    var joint_object = robot.joints[joint];
    var joint_xyz = joint_object.origin.xyz;
    var joint_rpy = joint_object.origin.rpy;
    var joint_axis = joint_object.axis;
    var joint_angle = joint_object.angle;

    var trans_mtx = kineval.get_transformation_matrix (joint_xyz, joint_rpy);
    var joint_mtx_def = matrix_multiply(mat, trans_mtx);
    // checkJointLimits(joint_type,joint_limit_upper,joint_limit_lower,joint_axis,joint_angle);
    // if joint_type is prismatic then create translation matrix, else controlFKJointAngle
    var control_joint_mtx = kineval.checkJointLimits(joint_object, joint_axis, joint_angle)

    // var control_joint_mtx = kineval.controlFKJointAngle(joint_axis,joint_angle);
    var joint_mtx = matrix_multiply(joint_mtx_def,control_joint_mtx);
    joint_object.xform = joint_mtx;
    kineval.traverseFKLink(joint_object.child, joint_mtx);
    inv_trans_mtx = matrix_invert_affine(trans_mtx);
    var prior_joint_mtx = matrix_multiply(joint_mtx, inv_trans_mtx); 

    return prior_joint_mtx;
}

kineval.controlFKJointAngle = function controlFKJointAngle(joint_axis,joint_angle) {

    var joint_quaternion = quaternion_from_axisangle(joint_axis,joint_angle);
    var unit_joint_quaternion = quaternion_normalize(joint_quaternion);
    var control_transformation_matrix = quaternion_to_rotation_matrix(unit_joint_quaternion);
    return control_transformation_matrix;
}

kineval.checkJointLimits = function checkJointLimits(joint_object, joint_axis, joint_angle) {
    var joint_type = joint_object.type;
    // var joint_limit_upper = joint_object.limit.upper;
    // var joint_limit_lower = joint_object.limit.lower;

    if (joint_type === "prismatic") {
        if (joint_angle > joint_object.limit.upper) { // apply saturation to joint limits in translation
            var control_angle = joint_object.limit.upper;
        } else if (joint_angle < joint_object.limit.lower) {
            var control_angle = joint_object.limit.lower;
        } else {
            var control_angle = joint_angle; // note: actually is translation
        } 
        var xyz_control = [joint_axis[0] * control_angle, joint_axis[1] * control_angle, joint_axis[2] * control_angle];
        var rpy_control = [0,0,0];
        var control_joint_mtx = kineval.get_transformation_matrix(xyz_control, rpy_control);
    }
    else if (joint_type === "revolute") { // apply saturation to joint limit in rotation
        if (joint_angle > joint_object.limit.upper) { // apply saturation to joint limits in rotation
            var control_angle = joint_object.limit.upper;
        } else if (joint_angle < joint_object.limit.lower) {
            var control_angle = joint_object.limit.lower;
        } else {
            var control_angle = joint_angle;
        } 
        var control_joint_mtx = kineval.controlFKJointAngle(joint_axis,control_angle);
    }
    else if (joint_type === "fixed") { // no control allowed of fixed joint
        var control_angle = 0;
        var control_joint_mtx = kineval.controlFKJointAngle(joint_axis,control_angle);
    }
    else { // assumed to be continuous by default, no joint limits
        var control_angle = joint_angle;
        var control_joint_mtx = kineval.controlFKJointAngle(joint_axis,control_angle);
    }

    return control_joint_mtx;
}
kineval.get_transformation_matrix = function get_transformation_matrix (xyz,rpy) {
    var translation_matrix = generate_translation_matrix(xyz[0],xyz[1],xyz[2]);

    var roll_transform = generate_rotation_matrix_X(rpy[0]);
    var pitch_transform = generate_rotation_matrix_Y(rpy[1]);
    var yaw_transform = generate_rotation_matrix_Z(rpy[2]);

    var rotation_matrix = matrix_multiply(matrix_multiply(yaw_transform,pitch_transform),roll_transform);

    var transformation_matrix = matrix_multiply(translation_matrix,rotation_matrix);

    return transformation_matrix;
}