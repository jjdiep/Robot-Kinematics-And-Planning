// PI ROBOT
// BY Justin Diep
// Note: I had to modify the link sizes from the original ROS file in an attempt to make the robot look realistic and as a result, the joints are offset visually from the link ends.
//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object();

robot.name = "pi_robot";

robot.origin = {xyz: [0,0,0.0425], rpy:[0,0,0]};  // held a bit over the ground plane

robot.base = "base_link";

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////
robot.links = {
    "base_link": {},  
    "base_laser": {}, 
    "cpu_link": {}, 
    "upper_base_link": {}, 
    "torso_link": {}, 
    "head_pan_link": {}, 
    "head_tilt_link": {}, 
    "neck_link": {}, 
    "head_link": {}, 
    "antenna_link": {}, 
    "left_shoulder_link": {}, 
    "right_shoulder_link": {}, 
    "left_shoulder_forward_link": {}, 
    "right_shoulder_forward_link": {}, 
    "left_shoulder_up_link": {}, 
    "right_shoulder_up_link": {}, 
    "left_upper_arm_link": {}, 
    "right_upper_arm_link": {}, 
    "left_elbow_link": {}, 
    "right_elbow_link": {}, 
    "left_lower_arm_link": {}, 
    "right_lower_arm_link": {}, 
    "left_wrist_link": {}, 
    "right_wrist_link": {},
    "left_hand_link": {}, 
    "right_hand_link": {},  
};
  
// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "left_hand_joint";
robot.endeffector.position = [[0],[0],[0.9],[1]]

// * * * Joint Definitions * * * -->

robot.joints = {};

robot.joints.cpu_joint = {parent: "base_link", child:"cpu_link"};
robot.joints.cpu_joint.origin = {xyz: [0.025,0,0.085], rpy: [0,0,0]};
robot.joints.cpu_joint.axis = [0.0,0,1]; 

    // <joint name="cpu_joint" type="fixed">
    //     <parent link="base_link"/>
    //     <child link="cpu_link"/>
    //     <origin xyz="0.025 0 0.085" rpy="0 0 0"/>
    // </joint>

robot.joints.base_laser_joint = {parent: "base_link", child:"base_laser"};
robot.joints.base_laser_joint.origin = {xyz: [0,0,0.07], rpy: [0,0,0]};
robot.joints.base_laser_joint.axis = [0.0,1,0];

    // <joint name="base_laser_joint" type="fixed">
    //     <parent link="base_link"/>
    //     <child link="base_laser"/>
    //     <origin xyz="0.18 0 0.07" rpy="0 0 0"/>
    // </joint>

robot.joints.upper_base_joint = {parent: "cpu_link", child:"upper_base_link"};
robot.joints.upper_base_joint.origin = {xyz: [0,0,0.07], rpy: [0,0,0]};
robot.joints.upper_base_joint.axis = [0.0,1,0];

    // <joint name="upper_base_joint" type="fixed">
    //     <parent link="cpu_link"/>
    //     <child link="upper_base_link"/>
    //     <origin xyz="0 0 0.07" rpy="0 0 0"/>
    // </joint>

robot.joints.torso_joint = {parent: "upper_base_link", child:"torso_link"};
robot.joints.torso_joint.origin = {xyz: [0,0,0.10], rpy: [0,0,0]};
robot.joints.torso_joint.axis = [0.0,0,1];

    // <joint name="torso_joint" type="revolute">
    //     <parent link="upper_base_link"/>
    //     <child link="torso_link"/>
    //     <origin xyz="0 0 0.10" rpy="0 0 0"/>
    //     <axis xyz="0 0 1"/>
    //     <limit lower="-3.1416" upper="3.1416" effort="10" velocity="3"/>
    // </joint>

robot.joints.head_pan_servo = {parent: "torso_link", child:"head_pan_link"};
robot.joints.head_pan_servo.origin = {xyz: [0,0,0.225], rpy: [0,0,0]};
robot.joints.head_pan_servo.axis = [0.0,1,0];

    // <joint name="head_pan_servo" type="fixed">
    //     <parent link="torso_link"/>
    //     <child link="head_pan_link"/>
    //     <origin xyz="0 0 0.225" rpy="0 0 0"/>
    // </joint>

robot.joints.head_pan_joint = {parent: "head_pan_link", child:"head_tilt_link"};
robot.joints.head_pan_joint.origin = {xyz: [0,0,0.045], rpy: [0,0,0]};
robot.joints.head_pan_joint.axis = [0.0,0,1];

    // <joint name="head_pan_joint" type="revolute">
    //     <parent link="head_pan_link"/>
    //     <child link="head_tilt_link"/>
    //     <origin xyz="0 0 0.045" rpy="0 0 0"/>
    //     <axis xyz="0 0 1"/>
    //     <limit lower="-3.1416" upper="3.1416" effort="10" velocity="3"/>
    // </joint>

robot.joints.head_tilt_joint = {parent: "head_tilt_link", child:"neck_link"};
robot.joints.head_tilt_joint.origin = {xyz: [0,0,0.04], rpy: [0,0,0]};
robot.joints.head_tilt_joint.axis = [0.0,1,0];

    // <joint name="head_tilt_joint" type="revolute">
    //     <parent link="head_tilt_link"/>
    //     <child link="neck_link"/>
    //     <origin xyz="0 0 0.04" rpy="0 0 0"/>
    //     <axis xyz="0 1 0"/>
    //     <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    // </joint>

robot.joints.neck_joint = {parent: "neck_link", child:"head_link"};
robot.joints.neck_joint.origin = {xyz: [0.05,0,0.015], rpy: [0,0,0]};
robot.joints.neck_joint.axis = [0.0,0,1];

    // <joint name="neck_joint" type="fixed">
    //     <parent link="neck_link"/>
    //     <child link="head_link"/>
    //     <origin xyz="0.05 0 0.015" rpy="0 0 0"/>
    // </joint>
// <!--    
//     <joint name="eyes_joint" type="fixed">
//         <parent link="head_link" />
//         <child link="eyes_link" />
//         <origin xyz="0.02 0 0.03" rpy="0 0 0" />
//     </joint>
// -->

robot.joints.antenna_joint = {parent: "head_link", child:"antenna_link"};
robot.joints.antenna_joint.origin = {xyz: [0.0,-0.025,0.065], rpy: [0,0,0]};
robot.joints.antenna_joint.axis = [0.0,0,1];

    // <joint name="antenna_joint" type="fixed">
    //     <parent link="head_link"/>
    //     <child link="antenna_link"/>
    //     <origin xyz="0.0 -0.025 0.065" rpy="0 0 0"/>
    // </joint>

robot.joints.left_shoulder_joint = {parent: "torso_link", child:"left_shoulder_link"};
robot.joints.left_shoulder_joint.origin = {xyz: [0.0,0.055,0.165], rpy: [0,0,0]};
robot.joints.left_shoulder_joint.axis = [0.0,1,0];

    // <joint name="left_shoulder_joint" type="fixed">
    //     <parent link="torso_link"/>
    //     <child link="left_shoulder_link"/>
    //     <origin xyz="0 0.055 0.165" rpy="0 0 0"/>
    // </joint>
robot.joints.right_shoulder_joint = {parent: "torso_link", child:"right_shoulder_link"};
robot.joints.right_shoulder_joint.origin = {xyz: [0.0,-0.055,0.165], rpy: [0,0,0]};
robot.joints.right_shoulder_joint.axis = [0.0,-1,0];

    // <joint name="right_shoulder_joint" type="fixed">
    //     <parent link="torso_link"/>
    //     <child link="right_shoulder_link"/>
    //     <origin xyz="0 -0.055 0.165" rpy="0 0 0"/>
    // </joint>

robot.joints.left_shoulder_forward_joint = {parent: "left_shoulder_link", child:"left_shoulder_forward_link"};
robot.joints.left_shoulder_forward_joint.origin = {xyz: [0.0,0.025,0], rpy: [0,0,0]};
robot.joints.left_shoulder_forward_joint.axis = [0.0,0,1];

    // <joint name="left_shoulder_forward_joint" type="revolute">
    //     <parent link="left_shoulder_link"/>
    //     <child link="left_shoulder_forward_link"/>
    //     <origin xyz="0 0.025 0" rpy="0 0 0"/>
    //     <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    //     <axis xyz="0 0 1"/>
    // </joint>

robot.joints.right_shoulder_forward_joint = {parent: "right_shoulder_link", child:"right_shoulder_forward_link"};
robot.joints.right_shoulder_forward_joint.origin = {xyz: [0.0,-0.025,0], rpy: [0,0,0]};
robot.joints.right_shoulder_forward_joint.axis = [0.0,0,1];

    // <joint name="right_shoulder_forward_joint" type="revolute">
    //     <parent link="right_shoulder_link"/>
    //     <child link="right_shoulder_forward_link"/>
    //     <origin xyz="0 -0.025 0" rpy="0 0 0"/>
    //     <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    //     <axis xyz="0 0 1"/>
    // </joint>

robot.joints.left_shoulder_up_joint = {parent: "left_shoulder_forward_link", child:"left_shoulder_up_link"};
robot.joints.left_shoulder_up_joint.origin = {xyz: [0,0.04,-0.01], rpy: [0,-0.707,0]};
robot.joints.left_shoulder_up_joint.axis = [0.0,1,0];

    // <joint name="left_shoulder_up_joint" type="revolute">
    //     <parent link="left_shoulder_forward_link"/>
    //     <child link="left_shoulder_up_link"/>
    //     <origin xyz="0 0.04 -0.01" rpy="0 -0.707 0"/>
    //     <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    //     <axis xyz="0 1 0"/>
    // </joint>

robot.joints.right_shoulder_up_joint = {parent: "right_shoulder_forward_link", child:"right_shoulder_up_link"};
robot.joints.right_shoulder_up_joint.origin = {xyz: [0,-0.04,-0.01], rpy: [0,-0.707,0]};
robot.joints.right_shoulder_up_joint.axis = [0.0,1,0];

    // <joint name="right_shoulder_up_joint" type="revolute">
    //     <parent link="right_shoulder_forward_link"/>
    //     <child link="right_shoulder_up_link"/>
    //     <origin xyz="0 -0.04 -0.01" rpy="0 -0.707 0"/>
    //     <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    //     <axis xyz="0 1 0"/>
    // </joint>

robot.joints.left_upper_arm_joint = {parent: "left_shoulder_up_link", child:"left_upper_arm_link"};
robot.joints.left_upper_arm_joint.origin = {xyz: [0,0,-0.05], rpy: [0,0,0]};
robot.joints.left_upper_arm_joint.axis = [0.0,1,0];

    // <joint name="left_upper_arm_joint" type="fixed">
    //     <parent link="left_shoulder_up_link"/>
    //     <child link="left_upper_arm_link"/>
    //     <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    // </joint>

robot.joints.right_upper_arm_joint = {parent: "right_shoulder_up_link", child:"right_upper_arm_link"};
robot.joints.right_upper_arm_joint.origin = {xyz: [0,0,-0.05], rpy: [0,0,0]};
robot.joints.right_upper_arm_joint.axis = [0.0,1,0];

    // <joint name="right_upper_arm_joint" type="fixed">
    //     <parent link="right_shoulder_up_link"/>
    //     <child link="right_upper_arm_link"/>
    //     <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    // </joint>

robot.joints.left_elbow_joint = {parent: "left_upper_arm_link", child:"left_elbow_link"};
robot.joints.left_elbow_joint.origin = {xyz: [-0.005,0,-0.05], rpy: [0,0,0]};
robot.joints.left_elbow_joint.axis = [0.0,1,0];

    // <joint name="left_elbow_joint" type="revolute">
    //     <parent link="left_upper_arm_link"/>
    //     <child link="left_elbow_link"/>
    //     <origin xyz="-0.005 0 -0.05" rpy="0 0 0"/>
    //     <limit lower="-3.146" upper="3.146" effort="10" velocity="3"/>
    //     <axis xyz="0 0 1"/>
    // </joint>

robot.joints.right_elbow_joint = {parent: "right_upper_arm_link", child:"right_elbow_link"};
robot.joints.right_elbow_joint.origin = {xyz: [-0.005,0,-0.05], rpy: [0,0,0]};
robot.joints.right_elbow_joint.axis = [0.0,1,0];

    // <joint name="right_elbow_joint" type="revolute">
    //     <parent link="right_upper_arm_link"/>
    //     <child link="right_elbow_link"/>
    //     <origin xyz="-0.005 0 -0.05" rpy="0 0 0"/>
    //     <limit lower="-3.146" upper="3.146" effort="10" velocity="3"/>
    //     <axis xyz="0 0 1"/>
    // </joint>

robot.joints.left_lower_arm_joint = {parent: "left_elbow_link", child:"left_lower_arm_link"};
robot.joints.left_lower_arm_joint.origin = {xyz: [0,0,-0.08], rpy: [0,0,0]};
robot.joints.left_lower_arm_joint.axis = [0.0,1,0];

    // <joint name="left_lower_arm_joint" type="fixed">
    //     <parent link="left_elbow_link"/>
    //     <child link="left_lower_arm_link"/>
    //     <origin xyz="0 0 -0.08" rpy="0 0 0"/>
    // </joint>

robot.joints.right_lower_arm_joint = {parent: "right_elbow_link", child:"right_lower_arm_link"};
robot.joints.right_lower_arm_joint.origin = {xyz: [0,0,-0.08], rpy: [0,0,0]};
robot.joints.right_lower_arm_joint.axis = [0.0,1,0];

    // <joint name="right_lower_arm_joint" type="fixed">
    //     <parent link="right_elbow_link"/>
    //     <child link="right_lower_arm_link"/>
    //     <origin xyz="0 0 -0.08" rpy="0 0 0"/>
    // </joint>

robot.joints.left_wrist_joint = {parent: "left_lower_arm_link", child:"left_wrist_link"};
robot.joints.left_wrist_joint.origin = {xyz: [0,0,-0.05], rpy: [0,0,0]};
robot.joints.left_wrist_joint.axis = [1,0,0];

    // <joint name="left_wrist_joint" type="revolute">
    //     <parent link="left_lower_arm_link"/>
    //     <child link="left_wrist_link"/>
    //     <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    //     <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    //     <axis xyz="1 0 0"/>
    // </joint>
 
robot.joints.right_wrist_joint = {parent: "right_lower_arm_link", child:"right_wrist_link"};
robot.joints.right_wrist_joint.origin = {xyz: [0,0,-0.05], rpy: [0,0,0]};
robot.joints.right_wrist_joint.axis = [1,0,0];

    // <joint name="right_wrist_joint" type="revolute">
    //     <parent link="right_lower_arm_link"/>
    //     <child link="right_wrist_link"/>
    //     <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    //     <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
    //     <axis xyz="1 0 0"/>
    // </joint>

robot.joints.left_hand_joint = {parent: "left_wrist_link", child:"left_hand_link"};
robot.joints.left_hand_joint.origin = {xyz: [0,0,-0.055], rpy: [0,0,0]};
robot.joints.left_hand_joint.axis = [1,0,0];

    // <joint name="left_hand_joint" type="fixed">
    //     <parent link="left_wrist_link"/>
    //     <child link="left_hand_link"/>
    //     <origin xyz="0 0 -0.055" rpy="0 0 0"/>
    // </joint>

robot.joints.right_hand_joint = {parent: "right_wrist_link", child:"right_hand_link"};
robot.joints.right_hand_joint.origin = {xyz: [0,0,-0.055], rpy: [0,0,0]};
robot.joints.right_hand_joint.axis = [1,0,0];

    // <joint name="right_hand_joint" type="fixed">
    //     <parent link="right_wrist_link"/>
    //     <child link="right_hand_link"/>
    //     <origin xyz="0 0 -0.055" rpy="0 0 0"/>
    // </joint>
 
// </robot>

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////
robot.links_geom_imported = true;

links_geom = {};

links_geom["base_link"] = new THREE.CubeGeometry( 2*0.32, 2*0.26, 2*0.085 );
links_geom["base_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0,0,0.0425) );

links_geom["base_laser"] = new THREE.CubeGeometry(2*0.025,2*0.025, 2*0.07);
links_geom["base_laser"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["cpu_link"] = new THREE.CubeGeometry( 2*0.19, 2*0.19, 2*0.07 );
links_geom["cpu_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.035) );

links_geom["upper_base_link"] = new THREE.CubeGeometry( 2*0.085, 2*0.085, 2*.1);
links_geom["upper_base_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["torso_link"] = new THREE.CubeGeometry( 2*0.05, 2*0.05, 2*0.24 );
links_geom["torso_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["head_pan_link"] = new THREE.CubeGeometry( 2*0.05, 2*0.045, 2*0.045 );
links_geom["head_pan_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["head_tilt_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.038, 2*0.04 );
links_geom["head_tilt_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["neck_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.05, 2*0.042 );
links_geom["neck_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["head_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.07, 2*0.11 );
links_geom["head_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["antenna_link"] = new THREE.CubeGeometry( 2*0.02, 2*0.02, 2*.05 );
links_geom["antenna_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["left_shoulder_link"] = new THREE.CubeGeometry( 2*0.025, 2*0.015, 2*0.05 );
links_geom["left_shoulder_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["right_shoulder_link"] = new THREE.CubeGeometry( 2*0.025, 2*0.015, 2*0.05 );
links_geom["right_shoulder_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["left_shoulder_forward_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.05, 2*0.03 );
links_geom["left_shoulder_forward_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["right_shoulder_forward_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.05, 2*0.03 );
links_geom["right_shoulder_forward_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["left_shoulder_up_link"] = new THREE.CubeGeometry(2*0.03, 2*0.05, 2*0.03 );
links_geom["left_shoulder_up_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["right_shoulder_up_link"] = new THREE.CubeGeometry(2*0.03, 2*0.05, 2*0.03);
links_geom["right_shoulder_up_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["left_upper_arm_link"] = new THREE.CubeGeometry( 2*0.0075, 2*0.0075,2*0.05 );
links_geom["left_upper_arm_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0 ) );

links_geom["right_upper_arm_link"] = new THREE.CubeGeometry( 2*0.0075, 2*0.0075, 2*0.05 );
links_geom["right_upper_arm_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["left_elbow_link"] = new THREE.CubeGeometry( 2*0.035, 2*0.035, 2*0.05 );
links_geom["left_elbow_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["right_elbow_link"] = new THREE.CubeGeometry( 2*0.035, 2*0.035, 2*0.05 );
links_geom["right_elbow_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["left_lower_arm_link"] = new THREE.CubeGeometry( 2*0.0075, 2*0.0075, 2*.11);
links_geom["left_lower_arm_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["right_lower_arm_link"] = new THREE.CubeGeometry( 2*0.0075, 2*0.0075, 2*.11 );
links_geom["right_lower_arm_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["left_wrist_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.05, 2*0.03 );
links_geom["left_wrist_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["right_wrist_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.05, 2*0.03 );
links_geom["right_wrist_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.0) );

links_geom["left_hand_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.01, 2*0.06 );
links_geom["left_hand_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["right_hand_link"] = new THREE.CubeGeometry( 2*0.03, 2*0.01, 2*0.06 );
links_geom["right_hand_link"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );


