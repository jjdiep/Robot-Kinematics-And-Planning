
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;
    // if (robot.name == "sawyer") {
    //     q_goal_config[1] = .91488; // sawyer
    // }
    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);
    collision_tree = tree_init(q_goal_config);
    // Store pointers to original trees for dfs later
    T_a_original = T_a;
    T_b_original = T_b;
    iterate = 0;
    iterate_rrt = 1;
    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)
    rrt_result = "searching";
    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
        if (rrt_alg == 0) {
            rrt_eps = .7;
            rrt_result = iterate_rrt_basic();
        } else if (rrt_alg == 1) {
            rrt_eps = .7;
            rrt_result = iterate_rrt_connect();
        } else if (rrt_alg == 2) {
            rrt_eps = .7;
            rrt_result = iterate_rrt_star();
        }
    }
    return rrt_result;
}

function iterate_rrt_basic() {
    // not needed, but implemented in 2D
    iterate++
    if (iterate >= 2000) {
        var dist_goal = calc_L2_norm(rrt_q_new,q_goal_config);
        var prob = 1-1/(dist_goal*.7); 
        console.log("Iterations over 2000", iterate);
        console.log("Goal Sampling Probability: ", 1-prob);
    }
    var q_random = random_config();
    // Goal bias to aid in completion within some reasonable iterations. Yes it's not part of the original implementation.
    var goal_sample_prob = rand_num(0,1);
    if (goal_sample_prob > prob) {
        q_random = q_goal_config;
    }
    var ext_flag = extendRRT(T_a,q_random);
    // while (ext_flag === "advanced") {
    //     ext_flag = extendRRT(T_a,q_random);
    // } 
    if (rrt_q_new != null) {
        var dist_goal = calc_L2_norm(rrt_q_new,q_goal_config);
    }
    if (ext_flag === "trapped") {
    } else if (dist_goal < rrt_eps) {
        rrt_iterate = false;
        find_path();
        return "reached";
    }
    rrt_iterate = true;
    return "iterating";
}

function iterate_rrt_connect() {
    iterate++
    var q_random = random_config();
    var ext_flag = extendRRT(T_a,q_random);

    if (ext_flag != "trapped") {
        ext_flag = connectRRT(T_b,rrt_q_new);
    } 
    if (ext_flag === "reached") {
        rrt_iterate = false;
        find_path();

        return "reached";
    }

    var alt_tree = T_a;
    T_a = T_b;
    T_b = alt_tree;

    rrt_iterate = true;
    return "iterating";
}

function iterate_rrt_star() {
    var terminate = false;
    final_path_check = false;
    iterate++
    if (iterate >= 3000) {
        iterate_rrt++
    }

    var q_random = random_config();
    var goal_sample_prob = rand_num(0,1);
    var iter_prob = (2000-iterate_rrt)/iterate_rrt;
    if (iter_prob < .7) {
        iter_prob = .7;
    }
    console.log(iter_prob);
    if (goal_sample_prob > iter_prob) {
        q_random = q_goal_config;
    }
    var ext_flag = extendRRT(T_a,q_random);
    // while (ext_flag === "advanced") {
    //     ext_flag = extendRRT(T_a,q_random);
    // } 
    var dist_goal = calc_L2_norm(rrt_q_new,q_goal_config);
    if (iterate > 3000) {
        var near_goal_index = nearest_neighbor(T_a,q_goal_config);
        var q_near_goal = T_a.vertices[near_goal_index].vertex;
        var dist_near_goal = calc_L2_norm(q_near_goal, q_goal_config)
        final_path_check = true;
        if (interpolate_q_check(q_near_goal,q_goal_config) === false && dist_near_goal < 3) {
            var node_to_goal_dist =  + q_near_goal.distance;
            tree_add_vertex(T_a,q_goal_config,node_to_goal_dist); // maybe just push x,y?
            tree_add_edge(T_a,near_goal_index,T_a.newest);
            terminate = true;
        }
    }
    if (ext_flag === "trapped" && terminate === false) {
    } else if (dist_goal < rrt_eps || terminate === true) {
        rrt_iterate = false;
        find_path();
        return "reached";
    }
    rrt_iterate = true;
    console.log("Number of Iterations: ", iterate);
    return "iterating";
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];
    tree.vertices[0].distance = 0;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q,dist) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;
    new_vertex.distance = dist;
    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    if (kineval.poseIsCollision(q) === false) {
        tree.vertices.push(new_vertex);
        tree.newest = tree.vertices.length - 1;
    }
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}

function tree_add_bad_vertex(tree,q,dist) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;
    new_vertex.distance = dist;
    // create rendering geometry for base location of vertex configuration
    add_bad_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_bad_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0x00afaf, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}

function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

function extendRRT(tree,q_target) {
    var q_near_index = nearest_neighbor(tree,q_target);
    var q_near = tree.vertices[q_near_index].vertex;
    rrt_q_new = new_config(q_target,q_near); //global?

    // if (kineval.poseIsCollision(rrt_q_new_test) !== undefined) {
    var is_collision = kineval.poseIsCollision(rrt_q_new);
    // } else {
    //     var is_collision = true;
    // }
    var path_collision = interpolate_q_check(q_near,rrt_q_new);
    var valid_joint_config = check_valid_joint_config(rrt_q_new);
    // var valid_joint_config = true;
    // is_collision = false;
    // path_collision = false;
    if (is_collision === false && path_collision === false && valid_joint_config === true) {
        if (rrt_alg !== 2) { // for RRT, RRT-Connect
            var node_dist = calc_L2_norm(rrt_q_new,q_near);
        } else { // RRT-Star
            var z_neighbor_list = Reach(tree);
            var z_min_cost_index = Near(tree,z_neighbor_list);
            var z_near = tree.vertices[z_min_cost_index].vertex;
            var node_dist = calc_L2_norm(rrt_q_new, z_near) + z_near.distance;
        }
        // var doublecheck_collision = kineval.poseIsCollision(rrt_q_new);
        // if (doublecheck_collision === false) {
            // Add new_point to tree
        console.log("New Vertex Collision? ", JSON.stringify(kineval.poseIsCollision(rrt_q_new)));
        tree_add_vertex(tree,rrt_q_new,node_dist); // maybe just push x,y?
        // rrt_q_new = rrt_q_new_test; // set global variable

        if (rrt_alg !== 2) {
            // Add edge between the points associated with these two indices
            tree_add_edge(tree, q_near_index, tree.newest);
        } else {
            tree_add_edge(tree,z_min_cost_index,tree.newest); //interpolate?
            Rewire(tree,z_neighbor_list); //interpolate
        }
        // }
        //Check if new_point = target_point
        var dist_target = calc_L2_norm(rrt_q_new,q_target);
        if(dist_target < rrt_eps) {
            return "reached";
        } else {
            return "advanced";
        }
        // }
    } else {
        // rrt_q_new = null;
        return "trapped";
    }
}

function Near(tree,z_neighbor_list) {
    var prior_z_dist = calc_L2_norm(rrt_q_new,tree.vertices[z_neighbor_list[0]].vertex);
    var prior_z_cost = tree.vertices[z_neighbor_list[0]].distance;
    var prior_z_total_cost = prior_z_dist + prior_z_cost;
    var z_min_cost_index = z_neighbor_list[0];
    for (var iind = 0; iind < z_neighbor_list.length; iind++) {
        var z_dist = calc_L2_norm(rrt_q_new,tree.vertices[z_neighbor_list[iind]].vertex);
        var z_cost = tree.vertices[z_neighbor_list[iind]].distance;
        var z_total_cost = z_dist + z_cost;
        var pathCollision = interpolate_q_check(tree.vertices[z_neighbor_list[iind]].vertex,rrt_q_new);
        if (z_total_cost < prior_z_total_cost && pathCollision === false) {
            var z_min_cost_index = z_neighbor_list[iind];
            prior_z_total_cost = z_total_cost;
        }
    }

    return z_min_cost_index;
}

function Reach(tree) {
    var neighbor_range = 1.5*rrt_eps;
    var z_neighbor_list = [];
    for (var iind = 0; iind < tree.vertices.length; iind++) {
        var q_s = tree.vertices[iind].vertex;
        var q_dist = calc_L2_norm(rrt_q_new,q_s);
        if (q_dist > 0 && q_dist < neighbor_range) { 
            z_neighbor_list.push(iind);
        }
    }
    return z_neighbor_list;
}

function Rewire(tree,z_neighbor_list) {
    for (var iind = 0; iind < z_neighbor_list.length; iind++) {
        var prior_z_path_cost = tree.vertices[z_neighbor_list[iind]].distance;
        var z_new_dist = calc_L2_norm(rrt_q_new,tree.vertices[z_neighbor_list[iind]].vertex);

        var z_new_cost = tree.vertices[tree.newest].distance;
        var z_new_path_cost = z_new_dist + z_new_cost;
        var pathCollision = interpolate_q_check(rrt_q_new,tree.vertices[z_neighbor_list[iind]].vertex);
        if (z_new_path_cost < prior_z_path_cost && pathCollision === false) {
            tree.vertices[z_neighbor_list[iind]].edges.shift(); // test!
            tree_add_edge(tree, tree.newest, z_neighbor_list[iind]);
            var new_edge = tree.vertices[z_neighbor_list[iind]].edges.pop();
 
            tree.vertices[z_neighbor_list[iind]].edges.unshift(new_edge);
            tree.vertices[z_neighbor_list[iind]].distance = z_new_path_cost;
            var rewired_z_path = [];
            var destination_node = tree.vertices[0].vertex;
            rewired_cost(tree,tree.vertices[z_neighbor_list[iind]],destination_node,rewired_z_path); 
            // bfs_fringe(tree,tree.vertices[z_neighbor_list[iind]]);
        }
    }
}

function rewired_cost(tree,curr_node,destination_node,path) {
    path_dfs(tree, curr_node,destination_node,false,path);
    path[path.length-1].distance=0;// not sure if this line is necessary or not, just in case;
    for (i = path.length-1; i >=1; i--) {
       path[i-1].distance = path[i].distance+calc_L2_norm(path[i].vertex, path[i-1].vertex);
    }
}

function bfs_fringe(tree,curr_node) {
    curr_node.distance = curr_node.edges[0].distance + calc_L2_norm(curr_node.edges[0].vertex, curr_node.vertex);
    to_visit_nodes = [];
    for (var i = 1; i < curr_node.edges.length; i++) {
        to_visit_nodes.unshift(curr_node.edges[i]);
    }
    while (to_visit_nodes.length > 0) {
        var child_nodes = to_visit_nodes.shift();
        child_nodes.distance = child_nodes.edges[0].distance + calc_L2_norm(child_nodes.edges[0].vertex, child_nodes.vertex);
        to_visit_nodes.concat(child_nodes.edges.slice(1,child_nodes.edges.length));
    }
}

function interpolate_q_check(q_near,rrt_q_new) {
    if (rrt_alg !== 2) {
        var num_interp = 5;
    } else {
        var num_interp = 2;
        if (final_path_check) {
            var num_interp = 5;
        }
    }
    for (var iind = 0; iind <= num_interp; iind++) {
        var interp_q = [];
        for (var jind = 0; jind < q_near.length; jind++) {
            var interp_state = (rrt_q_new[jind]-q_near[jind])/num_interp*(iind)+q_near[jind];
            interp_q.push(interp_state);
        }
        var is_collision = kineval.poseIsCollision(interp_q);
        if (is_collision != false) {
            return true;
        }
    }

    return false;
}

function connectRRT(tree,connect_node) {
    var counter = 0;
    var extend_ret = extendRRT(tree,connect_node);
    counter++;
    while (extend_ret === "advanced") {
        extend_ret = extendRRT(tree,connect_node);
        counter++
    }
    return extend_ret;
}

function random_config() {
    var q_rand = new Array(q_start_config.length);
    if (rrt_alg === 2) {
        var search_scale = 1.2;
        if (iterate > 4000) search_scale = 1.2;
    } else {
        var search_scale = 1.3;
        if (iterate > 4000) search_scale = 1.5;
    }
    // Get a random point from the graph
    for(var i = 0; i < 3; ++i) {
        q_rand[i] = rand_num(robot_boundary[0][i]*search_scale, robot_boundary[1][i]*search_scale);
    }
    //Set y coordinate to 0
    q_rand[1] = 0; 

    // Generate orientation of robot base (3,4,5) and the rest of the joints
    for(var i = 3; i < 6; ++i) {
        q_rand[i] = rand_num(-Math.PI, Math.PI);
    } 
    // for(var i = 6; i < q_rand.length; ++i) { // test, remove asap
    //     q_rand[i] = rand_num(-Math.PI, Math.PI);
    // } 
    // need to consider joint type and limits!
    for (var name in robot.joints) {
        var joint_object = robot.joints[name];
        var j_type = joint_object.type
        var q_indices = q_names[name];
        if (j_type === "prismatic" || j_type === "revolute") {
            q_rand[q_indices] = rand_num(joint_object.limit.lower,joint_object.limit.upper);
        } else if (j_type === "fixed") {
            q_rand[q_indices] = 0;
        } else { // continuous
            q_rand[q_indices] = rand_num(-Math.PI,Math.PI);
        }
    }

    return q_rand;
}

function check_valid_joint_config(q) {
    var joint_check_ret = true;
    for (var name in robot.joints) {
        var joint_object = robot.joints[name];
        var j_type = joint_object.type
        var q_indices = q_names[name];
        if (j_type === "prismatic" || j_type === "revolute") {
            if (q[q_indices] > joint_object.limit.upper) {
                joint_check_ret = false;
                console.log("out of bounds revolute/prismatic!")
            } else if (q[q_indices] < joint_object.limit.lower) {
                joint_check_ret = false;
                console.log("out of bounds revolute/prismatic!")
            }
        } else if (j_type === "fixed") {
            if (q[q_indices] != 0) {
                joint_check_ret = false;
                console.log("out of bounds fixed joint!")
            }
        } else { // continuous
            if (q[q_indices] > Math.PI || q[q_indices] < -Math.PI) {
                joint_check_ret = false;
                console.log("not within 2*pi!")
            }
        }
    }
    return joint_check_ret;
}

function new_config(q_rand,q_near) {
    var q_rand_norm = normalize_joint_state(q_rand)
    var dir_vec = vector_subtract(q_rand_norm,q_near); // normalize!
    var normalized = vector_normalize(dir_vec);
    // var check = vector_magnitude(normalized);
    // var n_squares = Math.floor(eps / 0.1); // change to eps?
    // var unit_dir_vec = vector_normalize(dir_vec); 
    //Add dir_vec elements to i_index, j_index
    // var q_total = [];
    var q_new = [];
    for (var iind = 0; iind < q_near.length; iind++) {
        var q_added = q_near[iind] + rrt_eps * normalized[iind];
        q_new.push(q_added);
    }
    // var norm_q_total = normalize_joint_state(q_total)
    // var normalized = vector_normalize(norm_q_total);
    // for (var iind = 0; iind < q_near.length; iind++) {
    //     var q_step = q_near[iind] + rrt_eps*normalized[iind];
    //     q_new.push(q_step);
    // }
    // if (kineval.poseIsCollision(q_new) === false) {
    //     if (interpolate_q_check(q_near,q_new) === false) { 
    return q_new;
        // }
    // }
}

function isEqualArray(a1,a2) { // maybe needed?
    for (var aind = 0; aind < a2.length; aind++) {
        if (Math.abs(a1[aind] - a2[aind]) > rrt_eps/2) {
            return false;
        }
    }
    return true;
}

function arrayCopy(a1) {
    var acopy = [];
    for (var iind = 0; iind < a1.length; iind++) {
        acopy.push(a1[iind]);
    }
    return acopy
}

function nearest_neighbor(tree,q_node) {
    var q_near_index = 0;
    var q_first = tree.vertices[q_near_index].vertex;
    var prior_q_dist = calc_L2_norm(q_first,q_node);
    for (var iind = 0; iind < tree.vertices.length; iind++) {
        var q_s = tree.vertices[iind].vertex;
        var q_dist = calc_L2_norm(q_s,q_node);
        if (q_dist < rrt_eps) {  
            var q_near = q_s;
            var q_near_index = iind;
            return q_near_index;
        } else if (q_dist < prior_q_dist) {
            var q_near_index = iind;
            var prior_q_dist = q_dist;
        }
    }
    var q_near = tree.vertices[q_near_index].vertex;
    return q_near_index;
}

function path_dfs(tree, curr_node) {   
    var in_path = []; 
    in_path.push(curr_node);

    curr_node.vertex.visited = true;
    var destination_dist = calc_L2_norm(curr_node.vertex,destinationNode);
    while(destination_dist > rrt_eps) {
        var current_edge_pt = curr_node.edges[0];
        in_path.push(current_edge_pt);
        curr_node = current_edge_pt;
        destination_dist = calc_L2_norm(curr_node.vertex,destinationNode);
    }
    return in_path;

}


// function getRndInt(min, max) {
//   return Math.floor(Math.random() * (max - min) ) + min;
// }

function rand_num(lower,upper) {
    var num = Math.random() * (upper - lower) + lower;
    return num;
}


// Loops through all angles and puts them in the range 0 to 2pi
function normalize_joint_state(q_joint) {
    var tpi = 2 * Math.PI;
    for(var i = 3; i < q_joint.length; ++i) {
        // q[i] = Math.abs(q[i] % tpi);
        q_joint[i] = Math.abs(q_joint[i] % Math.PI);
    }
    // Set y to 0.
    q_joint[1] = 0;
    // if (robot.name == "sawyer") {
    //     q[1] = .91488; // sawyer
    // }
    // Add joint limit planning!

    // No rotations allowed about any axis other than pitch
    q_joint[3] = 0;
    // q[4] = 0;
    q_joint[5] = 0;
    for (var name in robot.joints) {
        var j_object = robot.joints[name];
        var j_type = j_object.type
        var q_indices = q_names[name];
        if (j_type == "prismatic" || j_type == "revolute") {
            if (q_joint[q_indices] > j_object.limit.upper) {
                q_joint[q_indices] = j_object.limit.upper;
            } else if (q_joint[q_indices] < j_object.limit.lower) {
                q_joint[q_indices] = j_object.limit.upper;
            }
        } else if (j_type == "fixed") {
            q_joint[q_indices] = 0;
        } else { // continuous
            q_joint[q_indices] = q_joint[q_indices] % Math.PI;
        }
    }

    // var norm_q = vector_normalize(q_joint);

    return q_joint;
}

function find_path() {
    if (rrt_alg == 1) {
        // Do dfs on each tree back to it's root node from new_point,
        // which is in both trees at the last index (tree.newest)
        // var found = false;
        
        // var path1 = [];

        destinationNode = T_a_original.vertices[0].vertex;
        dfsPath1 = path_dfs(T_a_original,T_a_original.vertices[T_a_original.newest]);
        
        // destinationNode = T_a.vertices[0].vertex;
        // dfsPath1 = path_dfs(T_a,T_a.vertices[T_a.newest],path1);
        

        // var path2 = [];

        destinationNode = T_b_original.vertices[0].vertex;
        dfsPath2 = path_dfs(T_b_original,T_b_original.vertices[T_b_original.newest]);
        // destinationNode = T_b.vertices[0].vertex;
        // dfsPath2 = path_dfs(T_b,T_b.vertices[T_b.newest],path2);

        var final_path = (dfsPath1.reverse()).concat(dfsPath2);
        my_final_path_test = [];
        final_val = [];
        for (var iind = 0; iind < final_path.length; iind++) {
            my_final_path_test.push(final_path[iind].vertex);
            final_val.push(kineval.poseIsCollision(my_final_path_test[iind]));
            // console.log("Collisions when run on path? ", JSON.stringify(final_val[iind]));
        }
        kineval.motion_plan = final_path;
        // Loop through kineval.motion_plan and highlight the path in red
        for(var i = 0; i < kineval.motion_plan.length; ++i) {
            console.log("Collisions when run on motion plan?", kineval.poseIsCollision(kineval.motion_plan[i].vertex));
            kineval.motion_plan[i].geom.material.color = {r:1,g:0,b:0};
        }
    } else if (rrt_alg == 2 || rrt_alg == 0) { // RRT-star and RRT
        destinationNode = T_a.vertices[0].vertex;
        dfsPath = path_dfs(T_a_original,T_a_original.vertices[T_a_original.newest]);
        var final_path = dfsPath.reverse();
        kineval.motion_plan = final_path;
        // Loop through kineval.motion_plan and highlight the path in red
        for(var i = 0; i < kineval.motion_plan.length; ++i) {
            kineval.motion_plan[i].geom.material.color = {r:1,g:0,b:0};
        }
    }

}