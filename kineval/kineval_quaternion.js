//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

//   quaternion_from_axisangle
function quaternion_from_axisangle(axis,theta) {
    // returns quaternion array that is rotated by theta angle around specified axis
    var norm_axis = vector_normalize(axis);
    var mat = [];
    mat[0] = Math.cos(theta/2); 
    mat[1] = norm_axis[0] * Math.sin(theta/2);
    mat[2] = norm_axis[1] * Math.sin(theta/2);
    mat[3] = norm_axis[2] * Math.sin(theta/2);
    return mat;
}

//   quaternion_normalize
function quaternion_normalize(q1) {
    var q_normalized = [];
    var q_norm = Math.sqrt(q1[0]*q1[0] + q1[1]*q1[1] + q1[2]*q1[2] + q1[3]*q1[3]);
    q_normalized[0] = q1[0]/q_norm;
    q_normalized[1] = q1[1]/q_norm;
    q_normalized[2] = q1[2]/q_norm;
    q_normalized[3] = q1[3]/q_norm;

    return q_normalized;
}

//   quaternion_to_rotation_matrix
function quaternion_to_rotation_matrix(q1) { // homogeneous conversion
    var mat = [];
    var i_init, j_init;
    var n = 4;
    for (i_init=0;i_init<n;i_init++) {
        mat[i_init] = [];
        for (j_init=0;j_init<n;j_init++) {
            if (i_init == j_init) {
                mat[i_init][j_init] = 1;
            } else {
                mat[i_init][j_init] = 0;
            }
        }
    }
    mat[0][0] = q1[0]*q1[0] + q1[1]*q1[1] - q1[2]*q1[2] - q1[3]*q1[3];
    mat[0][1] = 2 * (q1[1]*q1[2] - q1[0]*q1[3]);
    mat[0][2] = 2 * (q1[0]*q1[2] + q1[1]*q1[3]);
    mat[1][0] = 2 * (q1[1]*q1[2] + q1[0]*q1[3]);
    mat[1][1] = q1[0]*q1[0] - q1[1]*q1[1] + q1[2]*q1[2] - q1[3]*q1[3];
    mat[1][2] = 2 * (q1[2]*q1[3] - q1[0]*q1[1]);
    mat[2][0] = 2 * (q1[1]*q1[3] - q1[0]*q1[2]);
    mat[2][1] = 2 * (q1[0]*q1[1] + q1[2]*q1[3]);
    mat[2][2] = q1[0]*q1[0] - q1[1]*q1[1] - q1[2]*q1[2] + q1[3]*q1[3];

    return mat;
}

//   quaternion_multiply
function quaternion_multiply(q1,q2) {
    var q_product = [];
    q_product[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_product[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_product[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_product[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

    return q_product;
}