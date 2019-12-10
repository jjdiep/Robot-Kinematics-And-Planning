//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


    // STENCIL: reference matrix code has the following functions:
    
    //   matrix_multiply, added additional code to work for row vector case
function matrix_multiply(m1, m2) {

    var i,j,k;
    var m1_col_len;
    var f_m1_row, f_m2_row;
    var mat = [];
    if (m1[0].length == null) {
        f_m1_row = true; 
        m1_col_len = m1.length;
        m1_row_len = 1;
    } else {
        m1_col_len = m1[0].length;
        m1_row_len = m1.length;
    }
    if (m2[0].length == null) {
        f_m2_row = true; 
        m2_col_len = m2.length;
        m2_row_len = 1;
    } else {
        m2_col_len = m2[0].length;
        m2_row_len = m2.length;
    }

    if (m1_col_len == m2_row_len) {
        var mat_m = m1_row_len;
        var mat_n = m2_col_len;
        mat = matrix_zeroes(mat_m, mat_n);
        for (i=0;i<m1_row_len;i++) { // for each row of m1
            // mat[i] = [];
            for (j=0;j<m2_col_len;j++) { // for each column of m2
                for (k=0;k<m1_col_len;k++) { // for each column of m1
                    if (f_m1_row) {
                        var m1vec = m1[k];
                    } else {
                        var m1vec = m1[i][k];
                    }
                    if (f_m2_row) {
                        var m2vec = m2[k];
                    } else {
                        var m2vec = m2[k][j];
                    }
                    mat[i][j] += m1vec * m2vec;
                }
            }
        }
    
        return mat;
    } else {
        var f_fail_multiply = true;
        return null; // not dimensionally correct
    }
}

// helper function for matrix initialization with zeroes
function matrix_zeroes(m, n) {
    var mat = [];
    var i_init, j_init;
    for (i_init=0;i_init<m;i_init++) {
        mat[i_init] = [];
        for (j_init=0;j_init<n;j_init++) {
            mat[i_init][j_init] = 0;
        }
    }
    return mat;
}
    //   matrix_transpose
function matrix_transpose(m1) {
    var m1_col_len, m1_row_len;
    var mat = [];

    if (m1[0].length == null) {
        var f_m1_row = true; 
        m1_col_len = m1.length;
        m1_row_len = 1;
    } else {
        m1_col_len = m1[0].length;
        m1_row_len = m1.length;
    }
    if (m1_col_len === 1) { // for proper row vector transpose
        var mat = [];
        for (var i_init=0;i_init<m1_row_len;i_init++) {
            mat.push(m1[i_init][0]);
        }

    } else {
        var mat = matrix_zeroes(m1_col_len,m1_row_len);
        for (var i_init=0;i_init<m1_row_len;i_init++) {
            for (var j_init=0;j_init<m1_col_len;j_init++) {
                if (f_m1_row) {
                    mat[j_init][i_init] = m1[j_init];
                } else {
                    mat[j_init][i_init] = m1[i_init][j_init];
                }
            }
        }
    }
    return mat;
}

    //   matrix_pseudoinverse
function matrix_pseudoinverse(m1) {
    
    var mat = [];
    var f_m1_row, f_m2_row;
    var m1_col_len, m1_row_len;
    var m1_T = matrix_transpose(m1);

    if (m1[0].length == null) {
        f_m1_row = true; 
        m1_col_len = m1.length;
        m1_row_len = 1;
    } else {
        m1_col_len = m1[0].length;
        m1_row_len = m1.length;
    }
    if (m1_col_len == m1_row_len) { // square
        mat = numeric.inv(m1);
    } else if (m1_col_len > m1_row_len) { // broad
        mat = matrix_multiply( m1_T, numeric.inv( matrix_multiply(m1, m1_T) ) );
    } else { // tall 
        mat = matrix_multiply( numeric.inv( matrix_multiply(m1_T, m1) ), m1_T );
    }

    return mat; 
}

    //   matrix_invert_affine
function matrix_invert_affine(m1) {
    var m1_r, m1_t, m1_rt, m1_tt;
    var m1_affine_inv;

    m1_r = [[m1[0][0],m1[0][1],m1[0][2],0],[m1[1][0],m1[1][1],m1[1][2],0],[m1[2][0],m1[2][1],m1[2][2],0],[0,0,0,1]];
    m1_t = [[1,0,0,m1[0][3]],[0,1,0,m1[1][3]],[0,0,1,m1[2][3]],[0,0,0,1]];
    m1_rt = matrix_transpose(m1_r);
    m1_tt = [[1,0,0,-m1[0][3]],[0,1,0,-m1[1][3]],[0,0,1,-m1[2][3]],[0,0,0,1]];
    m1_affine_inv = matrix_multiply(m1_rt,m1_tt);
    return m1_affine_inv;
}

    //   vector_normalize
function vector_normalize(v1) {
    var vd, vn;
    var vadd = 0;
    var vn = []
    for (var iind = 0; iind < v1.length; iind++) {
        vd = v1[iind]*v1[iind];
        vadd = vd + vadd;
    }
    var vmag = Math.sqrt(vadd);
    for (var jind = 0; jind < v1.length; jind++) {
        vn.push(v1[jind]/vmag);
    }
    return vn;
}

    //   vector_cross
function vector_cross(v1,v2) {
    var vc;
    var cx, cy, cz;
    cx = v1[1]*v2[2] - v1[2]*v2[1];
    cy = v1[2]*v2[0] - v1[0]*v2[2];
    cz = v1[0]*v2[1] - v1[1]*v2[0];
    vc = [cx,cy,cz];
    return vc;
}

    //   generate_identity
function generate_identity(n) {
    var mat = [];
    var i_init, j_init;
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
    return mat;
}

    //   generate_translation_matrix
function generate_translation_matrix(x_trans,y_trans,z_trans) {
    var mat = generate_identity(4);
    mat[0][3] = x_trans;
    mat[1][3] = y_trans;
    mat[2][3] = z_trans;
    return mat;
}

    //   generate_rotation_matrix_X
function generate_rotation_matrix_X(theta) {
    var mat = generate_identity(4);
    mat[1][1] = Math.cos(theta);
    mat[1][2] = -Math.sin(theta);
    mat[2][1] = Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}

    //   generate_rotation_matrix_Y
function generate_rotation_matrix_Y(theta) {
    var mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][2] = Math.sin(theta);
    mat[2][0] = -Math.sin(theta);
    mat[2][2] = Math.cos(theta);
    return mat;
}

    //   generate_rotation_matrix_Z
function generate_rotation_matrix_Z(theta) {
    var mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][1] = -Math.sin(theta);
    mat[1][0] = Math.sin(theta);
    mat[1][1] = Math.cos(theta);
    return mat;
}

function vector_subtract(v1,v2) {
    var v_diff = [];
    // Check if vector is same length!
    if (v1.length === v2.length) {
        for (var i = 0; i < v1.length; i++) {
            var diff = v1[i] - v2[i];
            v_diff.push(diff);
        }  
    } else {
        
        return null;
    }

    return v_diff;
}

function vector_add(v1,v2) {
    var v_sum = [];
    // Check if vector is same length!
    if (v1.length === v2.length) {
        for (var i = 0; i < v1.length; i++) {
            var sum = v1[i] - v2[i];
            v_sum.push(sum);
        }  
    } else {

        return null;
    }

    return v_sum;
}

function matrix_append_vector(m1,v1) {
    // var mat = matrix_zeroes(v1.length,m1[0].length+1);
    if (m1.length == 0) {
        m1 = v1;
    } else {
        if (m1.length == v1.length) {
            for (var i = 0; i < v1.length; i++) {
                m1[i].push(v1[i][0]);
            }
        } else {
            return null;
        }
    }
    return m1; 
}

function scalar_multiply(s1,v1) {
    var vec = [];
    for (var i = 0; i < v1.length; i++) {
        vec.push(s1*v1[i]);
    }
    return vec;
}

function extract_euler_angles(m1) {
    // My implementation of "Computing Euler angles from a rotation matrix" by Gregory G. Slabaugh
    var theta_1, theta_2, psi_1, psi_2, phi_1, phi_2;
    // Version 1
    if (Math.abs(m1[2][0]) !== 1) {
        theta_1 = -Math.asin(m1[2][0]);
        theta_2 = Math.PI - theta_1;
        psi_1 = Math.atan2(m1[2][1]/Math.cos(theta_1),m1[2][2]/Math.cos(theta_1));
        psi_2 = Math.atan2(m1[2][1]/Math.cos(theta_2),m1[2][2]/Math.cos(theta_2));
        phi_1 = Math.atan2(m1[1][0]/Math.cos(theta_1),m1[0][0]/Math.cos(theta_1));
        phi_2 = Math.atan2(m1[1][0]/Math.cos(theta_2),m1[0][0]/Math.cos(theta_2));

    } else {    // Gimbal Lock case
        phi_1 = 0;
        if (m1[2][0] == -1) {
            theta_1 = Math.PI/2;
            psi_1 = phi_1 + Math.atan2(m1[0][1],m1[0][2]);
        } else {
            theta_1 = -Math.PI/2;
            psi_1 = -phi_1 + Math.atan2(-m1[0][1],-m1[0][2]);
        }
    }
    var rpy = [psi_1, theta_1, phi_1];

    // Version 2
    // theta_1 = Math.atan2(m1[1][2],m1[2][2]);
    // c_2 = Math.sqrt(m1[0][0]*m1[0][0] + m1[0][1]*m1[0][1]);
    // psi_1 = Math.atan2(-m1[0][2],c_2); 
    // s_1 = Math.sin(theta_1);
    // c_1 = Math.cos(theta_1);
    // phi_1 = Math.atan2(s_1*m1[2][0]-c_1*m1[1][0],c_1*m1[1][1]-s_1*m1[2][1]);
    // var rpy = [theta_1, psi_1, phi_1]; 
    return rpy;
}