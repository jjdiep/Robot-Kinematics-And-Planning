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
        for (var i=0;i<m1_row_len;i++) { // for each row of m1
            // mat[i] = [];
            for (var j=0;j<m2_col_len;j++) { // for each column of m2
                for (var k=0;k<m1_col_len;k++) { // for each column of m1
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

function matrix_vector_multiply(m1, v1) {
    var mat = matrix_zeroes(m1.length,1);
    var test = m1[0].length;
    for (var i=0;i<m1.length;i++) { // for each row of m1
        for (var j=0;j<m1[0].length;j++) { // for each col of m1
            mat[i][0] += m1[i][j] * v1[j][0];
        }
    }
    return mat; 
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
        mat = matrix_multiply( m1_T, matrix_inverse( matrix_multiply(m1, m1_T) ) );
    } else { // tall 
        mat = matrix_multiply( matrix_inverse( matrix_multiply(m1_T, m1) ), m1_T );
    }

    return mat; 
}

// Added to integrate advanced extension LU Decomp and Inverse
function matrix_inverse(m1) {
    var mat = [];
    if (kineval.params.LU_on) {
        mat = LU_inv(m1);
    } else {
        mat = numeric.inv(m1);
    }
    return mat
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

function vector_magnitude(v1) {
    var v_mag = 0;
    for (var i = 0; i < v1.length; i++) {
        v_mag = v1[i] * v1[i] + v_mag;
    }
    return v_mag;
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

function calc_L2_norm(a1,a2) { // assumes same length a1 and a2
    var adiff = [];
    var asum = 0;
    for (var iind = 0; iind < a1.length; iind++) {
        adiff.push((a1[iind] - a2[iind]) * (a1[iind] - a2[iind]));
        asum += adiff[iind];
    }
    var adist = Math.sqrt(asum);
    return adist;
}

function LU_decomp(A) {
    if (A.length !== A[0].length) {
        console.log("LU decomposition failed: Not square matrix");
        return null;
    } else {
        var n = A.length;
        var L = generate_identity(n);
        var P = generate_identity(n);
        var U = matrix_copy(A);
    }

    for (var i = 0; i < n; i++) {
        if (U[i][i] === 0) {
            var U_s = getColSlice(U,i,U.length,0); // test this portion!
            var U_s_abs = vector_absolute(U_s)  //
            var maximum = arr => Math.max(...U_s_abs);  //
            for (var k = 0; k < n; k++) {
                if (maximum === Math.abs(U[k][i])) {
                    temp = getColSlice(P,0,P.length,0);
                    tmp_P_k = matrix_copy(P[k]); // check this for pointer assignment issues
                    for (var iind = 0; iind < P[0].length; iind++) { // P(1,:) 
                        P[0][iind] = tmp_P_k;
                        P[k][iind] = temp;
                    }
                }        
            }
        }
        
        if (U[i][i] !== 1) {
            var tmp = generate_identity(n);
            tmp[i][i] = U[i][i];
            L = matrix_multiply(L,tmp);
            // for (var jind = 0; jind < U[0].length; jind++) {
            //     U[i][jind] = U[i][jind]/U[i][i];
            // }
            U[i] = row_division(U[i],U[i][i]);
        }
        
        if (i !== n) {
            for (var j = i+1; j < U.length; j++) {
                var temp_mat = generate_identity(n);
                temp_mat[j][i] = U[j][i];
                L = matrix_multiply(L,temp_mat);
                var U_temp_vec = [];
                for (kind = 0; kind < U[0].length; kind++) { 
                    U_temp_vec.push(U[j][kind] - U[j][i] * U[i][kind]);
                }
                U[j] = U_temp_vec;
            }
        }
    }
    
    P = matrix_transpose(P); // double check this
    return [P,L,U];
}

function row_division(v1,s1) {
    var vec = []
    for (var jind = 0; jind < v1.length; jind++) {
        vec.push(v1[jind]/s1);
    }
    return vec
}
function vector_absolute(v1) {
    var vec = [];
    for (var i = 0; i < v1.length; i++) {
        vec.push(Math.abs(v1[i]));
    }
    return vec;
}

function getColSlice(matrix,s_ind,e_ind,col){ // note: returns as vector, not col vec
   var col_vec = [];
   for(var i=s_ind; i < e_ind; i++){
      col_vec.push(matrix[i][col]);
   }
   return col_vec;
}

// advanced extension LU Decomposition
function LU_linear_solver(LU,b) {
    var d = [];
    var L = LU[1]; var U = LU[2];
    var prod_Ld = 0; var prod_Ux = 0;
    var n = L.length;

    // forward-substitution
    d.push(b[0]/L[0][0]);
    for (var iind = 1; iind < L.length; iind++) {
        prod_Ld = 0;
        for (var i = 0; i < iind; i++) {
            prod_Ld += L[iind][i]*d[i];
        }
        var d_i = (b[iind] - prod_Ld)/L[iind][iind];
        d.push(d_i)
    }
    
    // back-substitution
    var x = [];
    for (var xind = 0; xind < n; xind++) { // init x vector
        x.push(0);
    }
    x[L.length-1] = d[L.length-1];
    for (var jind = L.length-2; jind >= 0; jind--) {
        prod_Ux = 0;
        for (var j = jind+1; j < L.length; j++) {
            prod_Ux += U[jind][j]*x[j];
        }
        var x_i = d[jind] - prod_Ux;
        x[jind] = x_i;
    }
    return x;
}

function LU_inv(A) {
    var b_inv = [];
    var n = A[0].length;
    var b = generate_identity(n);
    var LU = LU_decomp(A);
    for (var i = 0; i < n; i++) {
        var b_col_vec = getColSlice(b,0,n,i);
        var x_i_vec = LU_linear_solver(LU,b_col_vec);
        var x_i_T = matrix_transpose(x_i_vec)
        b_inv = matrix_append_vector(b_inv,x_i_T);
    }
    return b_inv;
}