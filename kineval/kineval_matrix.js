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
        return 0; // not dimensionally correct
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

}

    //   matrix_pseudoinverse
function matrix_pseudoinverse(m1) {

}

    //   matrix_invert_affine
function matrix_invert_affine(m1) {

}

    //   vector_normalize
function vector_normalize(v1) {

}

    //   vector_cross
function vector_cross(v1,v2) {

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
}

    //   generate_rotation_matrix_Z
function generate_rotation_matrix_Z(theta) {
    var mat = generate_identity(4);
    mat[0][0] = Math.cos(theta);
    mat[0][1] = -Math.sin(theta);
    mat[1][0] = Math.sin(theta);
    mat[1][1] = Math.cos(theta);
}
