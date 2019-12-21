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