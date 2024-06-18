function status = checkSkewSymmetric(Matrix)
%% -----------------------------------------------------------------------
% Check if the input matrix M is skew-symmetric
%
%% -----------------------------------------------------------------------
MatrixTranspose = Matrix';
status = isequal(Matrix, -MatrixTranspose);
end
