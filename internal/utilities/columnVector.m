function colVector = columnVector(vec)
%% --------------------------------------------------------------------
% COLUMNVECTOR( X ), with X a vector, Convert to a column vector.
%% --------------------------------------------------------------------
assert(isvector(vec), "Input must be a vector.");
colVector = vec(:);
end