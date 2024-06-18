% Define the number of degrees of freedom
ndof = 7; % Example value

% Declare symbolic variables
q = sym('q', [1, ndof]);
alpha = sym('alpha', [1, ndof]);
a = sym('a', [1, ndof]);
d = sym('d', [1, ndof]);
m = sym('m', [1, ndof]);
I = sym('I', [3, 3, ndof]);
q_cell = num2cell(q);
alpha_cell = num2cell(alpha);
a_cell = num2cell(a);
d_cell = num2cell(d);
q_slice = q_cell(1:ndof);
alpha_slice = alpha_cell(1:ndof);
a_slice = a_cell(1:ndof);
d_slice = d_cell(1:ndof);
% Slice the arrays according to the number of degrees of freedom
 
% Call the function with the appropriate number of arguments
J = geometricJacobian(q_slice{:}, alpha_slice{:}, a_slice{:}, d_slice{:});
