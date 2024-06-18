function wrapped_angle = wrapXpi(angle, X)
%% 
% WRAPXPI( ANGLE )
% Wrap angle to the interval [-X*pi, X*pi]
%%
wrapped_angle = angle - X.*pi*floor((angle + X./2.*pi) / (X.*pi));
end
