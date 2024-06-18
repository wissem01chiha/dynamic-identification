function wrapped_angle = wrap2deg(angle)
%% WRAP2DEG( ANGLE ) 
%  Wrap angle to the the interval [-180, 180].
    wrapped_angle = angle - 360*floor((angle + 180) / (360));
end