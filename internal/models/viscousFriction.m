function force = viscousFriction(V, Fc, Fs)
%% -----------------------------------------------------------------------
% VISCOUSFRICTION(V, FC, FS) Compute the Coulomb and viscous 
% friction model.
%
%% -----------------------------------------------------------------------
force = Fc .* sign(V) + Fs .* V;
end

