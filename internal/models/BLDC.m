function [Td, I] = BLDC(Q_t, dQ_dt, d2Q_d2t,varargin)
%% -----------------------------------------------------------------------
% BLDC 
%   Brushless Direct Current Motor Model Function.
%
% Inputs:
%   samplingRate - Sampling frequancy of the input time-vetors.
%   dQ_dt        - Motor rotaor velocity.
%   Q_t          - Motor roator velocity.
%
% Options:
%   Jm           - Robot Inertia factor.
%   kf             Motor damping coefficient.
%   Kt           - Motor current coefficient.
%   Tck          - Motor cogging torque coefficents.
%   Ta, Tb       - Motor mechanical disturbance cofficents.
%
% Returns:
%   Ia         - Armature Current vector.
%   Td         - Motor devlopped Torque vector.
%
% Ref:
%   Practical Modeling and Comprehensive System Identification of a BLDC 
%   Motor - C.iang, X.Wang, Y.Ma, B.Xu - 2015.
%
% Author : Wissem CHIHA
%% -----------------------------------------------------------------------
p   = inputParser;

default_Jm     = 0.000558;
default_Kt     = 0.11;
default_Kf     = 0.14;
default_Tck    = [0.02, 0.02, 0.02, 0.02, 0.02];
default_Ta     = 0.22;
default_Tb     = 0.21;

addOptional(p,'inertia', default_Jm);
addOptional(p,'torqueConstant',default_Kt);
addOptional(p,'damping',default_Kf);
addOptional(p,'Tck',default_Tck);
addOptional(p,'Ta',default_Ta);
addOptional(p,'Tb',default_Tb);

parse(p, varargin{:});
J   = p.Results.inertia;
Kt  = p.Results.torqueConstant;
Kf  = p.Results.damping;
Tck = p.Results.Tck;
Ta  = p.Results.Ta;
Tb  = p.Results.Tb;
  
Td  = J .* d2Q_d2t + Kf .* dQ_dt - Ta.*sin(Q_t)- Tb.*cos(Q_t);
for j=1:length(Tck)
    Td = Td + Tck(j).*cos( j .* Q_t);
end
I = Td./Kt;
end