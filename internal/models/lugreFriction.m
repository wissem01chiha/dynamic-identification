function [t,F,Fss,err] = lugreFriction(Fc,Fs,v,vs,sigma1,sigma0,sigma2,...
                                          tinit,ts,tspan,z0)
%% ------------------------------------------------------------------------
% LUGREFRICTION - Compute LuGre Friction Model 
%
% Inputs:
%    Fc         - Coulomb friction coefficent 
%    Fs         - stribek coefficent 
%    v          - Joint velocity
%    vs         - kinetic velocity transistion 
%    sigma      - Model fixed paremters
%    tinit      - Intial simulation time  
%    ts         - step time simulation 
%    tspan      - final  simulation time
%     
% Returns:
%    t          - Simulation time dates vector.
%    F          - Friction Force for the given velocity 
%    Fss        - steady state friction force 
%    err        - diff between friction force and steady state force.
%
% Ref :
%   A new model for control systems with friction,Canudas de Wit et al.
%   IEEE Transactions on Automatic Control - 1995.
%
% Author: Wissem CHIHA
%% ------------------------------------------------------------------------
t = tinit:ts:tspan;
Fss = Fc * sign(v)+ (Fs - Fc) * exp( (-v/vs)^2) * sign(v) + sigma2 * v;
z = z0;
F   = nan(size(t));
err = nan(size(t));
for j = 1 : length(t)
    [F(j),z] = luGre(z,v,Fc,Fs,vs,sigma0,sigma1,sigma2,ts);
    err(j)= abs( F(j) - Fss );
end 
end 
function [F,z] = luGre(z,v,Fc,Fs,vs,sigma0,sigma1,sigma2,ts)
    gv = (Fc + (Fs - Fc) * exp(-(v/vs)^2))/sigma0;          
    z_dot = v - abs(v)* z / gv;                      
    z = z + z_dot * ts;
    F = sigma0 * z + sigma1 * z_dot + sigma2 * v;    
end