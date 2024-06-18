function upt = discreteTimeDerivative(ut, ut_1, dt)
%% ------------------------------------------------------------------------
% DISCRETETIMEDERIVATIVE  Compute the discrete-time derivative.
% DISCRETETIMEDERIVATIVE(UT, UT_1, DT), For scalar inputs UT 
%   Ut_1 and DT is (UT - UT_1)/DT
%
% Author : Wisseml CHIHA
%% ------------------------------------------------------------------------
upt  = (ut - ut_1)./dt;
end

