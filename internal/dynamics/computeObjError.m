function RMSE = computeObjError(robot, Q_t, dQ_dt, d2Q_d2t,torques, x)
    %%
    %
    %
    %%
    N = length(Q_t);
    RMSE = 0;
    for t = 1:N
        err = computeModelError(robot, Q_t(t,:),dQ_dt(t,:), d2Q_d2t(t,:),...
            torques(t,:), x);
    RMSE= RMSE + err;
    end
end