function plotFrictionTorques(varargin)
%% -----------------------------------------------------------------------
%
%
%% ----------------------------------------------------------------------

    close all;
    figure(1);
    for  i=1:7
        subplot(3,3,i);
        plot(data.torque(:,i),'g--');
        hold on;
        plot(torques(:,i),'r-');
        hold on;
        plot(torques(:,i)+data.torque(:,i),'b-');
        legend('recorded','friction','frictionless');
        xlabel("Time (seconds)");
        ylabel("Torque (N.m) ");
        title(['Joint'  num2str(i)]) ;
    end
    set(1, 'Position', [200, 150, 1000, 600]);
    sgtitle(['Joints Torques, sampling frequency = ', ...
     num2str(1/data.timeStep), 'Friction Model ', model ], 'FontSize', 11);


end

