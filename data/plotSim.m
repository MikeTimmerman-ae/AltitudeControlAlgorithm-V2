clear;

%% Plot simulated state trajectories and control inputs

t = 0:0.05:20;
x = csvread("state.csv");
u = csvread("input.csv");
r = csvread("OptimalTrajectoryDelayed_0.05.csv");

figure

subplot(3,2,1);
plot(t, x(2,:), t, r(1,:));
title("Vertical displacement")
ylim([0, 4000]);
grid on
ylabel('Displacement [m]'); 
xlabel('Time [s]');
legend({'Controlled Altitude', 'Reference Altitude'},'Location','southeast')

subplot(3,2,3);
plot(t, x(1,:));
title("Horizontal displacement")
grid on
ylabel('Displacement [m]');
xlabel('Time [s]');
legend({'Controlled Displacement'},'Location','southeast')

subplot(3,2,2);
plot(t, x(4,:), t, r(2,:));
title("Vertical velocity")
grid on
ylabel('Velocity [m/s]'); 
xlabel('Time [s]');
legend({'Controlled Velocity', 'Reference Velocity'},'Location','northeast')

subplot(3,2,4);
plot(t, x(3,:));
title("Horizontal velocity")
grid on
ylabel('Velocity [m/s]');
xlabel('Time [s]');
legend({'Controlled Velocity'},'Location','northeast')

subplot(3,2,5);
plot(t, u);
title("Airbrake linear extension")
ylim([-0.05, 0.1]);
grid on
yline(0.05, '--k', 'upper constraint');
yline(0, '--k', 'lower constraint');
ylabel('Extension [m]') 
xlabel('Times [s]') 
legend({'Control Input'},'Location','northeast')

subplot(3,2,6);
plot(t, x(5,:));
title("Electrical motor rotational speed")
ylim([-15, 15]);
grid on
yline(5, '--k', 'upper constraint');
yline(-5, '--k', 'lower constraint');
ylabel(' Angular velocity [rad/s]') 
xlabel('Times [s]') 
legend({'Motor input speed'},'Location','northeast')

sgtitle('Controlled Rocket Trajectory');


disp(max(x(2,:)));
disp((max(x(2,:))-3500)/35);

