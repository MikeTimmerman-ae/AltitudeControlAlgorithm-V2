clear;

%% Get trajectory approximated by Nth-order polynomial

% Get trajectory data
N = 5;
t = 5.5:0.05:25.5;
x = csvread("OptimalTrajectoryDelayed_0.05.csv");

% Approximate altitude trajectory
p1 = polyfit(t,x(1,:),N);
y1 = polyval(p1, t);

% Approximate velocity trajectory
p2 = polyfit(t,x(2,:),N);
y2 = polyval(p2, t);

% Plot result to compare
figure

subplot(2,1,1);
plot(t,x(1,:),t,y1);
title("Vertical Displacement");
ylim([0, 4000]);
grid on
ylabel('Displacement [m]'); 
xlabel('Time [s]');
legend({'Actual Trajectory', 'Approx Trajectory'},'Location','southeast')

subplot(2,1,2);
plot(t,x(2,:),t,y2);
title("Vertical Velocity");
ylim([-20, 350]);
grid on
ylabel('Velocity [m/s]'); 
xlabel('Time [s]');
legend({'Actual Trajectory', 'Approx Trajectory'},'Location','northeast')
