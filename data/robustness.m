clear;

%% Plot difference in target apogee against variation in initial conditions of the state

stateOffsets = csvread("stateOffsets.csv");
deviations = csvread("deviations.csv");

[Cd_fit,gof] = fit([stateOffsets(:,4),stateOffsets(:,2)],deviations/35,'linearinterp');

figure;
plot( Cd_fit,[stateOffsets(:,4),stateOffsets(:,2)],deviations/35 );
zlabel('percentage deviation from target [%]')
ylabel('deviation in initial altitude [m]') 
xlabel('deviation in initial velocity [s]')