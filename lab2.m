% BME 550: Lab 2 Matlab Code for Appendix

% Clear everything to start with
clc
clearvars
close all

%% Data
labData = {
    [163.4, 13.8, -3.8, 1777, -158, 0, 286.94, 35.3, -7.6];
    [164.5, 14.6, -3.3, 3095, -635, 0, 262.75, 47.9, 12.64];
    [155.8, 17, -2.8, 1757, -313, 0, 275.81, 41.7, 7.12];
    [159.22, 10.53, 0.08, 2104.38, -275.7, 0, 254.83, 23.2, 11];
    [147.16, 12.79, -3.01, 2287.03, -180.27, 0, 239.13, 24.85, -6.18];
    [152.73, 13.46, -0.24, 1622.57, 255.25, 0, 249.21, 25.47, -11.8];
    [161.01, 12.49, 1.63, 2996.35, 608.31, 0, 265.76, 35, -17.17];
    [173.05, 10.66, -0.58, 2001.54, -221.47, 0, 289.2, 28.08, 8.11];
    [143.3, 14.1, -1.8, 3342, -345, 0, 231.5, 36.4, 4.48];
    [168.73, 7.56, 2.32, 2307.1, 251.24, 0, 263.49, 19.6, 1.03]
};

%% Part 1
% Total number of shots
n = 10;

% Constant aerodynamic coeffecients from part 1 
Cd = 0.25;
Cl = 0.22;
Cm = 0.1;

% Params from test_golfball_sim
grav = 32.17;            % gravity, in English units
radius = (1.68/2)/12;    % diameter of 1.68 inches, radius in feet
mass = (1.62/16)/grav;   % weight of 1.62 ounces, mass in slugs
rho = 0.0023769;         % density of air (slugs/ft^3)
area = pi*radius*radius;
inertia = 2*mass*radius*radius/5;   % inertia of a sphere

params.radius = radius;
params.mass = mass;
params.rho = rho;
params.area = area;
params.inertia = inertia;
params.grav = grav;

aero_a = 0.171; % Quintavalla -> 0.171
aero_b = 0.62; % Quintavalla -> 0.62
aero_c = 0.083; % Quintavalla -> 0.083
aero_d = 0.885; % Quintavalla -> 0.885
aero_e = 0.0125; % Quintavalla -> 0.0125
params.use_quintavalla = false; % set to false to use static/constant coefficients
                                % set to true to use spin-dependent coefficients

params.a = aero_a;
params.b = aero_b; 
params.c = aero_c; 
params.d = aero_d; 
params.e = aero_e; 

params.static_Cd = Cd; 
params.static_Cl = Cl;
params.static_Cm = Cm;

LPEsum=0;

hold on;

for i = 1:n
    % Simulation shot details
    simulationDetails = labData{i};
    v0 = simulationDetails(1);
    elev = simulationDetails(2);
    azim = simulationDetails(3);
    back = simulationDetails(4);
    side = simulationDetails(5);
    rifle = simulationDetails(6);

    % Run simulation
    [X, Y, Z] = sim_golfball_flight(v0, elev, azim, back, side, rifle, params);
    carry = X(end); % X driving distance in yards
    offline = Z(end); % Z offline distance in yards

    % Plot the trajectory
    plot3(X, Z, Y);
    
    % Plot apex
    [~, apex_id] = max(Y);
    text(X(apex_id), Z(apex_id), Y(apex_id), 'o', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    
    % Plot landing position
    text(X(end), Z(end), Y(end), 'x', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
   
    labCarry = simulationDetails(7);
    labApex = simulationDetails(8);
    labOffline = simulationDetails(9);

    % Add to the sum amount for each iteration (landing position error)
    LPEsum = LPEsum + ((labCarry-carry)^2+(labOffline-offline)^2)^(1/2);
end

LPE = (1/n)*(LPEsum)

set(gca, 'YDir','reverse');
axis([0 320 -70 70 0 80]);
xlabel('Carry (yds)');
ylabel('Offline (yds)');
zlabel('Height (yds)');
title('Part 1: 3D Golf Shot Trajectories for 10 Shots ');
view([-40, 10]);
axis equal;
grid on;
hold off;


%% PART 2

% First shot
simulationDetails = labData{1};

% Rho ranges from 0.5defaultRho to 1.5defaultRho
defaultRho = params.rho;
startFactor = 0.5;
endFactor = 1.5;
rhoFactor = startFactor;
rhoStart = defaultRho*startFactor;
rhoEnd = defaultRho*endFactor;

% Simulation Details for part 2
v0 = simulationDetails(1);
elev = simulationDetails(2);
azim = simulationDetails(3);
back = simulationDetails(4);
side = simulationDetails(5);
rifle = simulationDetails(6);
figure();
hold on;

% Uses same aerodynamic conditions from part 1
% Range rho factor from 0.5 to 1.5
while rhoFactor <= 1.49
    currentRho = rho*rhoFactor;
    params.rho = currentRho;
    [X, Y, Z] = sim_golfball_flight(v0, elev, azim, back, side, rifle, params);
    normalizedRho = (currentRho - rhoStart)/(rhoEnd - rhoStart);
    plot(normalizedRho, X(end), '.', 'Color', 'black');
    rhoFactor = rhoFactor + 0.01;

end

title('Part 2: Carry Distance as a Function of Air Density, Shot 1');
xlabel('Normalized Air Density');
ylabel('Carry (yds)');
hold off;

%% Part 3

% Ensure rho is back to what it was at before part 2
rho = 0.0023769;         % density of air (slugs/ft^3)
params.rho = rho;

% Use aero params
initialAeroParams = [aero_a, aero_b, aero_c, aero_d, aero_e];

% Ranges
startRange = [0, 0, 0, 0, 0];
endRange = [1, 1, 1, 1, 1];

% fmincon
aeroCoeffs = fmincon(@(x) opFuncP3(x), initialAeroParams, [], [], [], [], startRange, endRange)

% Reset the LPEsum to 0 as we will go through all iterations for part 3
LPEsum=0;

for i = 1:n
    simulationDetails = labData{i};
    v0 = simulationDetails(1);
    elev = simulationDetails(2);
    azim = simulationDetails(3);
    back = simulationDetails(4);
    side = simulationDetails(5);
    rifle = simulationDetails(6);
    
    % Use new aero params
    params.use_quintavalla = true;
    params.a = aeroCoeffs(1);
    params.b = aeroCoeffs(2); 
    params.c = aeroCoeffs(3); 
    params.d = aeroCoeffs(4); 
    params.e = aeroCoeffs(5);
    
    % Run simulation
    [X, Y, Z] = sim_golfball_flight(v0, elev, azim, back, side, rifle, params);

    simulationCarry = simulationDetails(7);
    simulationApex = simulationDetails(8);
    simulationOffline = simulationDetails(9);

    % Add to LPE sum for each shot
    LPEsum = LPEsum + ((simulationCarry - X(end))^2 + (simulationOffline - Z(end))^2)^(1/2);

end

% LPE for part 3 for all 10 shots
part3LPE = (1/n)*(LPEsum)

%% Part 4
speedSum=0;
for i = 1:n
    simulationDetails = labData{i};
    speedSum = speedSum + simulationDetails(1);
end

meanBallSpeed = speedSum/n;

% fmincon tolerance 1e-8
% optimal backspin [500,4000]
% vertical launch angle [0,45]
toleranceFmincon = 1e-8;
startRange = [0, 500];
endRange= [45, 4000];

options = optimoptions('fmincon','OptimalityTolerance',toleranceFmincon);
launchAngleAndBackSpin = fmincon(@(x) opFuncP4(x, meanBallSpeed), [22.5,2250], [], [], [], [], startRange, endRange, [], options)
launchAngle = launchAngleAndBackSpin(1)
backSpin = launchAngleAndBackSpin(2)

% Optimization function for part 3
function y=opFuncP3(inputAeroParms)

    labData = {
    [163.4, 13.8, -3.8, 1777, -158, 0, 286.94, 35.3, -7.6];
    [164.5, 14.6, -3.3, 3095, -635, 0, 262.75, 47.9, 12.64];
    [155.8, 17, -2.8, 1757, -313, 0, 275.81, 41.7, 7.12];
    [159.22, 10.53, 0.08, 2104.38, -275.7, 0, 254.83, 23.2, 11];
    [147.16, 12.79, -3.01, 2287.03, -180.27, 0, 239.13, 24.85, -6.18];
    [152.73, 13.46, -0.24, 1622.57, 255.25, 0, 249.21, 25.47, -11.8];
    [161.01, 12.49, 1.63, 2996.35, 608.31, 0, 265.76, 35, -17.17];
    [173.05, 10.66, -0.58, 2001.54, -221.47, 0, 289.2, 28.08, 8.11];
    [143.3, 14.1, -1.8, 3342, -345, 0, 231.5, 36.4, 4.48];
    [168.73, 7.56, 2.32, 2307.1, 251.24, 0, 263.49, 19.6, 1.03]
    };

    grav = 32.17;            % gravity, in English units
    radius = (1.68/2)/12;    % diameter of 1.68 inches, radius in feet
    mass = (1.62/16)/grav;   % weight of 1.62 ounces, mass in slugs
    rho = 0.0023769;         % density of air (slugs/ft^3)
    area = pi*radius*radius;
    inertia = 2*mass*radius*radius/5;   % inertia of a sphere

    params.radius = radius;
    params.mass = mass;
    params.rho = rho;
    params.area = area;
    params.inertia = inertia;
    params.grav = grav;

    params.use_quintavalla = true;
    params.a = inputAeroParms(1);
    params.b = inputAeroParms(2); 
    params.c = inputAeroParms(3); 
    params.d = inputAeroParms(4); 
    params.e = inputAeroParms(5);
    
    shot1 = labData{1};
    shot2 = labData{2};
    shot3 = labData{3};
    shot4 = labData{4};
    shot5 = labData{5};
    shot6 = labData{6};
    shot7 = labData{7};
    shot8 = labData{8};
    shot9 = labData{9};
    shot10 = labData{10};

    [X_1, ~, Z_1] = sim_golfball_flight(shot1(1), shot1(2), shot1(3), shot1(4), shot1(5), shot1(6), params);
    [X_2, ~, Z_2] = sim_golfball_flight(shot2(1), shot2(2), shot2(3), shot2(4), shot2(5), shot2(6), params);
    [X_3, ~, Z_3] = sim_golfball_flight(shot3(1), shot3(2), shot3(3), shot3(4), shot3(5), shot3(6), params);
    [X_4, ~, Z_4] = sim_golfball_flight(shot4(1), shot4(2), shot4(3), shot4(4), shot4(5), shot4(6), params);
    [X_5, ~, Z_5] = sim_golfball_flight(shot5(1), shot5(2), shot5(3), shot5(4), shot5(5), shot5(6), params);
    [X_6, ~, Z_6] = sim_golfball_flight(shot6(1), shot6(2), shot6(3), shot6(4), shot6(5), shot6(6), params);
    [X_7, ~, Z_7] = sim_golfball_flight(shot7(1), shot7(2), shot7(3), shot7(4), shot7(5), shot7(6), params);
    [X_8, ~, Z_8] = sim_golfball_flight(shot8(1), shot8(2), shot8(3), shot8(4), shot8(5), shot8(6), params);
    [X_9, ~, Z_9] = sim_golfball_flight(shot9(1), shot9(2), shot9(3), shot9(4), shot9(5), shot9(6), params);
    [X_10, ~, Z_10] = sim_golfball_flight(shot10(1), shot10(2), shot10(3), shot10(4), shot10(5), shot10(6), params);

    y =  (1/10)*( ...
          ((shot1(7) - X_1(end))^2 + (shot1(9) - Z_1(end))^2)^(1/2) ...
        + ((shot2(7) - X_2(end))^2 + (shot2(9) - Z_2(end))^2)^(1/2) ...
        + ((shot3(7) - X_3(end))^2 + (shot3(9) - Z_3(end))^2)^(1/2) ...
        + ((shot4(7) - X_4(end))^2 + (shot4(9) - Z_4(end))^2)^(1/2) ...
        + ((shot5(7) - X_5(end))^2 + (shot5(9) - Z_5(end))^2)^(1/2) ...
        + ((shot6(7) - X_6(end))^2 + (shot6(9) - Z_6(end))^2)^(1/2) ...
        + ((shot7(7) - X_7(end))^2 + (shot7(9) - Z_7(end))^2)^(1/2) ...
        + ((shot8(7) - X_8(end))^2 + (shot8(9) - Z_8(end))^2)^(1/2) ...
        + ((shot9(7) - X_9(end))^2 + (shot9(9) - Z_9(end))^2)^(1/2) ...
        + ((shot10(7) - X_10(end))^2 + (shot10(9) - Z_10(end))^2)^(1/2));

end

% Optimization function for part 4
function x=opFuncP4(inputLaunchAngleAndBackSpin, meanBallSpeed)    
    grav = 32.17;            % gravity, in English units
    radius = (1.68/2)/12;    % diameter of 1.68 inches, radius in feet
    mass = (1.62/16)/grav;   % weight of 1.62 ounces, mass in slugs
    rho = 0.0023769;         % density of air (slugs/ft^3)
    area = pi*radius*radius;
    inertia = 2*mass*radius*radius/5;   % inertia of a sphere

    params.radius = radius;
    params.mass = mass;
    params.rho = rho;
    params.area = area;
    params.inertia = inertia;
    params.grav = grav;
    
    % Aero param values for part 4 
    a = 0.1625;
    b = 0.5261;
    c = 0.0943;
    d = 0.6372;
    e = 0.01;

    params.use_quintavalla = true;
    params.a = a;
    params.b = b; 
    params.c = c; 
    params.d = d; 
    params.e = e;

    [X, ~, ~] = sim_golfball_flight(meanBallSpeed, inputLaunchAngleAndBackSpin(1), 0, inputLaunchAngleAndBackSpin(2), 0, 0, params);

    x = -1*X(end);
end


