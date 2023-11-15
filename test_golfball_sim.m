%  Numerical integration of equations of motion for
%  golf ball trajectory (derived in class).
%
%  Updated:  13 Nov 2022  by  K. Inkol
%
clc
clearvars
close all


%  Launch conditions (J. McPhee, 2021)
v0 = 160.8;  % in mph
elev = 22.7; % in deg
azim = -5.3;  % in deg, about Y (i.e. going left)
back = 1498; % in rpm, about global Z
side = 280;    % in rpm, about global Y
rifle = 0;   % in rpm, about global X


grav = 32.17;            % gravity, in English units
radius = (1.68/2)/12;    % diameter of 1.68 inches, radius in feet
mass = (1.62/16)/grav;   % weight of 1.62 ounces, mass in slugs
rho = 0.0023769;         % density of air (slugs/ft^3)
area = pi*radius*radius;
inertia = 2*mass*radius*radius/5;   % inertia of a sphere

% Spin-dependent aero coefficients
aero_a = 0.171; % Quintavalla -> 0.171
aero_b = 0.62; % Quintavalla -> 0.62
aero_c = 0.083; % Quintavalla -> 0.083
aero_d = 0.885; % Quintavalla -> 0.885
aero_e = 0.0125; % Quintavalla -> 0.0125
params.use_quintavalla = false; % set to false to use static/constant coefficients
                                % set to true to use spin-dependent coefficients

% Static aero coefficients
Cd = 0.2; % is high-speed value from Smith 2018
Cl = 0.2;
Cm = 0.1;



% Assign necessary parameters to struct before simulations
params.a = aero_a;
params.b = aero_b; 
params.c = aero_c; 
params.d = aero_d; 
params.e = aero_e; 
params.static_Cd = Cd; 
params.static_Cl = Cl;
params.static_Cm = Cm;
params.radius = radius;
params.mass = mass;
params.rho = rho;
params.area = area;
params.inertia = inertia;
params.grav = grav;

% Run simulation
[X, Y, Z] = sim_golfball_flight(v0, elev, azim, back, side, rifle, params);
carry = X(end); % X driving distance in yards
offline = Z(end); % Z offline distance in yards
fprintf(['Carry: ', num2str(carry), ' Yards\nOffline:  ', num2str(offline), ' Yards'])


figure
plot3(X, Z, Y)
set(gca, 'YDir','reverse')
axis equal
grid on
axis([0 320 -40 40 0 80])
xlabel('X (yds)')
ylabel('Z (yds)')
zlabel('Y (yds)')
title('3D Trajectory')
view([-65, 8])

% plot apex
[~, apex_id] = max(Y);
text(X(apex_id), Z(apex_id), Y(apex_id), 'o', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle')

% plot landing position
text(X(end), Z(end), Y(end), 'x', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle')