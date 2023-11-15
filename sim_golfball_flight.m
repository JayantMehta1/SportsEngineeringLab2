function [sim_X, sim_Y, sim_Z] = sim_golfball_flight(v0, elev, azim, back, side, rifle, params)
% Inputs:
%   v0 = launch speed (mph)
%   elev = vertical launch angle (elevation; degrees)
%   azim = horizontal launch angle (azimuth; degrees); -ve = going to right
%   back = initial backspin  (rpm)
%   side = initial sidespin (rpm)
%   rifle = initial rifle spin (rpm)
%   params = struct containing model parameters
%
% Returns:
%   sim_X = X position (carry) of ball (Yards)
%   sim_Y = Y position (height) of ball (Yards)
%   sim_Z = Z position (offline) of ball (Yards)


% Convert to x,y,z initial values
vx = (v0*cosd(elev)*cosd(azim))*88/60;  % convert mph to ft/s
vy = v0*sind(elev)*88/60;
vz = -(v0*cosd(elev)*sind(azim))*88/60;
wx = rifle*pi/30;   % convert rpm to rad/s
wy = side*pi/30;
wz = back*pi/30;

omega = sqrt(wx*wx + wy*wy + wz*wz);   % magnitude of angular velocity
tx = wx/omega;                         % constant directions
ty = wy/omega;
tz = wz/omega;
params.tx = tx; % add to param struct
params.ty = ty;
params.tz = tz;

t0 = 0;
tf = 15; % allow 15 seconds for ball to land (Y = 0)
x0 = [0, 0, 0, vx, vy, vz, omega]';   % launch conditions


% Run simulation
dynamic_eqn = @(t,x) golf_eqns(x, params);
land_events = @(t,X) stop_ball_flight(X,params); % get land_event function that has appropriate form
options = odeset('RelTol',1e-5,'AbsTol',1e-6, 'Events', land_events); % assign land_event function here
[t,x] = ode45(dynamic_eqn, [t0,tf], x0, options);

sim_X = x(:,1)/3;    % to get distances in yards
sim_Y = x(:,2)/3;
sim_Z = x(:,3)/3;

end



% Use subfunction here to capture event of ball landing during integration
function [position,isterminal,direction] = stop_ball_flight(X, params)
  position = X(2) - 0; % The value that we want to be zero
  isterminal = 1;  % Halt integration 
  direction = -1;   % The zero can be approached from either direction
end