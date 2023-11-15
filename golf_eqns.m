function xdot = golf_eqns(X,params)

%  Numerical integration of equations of
%  motion for golf ball using Quintavalla model
%
%  x(1)=X, x(2)=Y, x(3)=Z, x(4)=Vx, x(5)=Vy, x(6)=Vz, x(7)=omega
xdot = zeros(7,1);   % pre-define to make Matlab run faster

% offload parameters from struct
radius = params.radius;
mass = params.mass;
rho = params.rho;
area = params.area;
inertia = params.inertia;
grav = params.grav; 
tx = params.tx;
ty = params.ty;
tz = params.tz;


speed = norm(X(4:6));  % total speed
omega = X(7);                                 % total spin
spinratio = radius*omega/speed;   % Cl, CD may depend on this
Q = rho*speed*speed*area/2;

if params.use_quintavalla 
    
    a = params.a;
    b = params.b;
    c = params.c;
    d = params.d;
    e = params.e;
    Cd = a + b*spinratio;   % from Steve Quintavalla's paper.
    Cl = c + d*spinratio;
    Cm = e*spinratio;
 
else

    Cd = params.static_Cd;   % 0.2 is high-speed value from Smith 2018
    Cl = params.static_Cl;     % 0.2 is average value from Smith 2018
    Cm = params.static_Cm;

end



% Seven scalar ODEs used to describe ball flight
xdot(1) = X(4);
xdot(2) = X(5);
xdot(3) = X(6);

v_ball = X(4:6);
v_hat = v_ball/speed;

xdot(4) = (-Q*Cd*v_hat(1) + Q*Cl*(ty*v_hat(3)-tz*v_hat(2)))/mass ;
xdot(5) = (-Q*Cd*v_hat(2) + Q*Cl*(tz*v_hat(1)-tx*v_hat(3)))/mass - grav;
xdot(6) = (-Q*Cd*v_hat(3) + Q*Cl*(tx*v_hat(2)-ty*v_hat(1)))/mass;

xdot(7) = -Q*Cm*radius*2/inertia;

end

