%% 1. Start clean
clear;
clc;
close all;

%% 2. Physical parameters (Provisional Values)
p.M = 0.5;       % Cart mass [kg]
p.m = 0.2;       % Total pendulum mass [kg]
p.l = 0.3;       % Pivot-to-center-of-mass distance [m]
p.I = 0.006;     % Pendulum inertia about its center of mass [kg*m^2]
p.b = 0.1;       % Cart viscous friction coefficient [N*s/m]
p.g = 9.81;      % Gravity [m/s^2]

%% 3. Simulation settings
tEnd = 10;      % Simulation duration [s]
Fmax = 10;      % Assumed maximum cart force [N]

% State order: [cart position; cart velocity; angle; angular velocity]
z0 = [0; 0; deg2rad(10); 0];

%% 4. Linear model around upright
% Convention: x positive right; theta positive leaning left.
% Input: horizontal force on the cart [N].

M = p.M;
m = p.m;
l = p.l;
I = p.I;
b = p.b;
g = p.g;

J = I + m*l^2;          % Pendulum inertia about pivot
D = (M + m)*J - (m*l)^2;

A = [0,  1,             0,                 0;
     0, -J*b/D,         m^2*g*l^2/D,       0;
     0,  0,             0,                 1;
     0, -m*l*b/D,       (M+m)*m*g*l/D,     0];

B = [0;
     J/D;
     0;
     m*l/D];

%% 5. Check controllability
nControllable = rank(ctrb(A, B));
assert(nControllable == 4, 'The model is not fully controllable.');

%% 6. Initial LQR weights using acceptable scales
xScale     = 0.2;             % Cart position [m]
vScale     = 0.5;             % Cart velocity [m/s]
thetaScale = deg2rad(10);     % Pendulum angle [rad]
omegaScale = 1.0;             % Angular velocity [rad/s]

Q = diag(1 ./ [xScale, vScale, thetaScale, omegaScale].^2);
R = 1 / Fmax^2;

%% 7. Calculate gains and check linear closed-loop stability
K = lqr(A, B, Q, R);
closedLoopPoles = eig(A - B*K);

disp('LQR gains K:');
disp(K);

disp('Closed-loop poles:');
disp(closedLoopPoles);

assert(all(real(closedLoopPoles) < 0), ...
       'The linear closed-loop model is not stable.');

%% 8. Simulate nonlinear dynamics with LQR
options = odeset('RelTol', 1e-7, 'AbsTol', 1e-9, ...
                 'MaxStep', 0.01);

[t, z] = ode45(@(t,z) cartPendulumDynamics(t,z,p,K,Fmax), ...
              linspace(0,tEnd,1001), z0, options);

% Reconstruct requested and applied force
Frequested = -z*K';
Fapplied = max(-Fmax, min(Fmax, Frequested));

%% 9. Plot the results
figure('Name','Nonlinear LQR balancing');

subplot(3,1,1);
plot(t, rad2deg(z(:,3)), 'LineWidth', 1.5);
ylabel('Angle [deg]');
grid on;
title('Pendulum Balancing');

subplot(3,1,2);
plot(t, z(:,1), 'LineWidth', 1.5);
ylabel('Cart position [m]');
grid on;

subplot(3,1,3);
plot(t, Frequested, '--', t, Fapplied, '-', 'LineWidth', 1.2);
ylabel('Force [N]');
xlabel('Time [s]');
legend('Requested','Applied');
grid on;

%% 10. Animate the simulated motion
fig = figure('Name','Cart-pendulum animation');
ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
xlim(ax,[-0.4 0.4]);
ylim(ax,[-0.12 0.75]);
xlabel(ax,'Horizontal position [m]');
ylabel(ax,'Height [m]');

plot(ax,[-0.4 0.4],[-0.05 -0.05],'k-','LineWidth',2);

cartW = 0.12;
cartH = 0.06;
rodLength = 2*p.l;   % Illustrative length, not a model parameter

cart = rectangle(ax,'Position',[-cartW/2,-cartH,cartW,cartH], ...
                 'FaceColor',[0.2 0.6 0.9]);

rod = plot(ax,[0 0],[0 rodLength],'-','LineWidth',3);
tip = plot(ax,0,rodLength,'o','MarkerSize',10, ...
           'MarkerFaceColor',[0.9 0.4 0.1]);
label = title(ax,'');

for i = 1:3:length(t)
    if ~isgraphics(fig)
        break;
    end

    x = z(i,1);
    theta = z(i,3);

    % Positive theta means leaning left
    tipX = x - rodLength*sin(theta);
    tipY = rodLength*cos(theta);

    cart.Position = [x-cartW/2,-cartH,cartW,cartH];
    rod.XData = [x tipX];
    rod.YData = [0 tipY];
    tip.XData = tipX;
    tip.YData = tipY;

    label.String = sprintf('t = %.2f s | angle = %.2f deg', ...
                           t(i),rad2deg(theta));
    drawnow;
    pause(0.03);
end

%% Local function (keep at the bottom of the script)
function dz = cartPendulumDynamics(~, z, p, K, Fmax)
    v     = z(2);
    theta = z(3);
    omega = z(4);

    F = -K*z;
    F = max(-Fmax, min(Fmax, F));

    J = p.I + p.m*p.l^2;
    h = p.m*p.l;

    % Coupled equations: H * [cart acceleration; angular acceleration] = rhs
    H = [p.M + p.m,       -h*cos(theta);
         -h*cos(theta),    J];

    rhs = [F - p.b*v - h*sin(theta)*omega^2;
           h*p.g*sin(theta)];

    acceleration = H \ rhs;

    dz = [v;
          acceleration(1);
          omega;
          acceleration(2)];
end

