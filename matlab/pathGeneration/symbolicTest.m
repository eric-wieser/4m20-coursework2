
%% Clean up workspace
clc
clear

%% Set up the symbolic variables
syms phi1
syms phi2
syms phi3
syms l1
syms l2
syms l3
syms lambda1
syms lambda2
syms lambda3
syms theta1
syms theta2
syms theta3

%% Encode the local coordinates of each joint/endpoint
E1 = [0;l1];
E2 = [0;l2];
E3 = [0;l3];

%%  (1.1) The rotation matrices
disp('1.1 beginning')
R01 = [cos(phi1) sin(phi1); -sin(phi1) cos(phi1)];
R12 = [cos(phi2) sin(phi2); -sin(phi2) cos(phi2)];
R23 = [cos(phi3) sin(phi3); -sin(phi3) cos(phi3)];

R02 = simplify(R01*R12);
R03 = simplify(R02*R23);

%% Calculate the position of the endpoint
endPosition = simplify( R01*E1 + R02*E2 + R03*E3);

%% (1.2) Jacobian of the endpoint
disp('1.2 beginning')
endJac = simplify([diff(endPosition,phi1),diff(endPosition,phi2),diff(endPosition,phi3)]);

%% For the rest of the problem assume the following numerical parameters
m1 = 0.2;
m2 = m1;
m3 = m1;
l1 = 0.1;
l2 = l1;
l3 = l2;
lambda1 = l1/(2*m1);
lambda2 = l2/(2*m2);
lambda3 = l3/(2*m3);
theta1 = m1*(lambda1^2)/3;
theta2 = m2*(lambda2^2)/3;
theta3 = m3*(lambda3^2)/3;
q0 = [-pi/3;-pi/4;-pi/4];
q0_dot = [0;0;0];

%% (1.3) Use the jacobian inverse method to find the jont angles that lead to a specific position
disp('1.3 beginning')
rT = [0.05;0.15];
qnew = q0;
for i = 1:30
    phi1 = qnew(1);
    phi2 = qnew(2);
    phi3 = qnew(3);
    xt = double(subs(endPosition));
    qnew = qnew + pinv(double(subs(endJac)))*(rT - xt);
end
finalJointAngles = qnew;

%% (1.4) Move the end effector in positive y direction for 0.05 with speed 0.2, plot qdot vs q
disp('1.4 beginning')
rT = [0.05;0.15];
lT = 0.05;
vT = 0.2;
% Time parameters
npoints = 50;
totalTime = lT/vT;
timeStep = totalTime/npoints;
distanceStep = lT/npoints;
timeList = [1:npoints]*timeStep;
% Solve to find all joint angles
ouputJointAngles = finalJointAngles;
for j = 1:npoints
    % For each point, converge on solution and store it
    rT = rT + [0;distanceStep];
    for i = 1:20
        phi1 = qnew(1);
        phi2 = qnew(2);
        phi3 = qnew(3);
        xt = double(subs(endPosition));
        qnew = qnew + pinv(double(subs(endJac)))*(rT - xt);
    end
    finalJointAngles = qnew;
    ouputJointAngles = [ouputJointAngles,finalJointAngles];
end
[ouputJointAnglesDot,~] = gradient(ouputJointAngles,timeStep);
figure(1);
plot(ouputJointAngles(1,:),ouputJointAnglesDot(1,:))
grid on
xlabel('$q_1$','Interpreter','LaTex', 'FontSize', 20) 
ylabel('$\dot{q_1}$','Interpreter','LaTex', 'FontSize', 20) 
figure(2);
plot(ouputJointAngles(2,:),ouputJointAnglesDot(2,:))
xlabel('$q_2$','Interpreter','LaTex',  'FontSize', 20) 
ylabel('$\dot{q_2}$','Interpreter','LaTex', 'FontSize', 20) 
grid on
figure(3);
plot(ouputJointAngles(3,:),ouputJointAnglesDot(3,:))
xlabel('$q_3$','Interpreter','LaTex', 'FontSize', 20) 
ylabel('$\dot{q_3}$','Interpreter','LaTex', 'FontSize', 20) 
grid on

%% (1.5) Assume q0 = [0,-pi,pi] and calculate the Jacobian for the end effector
disp('1.5 beginning')
phi1 = 0
phi2 = -pi + 0.001*pi
phi3 = pi
double( subs(endJac) )
% No we are at a singularity

%% ( 2.1)Removing the uppermost link of the manipulator, such that the whole system acts as a two link
% manipulator. Define the Lagrangian of the system, and derive the equations of motion. 
% Ignore the gravitational force as g = 0 m/s2
disp('2.1 beginning')

% Set up the symbolic variables
syms phi1
syms phi2
syms phi1dot
syms phi2dot
syms phi1dotdot
syms phi2dotdot
syms l1
syms l2
syms m1
syms m2
syms lambda1
syms lambda2
syms theta1
syms theta2

% Calculate positions of the centres of mass
m1Position = R01*[0;lambda1];
m2Position = R01*E1 + R02*[0;lambda2];

% Calculate Jacobians of the centres of mass
m1Jac = simplify([diff(m1Position,phi1),diff(m1Position,phi2)]);
m2Jac = simplify([diff(m2Position,phi1),diff(m2Position,phi2)]);

% Convert Jacobians into speeds squared
m1Velocity = m1Jac*[phi1dot;phi2dot];
m2Velocity = m2Jac*[phi1dot;phi2dot];
m1Speedsqrd = simplify(m1Velocity(1)^2 + m1Velocity(2)^2);
m2Speedsqrd = simplify(m2Velocity(1)^2 + m2Velocity(2)^2);

% Calculate global roation
m1Omegasqrd = phi1dot^2;
m2Omegasqrd = (phi1dot+phi2dot)^2;

T = simplify( 0.5*(m1*m1Speedsqrd + m2*m2Speedsqrd + theta1*m1Omegasqrd + theta2*m2Omegasqrd) );
V = 0;
L = T - V;

% Calculate the equations of motion by taking derivatives
dLbydQ1dot = diff(L,phi1dot);
dLbydQ2dot = diff(L,phi2dot);
TwoPointOne1 = simplify(simplify( diff(dLbydQ1dot,phi1)*phi1dot + diff(dLbydQ1dot,phi2)*phi2dot + diff(dLbydQ1dot,phi1dot)*phi1dotdot + diff(dLbydQ1dot,phi2dot)*phi2dotdot - diff(L,phi1) ));
TwoPointOne2 = simplify(simplify( diff(dLbydQ2dot,phi1)*phi1dot + diff(dLbydQ2dot,phi2)*phi2dot + diff(dLbydQ2dot,phi1dot)*phi1dotdot + diff(dLbydQ2dot,phi2dot)*phi2dotdot - diff(L,phi2) ));

% Do substitutions to get it in a simpler state than it is now
syms m
syms l
for i = 1:20
    TwoPointOne1 = simplify(subs(TwoPointOne1,{m1,m2,theta1,theta2,lambda1,lambda2,l1,l2},{m,m,(m*lambda1^2)/3,(m*lambda2^2)/3,l1/2,l2/2,l,l}));
    TwoPointOne2 = simplify(subs(TwoPointOne2,{m1,m2,theta1,theta2,lambda1,lambda2,l1,l2},{m,m,(m*lambda1^2)/3,(m*lambda2^2)/3,l1/2,l2/2,l,l}));
end
latex(TwoPointOne1)
latex(TwoPointOne2)

%% ( 2.2)Consider the gravitational acceleration g=9.8 m/s2 acts on the whole system, and explain 
% how the equations of motion change. And, by assuming two motors are installed in the 
% lower two joints, what are the torques of these motors a01, a12 required to stand still 
% against the gravitational force? Derive a01, a12 as functions of phi1,phi2
disp('2.2 beginning')

syms g;
V = m1*g*m1Position(2) + m2*g*m2Position(2);
L = T - V;
dLbydQ1dot = diff(L,phi1dot);
dLbydQ2dot = diff(L,phi2dot);
a1 = simplify( diff(dLbydQ1dot,phi1)*phi1dot + diff(dLbydQ1dot,phi2)*phi2dot + diff(dLbydQ1dot,phi1dot)*phi1dotdot + diff(dLbydQ1dot,phi2dot)*phi2dotdot - diff(L,phi1) );
a2 = simplify( diff(dLbydQ2dot,phi1)*phi1dot + diff(dLbydQ2dot,phi2)*phi2dot + diff(dLbydQ2dot,phi1dot)*phi1dotdot + diff(dLbydQ2dot,phi2dot)*phi2dotdot - diff(L,phi2) );

phi1dot = 0;
phi2dot = 0;
phi1dotdot = 0;
phi2dotdot = 0;

subs(a1)
subs(a2)

%% (3.1)Derive the Newton-Euler equations of motion of the aforementioned two-link manipulator. 
% Assume that no gravity and no external forces are acting on the system
disp('3.1 beginning')

% Set up the symbolic variables
syms phi1
syms phi2
syms phi1dot
syms phi2dot
syms phi1dotdot
syms phi2dotdot
syms l1
syms l2
syms m1
syms m2
syms lambda1
syms lambda2
syms theta1
syms theta2

% Calculate global roation
m1Omega= phi1dot;
m2Omega = (phi1dot+phi2dot);
m1Omegadot= phi1dotdot;
m2Omegadot = (phi1dotdot+phi2dotdot);

% Calculate accelerations
m1Acceleration = diff(m1Velocity,phi1)*phi1dot + diff(m1Velocity,phi2)*phi2dot + diff(m1Velocity,phi1dot)*phi1dotdot + diff(m1Velocity,phi2dot)*phi2dotdot;
m2Acceleration = diff(m2Velocity,phi1)*phi1dot + diff(m2Velocity,phi2)*phi2dot + diff(m2Velocity,phi1dot)*phi1dotdot + diff(m2Velocity,phi2dot)*phi2dotdot;

% Newtons Equations
force23 = [0;0;0];
force12 = [m2*m2Acceleration;0];
force01 = force12 +[m1*m1Acceleration;0];

% Eulers Equations
end1 = [R01*E1;0];
r01 = end1;
end2 = [R01*E1;0] + [R02*E2;0];
r12 = end2 - end1;
rC1 = [m1Position;0];
rC2 = [m2Position;0];
torque23 = [0;0;0];%cross((r23 + r3C3),force23) + theta3*[0;0;-phi3dot] + cross([0;0;-phi3],theta3*[0;0;-phi3])
torque12 = simplify( torque23 + cross(rC2 - end1,force12) - cross(end2 - rC2,(-force23)) + theta2*[0;0;-m2Omegadot] + cross([0;0;m2Omega],theta2*[0;0;m2Omega]) );
torque01 = simplify( torque12 + cross(rC1,force01) - cross(end1 - rC1,(-force12)) + theta1*[0;0;-m1Omegadot] + cross([0;0;m1Omega],theta1*[0;0;m1Omega]) );

% Do substitutions to get it in a simpler state than it is now
syms m
syms l
for i = 1:20
    torque01 = simplify(subs(torque01,{m1,m2,theta1,theta2,lambda1,lambda2,l1,l2},{m,m,(m*lambda1^2)/3,(m*lambda2^2)/3,l1/2,l2/2,l,l}));
    torque12 = simplify(subs(torque12,{m1,m2,theta1,theta2,lambda1,lambda2,l1,l2},{m,m,(m*lambda1^2)/3,(m*lambda2^2)/3,l1/2,l2/2,l,l}));
end
latex(torque01(3))
latex(torque12(3))

%% (3.2)How do the equations change for the three-link manipulator by including the uppermost link
% in the figure?
disp('3.2 beginning')

% Set up the symbolic variables
syms phi1
syms phi2
syms phi3
syms phi1dot
syms phi2dot
syms phi3dot
syms phi1dotdot
syms phi2dotdot
syms phi3dotdot
syms l1
syms l2
syms l3
syms m1
syms m2
syms m3
syms lambda1
syms lambda2
syms lambda3
syms theta1
syms theta2
syms theta3

% Calculate global rotation
m1Omega= phi1dot;
m2Omega = (phi1dot+phi2dot);
m3Omega = (phi1dot+phi2dot+phi3dot);
m1Omegadot= phi1dotdot;
m2Omegadot = (phi1dotdot+phi2dotdot);
m3Omegadot = (phi1dotdot+phi2dotdot+phi3dotdot);

% Calculate m3 position
m3Position = R01*E1 + R02*E2 + R03*[0;lambda3];
m3Jac = simplify([diff(m3Position,phi1),diff(m3Position,phi2),diff(m3Position,phi3)]);
m3Velocity = m3Jac*[phi1dot;phi2dot;phi3dot];

% Calculate accelerations
m1Acceleration = diff(m1Velocity,phi1)*phi1dot + diff(m1Velocity,phi2)*phi2dot + diff(m1Velocity,phi1dot)*phi1dotdot + diff(m1Velocity,phi2dot)*phi2dotdot;
m2Acceleration = diff(m2Velocity,phi1)*phi1dot + diff(m2Velocity,phi2)*phi2dot + diff(m2Velocity,phi1dot)*phi1dotdot + diff(m2Velocity,phi2dot)*phi2dotdot;
m3Acceleration = diff(m3Velocity,phi1)*phi1dot + diff(m3Velocity,phi2)*phi2dot + diff(m3Velocity,phi1dot)*phi1dotdot + diff(m3Velocity,phi2dot)*phi2dotdot;

% Newtons Equations
force23 = [m3*m3Acceleration;0];
force12 = force23 + [m2*m2Acceleration;0];
force01 = force12 + [m1*m1Acceleration;0];

% Eulers Equations
end1 = [R01*E1;0];
end2 = [R01*E1;0] + [R02*E2;0];
rC1 = [m1Position;0];
rC2 = [m2Position;0];
rC3 = [m3Position;0];
torque23 = simplify( cross(rC3 - end2,force23) + theta3*[0;0;-m3Omegadot] + cross([0;0;m3Omega],theta3*[0;0;m3Omega]) );
torque12 = simplify( torque23 + cross(rC2 - end1,force12) - cross(end2 - rC2,(-force23)) + theta2*[0;0;-m2Omegadot] + cross([0;0;m2Omega],theta2*[0;0;m2Omega]) );
torque01 = simplify( torque12 + cross(rC1,force01) - cross(end1 - rC1,(-force12)) + theta1*[0;0;-m1Omegadot] + cross([0;0;m1Omega],theta1*[0;0;m1Omega]) );

% Do substitutions to get it in a simpler state than it is now
syms m
syms l
for i = 1:20
    torque01 = simplify(subs(torque01,{m1,m2,m3},{m,m,m}));
    torque01 = simplify(subs(torque01,{theta1,theta2,theta3},{(m*lambda1^2)/3,(m*lambda2^2)/3,(m*lambda3^2)/3})) ;
    torque01 = simplify(subs(torque01,{lambda1,lambda2,lambda3},{l1/2,l2/2,l3/2}));
    torque01 = simplify(subs(torque01,{l1,l2,l3},{l,l,l}));
    
    torque12 = simplify(subs(torque12,{m1,m2,m3},{m,m,m}));
    torque12 = simplify(subs(torque12,{theta1,theta2,theta3},{(m*lambda1^2)/3,(m*lambda2^2)/3,(m*lambda3^2)/3})) ;
    torque12 = simplify(subs(torque12,{lambda1,lambda2,lambda3},{l1/2,l2/2,l3/2}));
    torque12 = simplify(subs(torque12,{l1,l2,l3},{l,l,l}));
    
    torque23 = simplify(subs(torque23,{m1,m2,m3},{m,m,m}));
    torque23 = simplify(subs(torque23,{theta1,theta2,theta3},{(m*lambda1^2)/3,(m*lambda2^2)/3,(m*lambda3^2)/3})) ;
    torque23 = simplify(subs(torque23,{lambda1,lambda2,lambda3},{l1/2,l2/2,l3/2}));
    torque23 = simplify(subs(torque23,{l1,l2,l3},{l,l,l}));
end
latex(torque01(3))
latex(torque12(3))
latex(torque23(3))


%% (3.3)How do the equations of motion of the three-link manipulator change further if the 
% gravitational forces are acting on the system? What are the torques !!!, !!", !!" required 
% for joint actuators in order to maintain a posture? Assume !! and !! as given in the table 
% above. 
disp('3.3 beginning')

% Set up the symbolic variables
syms phi1
syms phi2
syms phi3
syms phi1dot
syms phi2dot
syms phi3dot
syms phi1dotdot
syms phi2dotdot
syms phi3dotdot
syms l1
syms l2
syms l3
syms m1
syms m2
syms m3
syms lambda1
syms lambda2
syms lambda3
syms theta1
syms theta2
syms theta3
syms g

% Calculate global rotation
m1Omega= phi1dot;
m2Omega = (phi1dot+phi2dot);
m3Omega = (phi1dot+phi2dot+phi3dot);
m1Omegadot= phi1dotdot;
m2Omegadot = (phi1dotdot+phi2dotdot);
m3Omegadot = (phi1dotdot+phi2dotdot+phi3dotdot);

% Calculate m3 position
m3Position = R01*E1 + R02*E2 + R03*[0;lambda3];
m3Jac = simplify([diff(m3Position,phi1),diff(m3Position,phi2),diff(m3Position,phi3)]);
m3Velocity = m3Jac*[phi1dot;phi2dot;phi3dot];

% Calculate accelerations
m1Acceleration = diff(m1Velocity,phi1)*phi1dot + diff(m1Velocity,phi2)*phi2dot + diff(m1Velocity,phi1dot)*phi1dotdot + diff(m1Velocity,phi2dot)*phi2dotdot;
m2Acceleration = diff(m2Velocity,phi1)*phi1dot + diff(m2Velocity,phi2)*phi2dot + diff(m2Velocity,phi1dot)*phi1dotdot + diff(m2Velocity,phi2dot)*phi2dotdot;
m3Acceleration = diff(m3Velocity,phi1)*phi1dot + diff(m3Velocity,phi2)*phi2dot + diff(m3Velocity,phi1dot)*phi1dotdot + diff(m3Velocity,phi2dot)*phi2dotdot;

% Newtons Equations
force23 = [m3*m3Acceleration;0] + [0;m3*g;0];
force12 = force23 + [m2*m2Acceleration;0] + [0;m2*g;0];
force01 = force12 + [m1*m1Acceleration;0] + [0;m2*g;0];

% Eulers Equations
end1 = [R01*E1;0];
end2 = [R01*E1;0] + [R02*E2;0];
rC1 = [m1Position;0];
rC2 = [m2Position;0];
rC3 = [m3Position;0];
torque23 = simplify( cross(rC3 - end2,force23) + theta3*[0;0;-m3Omegadot] + cross([0;0;m3Omega],theta3*[0;0;m3Omega]) );
torque12 = simplify( torque23 + cross(rC2 - end1,force12) - cross(end2 - rC2,(-force23)) + theta2*[0;0;-m2Omegadot] + cross([0;0;m2Omega],theta2*[0;0;m2Omega]) );
torque01 = simplify( torque12 + cross(rC1,force01) - cross(end1 - rC1,(-force12)) + theta1*[0;0;-m1Omegadot] + cross([0;0;m1Omega],theta1*[0;0;m1Omega]) );

torque01 = torque01(3);
torque12 = torque12(3);
torque23 = torque23(3);

% Do substitutions to get it in a simpler state than it is now
syms m
syms l

torque01 = simplify(subs(torque01,{m1,m2,m3},{m,m,m}));
torque01 = simplify(subs(torque01,{theta1,theta2,theta3},{(m*lambda1^2)/3,(m*lambda2^2)/3,(m*lambda3^2)/3})) ;
torque01 = simplify(subs(torque01,{lambda1,lambda2,lambda3},{l1/2,l2/2,l3/2}));
torque01 = simplify(subs(torque01,{l1,l2,l3},{l,l,l}));

torque12 = simplify(subs(torque12,{m1,m2,m3},{m,m,m}));
torque12 = simplify(subs(torque12,{theta1,theta2,theta3},{(m*lambda1^2)/3,(m*lambda2^2)/3,(m*lambda3^2)/3})) ;
torque12 = simplify(subs(torque12,{lambda1,lambda2,lambda3},{l1/2,l2/2,l3/2}));
torque12 = simplify(subs(torque12,{l1,l2,l3},{l,l,l}));

torque23 = simplify(subs(torque23,{m1,m2,m3},{m,m,m}));
torque23 = simplify(subs(torque23,{theta1,theta2,theta3},{(m*lambda1^2)/3,(m*lambda2^2)/3,(m*lambda3^2)/3})) ;
torque23 = simplify(subs(torque23,{lambda1,lambda2,lambda3},{l1/2,l2/2,l3/2}));
torque23 = simplify(subs(torque23,{l1,l2,l3},{l,l,l}));

latex(torque01)
latex(torque12)
latex(torque23)



phi1dot = 0;
phi2dot = 0;
phi3dot = 0;
phi1dotdot = 0;
phi2dotdot = 0;
phi3dotdot = 0;

m1 = 0.2;
m2 = m1;
m3 = m1;
l1 = 0.1;
l2 = l1;
l3 = l2;
lambda1 = l1/2;
lambda2 = l2/2;
lambda3 = l3/2;
theta1 = m1*(lambda1^2)/3;
theta2 = m2*(lambda2^2)/3;
theta3 = m3*(lambda3^2)/3;
phi1 = -pi/3;
phi2 = -pi/4;
phi3 = -pi/4;
g = -9.8;

disp([double( subs(torque01) );
double( subs(torque12) );
double( subs(torque23) )])

