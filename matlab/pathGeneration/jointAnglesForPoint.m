function [ reachablePoint, finalJointAngles ] = jointAnglesForPoint( endPoint, startingJointAngles, maxIterations, thresholdDist )
%JOINTANGLESFORPOINT Summary of this function goes here
%   Detailed explanation goes here

%% Set up the symbolic variables
syms phi1
syms phi2
syms phi3
syms m0
syms m1
syms m2
syms m3
syms lambda0
syms lambda1
syms lambda2
syms lambda3
syms theta1
syms theta2
syms theta3

%% Actual robot link lengths
l0 = 0.125;
l1 = 0.148;
l2 = 0.149;
l3 = 0.139;

%% Encode the local coordinates of each joint/endpoint
E0 = [0;l0];
E1 = [0;l1];
E2 = [0;l2];
E3 = [0;l3];

%% For the rest of the problem assume the following numerical parameters
lambda0 = l1/2;
lambda1 = l1/2;
lambda2 = l2/2;
lambda3 = l3/2;
theta1 = m1*(lambda1^2)/3;
theta2 = m2*(lambda2^2)/3;
theta3 = m3*(lambda3^2)/3;

%%  (1.1) The rotation matrices
R01 = [cos(phi1) -sin(phi1); sin(phi1) cos(phi1)];
R12 = [cos(phi2) -sin(phi2); sin(phi2) cos(phi2)];
R23 = [cos(phi3) -sin(phi3); sin(phi3) cos(phi3)];

R02 = simplify(R01*R12);
R03 = simplify(R02*R23);

%% Calculate the position of the endpoint
endPosition = simplify( E0 + R01*E1 + R02*E2 + R03*E3);

%% Calculate the position of the centre of mass
COMPosition = simplify( m0.*[0;lambda0] + m1.*R01*[0;lambda1] + m2.*R02*[0;lambda2] + m3.*R03*[0;lambda3]);

%% Jacobian of the endpoint
endJac = simplify([diff(endPosition,phi1),diff(endPosition,phi2),diff(endPosition,phi3)]);

%% Define the size of each step
stepSize = 0.2;

%%  Use the jacobian inverse method to find the jont angles that lead to a specific position
startingJointAngles = [1;1;1];
targetPosition = [stepSize;0];
qnew = startingJointAngles;
for i = 1:maxIterations
    phi1 = qnew(1);
    phi2 = qnew(2);
    phi3 = qnew(3);
    xt = double(subs(endPosition));
    qnew = qnew + pinv(double(subs(endJac)))*(targetPosition - xt);
end
reachablePoint = 0;
if ( norm(targetPosition - xt,2) < thresholdDist)
    reachablePoint = 1;
end
finalJointAngles = qnew;

end

