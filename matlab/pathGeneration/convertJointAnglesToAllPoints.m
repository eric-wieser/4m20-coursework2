function [ position0, position1, position2, endPosition, COMPosition, reachablePoint ] = convertJointAnglesToAllPoints( jointAngles , stabilityRegion)
%CONVERTJOINTANGLESTOALLPOINTS Summary of this function goes here
%   Detailed explanation goes here

%% Is this point reachable
jointLimit = 1.1;
if (max(abs(jointAngles)) > jointLimit)
    endPosition = [1000;1000];
    reachablePoint = 0;
    return
end
reachablePoint = 1;

phi1 = jointAngles(1);
phi2 = jointAngles(2);
phi3 = jointAngles(3);

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
%%  (1.1) The rotation matrices
R01 = [cos(phi1) -sin(phi1); sin(phi1) cos(phi1)];
R12 = [cos(phi2) -sin(phi2); sin(phi2) cos(phi2)];
R23 = [cos(phi3) -sin(phi3); sin(phi3) cos(phi3)];

R02 = R01*R12;
R03 = R02*R23;

%% Calculate the position of the endpoint
position0 = E0;
position1 = E0 + R01*E1;
position2 =  E0 + R01*E1 + R02*E2;
endPosition = E0 + R01*E1 + R02*E2 + R03*E3;


%% Check if it is unstable
m = 0.1;
COMPosition =  ([0;l0/2] + position0+R01*[0;l1/2] + position1+R02*[0;l2/2] + position2+R03*[0;l3/2] )/4;
if COMPosition(1) < min(stabilityRegion) || COMPosition(1) > max(stabilityRegion)
    reachablePoint = 0;
end


end

