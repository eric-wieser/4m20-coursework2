function [ extraCost ] = dist_between(current, neighbor, stabilityRegion)
%DIST_BETWEEN Summary of this function goes here
%   Detailed explanation goes here

%[ endPosition1, ~ ] = convertJointAnglesToEndPoint( current, stabilityRegion );

[ endPosition2, reachablePoint ] = convertJointAnglesToEndPoint( neighbor, stabilityRegion );

if (~reachablePoint) || (endPosition2(2) < 0)
    extraCost = 1000000;
else
    %extraCost = norm(endPosition1 - endPosition2,2);
    extraCost = 1.0;
end

end

