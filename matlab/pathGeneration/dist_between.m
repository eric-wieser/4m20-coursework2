function [ extraCost ] = dist_between(current, neighbor)
%DIST_BETWEEN Summary of this function goes here
%   Detailed explanation goes here

[ endPosition1, ~ ] = convertJointAnglesToEndPoint( current );

[ endPosition2, reachablePoint ] = convertJointAnglesToEndPoint( neighbor );

if (~reachablePoint) || (endPosition2(2) < 0)
    extraCost = 1000000;
else
    extraCost = norm(endPosition1 - endPosition2,2);
end

end

