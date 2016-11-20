function [ cost ] = heuristicGoalCost( jointAngles, stabilityRegion,stepSize )
%HEURISTICGOALCOST Summary of this function goes here
%   Detailed explanation goes here

endPoistion = [0.2;0];
thisPoistion = convertJointAnglesToEndPoint( jointAngles, stabilityRegion );
cost = norm(thisPoistion - endPoistion,2)/(0.15*3*3*stepSize);


end

