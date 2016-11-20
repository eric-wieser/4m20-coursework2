function [ cost ] = heuristicGoalCost( jointAngles )
%HEURISTICGOALCOST Summary of this function goes here
%   Detailed explanation goes here

endPoistion = [0.2;0];
thisPoistion = convertJointAnglesToEndPoint( jointAngles );
cost = norm(thisPoistion - endPoistion,2);


end

