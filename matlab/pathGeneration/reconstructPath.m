function [ total_path ] = reconstructPath(cameFrom, current)
%RECONSTRUCTPATH Summary of this function goes here
%   Detailed explanation goes here

    total_path = {current};
    while isKey(cameFrom,current)
        current = cameFrom(current);
        total_path{end+1} = current;
    end
    return


end

