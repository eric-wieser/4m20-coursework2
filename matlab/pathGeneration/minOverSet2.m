function [ minKey, minVal ] = minOverSet2( inputSet )
%MINOVERSET2 Summary of this function goes here
%   Detailed explanation goes here

[minVal,minIndex]  = min(cell2mat(values(inputSet)));
keyList = keys(inputSet);
minKey = keyList{minIndex};

end

