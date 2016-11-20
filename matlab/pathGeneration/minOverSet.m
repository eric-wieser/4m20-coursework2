function [ minKey, minVal ] = minOverSet( inputSet, keyList )
%MINOVERSET Summary of this function goes here
%   Detailed explanation goes here

valueSet = values(inputSet,num2cell(keyList));
[minVal,minIndex] = min( cell2mat( valueSet ) );
minKey = keyList(minIndex);

end

