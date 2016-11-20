function [ outputPoint ] = inverseHashFunction( inputHash )
%INVERSEHASHFUNCTION Summary of this function goes here
%   Detailed explanation goes here
Mask = (2^16)-1;
outputPoint = double( [bitshift(inputHash,-32)- 200; bitand(bitshift(inputHash,-16),Mask)- 200; bitand(inputHash,Mask)- 200] )/100;

% outputPoint = str2num(inputHash)';

end

