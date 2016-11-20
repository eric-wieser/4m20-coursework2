function [ hashOut ] = hashFunction( inputPoint )
%HASHFUNCTION Summary of this function goes here
%   Detailed explanation goes here
compressed = int64( fix(inputPoint*100) + 200 );
hashOut = bitshift(compressed(1),32) +  bitshift(compressed(2),16) + compressed(3);
%hashOut = num2str(inputPoint','%10.2f');

end

