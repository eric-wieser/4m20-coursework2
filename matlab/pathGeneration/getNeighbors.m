function [ cellout ] = getNeighbors( currentPoint, stepsize )
%GETNEIGHBORS Summary of this function goes here
%   Detailed explanation goes here
numCurr = inverseHashFunction( currentPoint );
cellout = cell(3*3*3,1);
n = 1;
for i = [stepsize,0,-stepsize]
    for j = [stepsize,0,-stepsize]
        for k = [stepsize,0,-stepsize]
            cellout{n} = hashFunction( [ numCurr(1) + i; numCurr(2) + j; numCurr(3) + k ] );
            n = n+1;
        end
    end
end

end

