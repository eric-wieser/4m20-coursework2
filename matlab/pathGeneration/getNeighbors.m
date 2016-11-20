function [ cellout ] = getNeighbors( currentPoint, stepsize )
%GETNEIGHBORS Summary of this function goes here
%   Detailed explanation goes here

numCurr = inverseHashFunction( currentPoint );
cellout{1} = hashFunction( [ numCurr(1) + stepsize; numCurr(2); numCurr(3) ] );
cellout{2} = hashFunction( [ numCurr(1) - stepsize; numCurr(2); numCurr(3) ]) ;

cellout{3} = hashFunction( [ numCurr(1) ; numCurr(2)+ stepsize; numCurr(3) ]) ;
cellout{4} = hashFunction( [ numCurr(1) ; numCurr(2)- stepsize; numCurr(3) ]) ;

cellout{5} = hashFunction( [ numCurr(1); numCurr(2); numCurr(3) + stepsize ]) ;
cellout{6} = hashFunction( [ numCurr(1); numCurr(2); numCurr(3) - stepsize ]) ;

end

