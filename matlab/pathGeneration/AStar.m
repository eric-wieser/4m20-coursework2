function [ outputPath ] = AStar(startJointAngles, goal, stepSize, threshold)
%ASTAR Summary of this function goes here
%   Detailed explanation goes here

    start = startJointAngles;

    % The set of nodes already evaluated.
    closedSet = containers.Map('KeyType','int64', 'ValueType', 'any');
    closedSetMaxIndex = 0;
    % For each node, which node it can most efficiently be reached from.
    % If a node can be reached from many nodes, cameFrom will eventually contain the
    % most efficient previous step.
    cameFrom = containers.Map('KeyType','int64', 'ValueType', 'any');
    % For each node, the cost of getting from the start node to that node.
    gScore = containers.Map('KeyType','int64', 'ValueType', 'any');
    % The cost of going from start to start is zero.
    gScore(hashFunction( start )) = 0 ;
    % For each node, the total cost of getting from the start node to the goal
    % by passing by that node. That value is partly known, partly heuristic.
    fScore = containers.Map('KeyType','int64', 'ValueType', 'any');
    % For the first node, that value is completely heuristic.
    fScore(hashFunction( start)) = heuristicGoalCost(start);
     % The set of currently discovered nodes still to be evaluated.
    % Initially, only the start node is known.
    openSet = containers.Map('KeyType','int64', 'ValueType', 'any');
    openSet(hashFunction(start)) = heuristicGoalCost(start);

    while size(openSet) ~= 0
        % the node in openSet having the lowest fScore[] value
        [ minKey, minVal ] = minOverSet2( openSet );
        current = minKey;
        [ currentendPosition, ~ ] = convertJointAnglesToEndPoint( inverseHashFunction(current) );
        if norm(currentendPosition - goal,2) < threshold
            outputPath = reconstructPath(cameFrom, current);
            return
        end
        remove(openSet,current);
        closedSet(current) = closedSetMaxIndex+1;
        closedSetMaxIndex = closedSetMaxIndex+1;
        currentNeighbors = getNeighbors( current, stepSize );
        for neighborIndex = 1:length(currentNeighbors)
            neighbor = currentNeighbors{neighborIndex};
            
            if isKey(closedSet,neighbor)
                continue % Ignore the neighbor which is already evaluated.
            end
            % The distance from start to a neighbor
            tentative_gScore = gScore(current) + dist_between( inverseHashFunction(current),  inverseHashFunction(neighbor));
            newNodeFlag = 0;
            if ~isKey(openSet,neighbor ) % Discover a new node
                newNodeFlag = 1;
            else
                if ( tentative_gScore >= gScore(neighbor) )
                    continue	% This is not a better path.
                end
            end
            % This path is the best until now. Record it!
            cameFrom(neighbor) = current;
            gScore(neighbor) = tentative_gScore;
            fScore(neighbor) = gScore(neighbor) + heuristicGoalCost( inverseHashFunction(neighbor));
            if newNodeFlag
                openSet(neighbor) = fScore(neighbor);
            end
        end
    end
    outputPath = [];
end