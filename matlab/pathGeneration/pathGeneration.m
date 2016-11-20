
%% Clean up workspace
clear

%% Run A*
stepSize = 0.1;
threshold = 0.05;
goal = [0.2;0];
startJointAngles = [1;1;1];
profile on
[ outputPath ] = AStar(startJointAngles, goal, stepSize, threshold);
profile off
profile viewer
vertexCollector = [];
for vertexIndex = 1:length(outputPath)
    thisVert = inverseHashFunction( outputPath{ vertexIndex} );
    vertexCollector = [vertexCollector,thisVert];
end

%% Generate cheeky plot
figure
for i = fliplr(1:length(vertexCollector))
    [ position0, position1, position2, position3, reachablePoint ] = convertJointAnglesToAllPoints( vertexCollector(:,i) );
    plot([0,position0(1),position1(1),position2(1),position3(1)],[0,position0(2),position1(2),position2(2),position3(2)])
    axis([-0.4 0.4,0,0.8])
    drawnow();
    pause(0.1);
end
