
%% Clean up workspace
clear

%% Run A*
stepSize = 0.1;
threshold = 0.05;
goal = [0.25;0];
startJointAngles = [0.9;1.10;1.10];
stabilityRegion = [-0.14,0.14];
[ outputPath ] = AStar(startJointAngles, goal, stepSize, threshold, stabilityRegion);
vertexCollector = [];
for vertexIndex = 1:length(outputPath)
    thisVert = inverseHashFunction( outputPath{ vertexIndex} );
    vertexCollector = [vertexCollector,thisVert];
end

%% Generate cheeky plot
figure
idx = 1;
for i = fliplr(1:length(vertexCollector))
    % Calculate the positions of the endpoints
    [ position0, position1, position2, position3, COMPosition, reachablePoint ] = convertJointAnglesToAllPoints( vertexCollector(:,i) , stabilityRegion);
    hold off
    plot([0,position0(1),position1(1),position2(1),position3(1)],[0,position0(2),position1(2),position2(2),position3(2)])
    hold on
    plot(stabilityRegion,[0,0],'b')
    % Calculate the position of the end foot
    outputCross = cross( [position3(1) - position2(1);position3(2) - position2(2) ; 0], [0;0;1]);
    footdirection = [outputCross(1);outputCross(2)]/norm([outputCross(1);outputCross(2)],2);
    f1 = position3 + 0.15*footdirection;
    f2 = position3 - 0.15*footdirection;
    plot([f1(1),f2(1)],[f1(2),f2(2)],'b')
    
    % Plot the centre of mass line
    plot([COMPosition(1),COMPosition(1)],[COMPosition(2),0],'r')
    axis([-0.4 0.4,-0.05,0.8])
    drawnow();
    pause(0.1);
    frame = getframe(1);
    im{idx} = frame2im(frame);
    idx = idx + 1;
end

filename = 'slinky.gif'; % Specify the output file name
for newIdx = 1:idx-1
    [A,map] = rgb2ind(im{newIdx},256);
    if newIdx == 1
        imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.1);
    else
        imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.1);
    end
end