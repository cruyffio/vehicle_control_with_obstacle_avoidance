function [inputs] = ROB599_ControlsProject_part2_Team20(TestTrack,Xobs)

load('TestTrack.mat');
set(groot, 'defaultFigureVisible', 'off');
%% Organize boundaries and obstacles
% interpolate more points: increase points on boundaries

bp = 2;
t_size = size(TestTrack.bl(1,:),2);

blnewx = interp1(1:t_size, TestTrack.bl(1,:),linspace(1, t_size, 1000));
blnewy = interp1(1:t_size, TestTrack.bl(2,:),linspace(1, t_size, 1000));
brnewx = interp1(1:t_size, TestTrack.br(1,:),linspace(1, t_size, 1000));
brnewy = interp1(1:t_size, TestTrack.br(2,:),linspace(1, t_size, 1000));
    

posPoint = [brnewx' brnewy'];
% negPoint: left boundary
% negPoint = TestTrack.bl';
negPoint = [blnewx' blnewy'];

for i = 1:length(Xobs)
    obs = Xobs{1,i};
    % if obs belongs to br, add it to posPoint, and plot red
    % other add it to negPoint, and plot blue
    if (judgeObs(obs))
        posPoint = [posPoint; obs];
        obsP = [obs;obs(1,:)];
        %plot(obsP(:,1),obsP(:,2),'r'); hold on;
    else
        negPoint = [negPoint; obs];
        obsP = [obs;obs(1,:)];
        %plot(obsP(:,1),obsP(:,2),'b'); hold on;
    end
end
axis equal;

totalPoint = [posPoint; negPoint];
label = [ones(length(posPoint),1); -ones(length(negPoint),1)];

x = totalPoint;
y = label;
% [x, y] = plotObstacles();
%disp('obstacle generated');


%% Generate desired path based on boundaries and obstacles

svmmodel = fitcsvm(x,y,'KernelFunction','rbf', 'KernelScale','auto');

[x1Grid,x2Grid] = meshgrid(linspace(min(x(:,1)),max(x(:,1)),1000),...
    linspace(min(x(:,2)),max(x(:,2)),1000));
xGrid = [x1Grid(:),x2Grid(:)];
[~,scores] = predict(svmmodel,xGrid);

[C, ~] = contour(x1Grid,x2Grid,reshape(scores(:,2),size(x1Grid)),[0,0],'k');
% legend('Right Boundary','Left Boundary','Optimized Reference Path');
% xlabel('x/m')
% ylabel('y/m')
% hold on

p = find(C(1,:)==0, 1, 'last');
C_center = C(:,p+1:end);
C_center = C_center(:,end:-3:1);   % path points / 3
% plot(C_center(1,:), C_center(2,:), 'x');


%% Path Following for a Differential Drive Robot

path = C_center';


robotCurrentLocation = path(1,:);
robotGoal = path(end,:);

initialOrientation = 2;


robotCurrentPose = [287, -176, 2];
robotCurrentRate = [5, 0, 0];

robot = OurVehicle(robotCurrentPose, robotCurrentRate);


controller = vehPurePursuit;


controller.Waypoints = path;


controller.DesiredLinearVelocity = 5;


controller.MaxAngularVelocity = 0.5;


controller.LookaheadDistance = 7;



goalRadius = 0.5;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

i = 1;
state_hist = zeros(100000,6);
input_hist = zeros(100000,2);
while( distanceToGoal > goalRadius )
    state_hist(i,:) = [robot.CurrentPose, robot.CurrentRate];
    % Compute the controller outputs, i.e., the inputs to the robot
    [delta, Fx] = step(controller, [robot.CurrentPose, robot.CurrentRate]);
    input_hist(i,:) = [delta, Fx];
    
    % Simulate the robot using the controller outputs.
    drive(robot, delta, Fx)
    
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    robotCurrentLocation = robot.CurrentPose(1:2);
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentLocation - robotGoal);
    i = i+1;
end
state_hist = state_hist(1:i-1, :);
input_hist = input_hist(1:i-1, :);

inputs = input_hist;


end



function [disMin] = calShortDis(point, trackPoint)
    % point: 1x2
    % trackPoint: 2xN (one boundary)
    numPoint = length(trackPoint);
    disMin = 1000;
    for i = 1:numPoint
        dis = sqrt( (point(1)-trackPoint(1,i))^2 + (point(2)-trackPoint(2,i))^2 );
        if (dis < disMin)
            disMin = dis;
        end
    end
end

function dist = distance(pointA, pointB)
%distance Find euclidean distance between two points

dist = norm(pointA - pointB);
end


function [isRight] = judgeObs(obs)
    load TestTrack;

    dis_obs1 = calShortDis(obs(1,:),TestTrack.bl);
    dis_obs4 = calShortDis(obs(4,:),TestTrack.bl);
    % obs(2,:) and obs(3,:) are compared with br
    dis_obs2 = calShortDis(obs(2,:),TestTrack.br);
    dis_obs3 = calShortDis(obs(3,:),TestTrack.br);
    

    if ( (dis_obs1+dis_obs4)/2 > (dis_obs2+dis_obs3)/2 )
        isRight = 1;
    else
        isRight = 0;
    end
end








function thetaWrap = wrapTo2Pi(theta)


twoPiVal = cast(2*pi,'like',theta);

% Check if inputs are positive
pos = (theta > 0);

% Wrap to 2*pi
thetaWrap = mod(theta, twoPiVal);

% Make sure that positive multiples of 2*pi map to 2*pi
thetaWrap((thetaWrap == 0) & pos) = twoPiVal;

end

function theta = wrapToPi(theta)

piVal = cast(pi,'like',theta);
theta = wrapTo2Pi(theta + piVal) - piVal;

end