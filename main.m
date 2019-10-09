%% main to run part 2 code
load TestTrack;

x0 = [287,5,-176,0,2,0];
Nobs = 25;

Xobs = generateRandomObstacles(Nobs,TestTrack);
inputs = ROB599_ControlsProject_part2_Team20(TestTrack,Xobs);
[Y] = forwardIntegrateControlInput(inputs,x0);

figure(1);
hold on;
plot(Y(:,1),Y(:,3),'r', 'LineWidth', 1.5);

%% Check
checkTrajectory(Y,inputs,Xobs,TestTrack)