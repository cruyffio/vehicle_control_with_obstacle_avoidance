classdef OurVehicle < handle
   
    
    properties(SetAccess = public)
        %Dt Time step for simulation of the robot
        Dt = 0.01;
        CurrentRate;
    end
    
    properties(SetAccess = private)
        %CurrentPose Current pose of the robot
        CurrentPose;
        
        %Trajectory Pose history of the robot
        Trajectory = [];
    end
    
    properties(Access = private)
        %HeadingVectorLength Length of the heading vector for plotting
        HeadingVectorLength = 0.3;
        
        %HRobot Graphics handle for robot
        HRobot;
        
        %HRobotHeading Graphics handle for robot heading
        HRobotHeading;
        
        %HRobotTrail Graphics handle for robot pose history
        HRobotTrail;
    end
    
    methods
        function obj = OurVehicle(currentPose, CurrentRate)
            %ExampleHelperDifferentialDriveRobot Constructor
            
            obj.CurrentRate = CurrentRate;
            obj.CurrentPose = currentPose;
            robotCurrentLocation = obj.CurrentPose(1:2);
            
            obj.Trajectory = obj.CurrentPose;
            
            % Get the current axes
            ax = gca;
            hold(ax, 'on');
            
            % Get robot x, y data for plotting
            [x, y] = obj.getCircleCoordinate(robotCurrentLocation, obj.HeadingVectorLength);
            
            % Draw robot
            obj.HRobot = fill(x,y, 'r', 'Parent', ax);
            
            % Draw heading vector
            [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);

            
            hold(ax, 'off');
        end
        
        function drive(obj, delta, Fx)

            curPose = obj.CurrentPose;
            curRate = obj.CurrentRate;
            x_state = [curPose(1), curRate(1), curPose(2), curRate(2), curPose(3), curRate(3)];
            xdot = veh_nl_dyn_robo(x_state, delta, Fx);
            x = curPose(1) + xdot(1)*obj.Dt;
            y = curPose(2) + xdot(3)*obj.Dt;
            theta = curPose(3) + xdot(5)*obj.Dt;
            curRate(1) = curRate(1) + xdot(2)*obj.Dt;
            curRate(2) = curRate(2) + xdot(4)*obj.Dt;
            curRate(3) = curRate(3) + xdot(6)*obj.Dt;
            obj.CurrentPose = [x, y, theta];
            obj.CurrentRate = curRate;
            obj.Trajectory = [obj.Trajectory; obj.CurrentPose];
%             updatePlots(obj);
        end
    end
    
    methods(Access = private)
        function updatePlots(obj)
            %updatePlots Update all plots
            
            updateHeading(obj);
            updateRobotCurrentLocation(obj);
            updateRobotTrail(obj);
            drawnow;
        end
        function updateHeading(obj)

            [xVec, yVec] = obj.computeHeadingVector(obj.CurrentPose, obj.HeadingVectorLength);

        end
        function updateRobotCurrentLocation(obj)

            [robotXData, robotYData] = obj.getCircleCoordinate(obj.CurrentPose, obj.HeadingVectorLength);
            obj.HRobot.XData =  robotXData;
            obj.HRobot.YData =  robotYData;
        end
        function updateRobotTrail(obj)
            %updateRobotTrail Update the X/YData for pose history
            robotCurrentLocation = obj.CurrentPose(1:2);

        end
    end
    
    methods(Access = private, Static)
        function [xVec, yVec] = computeHeadingVector(robotCurrentPose, robotDiameter)
            %computeHeadingVector Compute XData and YData for heading vector
            
            robotCurrentLocation = robotCurrentPose(1:2);
            cth = cos(robotCurrentPose(3));
            sth = sin(robotCurrentPose(3));
            
            dx = robotDiameter * cth;
            dy = robotDiameter * sth;
            
            xVec = [ robotCurrentLocation(1)   robotCurrentLocation(1)+dx];
            yVec = [ robotCurrentLocation(2)   robotCurrentLocation(2)+dy];
        end
        
        function [x, y] = getCircleCoordinate(pose, r)
            
            theta = linspace(0,2*pi,20);
            x = r*cos(theta) + pose(1);
            y = r*sin(theta) + pose(2);
        end
    end
end

function xdot = veh_nl_dyn_robo(x, delta, Fx)
xdot = zeros(6,1);

m = 1400;       W = 13720;      Nw = 2;
f = 0.01;       Iz = 2667;      a = 1.35;           b = 1.45;
By = 0.27;      Cy = 1.2;       Dy = 2921;          Ey = -1.6;
Shy = 0;        Svy = 0;


    
    xpos = x( 1 );
%     xpos_new = x( i*x_dim+1 );
    u = x( 2 );
%     u_new = x( i*x_dim+2 );
    ypos = x( 3 );
%     ypos_new = x( i*x_dim+3 );
    v = x( 4 );
%     v_new = x( i*x_dim+4 );
    phi = x( 5 );
%     phi_new = x( i*x_dim+5 );
    r = x( 6 );
%     r_new = x( i*x_dim+6 );
    
alpha_f = ( delta - atan( (v+a*r)/u ) )/pi*180;           % radian to degree
alpha_r = ( -atan( (v-b*r)/u ) )/pi*180;                  % radian to degree
phi_yf = (1-Ey)*(alpha_f+Shy)+Ey/By*atan(By*(alpha_f+Shy));
phi_yr = (1-Ey)*(alpha_r+Shy)+Ey/By*atan(By*(alpha_r+Shy));
Fyf = Dy*sin(Cy*atan(By*phi_yf))+Svy;
Fyr = Dy*sin(Cy*atan(By*phi_yr))+Svy;

xdot(1) = (u*cos(phi)-v*sin(phi));
xdot(2) = ( 1/m * ( -f*W + Nw*Fx - Fyf*sin(delta) ) + v*r );
xdot(3) = (u*sin(phi)+v*cos(phi));
xdot(4) = ( 1/m * ( Fyf*cos(delta) + Fyr ) - u*r );
xdot(5) = r;
xdot(6) = ( 1/Iz * ( a*Fyf*cos(delta) - b*Fyr ) );
end
