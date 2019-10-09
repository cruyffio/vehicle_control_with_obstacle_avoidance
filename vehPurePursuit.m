classdef vehPurePursuit < matlab.System
    
    properties (Nontunable)
        
        Waypoints
    end
    
    properties
        
        MaxAngularVelocity      = 1.0
        
       
        LookaheadDistance       = 1.0
        

        DesiredLinearVelocity   = 0.1
    end
    
    properties (Access = private)
        %Path X-Y coordinates of finely discretized waypoints
        Path
        
        %ProjPointIdx Index of a point on path closest to the current pose
        ProjPointIdx
        
        %DeltaDistance Distance between discretized waypoints
        DeltaDistance
        
        %LookaheadPoint Coordinates of the lookahead point
        LookaheadPoint
        
        %LastPose Last pose used to compute robot velocity commands
        LastPose
    end
    
    methods
        function set.Waypoints(obj, waypts)
            %set.Waypoints Setter for Waypoints property
            
            validateattributes(waypts, {'double'},{'finite', 'real','ncols',2}, ...
                'PurePursuit', 'Waypoints');
            
            isInvalidNumOfWaypoints = size(waypts,1) < 2;
            coder.internal.errorIf(isInvalidNumOfWaypoints, ...
                'robotics:robotalgs:purepursuit:NeedTwoWaypoints', 'waypoints');
            obj.Waypoints = waypts;
        end
        
        function set.MaxAngularVelocity(obj, omega)
            %set.MaxAngularVelocity Setter for Waypoints property
            
            validateattributes(omega, {'double'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty'}, 'PurePursuit', 'MaxAngularVelocity');
            obj.MaxAngularVelocity = omega;
        end
        
        function set.LookaheadDistance(obj, dist)
            %set.LookaheadDistance Setter for LookaheadDistance property
            
            validateattributes(dist, {'double'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty'},'PurePursuit', 'LookaheadDistance');
            obj.LookaheadDistance = dist;
        end
        
        function set.DesiredLinearVelocity(obj, val)
            %set.DesiredLinearVelocity Setter for DesiredLinearVelocity
            
            validateattributes(val, {'double'},{'nonnan', 'real', 'scalar', ...
                'positive', 'nonempty'},'PurePursuit', 'DesiredLinearVelocity');
            obj.DesiredLinearVelocity = val;
        end
        
        function obj = PurePursuit(varargin)
            %PurePursuit Constructor
            setProperties(obj,nargin,varargin{:},'Waypoints', ...
                'DesiredLinearVelocity', 'MaxAngularVelocity', ...
                'LookaheadDistance');
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj)
            %setupImpl Construct discretized waypoints
            
            coder.internal.errorIf(isempty(obj.Waypoints), ...
                'robotics:robotalgs:purepursuit:EmptyWaypoints')
            

            [xd, yd, delta] = ...
                equidistantPoints(obj.Waypoints);
            obj.Path = [xd(:) yd(:)];
            obj.DeltaDistance = delta;
            obj.ProjPointIdx = 0;
            obj.LookaheadPoint = zeros(1,2);
            obj.LastPose = zeros(1,3);
        end
        
        function [delta, Fx] = stepImpl(obj,curPose_Rate)
            %stepImpl Compute control commands
            curPose = curPose_Rate(1:3);
            curRate = curPose_Rate(4:6);
            validateattributes(curPose, {'double'}, {'nonnan', 'real','vector', 'numel',3}, ...
                'step', 'pose');
            
            %Update the last pose
            obj.LastPose = [curPose(1) curPose(2) curPose(3)];
            
            % Compute lookahead index increment
            lookAheadIdxDist = round(obj.LookaheadDistance/obj.DeltaDistance);

            if obj.ProjPointIdx == 0
                obj.ProjPointIdx = findNearest(obj, curPose);
            else
                % Search for nearest point only in few next
                % points
                updateProj(obj, curPose, lookAheadIdxDist);
            end
            

            lookAheadIdx = obj.ProjPointIdx + lookAheadIdxDist;
            if (lookAheadIdx < size(obj.Path, 1))
                obj.LookaheadPoint = obj.Path(lookAheadIdx, :);
            else
                obj.LookaheadPoint = obj.Path(end, :);
            end
            

            a = 1.35;           b = 1.45;
            L = a + b;
            lfw = b;
            Lfw = sqrt((obj.LookaheadPoint(2) - curPose(2))^2 + (obj.LookaheadPoint(1) - curPose(1))^2);
            slope = atan2((obj.LookaheadPoint(2) - curPose(2)), ...
                (obj.LookaheadPoint(1) - curPose(1)));
            alpha = angdiff(curPose(3), slope);
            delta = -atan2(-L*sin(alpha), Lfw/2+lfw*cos(alpha));
            
            if abs(abs(alpha) - pi) < 1e-12
                delta = sign(delta)*1;
            end
            delta = obj.saturate(delta);
            
            
            W = 13720;      Nw = 2;f = 0.01;   m = 1400;
            v = curRate(2); r = curRate(3);
            Fx = (f*W - v*r*m)/Nw;
        end
        
        function dat = infoImpl(obj)

            dat.RobotPose = obj.LastPose;
            dat.LookaheadPoint = obj.LookaheadPoint;
        end
        
        function num = getNumInputsImpl(~)
            num = 1;
        end
        
        function num = getNumOutputsImpl(~)
            num = 2;
        end
        
        function s = saveObjectImpl(obj)

            s = saveObjectImpl@matlab.System(obj);
            
            s.Path = obj.Path;
            s.ProjPointIdx = obj.ProjPointIdx;
            s.DeltaDistance = obj.DeltaDistance;
            s.LookaheadPoint = obj.LookaheadPoint;
            s.LastPose = obj.LastPose;
            
            if isLocked(obj)
                s.Waypoints = obj.Waypoints;
            end
        end
        
        function loadObjectImpl(obj, svObj, wasLocked)
            %loadObjectImpl Custom load implementation
            
            obj.Path = svObj.Path;
            obj.ProjPointIdx = svObj.ProjPointIdx;
            obj.DeltaDistance = svObj.DeltaDistance;
            obj.LookaheadPoint = svObj.LookaheadPoint;
            obj.LastPose = svObj.LastPose;
            
            % Load state only if locked when saved
            if wasLocked
                obj.Waypoints = svObj.Waypoints;
            end
            
            % Call base class method
            loadObjectImpl@matlab.System(obj,svObj,wasLocked);
        end
        
        function resetImpl(obj)
            %resetImpl Reset the internal state to defaults
            obj.ProjPointIdx = 0;
            obj.LookaheadPoint = zeros(1,2);
            obj.LastPose = zeros(1,3);
        end
        
    end
    
    methods (Access = private)
        function avel = saturate(obj, avel)
            %saturate Saturate angular velocity
            
            if abs(avel) > obj.MaxAngularVelocity
                avel = sign(avel)*obj.MaxAngularVelocity;
            end
        end
        
        function nearIdx = findNearest(obj, pose)
            %findNearest Find closest point in the entire path
            dist = sqrt((obj.Path(:,1) - pose(1)).^2 + ...
                (obj.Path(:,2) - pose(2)).^2);
            [~,nearIdx] = min(dist);
        end
        
        function updateProj(obj, pose, lookAheadIdxDist)
            %updateProj Find closest point in next few points
            
            searchDist = 2*lookAheadIdxDist;
            if (obj.ProjPointIdx + searchDist < size(obj.Path, 1))
                lookForward = obj.ProjPointIdx + searchDist;
            else
                lookForward = size(obj.Path, 1);
            end
            
            dist = sqrt((obj.Path(obj.ProjPointIdx:lookForward,1) - pose(1)).^2 + ...
                (obj.Path(obj.ProjPointIdx:lookForward,2) - pose(2)).^2);
            
            [~,curProjection] = min(dist);
            
            obj.ProjPointIdx = obj.ProjPointIdx + curProjection - 1;
        end
    end
end


function [x, y, delta] = equidistantPoints(origpath)

coder.varsize('x');
coder.varsize('y');

% Initialize empty array
x = origpath(1,1);
y = origpath(1,2);
delta = 0;

% Find minimum distance between points
if numel(origpath) == 2
    return;
end

% Compute minimum delta distance based on input points
dOrigPath = diff(origpath);
distOrigPath = sqrt(dOrigPath(:,1).^2 + dOrigPath(:,2).^2);
sortedDist = sort(distOrigPath);
sortedDist = sortedDist(sortedDist > 0);

minimumDelta = 0.0001*mean(sortedDist);
maximumDelta = 0.005*mean(sortedDist);

% Remove points that are closer than minimumDelta distance
pathTemp = origpath;

j = 1;
for i = 1:size(origpath, 1)-1
    if distance(pathTemp(j,:), origpath(i+1,:)) >= minimumDelta
        j = j+1;
        pathTemp(j, :) = origpath(i+1,:);
    end
end

% Extracting path
path = pathTemp(1:j,:);

% Check one more time after removing points from the path
if numel(path) == 2
    return;
end

dPath = diff(path);
distPath = sqrt(dPath(:,1).^2 + dPath(:,2).^2);

minPathDist = min(distPath);

% Maximum delta
maxDelta = min(maximumDelta, minPathDist);

% Final delta distance between all equidistant points
delta = max(minimumDelta, maxDelta);

% Loop over pair of points to compute discretized points in between
for i = 1:size(path,1)-1
    % If X coordinate is not the same
    if ~isEqualFloat(path(i,1), path(i+1,1))
        xcomputed = linspace(path(i,1), path(i+1,1), ceil(norm(path(i,:)-path(i+1,:))/delta));
        ycomputed = linspace(path(i,2), path(i+1,2),numel(xcomputed));
        
        % If Y coordinate is not the same
    elseif ~isEqualFloat(path(i,2), path(i+1,2))
        ycomputed = linspace(path(i,2), path(i+1,2), ceil(norm(path(i,:)-path(i+1,:))/delta));
        xcomputed = linspace(path(i,1), path(i+1,1), numel(ycomputed));
    else
        xcomputed = path(i,1);
        ycomputed = path(i,2);
    end
    
    if isEqualFloat(x(end), xcomputed(1)) && isEqualFloat(y(end), ycomputed(1))
        x = [x(1:end,1); xcomputed(2:end)'];
        y = [y(1:end,1); ycomputed(2:end)'];
    else
        x = [x(1:end,1); xcomputed'];
        y = [y(1:end,1); ycomputed'];
    end
end
end

function flag = isEqualFloat(a, b)

% Comparison tolerance
tolerance = eps(a);
flag = abs(a - b) < tolerance;
end

function delta = angdiff(x, y)

if nargin == 1

    d = diff(x);
else
  
    
    d = y - x;
end


delta = wrapToPi(d);

end

function d = distance(a,b)
% DISTANCE - computes Euclidean distance matrix
%
% E = distance(A,B)
%
%    A - (DxM) matrix 
%    B - (DxN) matrix
%
% Returns:
%    E - (MxN) Euclidean distances between vectors in A and B
%
%
% Description : 
%    This fully vectorized (VERY FAST!) m-file computes the 
%    Euclidean distance between two vectors by:
%
%                 ||A-B|| = sqrt ( ||A||^2 + ||B||^2 - 2*A.B )
%
% Example : 
%    A = rand(400,100); B = rand(400,200);
%    d = distance(A,B);
% Author   : Roland Bunschoten
%            University of Amsterdam
%            Intelligent Autonomous Systems (IAS) group
%            Kruislaan 403  1098 SJ Amsterdam
%            tel.(+31)20-5257524
%            bunschot@wins.uva.nl
% Last Rev : Oct 29 16:35:48 MET DST 1999
% Tested   : PC Matlab v5.2 and Solaris Matlab v5.3
% Thanx    : Nikos Vlassis
% Copyright notice: You are free to modify, extend and distribute 
%    this code granted that the author of the original code is 
%    mentioned as the original author of the code.
if (nargin ~= 2)
   error('Not enough input arguments');
end
if (size(a,1) ~= size(b,1))
   error('A and B should be of same dimensionality');
end
aa=sum(a.*a,1); bb=sum(b.*b,1); ab=a'*b; 
d = sqrt(abs(repmat(aa',[1 size(bb,2)]) + repmat(bb,[size(aa,2) 1]) - 2*ab));
end