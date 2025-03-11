function pioneerMapping(mode)
% pioneerMapping Runs mapping experiments with the Pioneer P3DX.
%
%   pioneerMapping('manual')  - Runs manual teleoperation via blocking input().
%   pioneerMapping('auto')    - Runs autonomous wandering with obstacle avoidance.
%
%   This script builds an occupancy grid map using an inverse sensor model
%   by reading range data from the robot's built-in ultrasonic sensors.
%   The occupancy grid is displayed as a grayscale image in real time.
%   At the end, the script prints the area coverage percentage and the total
%   elapsed time.
%
%   Note: The robot is teleoperated via blocking input() commands in manual mode.

    % Validate mode argument
    if nargin < 1
        error('Please specify the mode as ''manual'' or ''auto''.');
    end

    %% Connection & Simulation Start
    % Create the RemoteAPIClient and connect to CoppeliaSim.
    client = RemoteAPIClient();  % (Custom interface for your simulation environment)
    sim = client.require('sim');
    sim.startSimulation();
    pause(1);  % Allow simulation to initialize 

    %% Retrieve Object Handles
    % Use absolute names as defined in the CoppeliaSim scene.
    robotHandle = sim.getObject('./Pioneer_p3dx');
    leftWheelHandle = sim.getObject('./Pioneer_p3dx_leftMotor');
    rightWheelHandle = sim.getObject('./Pioneer_p3dx_rightMotor');
    
    % Set initial velocities to zero so the robot is stationary at start.
    sim.setJointTargetVelocity(leftWheelHandle, 0);
    sim.setJointTargetVelocity(rightWheelHandle, 0);
    
    % Loop through and retrieve all ultrasonic sensor handles (assuming sensors named sequentially)
    numSensors = 16;
    ultrasonicHandles = cell(1, numSensors);
    for i = 1:numSensors
        sensorName = sprintf('./Pioneer_p3dx_ultrasonicSensor%d', i);
        ultrasonicHandles{i} = sim.getObject(sensorName);
    end

    %% Set Up Occupancy Grid Map
    % Define grid parameters and initialize occupancy grid (using log-odds).
    gridResolution = 0.1;  % Cell size in meters
    gridWidth = round(10 / gridResolution);   % 10 m length -> ~100 cols
    gridHeight = round(5 / gridResolution);     % 5 m width -> ~50 rows
    occupancyGrid = zeros(gridHeight, gridWidth);  % Initialize all unknown (0 log-odds)

    % Create a figure (GUI) to display the occupancy grid map.
    hFigMap = figure('Name', 'Occupancy Grid Map', 'NumberTitle', 'off');

    %% Start Timer for Quantitative Analysis
    tStart = tic;

    %% Main Loop: Behavior Mode Selection
    switch lower(mode)
        case 'manual'
            % Manual Mode: Teleoperation via blocking input().
            disp('Manual Mode: Use keys w (forward), s (backward), a (smooth left turn), d (smooth right turn), q (quit).');
            while true
                key = lower(input('Enter command (w/s/a/d/q): ', 's'));
                if strcmp(key, 'q')
                    break;
                end
                % Process movement commands
                switch key
                    case 'w'  % Move forward one step
                        sim.setJointTargetVelocity(leftWheelHandle, 1);
                        sim.setJointTargetVelocity(rightWheelHandle, 1);
                    case 's'  % Move backward one step
                        sim.setJointTargetVelocity(leftWheelHandle, -1);
                        sim.setJointTargetVelocity(rightWheelHandle, -1);
                    case 'a'  % Smooth left turn one step
                        sim.setJointTargetVelocity(leftWheelHandle, 0.5);
                        sim.setJointTargetVelocity(rightWheelHandle, 1);
                    case 'd'  % Smooth right turn one step
                        sim.setJointTargetVelocity(leftWheelHandle, 1);
                        sim.setJointTargetVelocity(rightWheelHandle, 0.5);
                    otherwise
                        % For unrecognized commands, stop the robot.
                        sim.setJointTargetVelocity(leftWheelHandle, 0);
                        sim.setJointTargetVelocity(rightWheelHandle, 0);
                end
                pause(0.1);  % Allow movement to take effect

                % Stop the robot after a single movement step
                sim.setJointTargetVelocity(leftWheelHandle, 0);
                sim.setJointTargetVelocity(rightWheelHandle, 0);
                
                % Update occupancy grid mapping
                robotPose = getRobotPose(sim, robotHandle);
                sensorData = getUltrasonicData(sim, ultrasonicHandles);
                occupancyGrid = updateOccupancyGrid(occupancyGrid, robotPose, sensorData, gridResolution);

                % Display the updated occupancy grid in the GUI
                figure(hFigMap);
                imagesc(occupancyGrid);
                colormap('gray');
                axis equal tight;
                title('Occupancy Grid Map (Manual Mode)');
                drawnow;  
            end

        case 'auto'
            % Autonomous Mode: The robot wanders and avoids obstacles.
            disp('Autonomous Mode: Robot will wander for 60 seconds.');
            duration = 60;  % Duration in seconds.
            while toc(tStart) < duration
                % Read sensor data
                sensorData = getUltrasonicData(sim, ultrasonicHandles);

                % Check front sensors for obstacles.
                % Adjust the angle threshold if needed (currently ±pi/6).
                frontIndices = find(abs(sensorData.angles) < (pi/6));
                frontRanges = sensorData.ranges(frontIndices);
                
                % Debug: display front sensor ranges.
                disp(['Front sensor ranges: ', num2str(frontRanges)]);
                
                % Set an obstacle detection threshold.
                obstacleThreshold = 1.0;  % Increase if necessary
                
                if any(frontRanges < obstacleThreshold)
                    disp('Obstacle detected. Turning right.');
                    % Obstacle detected: perform a smooth right turn.
                    sim.setJointTargetVelocity(leftWheelHandle, 1);
                    sim.setJointTargetVelocity(rightWheelHandle, 0.5);
                else
                    disp('Path clear. Moving forward.');
                    % Clear path: move forward at half speed.
                    sim.setJointTargetVelocity(leftWheelHandle, 1);
                    sim.setJointTargetVelocity(rightWheelHandle, 1);
                end
                pause(0.1);

                % Update occupancy grid mapping
                robotPose = getRobotPose(sim, robotHandle);
                occupancyGrid = updateOccupancyGrid(occupancyGrid, robotPose, sensorData, gridResolution);

                % Debug: display robot pose (location and orientation)
                disp(['Robot Pose: x = ' num2str(robotPose(1)) ', y = ' num2str(robotPose(2)) ', theta = ' num2str(robotPose(3))]);

                % Display the updated occupancy grid
                figure(hFigMap);
                imagesc(occupancyGrid);
                colormap('gray');
                axis equal tight;
                title('Occupancy Grid Map (Autonomous Mode)');
                drawnow;
            end

        otherwise
            error('Unknown mode. Use ''manual'' or ''auto''.');
    end

    %% Quantitative Analysis and Simulation Stop
    tElapsed = toc(tStart);
    % Calculate area coverage: count cells with nonzero log-odds
    mappedCells = sum(sum(abs(occupancyGrid) > 0));
    totalCells = numel(occupancyGrid);
    coveragePercent = (mappedCells / totalCells) * 100;
    fprintf('Mapping completed.\nArea coverage: %.1f%%\nTime elapsed: %.1f seconds\n', coveragePercent, tElapsed);

    sim.stopSimulation();
    disp('Simulation stopped.');
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% getRobotPose
% Returns the robot's position and orientation (yaw) in world coordinates.
function robotPose = getRobotPose(sim, robotHandle)
    pos = sim.getObjectPosition(robotHandle, -1);     % World position
    orient = sim.getObjectOrientation(robotHandle, -1);  % World orientation (Euler angles)
    if iscell(pos)
        pos = cell2mat(pos);
    end
    if iscell(orient)
        orient = cell2mat(orient);
    end
    robotPose = [pos(1), pos(2), orient(3)];  % Return [x, y, theta]
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% getUltrasonicData
% Reads range data from each ultrasonic sensor and assigns each a beam angle.
%   maxRange is used if no obstacle is detected.
function sensorData = getUltrasonicData(sim, ultrasonicHandles)
    numSensors = length(ultrasonicHandles);
    ranges = zeros(1, numSensors);
    angles = zeros(1, numSensors);
    maxRange = 10;  % Maximum range if no detection
    for i = 1:numSensors
        [~, detectionState, detectedPoint, ~, ~] = sim.readProximitySensor(ultrasonicHandles{i});
        if detectionState
            if iscell(detectedPoint)
                detectedPoint = cell2mat(detectedPoint);
            end
            ranges(i) = norm(detectedPoint);
        else
            ranges(i) = maxRange;
        end
        % Assign angles uniformly over 360°
        angles(i) = -pi + (i - 0.5) * (2*pi/numSensors);
    end
    sensorData.ranges = ranges;
    sensorData.angles = angles;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% updateOccupancyGrid
% Updates the occupancy grid using the inverse sensor model.
%   Cells along each sensor beam are decremented for free space
%   and the last cell is incremented for an obstacle.
function occupancyGrid = updateOccupancyGrid(occupancyGrid, robotPose, sensorData, gridResolution)
    freeUpdate = -1;   % Log-odds decrement for free space
    occUpdate = 2;     % Log-odds increment for an obstacle
    maxSensorRange = 5;  % Maximum sensor range (m)
    for i = 1:length(sensorData.ranges)
        currentAngle = double(sensorData.angles(i));
        theta = double(robotPose(3));
        angle = theta + currentAngle;
        range = min(sensorData.ranges(i), maxSensorRange);
        xEnd = robotPose(1) + range * cos(angle);
        yEnd = robotPose(2) + range * sin(angle);
        [iRobot, jRobot] = worldToGrid(robotPose(1), robotPose(2), gridResolution, occupancyGrid);
        [iEnd, jEnd] = worldToGrid(xEnd, yEnd, gridResolution, occupancyGrid);
        rayIndices = bresenham(iRobot, jRobot, iEnd, jEnd);
        for k = 1:(size(rayIndices,1)-1)
            occupancyGrid(rayIndices(k,1), rayIndices(k,2)) = ...
                occupancyGrid(rayIndices(k,1), rayIndices(k,2)) + freeUpdate;
        end
        occupancyGrid(iEnd, jEnd) = occupancyGrid(iEnd, jEnd) + occUpdate;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% worldToGrid
% Converts world coordinates (x, y) into grid indices.
function [i, j] = worldToGrid(x, y, resolution, grid)
    j = round(x / resolution) + 1;
    i = round(y / resolution) + 1;
    i = min(max(i, 1), size(grid, 1));
    j = min(max(j, 1), size(grid, 2));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% bresenham
% Implements Bresenham's line algorithm to determine grid cells along a line.
function indices = bresenham(x1, y1, x2, y2)
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    sx = sign(x2 - x1);
    sy = sign(y2 - y1);
    err = dx - dy;
    x = x1;
    y = y1;
    indices = [];
    while true
        indices = [indices; x, y]; %#ok<AGROW>
        if (x == x2) && (y == y2)
            break;
        end
        e2 = 2 * err;
        if e2 > -dy
            err = err - dy;
            x = x + sx;
        end
        if e2 < dx
            err = err + dx;
            y = y + sy;
        end
    end
end
