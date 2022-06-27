% Run setup.m before running this script

robot_map = createMap();
position_tolerance = 85;
goal_tolerance = 100;
count = 1;
 
%Read initial orrientation data from robot
pose = readPose(c_esp)

% Write starting coordinates to the robot 
start_point = getPosition(cam);
writePose(c_esp, start_point(1)*1000, start_point(2)*1000, pose(3), start_point(1)*1000, start_point(2)*1000, pose(3), 0);

% Generate path trajectory
path = floor(getPathMatrix(start_point(1), start_point(2), pi/2, robot_map));
pose = readPose(c_esp);

% Check if robot is in goal position
if( (abs(pose(1)- 200) < goal_tolerance) && (abs(pose(2)- 200) < goal_tolerance) )
    goal_reached = 1;
    disp("Goal Reached!!")
else
    goal_reached = 0;
    node_reached = 0;
    writePose(c_esp, pose(1), pose(2), pose(3), pose(4), pose(5), path(count,3), 0);
end

pose = readPose(c_esp)
% Begin path following algorithm
while(~goal_reached)

    
    while(~node_reached)
        
        position = getPosition(cam);
        pose(1) = 1000*position(1);
        pose(2) = 1000*position(2);
        writePose(c_esp, pose(1), pose(2), pose(3), path(count,1), path(count,2), pose(6), 1);

        if((abs(pose(1) - pose(4)) < position_tolerance) && (abs(pose(2) - pose(5)) < position_tolerance) )
            node_reached = 1;
            disp("Node Reached!!")
            if( (abs(pose(1) - 200) < goal_tolerance) && (abs(pose(2) - 200) < goal_tolerance) )
                goal_reached = 1;
                node_reached = 1;
                disp("Goal Reached!!")
                writePose(c_esp, 0, 0, 0, 0, 0, 0, 3);
            else
                % Update Path Trajectory
                disp("Trajectory Update!!")
                count = count + 1;
                writePose(c_esp, pose(1), pose(2), pose(3), path(count,1), path(count,2), path(count-1,3), 0);
                pose = readPose(c_esp)
            end
        end

    end
    node_reached = 0;

end

% ----------------------------------------------------------
% ---------------------- Functions -------------------------
% ----------------------------------------------------------

% --- 
% --- RRT algorithm returns trajectory in the from of a pose matrix
% --- 
function states = getPathMatrix(x_0, y_0, theta_0, map)
    clear path;
    % Set start and goal poses
    start = [x_0, y_0, theta_0];
    goal = [0.2, 0.2, 0];
    
    % Show start and goal positions of robot.
    hold on
    plot(start(1),start(2),'ro')
    plot(goal(1),goal(2),'mo')
    
    % Show start and goal headings.
    r = 0.1;
    plot([start(1),start(1) + r*cos(start(3))],[start(2),start(2) + r*sin(start(3))],'r-')
    plot([goal(1),goal(1) + r*cos(goal(3))],[goal(2),goal(2) + r*sin(goal(3))],'m-')
    hold off
    
    % Define State Space
    bounds = [[-1.15 1.15]; [-0.57 0.57]; [-pi pi]];
    ss = stateSpaceDubins(bounds);
    ss.MinTurningRadius = 0.001;
    
    %Define Planner
    stateValidator = validatorOccupancyMap(ss); 
    stateValidator.Map = map;
    stateValidator.ValidationDistance = 0.001;
    
    planner = plannerRRT(ss,stateValidator);
    planner.MaxConnectionDistance = 0.175;
    planner.MaxIterations = 60000;
    
    planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
    
    %Plan the path between the start and goal. Reset the random number generator for reproducible results.
    rng default
    
    [pthObj, solnInfo] = plan(planner,start,goal);
    
    %Show the occupancy map. Plot the search tree from the solnInfo. Interpolate and overlay the final path.
    
    show(map)
    hold on
    
    % Plot entire search tree.
    %plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-');
    
    % Interpolate and plot path.
    interpolate(pthObj,300)
    plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2)
    
    % Show start and goal in grid map.
    plot(start(1),start(2),'ro')
    plot(goal(1),goal(2),'mo')
    hold off

    states = pthObj.States;
    
    % Convert Pose orrientation value to degrees
    states(:,3) = 57.2957795131*states(:,3);
    
    % --- Sort repeated orrientation values (Straight line)
    count = 1;
    newCell = 1;
    lastState = 0;
    newStates = [];
    for i = 1:300
        
        % Defaults pose [-pi/2 pi/2], change to -> [0 360]
        if states(i,3) < 0
           states(i,3) = states(i,3) + 360;
        end

        if states(i,3) == lastState
           count = count + 1;
        else
           newStates(newCell,3) = states(i,3);
           newStates(newCell,2) = states(i,2);
           newStates(newCell,1) = states(i,1);
           count = 1;
           newCell = newCell + 1;
        end
        
        lastState = states(i,3);
 
    end    

    states = newStates;
    states = [1000*states(:,1)  1000*states(:,2)  states(:,3)];
    [m , n] = size(states);
    states(m+1,1) = 200;
    states(m+1,2) = 200;
    states(m+1,3) = 0;
    disp(states);
    [m , n] = size(states);

    % Correct the angle of the robot pose for point to point movement
    for i = 1:(m-1)
        states(i,3) = floor(atan2d(states(i+1,2)-states(i,2),states(i+1,1)-states(i,1)));
        if(states(i,3) < 0)
            states(i,3) = 360 + states(i,3);
        end
    end

    % Filter oscillatory movements
    for i=2:(m-1)
        ang_1 = states(i-1,3);
        ang_2 = states(i,3);
        ang_3 = states(i+1,3);
        if(ang_1 > ang_2 && ang_2 < ang_3)
            states(i,3) = mean([ang_3 ang_1]);
        end
    end
end

% --- 
% ---  Goal is reached
% --- 
function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.1;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end

% --- 
% --- Create Map
% --- 
function cm = createMap()
    w = 13;     % width of the walls   (mm)
    x = 1150;   % length of the x-axis (mm)
    y = 570;    % length of the y-axis (mm)
    
    total_wall = 7*w;
    cell_size = (1150-total_wall)/6;
    
    a = zeros(y,x);         % Map size
    a(1:w,:)=1;             % Top wall
    a((y-w):y,:)=1;         % Bottom wall 
    a(:,1:w)=1;             % Left wall
    a(:,(x-w):x)=1;         % Right wall
    
    % Other walls
    home = 2*w + 2*cell_size; 
    a(floor(w+cell_size):y, home:floor(home+w))=1; 
    a(floor(w+cell_size):floor(2*w+cell_size),floor(home-cell_size):floor(home+w)) = 1;
    a(1:floor(w+cell_size),floor(3*(w+cell_size)):floor(3*(w+cell_size))+w)=1;
    a(floor(2*(cell_size+w)):y,floor(3*(w+cell_size)):floor(3*(w+cell_size))+w)=1;
    a(1:floor(2*(w+cell_size)),floor(4*(w+cell_size)):floor(4*(w+cell_size))+w)=1;
    a(floor(w+cell_size):floor(2*(w+cell_size))+w,floor(5*(w+cell_size)):floor(5*(w+cell_size))+w)=1;
    a(floor(2*(w+cell_size)):floor(2*(w+cell_size))+w, floor(6*w+5*cell_size):x)=1;
    
    cm = occupancyMap(a,'Resolution', 1000);
end

% --- 
% --- Write Pose Data to Robot
% --- 
function writePose(c_esp, x_0, y_0, theta_0, x_f, y_f, theta_f, RW)
    writeData = [num2str(x_0) '.' num2str(y_0) '.' num2str(theta_0) '.' num2str(x_f) '.' num2str(y_f) '.' num2str(theta_f) '.' num2str(RW) '.'];
    write(c_esp, writeData);
end

% --- 
% --- Read Pose Data from Robot
% --- 
function pose = readPose(c_esp)
    readData = char(read(c_esp));
    data = split(readData,'.');
    for i=1:7 
        pose(i) = str2num(char(data(i)));
    end
end

% --- 
% --- Integer map
% --- 
function f = map(x, in_min, in_max, out_min, out_max)
    f = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;    
end

% --- 
% --- Get Robot Position
% --- 
function robotPosition = getPosition(cam)
% Conditional variables to run image capture loop
    run_axis = 1;
    run_robot = 1;
    count = 0;

    %Image capture and Circle detection
    while(run_axis || run_robot)
        %The raspberry pi camera requires 5 sucessive image captures for an snapshot() update
        for k = 1:5
            smp = snapshot(cam);
        end
        
        % Coordinate axis circle detection using Hough Transform
        [centers,radii] = imfindcircles(smp,[8 13],'ObjectPolarity','dark', ...
            'Sensitivity',0.90);
        % Test that the three coordinate circle markers were located on the image
        temp = size(centers);
        if temp(1) == 3
            run_axis = 0;
        end
            
        % Robot circle detection using Hough Transform
        [centersRobot,radiiRobot] = imfindcircles(smp,[14 20],'ObjectPolarity','dark', ...
            'Sensitivity',0.90);
        % Test that the robots marker was located on the image
        temp = size(centersRobot);
        if temp(1) == 1
            run_robot = 0;
        end

        count = count + 1; 
        if(count > 10)
            disp("ERROR: Robot not Found");
            break;
        end
    end

    % Sort the values of the found circles to establish coordinates
    testx = sort(centers(:,1));
    testy = sort(centers(:,2));

    % max X value is the bottom right mapMarker
    %MM_bottom_right = [centers(find(centers == testx(3))) centers(find(centers == testx(3)), 2)];

    % min Y value is the top left mapMarker
    %MM_top_left = [centers(find(centers == testy(1)) - 3) centers(find(centers == testy(1)))];

    % Origin value is max Y AND min X
    %MM_origin = [centers(find(centers == testy(3)) - 3) centers(find(centers == testy(3)))];

    % Display Captured image AND discovered circles

    imshow(smp)
    h = viscircles(centers,radii);
    b = viscircles(centersRobot, radiiRobot);

    %Establish world limits 
    xWorldLimits = [-20 1130];
    yWorldLimits = [-20 555];

    % Create a reference using world coordinates to captured image
    RA = imref2d(size(smp), xWorldLimits, yWorldLimits);

    % Convert image coordinates to world coordinates (mm)
    [robotX, robotY] = intrinsicToWorld(RA, centersRobot(1), centersRobot(2));

    % Map Y coordinates from image data to standard cartesian
    robotY = map(robotY, yWorldLimits(1), yWorldLimits(2), yWorldLimits(2), yWorldLimits(1));

    robotPosition = [floor(robotX), floor(robotY)];
    %{
    figure 
    hold on
    h = viscircles(centers,radii);
    b = viscircles([robotPosition(1) robotPosition(2)], radiiRobot);
    quiver(robotPosition(1), robotPosition(2), 0, 50, 1, "filled", "LineWidth", 3);
    %}
    robotPosition = 0.001*[floor(robotX), floor(robotY)];

end