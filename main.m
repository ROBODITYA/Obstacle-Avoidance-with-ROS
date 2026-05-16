% =============================================================
%  main.m  —  ROS2 Toolbox IIb  :  Reactive Obstacle Avoidance
%
%  Prerequisites (run ONCE in a Ubuntu terminal before opening MATLAB):
%    Terminal 1:
%      export TURTLEBOT3_MODEL=burger
%      ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
%
%    Terminal 2:
%      ros2 launch turtlebot3_navigation2 navigation2.launch.py \
%        use_sim_time:=true map:=$HOME/map_world.yaml
%
%    In RViz2: click "2D Pose Estimate" and click where the robot is
%
%  Run this script from ~/matlab_ws :
%    cd ~/matlab_ws
%    run main.m       (or press F5 in the MATLAB editor)
%
%  Press Ctrl+C in MATLAB to stop the robot.
%
%  Files required in the same folder as this script:
%    ScanHandle.m          (handle class — must be separate file)
%    scanCallback.m        (callback function — must be separate file)
%    OdometryMsg2Pose.m    (helper — must be separate file)
% =============================================================

clear;  clc;


% ─────────────────────────────────────────────────────────────
%  SECTION 1 — ROS2 NODE SETUP
% ─────────────────────────────────────────────────────────────

fprintf('=== ROS2 Node Setup ===\n');

% Use Gazebo simulated time so that ros2rate and timers sync with
% the simulation clock.  If you pause Gazebo, MATLAB also pauses.
params.use_sim_time = true;
node = ros2node('/matlab_node', 'Parameters', params);
fprintf('Node created: /matlab_node\n');

% Publisher for motion commands
[velPub, velMsg] = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
fprintf('Publisher ready: /cmd_vel\n');

% Subscriber for odometry (used in forward/turn with feedback)
odomSub = ros2subscriber(node, '/odom', 'nav_msgs/Odometry');
fprintf('Subscriber ready: /odom\n');


% ─────────────────────────────────────────────────────────────
%  SECTION 2 — LASER SCAN CALLBACK SETUP
% ─────────────────────────────────────────────────────────────

fprintf('\n=== Laser Scan Setup ===\n');

% Parameters for the scaled obstacle distance formula
beta        = 0.5;   % directional weighting  (0 = circular zone, 0.5 = elongated forward)
robotradius = 0.2;   % TurtleBot3 burger radius in metres

% Create the shared handle object.
% The callback writes into it; the control loop reads from it.
laserScan             = ScanHandle();
laserScan.beta        = beta;
laserScan.robotradius = robotradius;

% Instantiate the callback subscriber.
% Every time a /scan message arrives, MATLAB automatically calls
% scanCallback(msg, laserScan, beta, robotradius) in the background.
% The curly-brace syntax passes extra arguments to the callback.
scanSubCallback = ros2subscriber(node, '/scan', ...
    'sensor_msgs/LaserScan', ...
    {@scanCallback, laserScan, beta, robotradius});

fprintf('Callback subscriber ready: /scan\n');
fprintf('Waiting 2s for first scan message...\n');
pause(2);   % give the callback time to fire at least once
fprintf('rmin = %.3f m,  phimin = %.3f rad\n', laserScan.rmin, laserScan.phimin);


% ─────────────────────────────────────────────────────────────
%  SECTION 3 — FSM PARAMETERS
% ─────────────────────────────────────────────────────────────
%
%  Three distance thresholds define five FSM regions:
%
%   |----completeLeft/Right----|---turnLeft/Right---|---goStraight---|
%   0          r_stop        r_turn              r_safe           inf
%
%   |------- stop ------------|---- slowDown -------|--- fullSpeed --|
%   0          r_stop                              r_safe           inf
%
%  Tune these if the robot stops too early or turns too late.

r_safe   = 0.50;   % metres — start reacting to obstacles
r_turn   = 0.25;   % metres — ramp to maximum turn rate
r_stop   = 0.15;   % metres — stop completely (safety critical)
v_max    = 0.18;   % m/s    — TurtleBot3 burger safe forward speed
omega_max = 1.0;   % rad/s  — maximum turn rate

% Initial FSM states — both start in their "all clear" state
v_state     = 'fullSpeed';
omega_state = 'goStraight';

fprintf('\n=== FSM Parameters ===\n');
fprintf('r_safe=%.2f  r_turn=%.2f  r_stop=%.2f\n', r_safe, r_turn, r_stop);
fprintf('v_max=%.2f m/s   omega_max=%.2f rad/s\n', v_max, omega_max);


% ─────────────────────────────────────────────────────────────
%  SECTION 4 — MAIN CONTROL LOOP
% ─────────────────────────────────────────────────────────────

control_rate = 10;   % Hz — how often we publish a new command
rateObj      = ros2rate(node, control_rate);

fprintf('\n=== Obstacle Avoidance Running  (Ctrl+C to stop) ===\n');
fprintf('%-12s %-10s %-12s %-14s %-14s\n', ...
    'rmin(m)', 'phi(rad)', 'v_state', 'omega_state', 'cmd(v, w)');

iteration = 0;

while true

    iteration = iteration + 1;

    % --- Read sensor data (written by scanCallback in background) ---
    rmin   = laserScan.rmin;
    phimin = laserScan.phimin;

    % Safety guard: if no valid scan yet, stop and wait
    if isnan(rmin) || isinf(rmin)
        velMsg.linear.x  = 0;
        velMsg.angular.z = 0;
        send(velPub, velMsg);
        waitfor(rateObj);
        continue;
    end

    % ── VelocityBehavior FSM ─────────────────────────────────
    %
    % Transitions depend only on rmin (distance to nearest obstacle).
    % Output: linear velocity v.

    switch v_state
        case 'fullSpeed'
            v = v_max;
            if rmin < r_safe
                v_state = 'slowDown';
            end

        case 'slowDown'
            % Linear interpolation: v_max at r_safe, 0 at r_stop
            v = v_max * (rmin - r_stop) / (r_safe - r_stop);
            v = max(0, min(v_max, v));   % clamp to [0, v_max]
            if rmin > r_safe
                v_state = 'fullSpeed';
            elseif rmin < r_stop
                v_state = 'stop';
            end

        case 'stop'
            v = 0;
            if rmin > r_stop
                v_state = 'slowDown';
            end

        otherwise
            v = 0;
            v_state = 'fullSpeed';
    end

    % ── TurningBehavior FSM ──────────────────────────────────
    %
    % Transitions depend on rmin AND sign of phimin.
    % Output: angular velocity omega.
    %
    % Sign convention:
    %   phimin < 0  →  obstacle to the robot's RIGHT
    %                →  turn LEFT (positive omega) to move away from it
    %   phimin > 0  →  obstacle to the robot's LEFT
    %                →  turn RIGHT (negative omega)
    %
    % omega_mag ramps from 0 (at r_safe) up to omega_max (at r_turn).

    if rmin < r_safe && rmin >= r_turn
        omega_mag = omega_max * (r_safe - rmin) / (r_safe - r_turn);
    elseif rmin < r_turn
        omega_mag = omega_max;
    else
        omega_mag = 0;
    end

    switch omega_state
        case 'goStraight'
            omega = 0;
            if rmin < r_safe
                % Choose turn direction based on obstacle angle
                if phimin <= 0
                    omega_state = 'turnLeft';     % obstacle right → turn left
                else
                    omega_state = 'turnRight';    % obstacle left  → turn right
                end
            end

        case 'turnLeft'
            omega = +omega_mag;    % positive = counterclockwise = left
            if rmin > r_safe
                omega_state = 'goStraight';
            elseif rmin < r_turn
                omega_state = 'completeLeft';
            elseif phimin > 0
                % Obstacle side has flipped — switch turn direction
                omega_state = 'turnRight';
            end

        case 'turnRight'
            omega = -omega_mag;    % negative = clockwise = right
            if rmin > r_safe
                omega_state = 'goStraight';
            elseif rmin < r_turn
                omega_state = 'completeRight';
            elseif phimin < 0
                omega_state = 'turnLeft';
            end

        case 'completeLeft'
            % Obstacle is dangerously close.
            % Commit to maximum left turn regardless of phimin.
            % This prevents oscillating left/right in a corner.
            omega = +omega_max;
            if rmin > r_turn
                omega_state = 'turnLeft';   % obstacle cleared — back to normal
            end

        case 'completeRight'
            omega = -omega_max;
            if rmin > r_turn
                omega_state = 'turnRight';
            end

        otherwise
            omega = 0;
            omega_state = 'goStraight';
    end

    % ── Publish velocity command ──────────────────────────────
    velMsg.linear.x  = v;
    velMsg.angular.z = omega;
    send(velPub, velMsg);

    % ── Print status every 10 iterations ─────────────────────
    if mod(iteration, 10) == 0
        fprintf('%-12.3f %-10.3f %-12s %-14s v=%.3f w=%.3f\n', ...
            rmin, phimin, v_state, omega_state, v, omega);
    end

    waitfor(rateObj);   % keep loop at 10 Hz (uses simulated clock)

end


% ─────────────────────────────────────────────────────────────
%  SECTION 5 — CLEANUP  (runs when you press Ctrl+C)
% ─────────────────────────────────────────────────────────────
% MATLAB will not reach here automatically due to the while true loop.
% You can also manually run the lines below after stopping with Ctrl+C.

% Stop the robot
velMsg.linear.x  = 0;
velMsg.angular.z = 0;
send(velPub, velMsg);
fprintf('\nRobot stopped.\n');

% Delete subscriber — this stops the callback from firing
clear scanSubCallback;
clear node;
fprintf('ROS2 node shut down.\n');
