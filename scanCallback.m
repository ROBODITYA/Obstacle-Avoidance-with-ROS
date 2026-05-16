function scanCallback(LaserScanMsg, laserScan, beta, robotradius)
% scanCallback  ROS2 subscriber callback for /scan topic.
%
% Called automatically every time a new LaserScan message arrives.
% Updates the shared ScanHandle object with:
%   - raw ranges and angles
%   - rmin  : true clearance to the most relevant obstacle (metres)
%   - phimin: heading toward that obstacle in the robot frame (radians)
%
% INPUTS
%   LaserScanMsg : sensor_msgs/LaserScan ROS2 message
%   laserScan    : ScanHandle object (shared with main loop via handle semantics)
%   beta         : directional scaling factor, 0..1
%   robotradius  : robot body radius in metres

    % --- 1. Extract raw data from message ---
    ranges = double(LaserScanMsg.ranges);
    angles = rosReadScanAngles(LaserScanMsg);   % returns angle for each beam (rad)

    % --- 2. Remove invalid beams (inf, nan, zero) ---
    valid   = isfinite(ranges) & (ranges > 0.01);
    ranges  = ranges(valid);
    angles  = angles(valid);

    % Store in handle object so main loop can read them
    laserScan.ranges = ranges;
    laserScan.angles = angles;

    if isempty(ranges)
        return;   % no valid readings this scan cycle
    end

    % --- 3. Restrict field of view to forward-facing beams only ---
    % Obstacles behind the robot are irrelevant (robot only moves forward).
    % Threshold = 45 degrees (pi/4 radians) either side of heading.
    threshold = pi / 4;
    fov       = abs(angles) < threshold;
    if any(fov)
        ranges_fov = ranges(fov);
        angles_fov = angles(fov);
    else
        % Fallback: use all beams if nothing in FOV
        ranges_fov = ranges;
        angles_fov = angles;
    end

    % --- 4. Compute scaled obstacle distance for every beam ---
    %
    % The raw range r_i measures distance from the robot centre to the wall.
    % We subtract robotradius to get the clearance from the robot's edge.
    %
    % Then we apply a directional weight:
    %   r_hat_i = (r_i - r_robot) * (1 - beta * cos(phi_i))
    %
    % Why? An obstacle straight ahead (phi=0) gives weight (1 - beta),
    % making it appear CLOSER than it really is — so the robot reacts
    % earlier to things in its path than to things beside it.
    % An obstacle at phi=90 deg gives weight 1.0 — no scaling.
    %
    clearance = ranges_fov - robotradius;
    r_hat     = clearance .* (1.0 - beta * cos(angles_fov));

    % --- 5. Find the beam with the minimum scaled distance ---
    [~, j] = min(r_hat);

    % Store the TRUE clearance (not the scaled one) and the angle
    laserScan.rmin   = clearance(j);
    laserScan.phimin = angles_fov(j);

end
