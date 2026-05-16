function [x, y, theta] = OdometryMsg2Pose(odomMsg)
% OdometryMsg2Pose  Extract (x, y, yaw) from a nav_msgs/Odometry message.
%
% ROS2 stores orientation as a quaternion (qx, qy, qz, qw).
% For a ground robot we only need the yaw (rotation around Z axis).
%
% Yaw formula:
%   theta = atan2( 2*(qw*qz + qx*qy),  1 - 2*(qy^2 + qz^2) )
%
% OUTPUT
%   x     : position in metres (map/odom frame)
%   y     : position in metres
%   theta : yaw in radians, range [-pi, pi]

    x = odomMsg.pose.pose.position.x;
    y = odomMsg.pose.pose.position.y;

    qx = odomMsg.pose.pose.orientation.x;
    qy = odomMsg.pose.pose.orientation.y;
    qz = odomMsg.pose.pose.orientation.z;
    qw = odomMsg.pose.pose.orientation.w;

    theta = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2));

end
