classdef ScanHandle < handle
% ScanHandle  Handle class to share laser scan data between
%             the callback subscriber and the main control loop.
%
% Because this is a HANDLE class (not a value class), all code that
% holds a reference to the same ScanHandle object sees the same data.
% If it were a normal class, the callback would update a private copy
% and the main loop would never see the changes.

    properties
        ranges      = []    % raw range readings (metres), one per beam
        angles      = []    % corresponding beam angles (radians)
        rmin        = Inf   % scaled minimum obstacle distance (metres)
        phimin      = 0     % heading toward nearest obstacle (radians)
        beta        = 0.5   % directional weighting factor (0=circular, 1=max elongated)
        robotradius = 0.2   % robot body radius (metres), subtracted from ranges
    end

end
