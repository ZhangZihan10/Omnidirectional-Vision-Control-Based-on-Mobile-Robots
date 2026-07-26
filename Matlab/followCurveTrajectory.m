function followCurveTrajectory(car, points, speed, totalDuration, sampleTime)
%FOLLOWCURVETRAJECTORY Follow an open-loop 2-D curve using 8-way movement.
%
% points is an N-by-2 matrix in the car body frame:
%   x > 0 means forward, y > 0 means left.
%
% This is open-loop timed control. It does not correct wheel slip or drift.

arguments
    car
    points (:, 2) double
    speed (1, 1) double = 35
    totalDuration (1, 1) double = 8.0
    sampleTime (1, 1) double = 0.12
end

if size(points, 1) < 2
    error("points must contain at least two rows.");
end
if totalDuration <= 0
    error("totalDuration must be positive.");
end
if sampleTime <= 0
    error("sampleTime must be positive.");
end

car.setSpeed(speed);

delta = diff(points, 1, 1);
segmentLength = hypot(delta(:, 1), delta(:, 2));
valid = segmentLength > 0;
delta = delta(valid, :);
segmentLength = segmentLength(valid);

if isempty(segmentLength)
    car.stop();
    return;
end

segmentDuration = totalDuration * segmentLength / sum(segmentLength);
headingDeg = atan2d(delta(:, 2), delta(:, 1));
states = arrayfun(@headingToState, headingDeg);

% Merge neighboring curve samples that quantize to the same car direction.
mergedStates = states(1);
mergedDurations = segmentDuration(1);
for k = 2:numel(states)
    if states(k) == mergedStates(end)
        mergedDurations(end) = mergedDurations(end) + segmentDuration(k);
    else
        mergedStates(end + 1, 1) = states(k); %#ok<AGROW>
        mergedDurations(end + 1, 1) = segmentDuration(k); %#ok<AGROW>
    end
end

try
    for k = 1:numel(mergedStates)
        durationLeft = mergedDurations(k);
        car.moveState(mergedStates(k));
        while durationLeft > 0
            pause(min(sampleTime, durationLeft));
            durationLeft = durationLeft - sampleTime;
        end
    end
catch ME
    car.stop();
    rethrow(ME);
end

car.stop();
end

function state = headingToState(angleDeg)
angleDeg = mod(angleDeg, 360);
directionAngles = [90 45 0 315 270 225 180 135];
directionStates = [0 1 2 3 4 5 6 7];
diffs = abs(mod(angleDeg - directionAngles + 180, 360) - 180);
[~, idx] = min(diffs);
state = directionStates(idx);
end
