% HC-08 BLE miniAuto demo.  小车控制基础
% Run this file, then use the car object from the MATLAB Command Window.

clear;
clc;

thisDir = fileparts(mfilename("fullpath"));
addpath(thisDir);

%% Connect
car = connectMiniAutoHC08();
cleanup = onCleanup(@() delete(car));

car.stop();
car.setSpeed(35);
disp("Connected. Try commands such as car.forward(1), car.left(1), car.stop().");

%% Basic motion test
input("Press Enter to run a short forward test, or Ctrl+C to cancel.", "s");
car.forward(0.5);
pause(0.2);
car.stop();

%% Curve trajectory demo
% Coordinate convention:
%   x > 0: forward
%   y > 0: left
% The controller approximates the curve by 8-way motion commands.
t = linspace(0, 1, 80);
x = 1000 * t;
y = 250 * sin(2 * pi * t);
curvePoints = [x(:), y(:)];

speed = 35;
totalDuration = 8.0;
sampleTime = 0.12;

input("Press Enter to run the S-curve trajectory, or Ctrl+C to cancel.", "s");
followCurveTrajectory(car, curvePoints, speed, totalDuration, sampleTime);

disp("Done.");
