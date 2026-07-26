classdef MiniAutoController < handle
    % MiniAuto Bluetooth/serial controller for the original app_control.ino.
    %
    % The Arduino firmware reads commands until "$" and splits fields by "|".
    % Examples:
    %   A|2|$   forward
    %   A|8|$   stop
    %   C|50|$  set speed to 50

    properties
        Serial
        Port
        BaudRate = 9600
        Timeout = 1.0
        CommandDelay = 0.03
    end

    properties (Constant, Access = private)
        StateLeft = 0
        StateForwardLeft = 1
        StateForward = 2
        StateForwardRight = 3
        StateRight = 4
        StateBackRight = 5
        StateBack = 6
        StateBackLeft = 7
        StateStop = 8
        StateRotateLeft = 9
        StateRotateRight = 10
    end

    methods
        function obj = MiniAutoController(port, baudRate, timeout)
            if nargin < 2 || isempty(baudRate)
                baudRate = 9600;
            end
            if nargin < 3 || isempty(timeout)
                timeout = 1.0;
            end

            obj.Port = char(port);
            obj.BaudRate = baudRate;
            obj.Timeout = timeout;
            obj.Serial = serialport(obj.Port, obj.BaudRate, "Timeout", obj.Timeout);
            configureTerminator(obj.Serial, "$");
            flush(obj.Serial);
            obj.stop();
        end

        function delete(obj)
            if ~isempty(obj.Serial)
                try
                    obj.stop();
                    flush(obj.Serial);
                catch
                end
            end
        end

        function setSpeed(obj, speed)
            % Set linear/rotation speed used by A-state commands, range 0..100.
            speed = obj.clampInteger(speed, 0, 100);
            obj.sendCommand("C", speed);
        end

        function moveState(obj, state, duration)
            % Send one of the app joystick states 0..10.
            if nargin < 3
                duration = [];
            end
            state = obj.clampInteger(state, 0, 10);
            obj.sendCommand("A", state);
            obj.stopAfterDuration(duration);
        end

        function moveAngle(obj, angleDeg, duration)
            % Move in the nearest 8-way translation direction.
            % 0=forward, 90=left, 180=back, 270=right.
            if nargin < 3
                duration = [];
            end
            angleDeg = mod(round(angleDeg), 360);
            angles = [90 45 0 315 270 225 180 135];
            states = [0 1 2 3 4 5 6 7];
            diffs = abs(mod(angleDeg - angles + 180, 360) - 180);
            [~, idx] = min(diffs);
            obj.moveState(states(idx), duration);
        end

        function forward(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateForward, duration);
        end

        function back(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateBack, duration);
        end

        function left(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateLeft, duration);
        end

        function right(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateRight, duration);
        end

        function forwardLeft(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateForwardLeft, duration);
        end

        function forwardRight(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateForwardRight, duration);
        end

        function backLeft(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateBackLeft, duration);
        end

        function backRight(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateBackRight, duration);
        end

        function rotateLeft(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateRotateLeft, duration);
        end

        function rotateRight(obj, duration)
            if nargin < 2
                duration = [];
            end
            obj.moveState(obj.StateRotateRight, duration);
        end

        function stop(obj)
            obj.sendCommand("A", obj.StateStop);
        end

        function setServo(obj, angleDeg)
            angleDeg = obj.clampInteger(angleDeg, 0, 180);
            obj.sendCommand("E", angleDeg);
        end

        function setUltrasoundRgb(obj, r, g, b)
            r = obj.clampInteger(r, 0, 255);
            g = obj.clampInteger(g, 0, 255);
            b = obj.clampInteger(b, 0, 255);
            obj.sendCommand("B", r, g, b);
        end

        function setAvoidance(obj, enabled)
            obj.sendCommand("F", double(logical(enabled)));
        end

        function [distanceMm, voltageMv] = readDistanceAndVoltage(obj)
            % Returns the firmware response to D|$: distance in mm and mV.
            flush(obj.Serial);
            obj.sendCommand("D");
            deadline = tic;
            distanceMm = NaN;
            voltageMv = NaN;

            while toc(deadline) < obj.Timeout
                if obj.Serial.NumBytesAvailable == 0
                    pause(0.02);
                    continue;
                end

                line = strtrim(char(readline(obj.Serial)));
                line = erase(line, "$");
                if isempty(line) || ~contains(line, ",")
                    continue;
                end

                values = sscanf(line, "%f,%f");
                if numel(values) >= 2
                    distanceMm = values(1);
                    voltageMv = values(2);
                    return;
                end
            end
        end
    end

    methods (Access = private)
        function sendCommand(obj, command, varargin)
            payload = char(command) + "|";
            for k = 1:numel(varargin)
                payload = payload + string(round(varargin{k})) + "|";
            end
            payload = payload + "$";
            write(obj.Serial, uint8(char(payload)), "uint8");
            pause(obj.CommandDelay);
        end

        function stopAfterDuration(obj, duration)
            if ~isempty(duration)
                pause(duration);
                obj.stop();
            end
        end
    end

    methods (Static)
        function ports = availablePorts()
            ports = serialportlist("available");
        end
    end

    methods (Static, Access = private)
        function value = clampInteger(value, minValue, maxValue)
            value = round(double(value));
            value = max(minValue, min(maxValue, value));
        end
    end
end
