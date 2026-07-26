classdef MiniAutoHC08Controller < handle
    % MiniAuto controller for HC-08 BLE transparent UART modules.
    %
    % The car firmware reads ASCII commands from Arduino Serial until "$".
    % Examples:
    %   A|2|$   forward
    %   A|8|$   stop
    %   C|50|$  set speed to 50

    properties
        Device
        TxCharacteristic
        Address = "48872D810806"
        ServiceUUID = "FFE0"
        CharacteristicUUID = "FFE1"
        WriteType = "withoutresponse"
        CommandDelay = 0.05
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
        function obj = MiniAutoHC08Controller(address, serviceUUID, characteristicUUID)
            if nargin >= 1 && ~isempty(address)
                obj.Address = string(address);
            end
            if nargin >= 2 && ~isempty(serviceUUID)
                obj.ServiceUUID = string(serviceUUID);
            end
            if nargin >= 3 && ~isempty(characteristicUUID)
                obj.CharacteristicUUID = string(characteristicUUID);
            end

            obj.Device = ble(obj.Address);
            obj.TxCharacteristic = characteristic( ...
                obj.Device, obj.ServiceUUID, obj.CharacteristicUUID);
            obj.stop();
        end

        function delete(obj)
            try
                obj.stop();
            catch
            end
            obj.TxCharacteristic = [];
            obj.Device = [];
        end

        function setSpeed(obj, speed)
            speed = obj.clampInteger(speed, 0, 100);
            obj.sendCommand("C", speed);
        end

        function moveState(obj, state, duration)
            if nargin < 3
                duration = [];
            end
            state = obj.clampInteger(state, 0, 10);
            obj.sendCommand("A", state);
            obj.stopAfterDuration(duration);
        end

        function moveAngle(obj, angleDeg, duration)
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

        function sendRaw(obj, payload)
            payload = char(payload);
            write(obj.TxCharacteristic, uint8(payload), "uint8", obj.WriteType);
            pause(obj.CommandDelay);
        end
    end

    methods (Access = private)
        function sendCommand(obj, command, varargin)
            payload = char(command) + "|";
            for k = 1:numel(varargin)
                payload = payload + string(round(varargin{k})) + "|";
            end
            payload = payload + "$";
            obj.sendRaw(payload);
        end

        function stopAfterDuration(obj, duration)
            if ~isempty(duration)
                pause(duration);
                obj.stop();
            end
        end
    end

    methods (Static)
        function devices = scan(timeoutSeconds)
            if nargin < 1 || isempty(timeoutSeconds)
                timeoutSeconds = 10;
            end
            devices = blelist("Timeout", timeoutSeconds);
        end

        function value = clampInteger(value, minValue, maxValue)
            value = round(double(value));
            value = max(minValue, min(maxValue, value));
        end
    end
end
