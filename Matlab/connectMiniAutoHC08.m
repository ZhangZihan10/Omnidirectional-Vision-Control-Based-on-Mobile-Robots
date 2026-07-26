function car = connectMiniAutoHC08(address, serviceUUID, characteristicUUID)
%CONNECTMINIAUTOHC08 Connect to the miniAuto HC-08 BLE transparent module.
%
% Usage:
%   car = connectMiniAutoHC08();
%   car.forward(1);
%   car.stop();

if nargin < 1 || isempty(address)
    address = "48872D810806";
end
if nargin < 2 || isempty(serviceUUID)
    serviceUUID = "FFE0";
end
if nargin < 3 || isempty(characteristicUUID)
    characteristicUUID = "FFE1";
end

address = string(address);
serviceUUID = string(serviceUUID);
characteristicUUID = string(characteristicUUID);

fprintf("Connecting to Hiwonder HC-08: %s ...\n", address);
car = MiniAutoHC08Controller(address, serviceUUID, characteristicUUID);
fprintf("Connected.\n");
end
