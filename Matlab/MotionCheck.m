%% 小车运动与电机接线自检 MotionCheck
% 第一步架空车轮，逐个验证电机编号、安装位置和正反极性。
% 第二步落地，在低速下验证固件定义的 8 个平移方向。
% 需要先把 app_control2.ino 的当前版本上传到小车控制板。

clear; clc;
thisDir = fileparts(mfilename("fullpath"));
addpath(thisDir);

TEST_SPEED = 35;
WHEEL_HOLD = 0.8;
MOTION_HOLD = 1.4;
GAP = 0.8;

% 原厂物理编号: 0左前，1右前，2右后，3左后。
wheelNames = {'左前轮', '右前轮', '右后轮', '左后轮'};

% state 与 app_control2 的 Rockerandgravity_Task 完全对应。
% expected 给出逻辑轮速符号 [左前 右前 右后 左后]。
seq = { ...
    '前进', 2, '[+ + + +]'; ...
    '后退', 6, '[- - - -]'; ...
    '左移', 0, '[+ - + -]'; ...
    '右移', 4, '[- + - +]'; ...
    '左前', 1, '[+ 0 + 0]'; ...
    '右前', 3, '[0 + 0 +]'; ...
    '左后', 7, '[0 - 0 -]'; ...
    '右后', 5, '[- 0 - 0]'  ...
};

car = connectMiniAutoHC08();
cleanup = onCleanup(@() safeShutdown(car));
txChar = getTxCharacteristic(car);

sendCmd(txChar, 'A|8|$');
sendCmd(txChar, sprintf('C|%d|$', TEST_SPEED));
pause(0.3);

fprintf('\n[1/2] 单轮编号和极性检查\n');
input(['请把整车架空，确保四个轮子均可自由旋转，然后按 Enter。' ...
       '若 M 命令下没有轮子转动，请先上传新版 app_control2。'], 's');

for motorIndex = 0:3
    fprintf('\n电机 %d：应当只有%s转动。\n', motorIndex, wheelNames{motorIndex + 1});
    fprintf('  正向：轮胎接地点应向车后运动，使该轮产生前进推力。\n');
    sendCmd(txChar, sprintf('M|%d|%d|$', motorIndex, TEST_SPEED));
    pause(WHEEL_HOLD);
    sendCmd(txChar, 'M|-1|0|$');
    pause(0.4);

    fprintf('  反向：同一车轮应反转。\n');
    sendCmd(txChar, sprintf('M|%d|%d|$', motorIndex, -TEST_SPEED));
    pause(WHEEL_HOLD);
    sendCmd(txChar, 'M|-1|0|$');
    pause(GAP);
end

fprintf('\n[2/2] 8 方向落地检查\n');
input('请将小车放到四周至少留出 1 m 的平整地面，然后按 Enter。', 's');

for i = 1:size(seq, 1)
    label = seq{i, 1};
    state = seq{i, 2};
    expected = seq{i, 3};
    fprintf('>>> %-4s state=%d，期望轮速 %s\n', label, state, expected);
    sendCmd(txChar, sprintf('A|%d|$', state));
    pause(MOTION_HOLD);
    sendCmd(txChar, 'A|8|$');
    pause(GAP);
end

sendCmd(txChar, 'A|8|$');
fprintf('\n[DONE] 自检结束。请记录每个电机编号实际对应的位置及正向是否正确。\n');

%% 局部函数
function sendCmd(txChar, str)
    write(txChar, uint8(char(str)), "WithoutResponse");
end

function txChar = getTxCharacteristic(car)
    txChar = characteristic(car.Device, "FFE0", "FFE1");
end

function safeShutdown(car)
    try
        txChar = getTxCharacteristic(car);
        sendCmd(txChar, 'M|-1|0|$');
        sendCmd(txChar, 'A|8|$');
    catch
    end
    try
        car.stop();
    catch
    end
    try
        delete(car);
    catch
    end
end