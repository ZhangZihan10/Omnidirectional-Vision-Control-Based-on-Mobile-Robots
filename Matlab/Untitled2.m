clc
clear all
X1 = 0.07; %
Y1 = 0.03; %
Z1 = -0.0375; % - GREEN
% Z1 = 0.118; % - RED
% Z1 = 0.039; % - BLUE
Tx = transl(X1, Y1, Z1);
T= trotx(180,'deg');


url = 'http://sentry.local:8888/stream';  % 网址
% 捕捉网页的屏幕截图并保存为图片文件

data = webread(url);
web(url);

% 设置串口参数
comPort = 'COM12';  % 设置串口号
baudRate = 115200;  % 设置波特率

cam = webcam;