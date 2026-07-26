
% 设置网址
baseUrl = 'http://sentry.local:8888';  % 设置网页的基础URL
url = 'http://sentry.local:8888/stream';  % 设置网址

% 设置webread选项
options = weboptions('Timeout', 7);  % 设置超时时间为10秒

% 获取网页的HTML内容
html = webread(url, options);

% 使用正则表达式提取图片的URL地址
pattern = 'src="([^"]+\.(jpg|jpeg|png|gif))"';  % 设置正则表达式，提取以.jpg、.jpeg、.png、.gif结尾的图片URL地址
matches = regexpi(html, pattern, 'tokens');
imageUrls = cellfun(@(x) x{1}, matches, 'UniformOutput', false);
i=1;
% 循环下载并保存图片
%for i = 1:numel(imageUrls)
    imageUrl = [baseUrl imageUrls{i}];  % 构建完整的URL地址
    imageData = webread(imageUrl);  % 下载图片数据
    
    % 保存图像数据到本地文件
    filename = sprintf('image%d.jpg', i);  % 设置保存文件名
    imwrite(imageData, filename);  % 保存图像
    
    % 读取图像矩阵
    image = imread(filename);
    
    % 显示图像
    imshow(image);
%end

%%%%%%%%%%%%
url = 'http://sentry.local:8888/stream';  % 图像URL http://sentry.local:8888/stream
image = imread(url);  % 打开URL中的图像并将其复制到变量中

imshow(image);  % 显示图像

%%%%%%%%%%%%%
% 设备的IP地址和端口号

name = "Matlab";
TCP_Handle = TCPInit('192.168.4.1',8888,name);
while(TCP_Handle.BytesAvailable == 0)    
end
data = read(TCP_Handle);
% pause(0.005);
while(TCP_Handle.BytesAvailable ~= 0)
    data1 = read(TCP_Handle);
    data = [data,data1];
%    pause(0.005);
end
fid = fopen('image4.png','w');
fwrite(fid,data,'uint8');
fclose(fid);
image = imread('image4.png');


%%%%%%%%%%%
%%%% 配置串口参数
port = "COM12";  % 串口号
baudrate = 115200;  % 波特率

% 创建serialport对象
s = serialport(port, baudrate, 'Timeout', 2);

% 等待Sentry2摄像头初始化完成
disp('Waiting for Sentry2 initialization...');
pause(2);  % 延迟2秒

writeline(s, 'I');
%imageData = readline(s);
%imageData =str2double(imageData);


imageData=[];

while s.BytesAvailable ~= 0
    % 发送获取图像命令
   data = read(s,1, "uint8");  % 读取单个字节
   imageData = [imageData; data];  % 拼接图像数据帧
end

% 关闭串口
%delete(s);clear s;

% 图像属性
rx_image_type = imageData(1);
rx_image_width = 320;%typecast(imageData(2:3), 'uint16');
rx_image_height = 240;%typecast(imageData(4:5), 'uint16');
rx_image_size =1321;%hex2dec('0*000032E9');% typecast(imageData(6:9), 'int32');

% 显示结果
disp(['image_type: ', num2str(rx_image_type), ', image_width: ', num2str(rx_image_width), ', image_height: ', num2str(rx_image_height), ', image_size: ', num2str(rx_image_size)]);
imshow(reshape(imageData(10:end), rx_image_width, rx_image_height)');