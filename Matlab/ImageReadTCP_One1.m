function image = ImageReadTCP_One1(TCP_Handle,CameraSelect)
writeTCP(TCP_Handle,sprintf("Camera1:%s,%s,%s",CameraSelect,'test','test'));

while(TCP_Handle.BytesAvailable == 0)    
end
data = read(TCP_Handle);
% pause(0.005);
while(TCP_Handle.BytesAvailable ~= 0)
    data1 = read(TCP_Handle);
    data = [data,data1];
%    pause(0.005);
end
fid = fopen('image1.png','w');
fwrite(fid,data,'uint8');
fclose(fid);
image = imread('image1.png');