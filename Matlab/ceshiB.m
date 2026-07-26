function [VBX,VBY]=ceshiB(img)
load maskRCNNModel.mat;
%[file,path]=uigetfile('D:\桌面文件夹\robot course\arduino\视觉识别\语义分割测试\测试图片\');
%filepath=fullfile(path,file);
%I=imread(filepath);
%img=imread('test1.jpg');
I=img;
figure;
imshow(I);

I=imresize(I,[1080, 1920]);

C=semanticseg(I,net,'MiniBatchSize', 32);
%pxds =pixelLabelDatastore(I,classes,labelIDs);

classes=["Bei","Red", "Green","Black"];
cmap=camvidColorMap;%需要更改内参数
B=labeloverlay(I,C,'ColorMap',cmap,'Transparency',0.4);
figure;
imshow(B),title("Semantic segmentation Result");
pixelLabelColorbar(cmap,classes);

    
%寻找黑色方块
C1=cellstr(C);
LB = [];
for i =7:631 %1:size(C1, 1)
    for j = 600:1250  %1:size(C1, 2)
        if strcmp(C1(i, j), "Black")%选定检测对象
            LB = [LB; i, j];
        end
    end
end

%黑色方块中心点
meanValueB = mean(LB, 1);
%图片尺寸信息为列*行
Valuex=[meanValueB(2)-70,meanValueB(2)+70];%确定区域范围
Valuey=[meanValueB(1)-70,meanValueB(1)+70];
VBX=Valuex;
VBY=Valuey;
end