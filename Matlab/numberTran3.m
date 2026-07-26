function numberTran3(arduino,numberz,numbery,numberx,numbert)

letter1 = 'z';
letter2 = 'y';
letter3 = 'x';
letter4 = 't';
letter5 = 'q';  % 新增的字母
%需要计算将算的角度值变为电机的值
%y1=numbery;
numberz=numberz*205560-11667;%原为100000
numberz=round(numberz, 2);
numberz = sprintf('%.2f', numberz);
numberz = regexprep(numberz, '\.0*$', '');


numbery=numbery*945-1900;


numberx=0.8*numbery+3419.6+numberx*1455;%由于机械手臂问题，L3会有偏角
numberx=round(numberx, 2);
numberx = sprintf('%.2f', numberx);
numberx = regexprep(numberx, '\.0*$', '');

numbery=round(numbery, 2);
numbery = sprintf('%.2f', numbery);
numbery = regexprep(numbery, '\.0*$', '');

%numberx=(numberx+y1*0.517)*1455+2000;%由于机械手臂问题，L3会有偏角
numbert=numbert*180/pi;

numbert=round(numbert, 3);
numbert = sprintf('%.3f', numbert);
numbert = regexprep(numbert, '\.0*$', '');

% 合并并转换为字符型
combined1 = [letter1, numberz];
combined2 = [letter2, numbery];
combined3 = [letter3, numberx];
combined4 = [letter4, numbert];
combined5 = sprintf('%s,%s,%s,%s,%s', letter5, numberz, numbery, numberx, numbert); % 新增的合并字符串

%writeline(arduino,combined1);pause(2);%将数据以文本形式写入文件。
%writeline(arduino,combined2);pause(2);
%writeline(arduino,combined3);pause(2);
%writeline(arduino,combined4);pause(2);
writeline(arduino, combined5); pause(2); % 新增的合并字符串写入

c="m";
writeline(arduino,c);


% 显示结果
%disp(combined1);
%disp(combined2);
%disp(combined3);
%disp(combined4);
disp(combined5);
end
