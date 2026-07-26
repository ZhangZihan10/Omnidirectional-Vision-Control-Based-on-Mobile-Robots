function color = color_check(Client)

test = ImageReadTCP_One(Client,'Center');
imshow(test);
%test=ImageR；
R = test(:,:,1);
G = test(:,:,2);
B = test(:,:,3);
if R > G & R > B
    color = 3; % Red Color
elseif G > R & G > B
    color = 4; % Green Color
else
    color = 5; % Blue Color
end
%调用第二个照相机
test1 = ImageReadTCP_One1(Client,'Center');
figure;
imshow(test1);