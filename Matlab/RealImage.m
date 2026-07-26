function image=RealImage
imageData = screencapture(0,  [1635,-380,470,356]);  % c
imshow(imageData);
image=imageData;
%gray_img=rgb2gray(imageData);
%imshow(gray_img);
%imtool(gray_img);
%figure;
%bw_img=gray_img>50;
%imtool(gray_img);
%imshow([gray_img,bw_img]);

%img = las_segm(imageData);

%imgr=ImageR(:,:,1);

%imgg=imageData(:,:,2);

%imgb=imageData(:,:,3);

%imtool(imgr);

%imtool(imgg);

%imtool(imgb);
%R = imageData(:,:,1);
%G = imageData(:,:,2);
%B = imageData(:,:,3);

%if R > G & R > B
    %color = 3; % Red Color
%elseif G > R & G > B
   % color = 4; % Green Color
%else
   % color = 5; % Blue Color
%end
%cam=webcam(2);
% preview(cam);
%img = snapshot(cam);
%imshow(img)



end