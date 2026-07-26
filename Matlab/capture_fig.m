function picture = capture_fig(Client)

picture = ImageReadTCP_One(Client,'Center');
imshow(picture);
figure;
picture2 = ImageReadTCP_One1(Client,'Center');
imshow(picture2);
%figure;
gray_img=rgb2gray(picture);
%imshow(gray_img);
%imtool(gray_img);
%figure;
bw_img=gray_img<150;
%imshow([picture,gray_img,bw_img]);

imgr=picture(:,:,1);

imgg=picture(:,:,2);

imgb=picture(:,:,3);

imtool(imgr);

imtool(imgg);

imtool(imgb);
end


