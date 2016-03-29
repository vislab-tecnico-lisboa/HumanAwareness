image=imread('peppers.png');
mask=zeros(size(image));
sigma=[30 0;0 30];
center=[size(image,1) size(image,2)];
radius=round(3*sqrt(sigma));
z = fspecial('gaussian',[7 7], [0.8 0.1]);