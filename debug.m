%img = imread('kitti/00/image_0/000000.png');

%harris = getHarrisScores(double(img), 9, 0.08);

%keypoints = selectKeypoints(harris, 200, 9);

%descriptors = getDescriptors(img, keypoints, 9);

%figure(1);
%imagesc(mat2gray(harris));
%axis equal;
%axis off;
%hold on;
%plot(keypoints(2, :), keypoints(1, :), 'rx', 'Linewidth', 2);

%for i = 1:16
   % subplot(4, 4, i);
    %patch_size = 2 * 9 + 1;
    %imagesc(uint8(reshape(descriptors(:,i), [patch_size patch_size])));
    %axis equal;
    %axis off;
%end


initialFrame = imread('kitti/00/image_0/000000.png');
%initialFrame = imread('parking/images/img_00047.png');
secondFrame = imread('kitti/00/image_0/000004.png');
%secondFrame = imread('kitti/00/image_1/000000.png');
%secondFrame = imread('parking/images/img_00050.png');


[ transformWorld2Camera, initialState ] = initializeVO( initialFrame, secondFrame );