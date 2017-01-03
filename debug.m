img = imread('kitti/00/image_0/000000.png');

harris = getHarrisScores(double(img), 9, 0.08);

keypoints = selectKeypoints(harris, 200, 9);

figure(1);
imagesc(mat2gray(harris));
axis equal;
axis off;
hold on;
plot(keypoints(2, :), keypoints(1, :), 'rx', 'Linewidth', 2);



pause;

