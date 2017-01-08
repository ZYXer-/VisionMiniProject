function plotMatching(keypoints1, keypoints2, frame)
% PLOTMATCHING - plots a frame with matching pairs of keypoints
%
% Usage:
%   plotMatching(keypoints1, keypoints2, frame)
%
% Input:
%   - keypoints1(2,N) :  array of 2D keypoints
%   - keypoints2(2,N) :  array of 2D keypoints matching keypoints1
%   - frame -  frame as 2D image 
%

    figure(1);
    subplot(3,1,1);
    imshow(frame);
    hold on;
    plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 1);
    plot(keypoints2(2, :), keypoints2(1, :), 'yx', 'Linewidth', 1);

    xFrom = keypoints2(1, :);
    xTo = keypoints1(1, :);
    yFrom = keypoints2(2, :);
    yTo = keypoints1(2, :);
    plot([yFrom; yTo], [xFrom; xTo], 'g-', 'Linewidth', 1);
    hold off;
end

