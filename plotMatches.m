function plotMatches(keypoints2, keypoints1)

% draw lines from new keypoint to old keypoint
xFrom = keypoints2(1, :);
xTo = keypoints1(1, :);
yFrom = keypoints2(2, :);
yTo = keypoints1(2, :);
plot([yFrom; yTo], [xFrom; xTo], 'g-', 'Linewidth', 3);

end

