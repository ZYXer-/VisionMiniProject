function plotMatches(matches, keypoints2, keypoints1)

% find nonzero elements in matches, pull locations
[~, queryIndices, matchIndices] = find(matches);

% draw lines from new keypoint to old keypoint
xFrom = keypoints2(1, queryIndices);
xTo = keypoints1(1, matchIndices);
yFrom = keypoints2(2, queryIndices);
yTo = keypoints1(2, matchIndices);
disp(xFrom);
plot([yFrom; yTo], [xFrom; xTo], 'g-', 'Linewidth', 3);

end

