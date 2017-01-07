function plotMatching(keypoints2, keypoints1, frame)

    figure(1);
    imshow(frame);
    hold on;
    plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);
    plot(keypoints2(2, :), keypoints2(1, :), 'yx', 'Linewidth', 2);

    xFrom = keypoints2(1, :);
    xTo = keypoints1(1, :);
    yFrom = keypoints2(2, :);
    yTo = keypoints1(2, :);
    plot([yFrom; yTo], [xFrom; xTo], 'g-', 'Linewidth', 3);

end

