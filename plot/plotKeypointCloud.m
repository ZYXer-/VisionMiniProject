function plotKeypointCloud(homoKeypoints1, homoKeypoints2, inliers, K, cameraRotation, cameraTranslation)
% PLOTKEYPOINTCLOUD - Plots keypoint cloud in 3D space
%
% Input: 
%   - homoKeypoints1(3,N) : Homogenious 2D keypoints
%   - homoKeypoints2(3,N) : Homogenious 2D keypoints matching homoKeypoints1
%   - inliers(1,N) : Indices of keypoints judged inliers by the RANSAC algorithm
%   - K(3,3) : Camera calibration
%   - cameraRotation(3,3) : The camera's 3x3 rotation matrix
%   - cameraTranslation(3,1) : The camera's 3x1 translation vector
%

    % Remove all outliers
    homoKeypoints1 = homoKeypoints1(:, inliers);
    homoKeypoints2 = homoKeypoints2(:, inliers);

    % Calculate camera transformation for initial frame and second frame
    cameraTransform1 = K * eye(3,4);
    cameraTransform2 = K * [cameraRotation, cameraTranslation];

    % Triangulate the keypoints using the camera transformations
    worldKeypoints = linearTriangulation( ...
        homoKeypoints1, homoKeypoints2, ...
        cameraTransform1, cameraTransform2);
    validPoints = worldKeypoints(3, : ) > 0 & worldKeypoints(3,:) <= 25;
    worldKeypoints = worldKeypoints(:, validPoints);

    % Make figure
    figure(2);
    axis equal;
    rotate3d on;
    grid on;

    % Plot keypoints
    plot3(worldKeypoints(1,:), worldKeypoints(2,:), worldKeypoints(3,:), 'o');
    
    % Plot first camera
    plotCoordinateFrame(eye(3), [0; 0; 0], 2.5);
    text(-0.1, -0.1, -0.1, 'Cam 1', 'fontsize', 12, 'color', 'k');

    % Plot second camera
    cam2 = -cameraRotation' * cameraTranslation;
    plotCoordinateFrame(cameraRotation',cam2, 2.5);
    text(cam2(1)-0.1, cam2(2)-0.1, cam2(3)-0.1, 'Cam 2', 'fontsize', 12, 'color', 'k');

end

