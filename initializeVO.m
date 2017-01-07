function [cameraRotation, cameraTranslation, inlierKeypoints, pointTrackerState] = ...
    initializeVO(K, initialFrame, secondFrame, debugging)
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here

    % include plotting functions
    addpath('plot/');
    

    %% Constants

    % constants for Harris scores
    harrisPatchSize = 9;
    harrisTraceWeight = 0.08;

    % constants for keypoint selection
    numOfKeypoints = 1000;
    minKeypointDistance = 6;


    %% Getting keypoints for initial frame

    % calculate Harris scores
    harrisScores = getHarrisScores(initialFrame, harrisPatchSize, harrisTraceWeight);

    % select keypoints
    keypoints1 = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance);


    %% Getting keypoints for second frame
    
    % initialize point tracker
    pointTracker = vision.PointTracker;
    initialize(pointTracker, fliplr(keypoints1'), initialFrame);
    
    % get keypoint correspondences by doing a step with second frame
    [trackedPoints, trackedPointValidity] = step(pointTracker, secondFrame);
    release(pointTracker);
    keypoints2 = fliplr(trackedPoints)';
    
    % remove all keypoints which aren't valid
    keypoints1 = keypoints1(:, trackedPointValidity == 1);
    keypoints2 = keypoints2(:, trackedPointValidity == 1);

    % debug keypoint matching
    if debugging == 1
        plotMatching(keypoints2, keypoints1, secondFrame);
    end


    %% Find camera transformation

    % get homogenized coordinates for all keypoints
    homoKeypoints1 = [keypoints1(1:2, :); ones(1, size(keypoints1, 2))];
    homoKeypoints2 = [keypoints2(1:2, :); ones(1, size(keypoints2, 2))];
    
    % using RANSAC get best camera transformation approximation and the inlier keypoints
    [cameraRotation, cameraTranslation, inliers] = performRANSAC( ...
        homoKeypoints1, homoKeypoints2, K);
    
    % Remove all outliers
    homoKeypoints1 = homoKeypoints1(:, inliers);
    homoKeypoints2 = homoKeypoints2(:, inliers);
    
    
    %% Triangulate keypoints

    % Calculate camera transformation for initial frame and second frame
    cameraTransform1 = K * eye(3,4);
    cameraTransform2 = K * [cameraRotation, cameraTranslation];
    
    % Triangulate the keypoints using the camera transformations
    worldKeypoints = linearTriangulation( ...
        homoKeypoints1, homoKeypoints2, ...
        cameraTransform1, cameraTransform2);


    %% Plot

    % Visualize the 3-D scene
    figure(1),
    subplot(2,2,3);
    validPoints = worldKeypoints(3, :) > 0 & worldKeypoints(3,:) <= 100;
    worldKeypoints =  worldKeypoints(:, validPoints);

    hold on; 
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(worldKeypoints(1,:), worldKeypoints(2,:), worldKeypoints(3,:), 'o');
    grid on;
    xlabel('x'), ylabel('y'), zlabel('z');

    % Display camera pose
    figure(1),
    subplot(2,2,4);
    hold on;
    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Initial','fontsize',10,'color','k','FontWeight','bold');

    center_cam2_W = -cameraRotation' * cameraTranslation;
    plotCoordinateFrame(cameraRotation', center_cam2_W, 0.8);
    %text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

    axis equal
    rotate3d on;
    %grid

    %transformWorld2Camera = T_C2_W;
    
    
    %% Set up initial state
   
    % store inlier keypoints of second frame for continious operation
    inlierKeypoints = keypoints2(1:2, inliers);
    
    % create point tracker state with the second frame and its keypoints
    pointTrackerState = vision.PointTracker;
    initialize(pointTrackerState, fliplr(inlierKeypoints'), secondFrame);

end

