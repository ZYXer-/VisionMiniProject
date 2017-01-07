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
    numOfKeypoints = 200;
    minKeypointDistance = 6;
    
    % constants for RANSAC
    ransacIterations = 1000;
    inlierToleranceInPx = 1.0;


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
        homoKeypoints1, homoKeypoints2, K, ...
        ransacIterations, inlierToleranceInPx);
    
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
    
    
    %% Set up initial state
   
    % store inlier keypoints of second frame for continious operation
    inlierKeypoints = keypoints2(1:2, inliers);
    
    % create point tracker state with the second frame and its keypoints
    pointTrackerState = vision.PointTracker;
    initialize(pointTrackerState, fliplr(inlierKeypoints'), secondFrame);

end

