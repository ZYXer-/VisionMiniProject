function [cameraRotation, cameraTranslation, inlierKeypoints, pointTrackerState] = ...
    initializeVO(K, initialFrame, secondFrame, debugging)
% INITIALIZEVO - Takes in two frames, outputs initial camera pose and the
% initial state for continious operation.
%
% Input: 
%   - K(3,3) : camera calibration
%   - initialFrame : first initialization frame
%   - secondFrame : second initialization frame
%   - debugging : if 1 keypoint cloud will be plotted
%
% Output: 
%   - cameraRotation(3,3) : initial camera rotation
%   - cameraTranslation(3,1) : initial camera translation
%   - inlierKeypoints(2,N) : array of 2D keypoints from the second frame
%   - pointTrackerState : vision.PointTracker object with keypoints from the second frame
%

    % include plotting functions
    addpath('plot/');
    

    %% Constants

    % constants for Harris scores
    harrisPatchSize = 9;
    harrisTraceWeight = 0.08;

    % constants for keypoint selection
    numOfKeypoints = 1000;
    minKeypointDistance = 8;
    
    % constants for RANSAC
    ransacIterations = 500;
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

    % plot keypoint matching
    plotMatching(keypoints1, keypoints2, secondFrame);


    %% Find camera transformation

    % get homogenized coordinates for all keypoints
    homoKeypoints1 = [keypoints1(1:2, :); ones(1, size(keypoints1, 2))];
    homoKeypoints2 = [keypoints2(1:2, :); ones(1, size(keypoints2, 2))];
    
    % using RANSAC get best camera transformation approximation and the inlier keypoints
    [cameraRotation, cameraTranslation, inliers] = performRANSAC( ...
        homoKeypoints1, homoKeypoints2, K, ...
        ransacIterations, inlierToleranceInPx);
    
    
    %% Plot triangulated keypoints for debugging
    if debugging
        plotKeypointCloud(homoKeypoints1, homoKeypoints2, inliers, K, cameraRotation, cameraTranslation);        
    end
    
    
    %% Set up initial state
   
    % store inlier keypoints of second frame for continious operation
    inlierKeypoints = keypoints2(1:2, inliers);
    
    % create point tracker state with the second frame and its keypoints
    pointTrackerState = vision.PointTracker;
    initialize(pointTrackerState, fliplr(inlierKeypoints'), secondFrame);

end

