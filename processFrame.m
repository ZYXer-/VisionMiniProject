function [ cameraRotation, cameraTranslation, landmarks, pointTrackerState ] = ...
    processFrame( cameraRotation, cameraTranslation, landmarks, pointTrackerState, newFrame, K, doPlot )
% PROCESSFRAME - Takes a new frame as well as the previous camera 
% transformation and state to generate the new camera transformation
% and state.
%
% Input: 
%   - cameraRotation(3,3) : The camera's previous 3x3 rotation matrix
%   - cameraTranslation(3,1) : The camera's previous 3x1 translation vector
%   - landmarks(2,N) : array of 2D keypoints from the previous frame
%   - pointTrackerState : vision.PointTracker object with keypoints from the previous frame
%   - newFrame : new frame to process
%   - K(3,3) : camera calibration
%   - doPlot : if 1, the keypoint correspondences will be plotted
%
% Output: 
%   - cameraRotation(3,3) : The camera's new 3x3 rotation matrix
%   - cameraTranslation(3,1) : The camera's new 3x1 translation vector
%   - landmarks(2,N) : array of 2D keypoints from the new frame
%   - pointTrackerState : vision.PointTracker object with keypoints from the new frame
%

    % include plotting functions
    addpath('plot/');

    %% Constants

    % constants for Harris scores
    harrisPatchSize = 9;
    harrisTraceWeight = 0.08;

    % constant for keypoint selection
    minKeypointDistance = 8;

    % constants for RANSAC
    ransacIterations = 100;
    inlierToleranceInPx = 1.0;
    
    % how many keypoints should be passed to next interation
    fillUpKeypointsToNum = 1000;

    % save old camera transformation information
    prevCameraRotation = cameraRotation;
    prevCameraTranslation = cameraTranslation;


    %% Match keypoints between new frame and previous frame

    % get keypoint correspondences by doing a point tracker step with the new frame
    [trackedPoints, trackedPointValidity] = step(pointTrackerState, newFrame);
    release(pointTrackerState);
    keypoints2 = fliplr(trackedPoints)';

    % remove all keypoints which aren't valid
    keypoints1 = landmarks(:, trackedPointValidity == 1);
    keypoints2 = keypoints2(:, trackedPointValidity == 1);

    % plot new frame with keypoint correspondences
    if doPlot
        plotMatching(keypoints1, keypoints2, newFrame);
    end


    %% Find camera transformation

    % get homogenized coordinates for all correspondences
    homoKeypoints1 = [keypoints1(1:2, :); ones(1, size(keypoints1, 2))];
    homoKeypoints2 = [keypoints2(1:2, :); ones(1, size(keypoints2, 2))];
    
    % using RANSAC get best camera transformation approximation and the inlier keypoints
    [cameraRotation, cameraTranslation, inliers] = ...
        performRANSAC(homoKeypoints1, homoKeypoints2, K, ransacIterations, inlierToleranceInPx);
    
    % get new camera transformation
    cameraRotation = cameraRotation * prevCameraRotation;
    cameraTranslation = prevCameraTranslation + (prevCameraRotation * cameraTranslation);
    
    
    %% Generate new landmarks from the new frame
    
    % only keep inlier keypoints
    keypoints2 = keypoints2(1:2, inliers);

    % calculate Harris scores
    harrisScores = getHarrisScores(newFrame, harrisPatchSize, harrisTraceWeight);

    % select new keypoints
    numOfKeypoints = fillUpKeypointsToNum - size(keypoints2, 2);
    newKeypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance, keypoints2);
    
    
    %% Set up next state
    
    % get landmarks for next frame by merging this frame's new keypoints with the existing inliers
    landmarks = [newKeypoints, keypoints2];

    % create point tracker state with this frame and the landmarks
    initialize(pointTrackerState, fliplr(landmarks'), newFrame);


end

