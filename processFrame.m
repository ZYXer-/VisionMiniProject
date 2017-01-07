function [ rotateCam2World, translateCam2World, landmarks, newState ] = ...
    processFrame( rotateCam2World, translateCam2World, prevKeypoints, prevState, newFrame, K, doPlot )
%PROCESSFRAME 
%   should we save the old camera pose before every re-iteration (in main?)? (only takes
%   in initial from initializeVO)
%   input: 
%       prevState
%        - 3D landmarks and descriptor (for each, from last detection)
%            - u v w 1 descriptorVector         <-- note: already homogeneous
%        - 2D keypoints from prev frame
       

% what do we want to "pass forward"? Just the pixel coordinates on
% prevImage, or the full descriptor sets? 

% can use pointracker at the end of initializeVO and pass this to
% processframe
% at each iteration of processframe, pointtrack existing landmarks on to
% new image
% at this point we have 3d world points, from which we can use p3p

addpath('plot/');

%% Constants

% constants for Harris scores
harrisPatchSize = 9;
harrisTraceWeight = 0.08;

% constants for keypoint selection

minKeypointDistance = 8;

    % constants for RANSAC
    ransacIterations = 100;
    inlierToleranceInPx = 1.0;

% save old camera pose information
prevRotateCam2World = rotateCam2World;
prevTranslateCam2World = translateCam2World;


%% Match keypoints between new frame and previous frame

pointTracker = prevState;

% get keypoint correspondences by doing a step with second frame
[trackedPoints, trackedPointValidity] = step(pointTracker, newFrame);
keypoints2 = fliplr(trackedPoints)';

% remove all keypoints which aren't valid
keypoints1 = prevKeypoints(:, trackedPointValidity == 1);
keypoints2 = keypoints2(:, trackedPointValidity == 1);

% plot new frame with keypoint correspondences
if doPlot
    plotMatching(keypoints2, keypoints1, newFrame);
end


%% Triangulation and outlier removal for new landmarks
% 2D currentFrame points + 3D world landmark points for camera pose
% 2D currentFrame points + 2D previousFrame points -> new 3D w landmarks


    %% Find camera transformation

    % get homogenized coordinates for all correspondences
    homoKeypoints1 = [keypoints1(1:2, :); ones(1, size(keypoints1, 2))];
    homoKeypoints2 = [keypoints2(1:2, :); ones(1, size(keypoints2, 2))];
    
    % using RANSAC get best camera transformation approximation and the inlier keypoints
    [rotateCam2World, translateCam2World, inliers] = ...
        performRANSAC(homoKeypoints1, homoKeypoints2, K, ransacIterations, inlierToleranceInPx);
    
    
    %% Triangulate keypoints

    % Get camera transformation for previous frame and new frame
    camTransform1 = K * [prevRotateCam2World, prevTranslateCam2World];
    camTransform2 = K * [rotateCam2World, translateCam2World];
    
    % Triangulate the keypoints using the transformation obtained from RANSAC
    worldKeypoints = linearTriangulation(homoKeypoints1, homoKeypoints2, camTransform1, camTransform2);
    worldKeypoints = worldKeypoints(:, inliers);
    validPoints = worldKeypoints(3, : ) > 0 & worldKeypoints(3,:) <= 25;
    worldKeypoints =  worldKeypoints(:, validPoints);
    
    keypoints2 = keypoints2(1:2, inliers);

    rotateCam2World =  rotateCam2World * prevRotateCam2World;
    translateCam2World = prevTranslateCam2World + (prevRotateCam2World * translateCam2World);
    
    %% Generate (potentially) new landmarks from the new frame

    % calculate Harris scores
    harrisScores = getHarrisScores(newFrame, harrisPatchSize, harrisTraceWeight);


    fillUpKeypointsToNum = 1000;


    % select keypoints
    numOfKeypoints = fillUpKeypointsToNum - size(keypoints2, 2);
    newKeypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance, keypoints2);
    
    % merge new frame keypoints with the existing inliers
    landmarks = [newKeypoints, keypoints2];

    % the initial state is the point tracker after the first step
    release(pointTracker);

    initialize(pointTracker, fliplr(landmarks'), newFrame);
    newState = pointTracker;


end

