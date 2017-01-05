function [ newCameraPose, newState ] = processFrame( prevCameraPose, newFrame, prevState )
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

%% Constants

% constants for Harris scores
harrisPatchSize = 9;
harrisTraceWeight = 0.08;

% constants for keypoint selection
numOfKeypoints = 200;
minKeypointDistance = 8;

% constants for descriptor generation
descriptorRadius = 9;

% multiplier of minimum match distance for matching threshold 
matchLambda = 5;

%% Pull descriptors for new frame

% calculate Harris scores
harrisScores = getHarrisScores(newFrame, harrisPatchSize, harrisTraceWeight);

% select keypoints
newKeypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance);

% get keypoint descriptors
newDescriptors = getDescriptors(newFrame, newKeypoints, descriptorRadius);


%% Match descriptors between new frame and previous frame

% extract descriptor image vectors
landmarkDescriptors = prevState(5:size(prevState,1),:);

correspondences = getCorrespondences(landmarkDescriptors, newDescriptors, matchLambda);


%% Remove outliers
% RANSAC

%% Triangulate new keypoint correspondences (2D x 2 -> 3D)
%exercise 4 part 4
%exercise 5 2-view geometry

%% Estimate camera pose
% PNP - lecture 3


end

