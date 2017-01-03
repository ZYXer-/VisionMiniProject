function [ transformWorld2Camera, initialState ] = initializeVO( initialFrame, secondFrame )
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here


%% Constants

% constants for Harris scores
harrisPatchSize = 9;
harrisTraceWeight = 0.08;

% constants for keypoint selection
numOfKeypoints = 200;
minKeypointDistance = 8;

% constants for descriptor generaton
descriptorRadius = 9;

% multiplier of minimum match distance for matching threshold 
matchLambda = 5;


%% Getting Descriptors for initial frame

% calculate Harris scores
harrisScores1 = getHarrisScores(initialFrame, harrisPatchSize, harrisTraceWeight);

% select keypoints
keypoints1 = selectKeypoints(harrisScores1, numOfKeypoints, minKeypointDistance);

% get keypoint descriptors
descriptors1 = getDescriptors(initialFrame, keypoints1, descriptorRadius);


%% Getting Descriptors for second frame

% calculate Harris scores
harrisScores2 = getHarrisScores(secondFrame, harrisPatchSize, harrisTraceWeight);

% select keypoints
keypoints2 = selectKeypoints(harrisScores2, numOfKeypoints, minKeypointDistance);

% get keypoint descriptors
descriptors2 = getDescriptors(secondFrame, keypoints2, descriptorRadius);



%% Match descriptors between first two images

correspondences = getCorrespondences(descriptors1, descriptors2, matchLambda);


%% DEBUGGING
figure(4);
imshow(secondFrame);
hold on;
plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);
plot(keypoints2(2, :), keypoints2(1, :), 'yx', 'Linewidth', 2);
plotMatches(correspondences, keypoints2, keypoints1);



%% Stereo matching to establish correspondences 

%% Remove outliers
% is this really necessary for initialization, or can we get by with just using
% the "best" matches? 

%% Triangulate the keypoint correspondences (2D x 2 -> 3D)
%exercise 4 part 4
%exercise 5 2-view geometry

end

