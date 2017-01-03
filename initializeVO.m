function [ transformWorld2Camera, initialState ] = initializeVO( initialFrame, secondFrame )
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here

clear all;
close all;

harrisPatchSize = 9;
harrisTraceWeight = 0.08;
numOfKeypoints = 200;
minKeypointDistance = 8;
descriptor_radius = 9;
match_lambda = 4;

%% Initial image processing

% Calculate Harris scores
harrisScores = getHarrisScores(initialFrame, harrisPatchSize, harrisTraceWeight);

% Select keypoints
keypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance);

% Describe keypoints
descriptors = describeKeypoints(img, keypoints, descriptor_radius);

%% Match descriptors between first two images

harris_scores_2 = getHarrisScores(secondFrame, harris_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_supression_radius);
descriptors_2 = describeKeypoints(secondFrame, keypoints_2, descriptor_radius);

matches = matchDescriptors(descriptors_2, descriptors, match_lambda);

%% Stereo matching to establish correspondences 

%% Remove outliers
% is this really necessary for initialization, or can we get by with just using
% the "best" matches? 

%% Triangulate the keypoint correspondences (2D x 2 -> 3D)
%exercise 4 part 4
%exercise 5 2-view geometry

end

