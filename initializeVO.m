function [ transformWorld2Camera, initialState ] = initializeVO( initialFrame, secondFrame )
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here

clear all;
close all;

harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 4;

%% Initial image processing

% Calculate Harris scores
harris_scores = getHarrisScores(initialFrame, harris_patch_size, harris_kappa);
assert(min(size(harris_scores) == size(initialFrame)));

% Select keypoints
keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);

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

