function [ newCameraPose, newState ] = processFrame( prevCameraPose, newFrame, prevState )
%PROCESSFRAME 
%   should we save the old camera pose before every re-iteration (in main?)? (only takes
%   in initial from initializeVO)


% what do we want to "pass forward"? Just the pixel coordinates on
% prevImage, or the full descriptor sets? 

%% Match descriptors between current image and previous image

%% Stereo matching to establish correspondences 
% ? is this necessary? 

%% Remove outliers
% RANSAC

%% Triangulate new keypoint correspondences (2D x 2 -> 3D)
%exercise 4 part 4
%exercise 5 2-view geometry

%% Estimate camera pose
% PNP - lecture 3


end

