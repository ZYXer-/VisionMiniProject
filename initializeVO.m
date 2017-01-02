function [ transformWorld2Camera, initialState ] = initializeVO( initialFrame, secondFrame )
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here


%% Initial image processing
% Select keypoints
keypoints = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius);

% Describe keypoints and show 16 strongest keypoint descriptors
descriptors = describeKeypoints(img, keypoints, descriptor_radius);

%% Match descriptors between first two images

harris_scores_2 = harris(img_2, harris_patch_size, harris_kappa);
keypoints_2 = selectKeypoints(...
    harris_scores_2, num_keypoints, nonmaximum_supression_radius);
descriptors_2 = describeKeypoints(img_2, keypoints_2, descriptor_radius);

matches = matchDescriptors(descriptors_2, descriptors, match_lambda);
end

