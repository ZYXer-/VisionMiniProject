function epiDistance = getEpipolarLineDistance(fundamentalMat, homoKeypoints1, homoKeypoints2)
% GETEPIPOLARLINEDISTANCE - Returns epipolar line distance for a set of keypoint pairs
%
% Input: 
%   - fundamentalMat(3,3) : Fundamental matrix of the camera
%   - homoKeypoints1(3,N) : Homogenious 2D keypoints
%   - homoKeypoints2(3,N) : Homogenious 2D keypoints matching homoKeypoints1
%
% Output:
%   - epiDistance(1,N) : Epipolar line distance for all keypoint pairs
%

    % Generate epipolar lines for each keypoint set
    epiLines1 = fundamentalMat.' * homoKeypoints2;
    epiLines2 = fundamentalMat * homoKeypoints1;

    % initialize distance vectors
    distances1 = zeros(size(homoKeypoints1, 2), 1);
    distances2 = zeros(size(homoKeypoints1, 2), 1);

    % get distance for each keypoint pair
    for i = 1:size(homoKeypoints1, 2)
        distances1(i) = getPointToLineDistance(epiLines1(:, i), homoKeypoints1(:, i));
        distances2(i) = getPointToLineDistance(epiLines2(:, i), homoKeypoints2(:, i));
    end

    % Get squared distance for each keypoint pair
    epiDistance = distances1.^2 + distances2.^2;
    
end