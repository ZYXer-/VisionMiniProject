function [cameraRotation, cameraTranslation, inliers] = performRANSAC(homoKeypoints1, homoKeypoints2, K, ransacIterations, inlierToleranceInPx)
% PERFORMRANSAC - Calculates a camera transformation by performing the 8-point
% algorithm on a set of homogenious keypoint pairs. This is repeated
% according to the RANSAC algorithm for a certain number of iterations and
% finally the camera transformation is returned that best fits the given
% keypoints.

    % convert homogenious keypoints to double for more precision
    homoKeypoints1 = double(homoKeypoints1);
    homoKeypoints2 = double(homoKeypoints2);
    
    % initialize RANSAC.
    fundamentalMat = eye(3);
    inliers = zeros(1, size(homoKeypoints1, 2));
    maxNumInliers = 0;

    %% RANSAC
    for i = 1 : ransacIterations
        
        % get eight samples of keypoints
        [~, sampleIndices] = datasample(homoKeypoints1, 8, 2, 'Replace', false);
        sample1 = homoKeypoints1(:, sampleIndices);
        sample2 = homoKeypoints2(:, sampleIndices);
        
        % normalize the eight pairs of keypoints so that the origin
        % is at centroid and mean distance from origin is sqrt(2).
        [sample1, normalizationMat1] = normalize2dPoints(sample1);
        [sample2, normalizationMat2] = normalize2dPoints(sample2);
        
        % get fundamental matrix with 8-point algorithm
        fundamentalMatGuess = getFundamentalMatWithEightPoint(sample1, sample2);
        
        % remove normalization
        fundamentalMatGuess = (normalizationMat2.') * fundamentalMatGuess * normalizationMat1;
        
        % calculate the epipolar line distance for all keypoints
        epiDistance = getEpipolarLineDistance(fundamentalMatGuess, homoKeypoints1, homoKeypoints2);
        
        % flag keypoints as inliers if epipolar line distance is below threshold
        inlierGuess = epiDistance < inlierToleranceInPx^2;
        
        % count inliers
        numOfInliers = nnz(inlierGuess);
        
        % if this iteration is the best until now, we keep its fundamental matrix
        if numOfInliers > maxNumInliers && numOfInliers >= 6
            maxNumInliers = numOfInliers;        
            inliers = inlierGuess;         
            fundamentalMat = fundamentalMatGuess;
        end
    end

    %% Get camera transformation
    
    % Get essential matrix from fundamental matrix
    essentialMat = K' * fundamentalMat * K;

    % Extract the relative camera rotation and translation from the essential matrix
    [cameraRotation, cameraTranslation] = decomposeEssentialMatrix(essentialMat);
    
    % remove outliers
    homoKeypoints1 = homoKeypoints1(:, inliers);
    homoKeypoints2 = homoKeypoints2(:, inliers);

    % Disambiguate among the four possible configurations
    [cameraRotation, cameraTranslation] = disambiguateRelativePose(cameraRotation, cameraTranslation, homoKeypoints1, homoKeypoints2, K);
    
end

