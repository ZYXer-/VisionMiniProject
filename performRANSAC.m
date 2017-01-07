function [cameraRotation, cameraTranslation, inliers] = performRANSAC(homoKeypoints1, homoKeypoints2, K, ransacIterations, inlierToleranceInPx)

    % convert homogenious keypoints to double for more precision
    homoKeypoints1 = double(homoKeypoints1);
    homoKeypoints2 = double(homoKeypoints2);
    
    % initialize RANSAC.
    fundamentalMat = eye(3);
    inliers = zeros(1, size(homoKeypoints1, 2));
    maxNumInliers = 0;

    % RANSAC
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
        
        % remove noramlization
        fundamentalMatGuess = (normalizationMat2.') * fundamentalMatGuess * normalizationMat1;
        
        
        epiDistance = getEpipolarLineDistance(fundamentalMatGuess, homoKeypoints1, homoKeypoints2);
        inlierGuess = epiDistance < inlierToleranceInPx;
                
        numOfInliers = nnz(inlierGuess);
        
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

    % Disambiguate among the four possible configurations (find the "real"
    % config)
    [cameraRotation, cameraTranslation] = disambiguateRelativePose(cameraRotation, cameraTranslation, homoKeypoints1, homoKeypoints2, K);
    
end

