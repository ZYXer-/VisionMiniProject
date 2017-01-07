function [rotateCam2World, translateCam2World, inliers] = performRANSAC(homoKeypoints1, homoKeypoints2, K)

    homoKeypoints1 = double(homoKeypoints1);
    homoKeypoints2 = double(homoKeypoints2);

    numIterations = 2000;
    pixel_tolerance = 1;
    
    fundamentalMat = eye(3);

    % Initialize RANSAC.
    inliers = zeros(1, size(homoKeypoints1, 2));
    %matched_query_keypoints = flipud(matched_query_keypoints);
    maxNumInliers = 0;
   

    % RANSAC
    for i = 1 : numIterations
        [~, sampleIndices] = datasample(homoKeypoints1, 8, 2, 'Replace', false);
        
        sample1 = homoKeypoints1(:, sampleIndices);
        sample2 = homoKeypoints2(:, sampleIndices);
        
        % Normalize each set of homogenious keypoints so that the origin
        % is at centroid and mean distance from origin is sqrt(2).
        [sample1, normalizationMat1] = normalize2dPoints(sample1);
        [sample2, normalizationMat2] = normalize2dPoints(sample2);

        %% Get fundamental matrix
        
        % Get fundamental matrix with 8-point algorithm
        fundamentalMatGuess = getFundamentalMatWithEightPoint(sample1, sample2);
        
        fundamentalMatGuess = (normalizationMat2.') * fundamentalMatGuess * normalizationMat1;
        
        epiDistance = getEpipolarLineDistance(fundamentalMatGuess, homoKeypoints1, homoKeypoints2);
        inlierGuess = epiDistance < pixel_tolerance;
                
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
    [rotateCam2World, translateCam2World] = decomposeEssentialMatrix(essentialMat);
    
    % remove outliers
    homoKeypoints1 = homoKeypoints1(:, inliers);
    homoKeypoints2 = homoKeypoints2(:, inliers);

    % Disambiguate among the four possible configurations (find the "real"
    % config)
    [rotateCam2World, translateCam2World] = disambiguateRelativePose(rotateCam2World, translateCam2World, homoKeypoints1, homoKeypoints2, K);
    
end

