function [rotCam2World, transCam2World, inliers] = performRANSAC(homoKeypoints1, homoKeypoints2, K)


    numIterations = 10;
    pixel_tolerance = 1;

    % Initialize RANSAC.
    inliers = zeros(1, size(homoKeypoints1, 2));
    %matched_query_keypoints = flipud(matched_query_keypoints);
    maxNumInliers = 0;

    % RANSAC
    for i = 1 : numIterations
        [~, sampleIndices] = datasample(homoKeypoints1, 8, 2, 'Replace', false);

        %% Get fundamental matrix

        % Normalize each set of homogenious keypoints so that the origin
        % is at centroid and mean distance from origin is sqrt(2).
        [normalized1, normalizationMat1] = normalize2dPoints(homoKeypoints1(:, sampleIndices));
        [normalized2, normalizationMat2] = normalize2dPoints(homoKeypoints2(:, sampleIndices));

        % Get fundamental matrix with 8-point algorithm
        fundamentalMat = getFundamentalMatWithEightPoint(normalized1, normalized2);
        fundamentalMat

        % Undo the normalization
        fundamentalMat = (normalizationMat2.') * fundamentalMat * normalizationMat1;


        %% Get camera transformation

        % Get essential matrix from fundamental matrix
        essentialMat = K' * fundamentalMat * K;

        % Extract the relative camera rotation and translation from the essential matrix
        [rotCam2WorldGuess, transCam2WorldGuess] = decomposeEssentialMatrix(essentialMat);

        % Disambiguate among the four possible configurations (find the "real"
        % config)
        [rotCam2WorldGuess, transCam2WorldGuess] = disambiguateRelativePose(rotCam2WorldGuess, transCam2WorldGuess, homoKeypoints1, homoKeypoints2, K);
        
        % Triangulate the keypoints using the guessed transformation
        M1 = K * eye(3,4);
        M2 = K * [rotCam2WorldGuess, transCam2WorldGuess];
        worldKeypointsGuess = linearTriangulation(homoKeypoints1, homoKeypoints2, M1, M2);
        
        %sizeOfRot = size(rotCam2WorldGuess);
        %sizeOfRot
        
        %sizeOfPs = size(worldKeypointsGuess);
        %sizeOfPs

        
        projected_points = projectPoints((rotCam2WorldGuess * worldKeypointsGuess(1:3, :)) + ...
        repmat(transCam2WorldGuess, [1 size(homoKeypoints1, 2)]), K);
   
    
    %homoKeypoints2
    %projected_points
        
        difference = homoKeypoints2 - projected_points;
        errors = sum(difference.^2, 1);
        is_inlier = errors < pixel_tolerance^2;
        
        numOfInliers = nnz(is_inlier);
        numOfInliers
        
        if nnz(is_inlier) > maxNumInliers && nnz(is_inlier) >= 6
            maxNumInliers = nnz(is_inlier);        
            inlierMask = is_inlier;
            inliers = inlierMask;
            
            rotCam2World = rotCam2WorldGuess;
            transCam2World = transCam2WorldGuess;
        end
    end
    
    maxNumInliers
    
    fundamentalMat

end

