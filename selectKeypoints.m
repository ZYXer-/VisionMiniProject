function keypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance)

    % Create list of keypoints
    keypoints = zeros(2, numOfKeypoints);
    
    % Create padded version of harrisScores
    paddedScores = padarray(harrisScores, [ minKeypointDistance minKeypointDistance ]);
    paddedScoresSize = size(paddedScores);
    
    % For each keypoint
    for i = 1 : numOfKeypoints
        
        % Get pixel with highest harris value
        [~, newKeypointIndex] = max(paddedScores(:));
        [newKeypointRow, newKeypointColumn] = ind2sub(paddedScoresSize, newKeypointIndex);
        
        % Add pixel as new keypoint
        newKeypoint = [newKeypointRow - minKeypointDistance; newKeypointColumn - minKeypointDistance];
        keypoints(:, i) = newKeypoint;
        
        % Fill in area around new keypoint with zeros
        zerosSize = (minKeypointDistance * 2);
        paddedScores( ...
            newKeypoint(1) : newKeypoint(1) + zerosSize, ...
            newKeypoint(2) : newKeypoint(2) + zerosSize) = ...
            zeros(zerosSize + 1);
        
    end

end

