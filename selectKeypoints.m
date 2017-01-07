function keypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance, discardPoints)

    % Create list of keypoints
    keypoints = zeros(2, numOfKeypoints);
    
    % Create padded version of harrisScores
    paddedScores = padarray(harrisScores, [ minKeypointDistance minKeypointDistance ]);
    paddedScoresSize = size(paddedScores);
    
    % Fill in area around discarded points
    if exist('discardPoints', 'var')
        for i = 1 : size(discardPoints, 2)
            discardPoint = round(discardPoints(:, i));
            zerosSize = (minKeypointDistance * 2);
            paddedScores( ...
                discardPoint(1) : discardPoint(1) + zerosSize, ...
                discardPoint(2) : discardPoint(2) + zerosSize) = ...
                zeros(zerosSize + 1);
        end
        
    end
    
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

