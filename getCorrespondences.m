function correspondences = getCorrespondences(descriptors1, descriptors2, lambda)

    % convert descriptors to double
    descriptors1 = double(descriptors1);
    descriptors2 = double(descriptors2);

    % get closest descriptor2 for each descriptor1
    [distances, matches] = pdist2(descriptors1', descriptors2', 'euclidean', 'Smallest', 1);
    
    % Get smallest non-zero distance
    sortedDistances = sort(distances);
    sortedNonZeroDistances = sortedDistances(sortedDistances ~= 0);
    minimumNonZeroDistance = sortedNonZeroDistances(1);
    
    % Set threshold based on lamda and smallest non-zero distance
    threshold = lambda * minimumNonZeroDistance;

    % Set all matches to zero where distance is above threshold
    matches(distances >= threshold) = 0;

    % create empty list of correspondences (one per descriptor1)
    correspondences = zeros(1, size(descriptors1, 2));
    
    % get indices of unique matches
    [~, uniqueMatchIndices, ~] = unique(matches, 'stable');
    
    % copy matches for each unique entry
    correspondences(uniqueMatchIndices) = matches(uniqueMatchIndices);

end

