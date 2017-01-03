function descriptors = getDescriptors(frame, keypoints, descriptorRadius)

    % Create list of descriptors
    numOfKeypoints = size(keypoints, 2);
    descriptorArea = ((2 * descriptorRadius) + 1) ^ 2;
    descriptors = zeros(descriptorArea, numOfKeypoints);
    descriptors = uint8(descriptors);
    
    % Create padded version of frame
    paddedFrame = padarray(frame, [descriptorRadius, descriptorRadius]);
    
    % For each keypoint
    for i = 1 : numOfKeypoints
        
        % Get descriptor
        keypoint = keypoints(:, i);
        descriptorSize = descriptorRadius * 2;
        descriptor = paddedFrame( ...
            keypoint(1) : keypoint(1) + descriptorSize, ...
            keypoint(2) : keypoint(2) + descriptorSize);
        descriptors(:, i) = reshape(descriptor, descriptorArea, 1);
        
    end

end
