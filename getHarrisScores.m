function [ harrisScores ] = getHarrisScores( frame, patchSize, traceWeight )

    % convert frame to double
    frame = double(frame);

    % sobel filters
    sobelX = [ -1 0 1; -2 0 2; -1 0 1 ]; 
    sobelY = sobelX.'; 
    
    % image derivates
    dX = conv2(frame, sobelX, 'valid');
    dY = conv2(frame, sobelY, 'valid');
    
    % covert to double
    dX = double(dX);
    dY = double(dY);
    
    % values for sum
    dXdX = dX .* dX;
    dXdY = dX .* dY; 
    dYdY = dY .* dY; 
    
    % get patch sums
    weightsForSum = ones(patchSize) / (patchSize^2);
    dXdXsum = conv2(dXdX, weightsForSum, 'valid');
    dXdYsum = conv2(dXdY, weightsForSum, 'valid');
    dYdYsum = conv2(dYdY, weightsForSum, 'valid');
    
    % calculate scores
    determinant = (dXdXsum .* dYdYsum) - (dXdYsum .* dXdYsum);
    trace = dXdXsum + dYdYsum;
    harrisScores = determinant - (traceWeight * (trace .* trace));
    
    % pad scores so that there is a score for each pixel
    imageSize = size(frame);
    scoresSize = size(harrisScores);
    padVertical = floor((imageSize(1) - scoresSize(1)) / 2.0);
    padHorizontal = floor((imageSize(2) - scoresSize(2)) / 2.0);
    harrisScores = padarray(harrisScores, [padVertical, padHorizontal]);
    
    % set all negative values to zero
    harrisScores(harrisScores < 0) = 0;

end

