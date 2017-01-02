function [ harrisScores ] = getHarrisScores( image, patchSize, kappa )

    % sobel filters
    sobelX = [ -1 0 1; -2 0 2; -1 0 1 ]; 
    sobelY = dx.'; 
    
    % image derivates
    dX = conv2(sobelX, image, 'valid');
    dY = conv2(sobelY, image, 'valid');
    
    % covert to double
    dX = double(dX);
    dY = double(dY);
    
    % values for sum
    dXdX = dX .* dX;
    dXdY = dX .* dY; 
    dYdY = dY .* dY; 
    
    % weights for patch
    weightsForSum = ones(patchSize) / (patchSize^2);
    dXdXsum = conv2(weightsForSum, dXdX, 'valid');
    dXdYsum = conv2(weightsForSum, dXdY, 'valid');
    dYdYsum = conv2(weightsForSum, dYdY, 'valid');
    
    % calculate scores;
    determinant = (dXdXsum * dYdYsum) - (dXdYsum * dXdYsum);
    trace = dXdXsum + dYdYsum;
    harrisScores = determinant - (kappa * trace * trace);

end

