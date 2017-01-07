%% Camera calibration
% will need to pull this from each dataset (e.g. K.txt, calib.txt, etc)
K = [7.188560000000e+02 0 6.071928000000e+02 
    0 7.188560000000e+02 1.852157000000e+02
    0 0 1];
    
    
%% Initialization Frames

initialFrame = imread('kitti/00/image_0/000000.png');
secondFrame = imread('kitti/00/image_0/000002.png');


%% Initialization    
    
[cameraRotation, cameraTranslation, keypoints, state] = ...
    initializeVO(K, initialFrame, secondFrame, 1);


%% Continuous operation
range = (2+1):150; % for everything after boot frames...
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    newFrame = imread(['kitti/00/image_0/' sprintf('%06d.png',i)]);
    
    % process the new frame (image) from the sequence
    [cameraRotation, cameraTranslation, keypoints, state] = ...
    processFrame( cameraRotation, cameraTranslation, keypoints, state, newFrame, K );
    
    % Makes sure that plots refresh.    
    pause(0.01);

end