%% Camera calibration
% will need to pull this from each dataset (e.g. K.txt, calib.txt, etc)
K = [7.188560000000e+02 0 6.071928000000e+02 
    0 7.188560000000e+02 1.852157000000e+02
    0 0 1];
    
    
%% Initialization Frames

initialFrame = imread('kitti/00/image_0/000000.png');
secondFrame = imread('kitti/00/image_0/000002.png');

ground_truth = load(['kitti/poses/00.txt']);
ground_truth = ground_truth(:, [end-8 end]);


%% Initialization    
    
[cameraRotation, cameraTranslation, keypoints, state] = ...
    initializeVO(K, initialFrame, secondFrame, 1);


groundTruthTranslation = ground_truth(1, [4 8 12]);
groundTruthRotation = ground_truth(1, [1 2 3 5 6 7 9 10 11]);

list = [[0; 0; 0; 0; 0; 0], [cameraTranslation; ground_truth(:, 1)]];
    


%% Continuous operation
range = (2+1):4535; % for everything after boot frames...
for i = range
    fprintf('\nProcessing frame %d\n=====================\n', i);
    newFrame = imread(['kitti/00/image_0/' sprintf('%06d.png',i)]);
    
    doPlot = mod(i, 25) == 3;
    
    list = [list, cameraTranslation];
    
    % process the new frame (image) from the sequence
    [cameraRotation, cameraTranslation, keypoints, state] = ...
    processFrame( cameraRotation, cameraTranslation, keypoints, state, newFrame, K, doPlot );

    if doPlot
        list = [list, cameraTranslation];
        figure(1),
        subplot(3, 1, [2; 3]);
        xlabel('x'), ylabel('y'), zlabel('z');
        axis equal;
        rotate3d on;
        hold on;
        % Display camera pose
        % multiply delta rotation and add delta translation

        for j = 1 : (size(list, 2) - 1)
            xFrom = list(1, j);
            xTo = list(1, j + 1);
            yFrom = list(2, j);
            yTo = list(2, j + 1);
            plot([yFrom; yTo], [xFrom; xTo], 'b-', 'Linewidth', 2);
        end
        
        list = [];
    end
    
    % Makes sure that plots refresh.    
    pause(0.01);

end