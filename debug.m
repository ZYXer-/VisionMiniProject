%img = imread('kitti/00/image_0/000000.png');

%harris = getHarrisScores(double(img), 9, 0.08);

%keypoints = selectKeypoints(harris, 200, 9);

%descriptors = getDescriptors(img, keypoints, 9);

%figure(1);
%imagesc(mat2gray(harris));
%axis equal;
%axis off;
%hold on;
%plot(keypoints(2, :), keypoints(1, :), 'rx', 'Linewidth', 2);

%for i = 1:16
   % subplot(4, 4, i);
    %patch_size = 2 * 9 + 1;
    %imagesc(uint8(reshape(descriptors(:,i), [patch_size patch_size])));
    %axis equal;
    %axis off;
%end

    %% Camera calibration
    % will need to pull this from each dataset (e.g. K.txt, calib.txt, etc)

    K = [7.188560000000e+02 0 6.071928000000e+02 
            0 7.188560000000e+02 1.852157000000e+02
            0 0 1];

initialFrame = imread('kitti/00/image_0/000000.png');
%initialFrame = imread('parking/images/img_00047.png');
secondFrame = imread('kitti/00/image_0/000002.png');
%secondFrame = imread('kitti/00/image_1/000000.png');
%secondFrame = imread('parking/images/img_00050.png');


[ rotateCam2World, translateCam2World, keypoints, state ] = ...
    initializeVO( initialFrame, secondFrame, K );

%% Continuous operation
range = (2+1):10; % for everything after boot frames...
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    newFrame = imread(['kitti/00/image_0/' sprintf('%06d.png',i)]);
    
    % process the new frame (image) from the sequence
    [ rotateCam2World, translateCam2World, keypoints, state ] = ...
    processFrame( rotateCam2World, translateCam2World, keypoints, state, newFrame, K );
    
    % Makes sure that plots refresh.    
    pause(0.01);

    prevFrame = newFrame;
end

