function [ rotateCam2World, translateCam2World, landmarks, newState ] = ...
    processFrame( rotateCam2World, translateCam2World, prevKeypoints, prevState, newFrame, K )
%PROCESSFRAME 
%   should we save the old camera pose before every re-iteration (in main?)? (only takes
%   in initial from initializeVO)
%   input: 
%       prevState
%        - 3D landmarks and descriptor (for each, from last detection)
%            - u v w 1 descriptorVector         <-- note: already homogeneous
%        - 2D keypoints from prev frame
       

% what do we want to "pass forward"? Just the pixel coordinates on
% prevImage, or the full descriptor sets? 

% can use pointracker at the end of initializeVO and pass this to
% processframe
% at each iteration of processframe, pointtrack existing landmarks on to
% new image
% at this point we have 3d world points, from which we can use p3p

addpath('plot/');

%% Constants

% constants for Harris scores
harrisPatchSize = 9;
harrisTraceWeight = 0.08;

% constants for keypoint selection
numOfKeypoints = 50;
minKeypointDistance = 8;

    % constants for RANSAC
    ransacIterations = 200;
    inlierToleranceInPx = 1.0;

% save old camera pose information
prevRotateCam2World = rotateCam2World;
prevTranslateCam2World = translateCam2World;

%% Generate (potentially) new landmarks from the new frame

% calculate Harris scores
harrisScores = getHarrisScores(newFrame, harrisPatchSize, harrisTraceWeight);

% select keypoints
newKeypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance);


%% Match keypoints between new frame and previous frame

pointTracker = prevState;

% get keypoint correspondences by doing a step with second frame
[trackedPoints, trackedPointValidity] = step(pointTracker, newFrame);
keypoints2 = fliplr(trackedPoints)';

% remove all keypoints which aren't valid
keypoints1 = prevKeypoints(:, trackedPointValidity == 1);
keypoints2 = keypoints2(:, trackedPointValidity == 1);

% plot new frame with keypoint correspondences
plotMatching(keypoints2, keypoints1, newFrame);


%% Triangulation and outlier removal for new landmarks
% 2D currentFrame points + 3D world landmark points for camera pose
% 2D currentFrame points + 2D previousFrame points -> new 3D w landmarks


    %% Find camera transformation

    % get homogenized coordinates for all correspondences
    homoKeypoints1 = [keypoints1(1:2, :); ones(1, size(keypoints1, 2))];
    homoKeypoints2 = [keypoints2(1:2, :); ones(1, size(keypoints2, 2))];
    
    % using RANSAC get best camera transformation approximation and the inlier keypoints
    [rotateCam2World, translateCam2World, inliers] = ...
        performRANSAC(homoKeypoints1, homoKeypoints2, K, ransacIterations, inlierToleranceInPx);
    
    
    %% Triangulate keypoints

    % Get camera transformation for previous frame and new frame
    camTransform1 = K * [prevRotateCam2World, prevTranslateCam2World];
    camTransform2 = K * [rotateCam2World, translateCam2World];
    
    % Triangulate the keypoints using the transformation obtained from RANSAC
    worldKeypoints = linearTriangulation(homoKeypoints1, homoKeypoints2, camTransform1, camTransform2);
    worldKeypoints = worldKeypoints(:, inliers);
    validPoints = worldKeypoints(3, : ) > 0 & worldKeypoints(3,:) <= 25;
    worldKeypoints =  worldKeypoints(:, validPoints);
    

    %% Plot

    % Visualize the 3-D scene
    figure(1),
    subplot(2,2,3);
    hold on;
    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(worldKeypoints(1,:), worldKeypoints(2,:), worldKeypoints(3,:), 'o');
    grid on;
    xlabel('x'), ylabel('y'), zlabel('z');


    figure(1),
    subplot(2,2,4);
    hold on;
    % Display camera pose
    % multiply delta rotation and add delta translation
    rotateCam2World =  rotateCam2World * prevRotateCam2World;
    translateCam2World = translateCam2World + prevTranslateCam2World;
    center_cam2_W = -rotateCam2World' * translateCam2World;
    plotCoordinateFrame(rotateCam2World', center_cam2_W, 0.8);
    %text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam','fontsize',10,'color','k','FontWeight','bold');

    %axis equal
    rotate3d on;
    
    % merge new frame keypoints with the existing inliers
    landmarks = [newKeypoints, keypoints2(1:2, inliers)];
    %landmarks = keypoints2(1:2, inliers);
    % SHOULD REMOVE DUPLICATE ENTRIES!!!
    % the initial state is the point tracker after the first step
    release(pointTracker);
    size(landmarks,2)
    initialize(pointTracker, fliplr(landmarks'), newFrame);
    newState = pointTracker;


end

