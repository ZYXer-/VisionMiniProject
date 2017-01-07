function [ rotateCam2World, translateCam2World, initialState ] = initializeVO( initialFrame, secondFrame )
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here


    %% Constants

    % constants for Harris scores
    harrisPatchSize = 9;
    harrisTraceWeight = 0.08;

    % constants for keypoint selection
    numOfKeypoints = 400;
    minKeypointDistance = 8;


    %% Getting keypoints for initial frame

    % calculate Harris scores
    harrisScores1 = getHarrisScores(initialFrame, harrisPatchSize, harrisTraceWeight);

    % select keypoints
    keypoints1 = selectKeypoints(harrisScores1, numOfKeypoints, minKeypointDistance);


    %% Getting keypoints for second frame
    
    % initialize point tracker
    pointTracker = vision.PointTracker;
    initialize(pointTracker, fliplr(keypoints1'), initialFrame);
    
    % get keypoint correspondences by doing a step with second frame
    [trackedPoints, trackedPointValidity, trackedPointScores] = step(pointTracker, secondFrame);
    keypoints2 = fliplr(trackedPoints)';
    
    % remove all keypoints which aren't valid
    keypoints1 = keypoints1(:, trackedPointValidity == 1);
    keypoints2 = keypoints2(:, trackedPointValidity == 1);
    
    % TODO TEST
    
    zenorm = sqrt(sum(abs(keypoints1 - keypoints2).^2, 1));
    testIndex = zenorm > 8.0;
    keypoints1 = keypoints1(:, testIndex);
    keypoints2 = keypoints2(:, testIndex);

    %% DEBUGGING
    figure(4);
    imshow(secondFrame);
    hold on;
    plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);
    plot(keypoints2(2, :), keypoints2(1, :), 'yx', 'Linewidth', 2);
    plotMatches(keypoints2, keypoints1);


    %% Triangulation and outlier removal for initial landmarks
    %(2D x 2 -> 3D)
    %uses the 8-point algorithm and RANSAC

    addpath('triangulation/');
    addpath('plot/');

    %% Camera calibration
    % will need to pull this from each dataset (e.g. K.txt, calib.txt, etc)

    K = [7.188560000000e+02 0 6.071928000000e+02 
            0 7.188560000000e+02 1.852157000000e+02
            0 0 1];


    %% Find camera transformation

    % get homogenized coordinates for all correspondences
    homoKeypoints1 = [keypoints1(1:2, :); ones(1, size(keypoints1, 2))];
    homoKeypoints2 = [keypoints2(1:2, :); ones(1, size(keypoints2, 2))];
    
    % using RANSAC get best camera transformation approximation and the inlier keypoints
    [rotateCam2World, translateCam2World, inliers] = performRANSAC(homoKeypoints1, homoKeypoints2, K);
    
    
    
    %% Triangulate keypoints

    % Triangulate the keypoints using the transformation obtained from RANSAC
    M1 = K * eye(3,4);
    M2 = K * [rotateCam2World, translateCam2World];
    worldKeypoints = linearTriangulation(homoKeypoints1, homoKeypoints2, M1, M2);


    %% Plot

    disp(worldKeypoints);

    % Visualize the 3-D scene
    figure(1),

    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(worldKeypoints(1,:), worldKeypoints(2,:), worldKeypoints(3,:), 'o');

    % Display camera pose

    plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
    text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

    center_cam2_W = -rotateCam2World' * translateCam2World;
    plotCoordinateFrame(rotateCam2World', center_cam2_W, 0.8);
    text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

    axis equal
    rotate3d on;
    %grid

    %transformWorld2Camera = T_C2_W;
    
    % the initial state is the point tracker after the first step
    initialState = pointTracker;
    
    translateCam2World
    
    rotateCam2World

end

