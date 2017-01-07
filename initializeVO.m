function [ rotateCam2World, translateCam2World, initialState ] = initializeVO( initialFrame, secondFrame, debugging )
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here

    % include plotting functions
    addpath('plot/');


    %% Constants

    % constants for Harris scores
    harrisPatchSize = 9;
    harrisTraceWeight = 0.08;

    % constants for keypoint selection
    numOfKeypoints = 1000;
    minKeypointDistance = 6;


    %% Getting keypoints for initial frame

    % calculate Harris scores
    harrisScores = getHarrisScores(initialFrame, harrisPatchSize, harrisTraceWeight);

    % select keypoints
    keypoints1 = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance);


    %% Getting keypoints for second frame
    
    % initialize point tracker
    pointTracker = vision.PointTracker;
    initialize(pointTracker, fliplr(keypoints1'), initialFrame);
    
    % get keypoint correspondences by doing a step with second frame
    [trackedPoints, trackedPointValidity] = step(pointTracker, secondFrame);
    keypoints2 = fliplr(trackedPoints)';
    
    % remove all keypoints which aren't valid
    keypoints1 = keypoints1(:, trackedPointValidity == 1);
    keypoints2 = keypoints2(:, trackedPointValidity == 1);

    % debug keypoint matching
    if debugging == 1
        plotMatching(keypoints2, keypoints1, secondFrame);
    end


    %% Triangulation and outlier removal for initial landmarks
    %(2D x 2 -> 3D)
    %uses the 8-point algorithm and RANSAC

    

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
    
    
    rotateCam2World
    translateCam2World
    
    
    %% Triangulate keypoints

    % Get camera transformation for initial frame and second frame
    camTransform1 = K * eye(3,4);
    camTransform2 = K * [rotateCam2World, translateCam2World];
    
    % Triangulate the keypoints using the transformation obtained from RANSAC
    worldKeypoints = linearTriangulation(homoKeypoints1, homoKeypoints2, camTransform1, camTransform2);
    worldKeypoints = worldKeypoints(:, inliers);
    validPoints = worldKeypoints(3, : ) > 0 & worldKeypoints(3,:) <= 100;
    worldKeypoints =  worldKeypoints(:, validPoints);


    %% Plot

    % Visualize the 3-D scene
    figure(2),

    % P is a [4xN] matrix containing the triangulated point cloud (in
    % homogeneous coordinates), given by the function linearTriangulation
    plot3(worldKeypoints(1,:), worldKeypoints(2,:), worldKeypoints(3,:), 'o');
    grid on;
    xlabel('x'), ylabel('y'), zlabel('z');

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

end

