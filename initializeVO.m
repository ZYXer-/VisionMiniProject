function [ transformWorld2Camera, initialState ] = initializeVO( initialFrame, secondFrame )
%INITIALIZEVO - takes in two frames, outputs initial camera pose and 2D->3D world state
%   Detailed explanation goes here


%% Constants

% constants for Harris scores
harrisPatchSize = 9;
harrisTraceWeight = 0.08;

% constants for keypoint selection
numOfKeypoints = 200;
minKeypointDistance = 8;

% constants for descriptor generaton
descriptorRadius = 9;

% multiplier of minimum match distance for matching threshold 
matchLambda = 5;


%% Getting Descriptors for initial frame

% calculate Harris scores
harrisScores1 = getHarrisScores(initialFrame, harrisPatchSize, harrisTraceWeight);

% select keypoints
keypoints1 = selectKeypoints(harrisScores1, numOfKeypoints, minKeypointDistance);

% get keypoint descriptors
descriptors1 = getDescriptors(initialFrame, keypoints1, descriptorRadius);


%% Getting Descriptors for second frame

% calculate Harris scores
harrisScores2 = getHarrisScores(secondFrame, harrisPatchSize, harrisTraceWeight);

% select keypoints
keypoints2 = selectKeypoints(harrisScores2, numOfKeypoints, minKeypointDistance);

% get keypoint descriptors
descriptors2 = getDescriptors(secondFrame, keypoints2, descriptorRadius);



%% Match descriptors between first two images

correspondences = getCorrespondences(descriptors1, descriptors2, matchLambda);


%% DEBUGGING
figure(4);
imshow(secondFrame);
hold on;
plot(keypoints1(2, :), keypoints1(1, :), 'rx', 'Linewidth', 2);
plot(keypoints2(2, :), keypoints2(1, :), 'yx', 'Linewidth', 2);
plotMatches(correspondences, keypoints2, keypoints1);


%% Triangulation and outlier removal for initial landmarks
%(2D x 2 -> 3D)
%uses the 8-point algorithm and RANSAC

addpath('8point/');
addpath('triangulation/');
addpath('plot/');


% get homogenized coordinates for all correspondences
[~, indices2, indices1] = find(correspondences);

x1 = keypoints1(1, indices1);
y1 = keypoints1(2, indices1);
p1 = [x1; y1; ones(size(x1))];

x2 = keypoints2(1, indices2);
y2 = keypoints2(2, indices2);
p2 = [x2; y2; ones(size(x1))];

% will need to pull this from each dataset (e.g. K.txt, calib.txt, etc)
K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];

E = estimateEssentialMatrix(p1, p2, K, K);

% Extract the relative camera positions (R,T) from the essential matrix
% Obtain extrinsic parameters (R,t) from E
[Rotations,Translation] = decomposeEssentialMatrix(E);

% Disambiguate among the four possible configurations (find the "real"
% config)
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rotations,Translation,p1,p2,K);

% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
P = linearTriangulation(p1,p2,M1,M2);   % world keypoints

%% Plot

disp(P);

% Visualize the 3-D scene
figure(1),

% P is a [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation
plot3(P(1,:), P(2,:), P(3,:), 'o');

% Display camera pose

plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

axis equal
rotate3d on;
%grid

transformWorld2Camera = T_C2_W;
initialState = P;

end

