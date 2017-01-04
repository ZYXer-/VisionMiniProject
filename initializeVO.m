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

addpath('8point/');
addpath('triangulation/');
addpath('plot/');
% need to pull and homogenize 2d coordinates from correspondences
[~, queryIndices, matchIndices] = find(correspondences);
x2 = keypoints2(1, queryIndices);
x1 = keypoints1(1, matchIndices);
y2 = keypoints2(2, queryIndices);
y1 = keypoints1(2, matchIndices);
oneVec = ones(size(x1));
p1 = [x1; y1; oneVec];  % these need to be U V 1, not X Y 
p2 = [x2; y2; oneVec];
F = fundamentalEightPoint(p1,p2);

% will need to pull this from each dataset (e.g. K.txt, calib.txt, etc)
K = [1379.74 0 760.35
    0 1382.08 503.41
    0 0 1 ];

E = estimateEssentialMatrix(p1, p2, K, K);

% Extract the relative camera positions (R,T) from the essential matrix

% Obtain extrinsic parameters (R,t) from E
[Rotations,Translation] = decomposeEssentialMatrix(E);

% Disambiguate among the four possible configurations (find the "real"
% config)
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rotations,Translation,p1,p2,K,K);

% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
P = linearTriangulation(p1,p2,M1,M2);
disp(P);

% Visualize the 3-D scene
figure(1),

% R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]

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

%% Remove outliers
% is this really necessary for initialization, or can we get by with just using
% the "best" matches? 

%% Triangulate the keypoint correspondences (2D x 2 -> 3D)
%exercise 4 part 4
%exercise 5 2-view geometry

end

