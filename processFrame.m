function [ newCameraPose, newState ] = processFrame( prevCameraPose, newFrame, prevState )
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

%% Constants

% constants for Harris scores
harrisPatchSize = 9;
harrisTraceWeight = 0.08;

% constants for keypoint selection
numOfKeypoints = 200;
minKeypointDistance = 8;

% constants for descriptor generation
descriptorRadius = 9;

% multiplier of minimum match distance for matching threshold 
matchLambda = 5;

%% Pull descriptors for new frame

% calculate Harris scores
harrisScores = getHarrisScores(newFrame, harrisPatchSize, harrisTraceWeight);

% select keypoints
newKeypoints = selectKeypoints(harrisScores, numOfKeypoints, minKeypointDistance);

% get keypoint descriptors
newDescriptors = getDescriptors(newFrame, newKeypoints, descriptorRadius);


%% Match descriptors between new frame and previous frame

% extract landmark descriptor image vectors from previousState
landmarkDescriptors = prevState(5:size(prevState,1),:);

newCorrespondences = getCorrespondences(landmarkDescriptors, newDescriptors, matchLambda);


%% Triangulation and outlier removal for new landmarks
% 2D currentFrame points + 3D world landmark points for camera pose
% 2D currentFrame points + 2D previousFrame points -> new 3D w landmarks

addpath('8point/');
addpath('triangulation/');
addpath('plot/');


% get homogenized coordinates for all correspondences
[~, indices2, indices1] = find(newCorrespondences);

% world frame landmarks
p1 = prevState(1:4, indices1);

% current frame 2D -> homogeneous points
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



%% Estimate camera pose
% PNP - lecture 3

poses = p3p( worldPoints, imageVectors );

%% Plot

disp(P);

% Visualize the 3-D scene
figure(1),

% P is a [4xN] matrix containing the triangulated point cloud (in
% homogeneous coordinates), given by the function linearTriangulation
plot3(P(1,:), P(2,:), P(3,:), 'o');

% Display current camera pose
center_cam2_W = -R_C2_W'*T_C2_W;
plotCoordinateFrame(R_C2_W',center_cam2_W, 0.8);
%text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam','fontsize',10,'color','k','FontWeight','bold');

axis equal
rotate3d on;
%grid

newCameraPose = [R_C2_W,T_C2_W];
newState = P;


end

