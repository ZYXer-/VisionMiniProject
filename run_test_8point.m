%% Test for 8 point algorithm

clear all % clear all variables in the workspace
close all % close all figures
clc       % clear the command window

addpath('8point/');
addpath('triangulation/');
addpath('plot/');

rng(42);

N = 40;         % Number of 3-D points
X = randn(4,N);  % Homogeneous coordinates of 3-D points

% Simulated scene with error-free correspondences
X(3, :) = X(3, :) * 5 + 10;
X(4, :) = 1;

P1 =   [500 0 320 0
        0 500 240 0
        0 0 1 0];

P2 =   [500 0 320 -100
        0 500 240 0
        0 0 1 0];
				
x1 = P1 * X;     % Image (i.e., projected) points
x2 = P2 * X;
disp(x1);
disp(x2);

%% Fundamental matrix estimation via the 8-point algorithm

% Estimate fundamental matrix
% Call the 8-point algorithm on inputs x1,x2
F = fundamentalEightPoint(x1,x2);

% Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
cost_algebraic = norm( sum(x2.*(F*x1)) ) / sqrt(N);
cost_dist_epi_line = distPoint2EpipolarLine(F,x1,x2);

fprintf('Algebraic error: %f\n', cost_algebraic);
fprintf('Geometric error: %f px\n\n', cost_dist_epi_line);


%% Normalized 8-point algorithm
% Call the normalized 8-point algorithm on inputs x1,x2
Fn = fundamentalEightPoint_normalized(x1,x2);

% Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
cost_algebraic = norm( sum(x2.*(Fn*x1)) ) / sqrt(N);
cost_dist_epi_line = distPoint2EpipolarLine(Fn,x1,x2);

fprintf('Algebraic error: %f\n', cost_algebraic);
fprintf('Geometric error: %f px\n\n', cost_dist_epi_line);
