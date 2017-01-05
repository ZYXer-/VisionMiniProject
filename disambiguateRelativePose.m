% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   rotations           -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   translation   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1          -  3xN homogeneous coordinates of point correspondences in image 1
%   p2          -  3xN homogeneous coordinates of point correspondences in image 2
%   K          -  3x3 calibration matrix for the camera
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_W = T_C2_C1 is a transformation that maps points
%   from the world coordinate system (coordinate system of image 1) to the
%   coordinate system of image 2
%

function [R,T] = disambiguateRelativePose(rotations,translation,p1,p2,K)

M1 = K * eye(3,4); % Projection matrix of camera

highestPositiveDepthCount = 0;
for iRotation = 1:2
    rotationTest = rotations(:,:,iRotation);
    
    for iSignT = 1:2
        T_test = translation * (-1)^iSignT;
        
        M2 = K * [rotationTest, T_test];
        P_test = linearTriangulation(p1,p2,M1,M2);
        
        PositiveDepthCount = sum(P_test(3,:) > 0);
        disp(PositiveDepthCount);      
        if (PositiveDepthCount > highestPositiveDepthCount)
            % Keep the rotation that gives the highest number of points
            % in front of both cameras
            R = rotationTest;
            T = T_test;
            highestPositiveDepthCount = PositiveDepthCount;
        end
    end
end

end

