% DISAMBIGUATERELATIVEPOSE- finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Arguments:
%   Rots -  3x3x2: the two possible rotations returned by decomposeEssentialMatrix
%   u3   -  a 3x1 vector with the translation information returned by decomposeEssentialMatrix
%   p1   -  3xN homogeneous coordinates of point correspondences in image 1
%   p2   -  3xN homogeneous coordinates of point correspondences in image 2
%   K1   -  3x3 calibration matrix for camera 1
%   K2   -  3x3 calibration matrix for camera 2
%
% Returns:
%   R -  3x3 the correct rotation matrix
%   T -  3x1 the correct translation vector
%
%   where [R|t] = T_C2_W = T_C2_C1 is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

function [R,T] = disambiguateRelativePose(rotation, translation, p1, p2, K)

M1 = K * eye(3,4); % Projection matrix of camera

highestPositiveDepthCount = 0;
for iRotation = 1:2
    rotationTest = rotation(:,:,iRotation);
    
    for iSignT = 1:2
        translationTest = translation * (-1)^iSignT;
        
        M2 = K * [rotationTest, translationTest];
        testPoints1 = linearTriangulation(p1, p2, M1, M2);
        testPoints2 = [rotationTest, translationTest] * testPoints1;
        
        positiveDepthCount1 = sum(testPoints1(3,:) > 0);
        positiveDepthCount2 = sum(testPoints2(3,:) > 0);
        positiveDepthCount = positiveDepthCount1 + positiveDepthCount2;

        if (positiveDepthCount > highestPositiveDepthCount)
            % Keep the rotation that gives the highest number of points
            % in front of both cameras
            R = rotationTest;
            T = translationTest;
            highestPositiveDepthCount = positiveDepthCount;
        end
    end
end

end
