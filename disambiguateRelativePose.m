function [R, T] = disambiguateRelativePose(rotation, translation, p1, p2, K)
% DISAMBIGUATERELATIVEPOSE - finds the correct relative camera pose (among
% four possible configurations) by returning the one that yields points
% lying in front of the image plane (with positive depth).
%
% Input:
%   - rotation(3,3,2)  : the two possible rotations returned by decomposeEssentialMatrix
%   - translation(3,1) : vector with the translation information returned by decomposeEssentialMatrix
%   - p1(3,N)     : homogeneous coordinates of point correspondences in image 1
%   - p2(3,N)     : homogeneous coordinates of point correspondences in image 2
%   - K(3,3)     : calibration matrix for camera
%
% Output:
%   - R(3,3)      : 3x3 the correct rotation matrix
%   - T(3,1)      : the correct translation vector
%
%   where [R|t] = T_C2_W = T_C2_C1 is a transformation that maps points
%   from the world coordinate system (identical to the coordinate system of camera 1)
%   to camera 2.
%

    M1 = K * eye(3,4); % Projection matrix of camera

    highestPositiveDepthCount = 0;
    for iRotation = 1:2
        rotationTest = rotation(:,:,iRotation);

        for iSignT = 1:2
            translationTest = translation * (-1)^iSignT;

            M2 = K * [rotationTest, translationTest];
            
            % test points in front of origin
            testPoints1 = linearTriangulation(p1, p2, M1, M2);
            positiveDepthCount1 = sum(testPoints1(3,:) > 0);
            
            % test points in front of transformed camera
            testPoints2 = [rotationTest, translationTest] * testPoints1;
            positiveDepthCount2 = sum(testPoints2(3,:) > 0);
            
            % add both positives
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
