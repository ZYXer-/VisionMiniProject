function [R,T] = decomposeEssentialMatrix(E)
% DECOMPOSEESSENTIALMATRIX
%
% Given an essential matrix, compute the camera motion, i.e.,  R and T such
% that E ~ T_x R
% 
% Input:
%   - E(3,3) : Essential matrix
%
% Output:
%   - R(3,3,2) : the two possible rotations
%   - u3(3,1)   : a vector with the translation information

    [U,~,V] = svd(E);

    % Translation
    T = U(:,3);

    % Rotations
    W = [0 -1 0; 1 0 0; 0 0 1];
    R(:,:,1) = U*W*V.';
    R(:,:,2) = U*W.'*V.';

    if det(R(:,:,1))<0
        R(:,:,1)=-R(:,:,1);
    end

    if det(R(:,:,2))<0
        R(:,:,2)=-R(:,:,2);
    end

    if norm(T) ~= 0
        T = T/norm(T);
    end

    return

end
