function descriptors = describeKeypoints(initialFrame, keypoints, radius)
% Returns a (2r+1)^2xN matrix of image patch vectors based on image
% img and a 2xN matrix containing the keypoint coordinates.
% r is the patch "radius".

N = size(keypoints, 2);
descriptors = uint8(zeros((2*radius+1) ^ 2, N));
paddedFrame = padarray(initialFrame, [radius, radius]);
for i = 1:N
    kp = keypoints(:, i) + radius;
    descriptors(:,i) = reshape(paddedFrame(kp(1)-r:kp(1)+radius, kp(2)-radius:kp(2)+radius), [], 1);
end

end
