function distance = getPointToLineDistance(line, homoPoint)
% GETPOOINTTOLINEDISTANCE - Get distance between point and closest point on
% line
%
% Input: 
%   - line(3,1) : Line given in a, b and c (ay + bx + c = 0)
%   - homoPoint(3,1) : Homogenious 2D point
%
% Output:
%   - distance(1,N) : Distance between point and closest point on line
%

    % get angle and offset of line
    angle = -line(1) ./ line(2);
    offset = -line(3) ./ line(2);
    
    % get angle and offset of the perpendicular line going through the point
    anglePerpend = -1 ./ angle;
    offsetPerpend = homoPoint(2) - (anglePerpend .* homoPoint(1));
    
    % get interception point
    xIntercept = (offset - offsetPerpend) ./ (anglePerpend - angle);
    yIntercept = (angle .* xIntercept) + offset;
    interceptPoint = [xIntercept; yIntercept; 1];
    
    % get distance between keypoint and interception point
    distance = sqrt((homoPoint(1) - interceptPoint(1)).^2 + (homoPoint(2) - interceptPoint(2)).^2);
    
end

