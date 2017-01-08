function plotCameraLocationHistory(locationHistory)
% PLOTCAMERALOCATIONHISTORY - plots the transitions betwen a list of camera location
%
% Usage:
%   plotCameraLocationHistory(locationHistory)
%
% Input:
%   - locationHistory(3,N) :  array of 3D camera translation coordinates
%
    
    figure(1),
    subplot(3, 1, [2; 3]);
    xlabel('x'), ylabel('y'), zlabel('z');
    axis equal;
    rotate3d on;
    hold on;

    for j = 1 : (size(locationHistory, 2) - 1)
        xFrom = locationHistory(1, j);
        xTo = locationHistory(1, j + 1);
        yFrom = locationHistory(2, j);
        yTo = locationHistory(2, j + 1);
        zFrom = locationHistory(3, j);
        zTo = locationHistory(3, j + 1);
        plot3([yFrom; yTo], [xFrom; xTo], [zFrom; zTo], 'b-', 'Linewidth', 2);
    end
end

