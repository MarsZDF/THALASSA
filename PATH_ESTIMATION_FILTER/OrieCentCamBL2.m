function [eulBL2] = OrieCentCamBL2(RotMatrix)
%Given a set of orbital states and a set of points (or a single point) to
%be observed by the on-board camera, this function returns the set of angles 
%required by Blender (Roll, Pitch, Yaw) to orient the simulated
%camera towards that point. Additional constraint in this case are:
% o The observed point is consistently the center of the target;
% o The Y axis is pointed upward and the target rotation axis belongs to
%   Y-Z in the camera RF
% o The boresight of the camera in Blender is aligned with -Z

L = size(RotMatrix);
[~, b] = size(L);
%PreMat = eye(3,3);
PreMat = [1 0 0; 0 -1 0; 0 0 -1]*[-1 0 0; 0 -1 0; 0 0 1];
%PreMat = [1 0 0; 0 -1 0; 0 0 -1];

if b > 2 
    eulBL2 = zeros(L(3),3);
    for i = 1:L(3)
        eulBL2(i,1:3) = -rad2deg(rotm2eul(PreMat*RotMatrix(:,:,i),'XYZ'));
    end
else
        eulBL2(1:3) = -rad2deg(rotm2eul(PreMat*RotMatrix(:,:),'XYZ'));
end
    
end


% Written by Marco Zaccaria Di Fraia in 2020
% Marco.Di-Fraia@cranfield.ac.uk