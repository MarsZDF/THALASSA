function [eulBL3] = OrieCentCamBL2Dyn(X)
% Returns the attitude of a center-pointing camera having a reference frame
% consistent with:
% Christian JA. Accurate planetary limb localization for image-based 
% spacecraft navigation. Journal of Spacecraft and Rockets. 2017 May;54(3):708-30.
% and having the same constraints as the one used for Blender simulation

L = size(X);
RotMatrix = zeros(3,3,L(1));
    for i = 1:L(1)
       eulBL3(i,1:3) = [-90 + rad2deg(atan2(X(i,3),(sqrt(X(i,2)^2 + X(i,1)^2)))), 0, 90 + rad2deg(atan2(X(i,2),X(i,1)))];
    end
    
end

% Written by Marco Zaccaria Di Fraia in 2020
% Marco.Di-Fraia@cranfield.ac.uk