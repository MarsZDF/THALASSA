function [D] = DELTA(Rx,Ry,Rz,u)
%Computes the value of Uppercase Delta (Eq. 2.34 in Scheeres DJ. Orbital
%motion in strongly perturbed environments: applications to asteroid, comet and planetary satellite orbiters_
D = sqrt((Rx^2 + u)*(Ry^2 + u)*(Rz^2 + u));
end

