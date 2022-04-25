function Orb2Table(X,Sunn, RotMatrix, p)
name = 'Orbit'; %Select a name for the file 
C = X(:, 1:3); 
%% Extract Euler Angles from points to be Observed and converts them to a Unreal Engine consistent definition
if nargin == 3
    [eulBL] = OrieCentCamBL2(RotMatrix);
else 
    %[eulUE] = OrientCamBL2(X,p);
end
%% Converts the position from a RH Coordinate System to a LH
%[Y] = R2LUE_P(C);
%% Collates together the LH position and the LH angles
XX(:,1:3) = C; 
XX(:,4:6) = deg2rad(eulBL); 
XX(:,7) = Sunn;
%% Writes the matrix defined above to a table with the name defined beforehand
Data2Table(XX, name)
end


% Written by Marco Zaccaria Di Fraia in 2020
% Marco.Di-Fraia@cranfield.ac.uk