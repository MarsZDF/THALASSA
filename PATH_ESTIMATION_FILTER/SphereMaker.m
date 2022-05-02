%This utility is used to create a sphere of points with angular spacing of λ1, λ2. 
%Be careful about which point is which (e.g., in a polar orbit there is a discontinuity) and about the camera reference frame

clearvars; close all;
tic
%% Constants
global Ancillary
Rx = 259; Ry = 251; Rz = 234; 
Rho = 1190; %Target Density
Om = 0/3600; %Target rotational speed (rps)
RAx = [0 0 1]; %Orientation of the target rotation axis
Ancillary(1) = Rx; Ancillary(2) = Ry; Ancillary(3) = Rz; Ancillary(4) = Rho; Ancillary(5) = Om;
Ancillary(6:8) = RAx; 

Lambda1 = 5;
Lambda2 = Lambda1;
u = (360/Lambda1)*((180/Lambda2) - 1) + 2; 

Ra = mean([Rx,Ry,Rz]);
r = 5*Ra;

%% Sphere Making
A1 = 0:Lambda1:360;
if A1(end) == 360
	A1(end) = [];
end

A2 = 0:Lambda2:180;
if A2(end) == 180
	A2(end) = [];
end 

A2(1) = [];
A2 = A2 - 90;

A1 = deg2rad(A1);
A2 = deg2rad(A2);
[G1, G2, G3] = sph2cart(0, -pi/2, r);
[H1, H2, H3] = sph2cart(0, pi/2, r);
Ii = [1, -cos(-pi/2)/sin(-pi/2) ,0];
Ll = [1, -cos(pi/2)/sin(pi/2) ,0];
Velocities = zeros(u-2,3);

for mno = 1:length(A1)
    for nop = 1:length(A2)
        MJO = length(A2)*(mno - 1) + nop; 
        [X(MJO), Y(MJO), Z(MJO)] = sph2cart(A1(mno), A2(nop), r);
        Vx(MJO) = 1;
        if A1(mno) ~= 0 && abs(A1(mno) - pi) > 1e-12
            if A1(mno) < pi
            Vy(MJO) = -1/tan(A1(mno));
            else
            Vy(MJO) = 1/tan(A1(mno));
            end
        else
            Vx(MJO) = 0;
            if A1(mno) == 0
                Vy(MJO) = -1;
                else
                Vy(MJO) = 1;
            end
        end
        Vz(MJO) = 0;
	end 
end

Positions = [X', Y', Z'];
Positions = vertcat(Positions, [G1, G2, G3]);
Positions = vertcat(Positions, [H1, H2, H3]);

Velocities = [Vx', Vy', Vz'];
Velocities = vertcat(Velocities, [Ii(1), Ii(2), Ii(3)]);
Velocities = vertcat(Velocities, [Ll(1), Ll(2), Ll(3)]);
for i = 1:u
    Velocities(i,:) = Velocities(i,:)/norm(Velocities(i,:));
end

%Graphics - use it to check that the acquisition N is where you expect it to be
plot3(Positions(:,1), Positions(:,2), Positions(:,3),'ko'); 
axis equal
for i = 1:u
    text(Positions(i,1), Positions(i,2), Positions(i,3),num2str(i),'HorizontalAlignment','left','FontSize',15);
end



[XdatagenObs] = [Positions, Velocities];
Sunn = -Om*ones(u,1); 
[RotMatrix] = OrieCentCamDynRotm(XdatagenObs);
Orb2Table(XdatagenObs, Sunn, RotMatrix); 
toc
