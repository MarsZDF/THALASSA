function  [a, b, c, d, e, f, PointsTrue] = EstEll(J, SunImAngle)
%[a2d, b2d, Angle, xC, yC] = EstEll(I)
experimental = 0;
%Azimuth is extracted from atan2 -> We convert it to [0 360]
Method = 'bicubic';
%J0 = J;
%J =  imresize(J, 4); %upsample the image
SZ = size(J);
I2 = rgb2gray(imrotate(J, -SunImAngle,Method, 'crop')); 
I2a = I2(1:SZ(1)/2,:);
I2b = I2((SZ(1)/2+1):end,:);
S1 = sum(sum(I2a));
S2 = sum(sum(I2b));
if S1 > S2 
    I = imrotate(I2, 90, Method,'crop'); 
else 
    I = imrotate(I2, -90, Method, 'crop'); 
end
    
%Crop preserves image size. The object is always centered and far enough 

Fi = zeros(1,SZ(1));
%La = zeros(1,SZ(1));

II = edge(I,'Sobel');

for j = 1:SZ(1)
    if ~isempty(nonzeros(II(j,:)))
        Fi(j) = find(II(j,:), 1, 'first');
        %La(j) = find(II(j,:), 1, 'last');
    end
end
    
Y = find(Fi,1):find(Fi,1,'last');
XU = Fi(Y);
%  Y2 = find(La,1):find(La,1,'last');
%  XU2 = La(Y2);
%Points = [XU1', Y1'; XU2', Y2'];
Points = [XU', Y'];

%Added on 26/10/2020
Der = diff(XU);
Extra = find(Der < (mean(Der) - 3*std(Der)));
Extra;
for je = 1:length(Extra)
   Points(Extra(length(Extra)-je+1),:) = [];
end

Zeros0 = find(~Points);
for ju = 1:length(Zeros0)
 Points(Zeros0(length(Zeros0)-ju+1),:) = [];
end 

Ik = zeros(SZ(1), SZ(2)); 
for i = 1:length(Points)
    Ik(Points(i,2), Points(i,1)) = 255;
end

if S1 > S2 
    Ik2 = imrotate(Ik, SunImAngle-90, Method, 'crop'); 
else 
    Ik2 = imrotate(Ik, SunImAngle+90, Method, 'crop'); 
end

%imshow(Ik2);
[rPointsNew, cPointsNew] = find(Ik2);
VectorE = EllipseDirectFit([cPointsNew, rPointsNew]);
close all 
imshow(J)
hold on
plot(cPointsNew,rPointsNew, 'o', 'Color', '#EFC93D')
PointsTrue = [rPointsNew, cPointsNew];
%axis equal

a = VectorE(1);
b = VectorE(2);
c = VectorE(3);
d = VectorE(4);
e = VectorE(5);
f = VectorE(6);
M = 600;
[Points2] = SamplingEllipse(a, b, c, d, e, f, M);
hold on 
plot(Points2(:,1), Points2(:,2), 'LineWidth', 1.6, 'Color', '#AF2C2C');
holder =3;

end
