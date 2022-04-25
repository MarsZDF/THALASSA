xxx1 = XdatagenObs(1,1:3); 
vvv1 = XdatagenObs(1,4:6); 
hhh1 = cross(xxx1, vvv1); 
iii1 = hhh1(3)/norm(hhh1);
mu = mean(abs(DeltaRadius));
err = std(abs(DeltaRadius));
med = median(abs(DeltaRadius));
MU = cat(1,MU,mu);
ERR = cat(1,ERR,err);
i3 = cat(1,i3,iii1);
MED = cat(1,MED,med);
errorbar(i3,MU,ERR)
clearvars -EXCEPT MU ERR i3 MED 

if ~1
mat = [i3, MU, ERR, MED];
[~,idx] = sort(mat(:,1)); % sort just the first column
sortedmat = mat(idx,:);
i3a = [];
i3a = rad2deg(sortedmat(:,1)); 
MUa = sortedmat(:,2); 
ERRa = sortedmat(:,3); 
MEDa = sortedmat(:,4); 
hold on
plot(i3a,MUa, 'o', 'Color', 'k', 'LineWidth', 1.24);
ernie = errorbar(i3a,MUa,ERRa, 'o', 'Color', 'k', 'LineWidth', 1.24);
set(get(get(ernie,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
%plot(i3a,MUa,'o', 'Color', 'k', 'LineWidth', 1.24);
errorbar(i3a,MUa,ERRa, 'Color', 'k', 'CapSize',10)
%plot(i3a, MEDa, 'Color', '#D55E00') 
plot(i3a, MEDa, 'p', 'Color', '#D55E00', 'MarkerSize', 10, 'LineWidth',1.24) 
xlim([min(i3a) - 2, max(i3a) + 2])
lgd = legend('\mu of \DeltaR^{+}','\sigma of \DeltaR^{+}','Median of \DeltaR^{+}');
lgd.FontSize = 14;
xlabel('i_0, Initial inclination [deg]');
ylabel('\DeltaR^{+} metrics [m]');
title('\DeltaR^{+} as a function of initial inclination i_0 (R8120)');
%area(i3a, MEDa ,'FaceColor',[53, 133, 151]/255,'LineWidth', 1, 'FaceAlpha',1,'EdgeAlpha',.9)
end