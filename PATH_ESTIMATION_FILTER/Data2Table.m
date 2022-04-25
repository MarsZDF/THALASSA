function Data2Table(M, name)
% This function converts a MATLAB matrix representing a sequence 
% of orbital states into a comma separated value (.csv) Data Table. 
cd 'C:/Users/ingma/OneDrive/Desktop/THALASSA/';
% Detailed explanation goes here
S = size(M); 
if S(2) ~= 7
    error('It looks like some information is missing from the state vector!');
end
writematrix(M,strcat(name,'.txt'))
%writematrix(M,'Orbit.csv')
end

% Written by Marco Zaccaria Di Fraia in 2020
% Marco.Di-Fraia@cranfield.ac.uk

%%OLD
% name = string(name);
% StrName = strcat(name, '.dat');
% C = num2cell(M);
% 
% L = S(1); % Number of States
% StateId = cell(L,1);
% for i=1:L
%     StateId{i} = ['State' num2str(i)];
% end
%T = cell2table(num2cell(M),'RowNames',StateId, 'VariableNames', ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]);
% Write the table to a CSV file
%writetable(T, StrName, 'WriteRowNames',true)
%While writing the file as a .csv the Row names would get lost. A workaround was found in
%saving the file as a .dat and then forcing a conversion to .csv
%!ren *.dat *.csv 
