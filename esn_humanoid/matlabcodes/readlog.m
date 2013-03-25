% Read the log file into matlab
% the txt file starts with the C# line 
% output: the sensor, motor values
%         the output matrix, the direct output matrix
%         the ESN error
% Quan Wang
% FIAS, 2013.03.25
% matlab v2012a
%

fileToRead1 = 'Humanoid0_13-03-25_14-50-01.txt';
DELIMITER = ',';
HEADERLINES = 442;

tic
% Import the file
rawData1 = importdata(fileToRead1, DELIMITER, HEADERLINES);

% initilized sensor and motor value matrix, and error as a vector
xvalues = zeros(length(rawData1),20);
yvalues = zeros(length(rawData1),20);
error = zeros(length(rawData1),1);
outputm = cell(length(rawData1),1);
direct_output = cell(length(rawData1),1);

for t = 2:length(rawData1)
    
    valuescell = regexp(rawData1{t}, '\ ', 'split');
    values = zeros(length(valuescell),1);

    for i = 1:length(valuescell)
        values(i) = str2double(valuescell{i}); 
    end
    
    % sensor value
    xvalues(t,:) = values(2:20+1);
    % motor value
    yvalues(t,:) = values(22:40+1);
    % output matrix in cells
    outputm{t} = reshape(values(42:2040+1),100,20);
    % direct output matrix in cells
    direct_output{t} = reshape(values(2042:2440+1),20,20);
    % error value
    error(t) = values(2442);
end

% calculate elapsed time
toc

% plot results
figure; imagesc(xvalues);figure(gcf);
colorbar;
xlabel('Joints','fontsize',18);
ylabel('Time Steps','fontsize',18);
title('Sensor Values','fontsize',18);

figure; imagesc(yvalues);figure(gcf);
colorbar;
xlabel('Joints','fontsize',18);
ylabel('Time Steps','fontsize',18);
title('Motor Values','fontsize',18);

figure; plot(error,'LineWidth',2); figure(gcf);
xlabel('Time Steps','fontsize',18);
ylabel('Error','fontsize',18);
title('Learning Error','fontsize',18);






