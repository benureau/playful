% generate the log file for playful machine humanoid 
% from the asf file and the amc file from CMU database
% in our case the amc data already stored in the excel file
% 
% input1: the excel file contains the amc angle data
% input2; readasf read the asf file with file name specified
% output: a txt file with the y[] (motor) values we can use 
%         by the replaycontroller of the playful humanoid
%
% Quan Wang
% matlab version 2007a
% FIAS, 2013.03.24


% load the file has all the amc angle data
[data txt]=xlsread('133_01WalkCrawl.xlsx', 'alljoints');

% call the asfread file
readasf;

% initiliaze new data matrix
data_new = data; %zeros(size(data,1), size(data,2));

for j = 1:length(Jointname)
    % initialize joint range
    rx_range = [];
    ry_range = [];
    rz_range = [];
    
    % calculate the mean of each direction of joint range
    if isempty(rotation.rx{j})==0
    rx_range = (rotation.rx{j}{1}+rotation.rx{j}{2})/2;
    end
    if isempty(rotation.ry{j})==0
    ry_range = (rotation.ry{j}{1}+rotation.ry{j}{2})/2;
    end
    if isempty(rotation.rz{j})==0
    rz_range = (rotation.rz{j}{1}+rotation.rz{j}{2})/2;
    end
    
    % find the current joint name
    Joint =  Jointname{j}{1};
    % Joint name length
    wordlength =  length(Joint);
    
    % substracting the range mean, with data in the amc txt file
    for i = 1:length(txt)
        txt_tmp = txt{i};
        if length(txt_tmp)==wordlength+4            
            if sum(txt_tmp==[Joint '(rx)'])==wordlength+4;
                if isempty(rx_range)==0
                data_new(1,i)= rx_range;   
                data_new(3:end,i)=data(3:end,i)-rx_range;
                end
             elseif sum(txt_tmp==[Joint '(ry)'])==wordlength+4;
                if isempty(ry_range)==0
                data_new(1,i)= ry_range;     
                data_new(3:end,i)=data(3:end,i)-ry_range;
                end
             elseif sum(txt_tmp==[Joint '(rz)'])==wordlength+4;
                if isempty(rz_range)==0
                data_new(1,i)= rz_range;     
                data_new(3:end,i)=data(3:end,i)-rz_range;
                end
            end
        end
    end
end

% select the joints and dimensions in the AMC file that is useful for us
Selected_dim = [56	57	49	50	59	52	60	53	40	39	28	27	42	30	14	9	7	8	23	22];

aa = data_new(:,Selected_dim);

% rescale the knees
rescale_id = [4,5];
rescale_id = rescale_id+1;
aa(:, rescale_id) = (aa(:, rescale_id)-118)*2;

% rescale the elbow
rescale_id = [12,13];
rescale_id = rescale_id+1;
aa(:, rescale_id) = (aa(:, rescale_id)-60)*2;

% rescale the back torson
rescale_id = [16];
rescale_id = rescale_id+1;
aa(:, rescale_id) = (aa(:, rescale_id)-45)*2;

%%% select the crawling time steps, ignor the working and standing up part
startline = 594; %1; % 550;
endline = 843; %1370; %849;
bb(:,1) = [0:endline-startline]';
bb(1:endline-startline+1,2:size(aa,2)+1) = deg2rad(aa(startline:endline,1:end));

% plot data
imagesc(bb(:,2:end)); figure(gcf)

% write output file
outputm = reshape(bb', 1, size(bb,1)*size(bb,2));
fid = fopen('crawlingdata_filename.txt', 'wt');
%fid = fopen('crawl_alljoints_range_5scale120v3.txt', 'wt');
fprintf(fid, '%4.0f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f \n', outputm);
fclose(fid)