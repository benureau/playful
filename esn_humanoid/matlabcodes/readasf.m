% parsing the asf file
% input: asf file(in txt format)
% output: the id, joint names 
% output: the directions (upper limit and the lower limit of joint movements)
% Quan Wang
% matlab version 2007a
% FIAS, 2013.03.24

filename = '133_asf.txt';

% initialize all the data vector or matrix
rotation.rx = cell(30,1);
rotation.ry = cell(30,1);
rotation.rz = cell(30,1);
direction = cell(30,1);
ID = cell(30,1);
Jointname= cell(30,1);

% open the file
fid=fopen(filename);


linenum = 18; % jump to whichever line you wish to read
 textscan(fid, '%s', 1, 'delimiter', '\n', 'headerlines', linenum-1);

% matlab function to go to next line in the text file (until the end)
tline = fgetl(fid);


i = 0;
while ischar(tline)
    if (length(tline)>7)
        
        % find the id lines
        if sum(tline(6:7)=='id')==2
            i = i + 1;
            ID(i) = textscan(tline,'id %d');
        end
    
        % find the joint names
        if sum(tline(6:9)=='name')==4
            Jointname(i) = textscan(tline,'name %s');

        end

        % the direction header line
        if sum(tline(6:9)=='dire')==4
            direction{i} = textscan(tline,'direction %n %n %n');
        end
    
        % if direction has all rx, ry and rz
        if sum(tline(5:7)=='dof')==3 && length(tline)==16
         %   dof{i}= textscan(tline,'%8c %3c %3c %2c','delimiter',' ');
            tline = fgetl(fid);
            rotation.rx{i}= textscan(tline, 'limits (%n %n)');
            tline = fgetl(fid);
            rotation.ry{i}= textscan(tline, '(%n %n)');
            tline = fgetl(fid);
            rotation.rz{i}= textscan(tline,'(%n %n)');
        end
        
        % if direction has two of rx, ry and rz
        if sum(tline(5:7)=='dof')==3 && length(tline)==13
         %   dof{i}= textscan(tline,'%8c %3c %2c','delimiter',' ');
            if sum(tline(9:13)=='rx ry')==5
                tline = fgetl(fid);
                rotation.rx{i}= textscan(tline, 'limits (%n %n)');
                tline = fgetl(fid);
                rotation.ry{i}= textscan(tline, '(%n %n)');
            elseif sum(tline(9:13)=='rx rz')==5
                tline = fgetl(fid);
                rotation.rx{i}= textscan(tline, 'limits (%n %n)');
                tline = fgetl(fid);
                rotation.rz{i}= textscan(tline, '(%n %n)');
            elseif sum(tline(9:13)=='ry rz')==5
                tline = fgetl(fid);
                rotation.ry{i}= textscan(tline, 'limits (%n %n)');
                tline = fgetl(fid);
                rotation.rz{i}= textscan(tline, '(%n %n)');
            end
        end
        
          % if direction has only of rx, ry or rz
        if sum(tline(5:7)=='dof')==3 && length(tline)==10
            doftline = tline;
            if sum(doftline(9:10)=='rx')==3
                tline = fgetl(fid);
                rotation.rx{i}= textscan(tline, 'limits (%n %n)');
            elseif sum(doftline(9:10)=='ry')==3
                tline = fgetl(fid);    
                rotation.ry{i}= textscan(tline, 'limits (%n %n)');
            elseif sum(doftline(9:10)=='rz')==3
                tline = fgetl(fid);    
                rotation.rz{i}= textscan(tline, 'limits (%n %n)');
            end
        end
    end
    
    % go to next line
    tline = fgetl(fid);
end

% close file
fclose(fid)