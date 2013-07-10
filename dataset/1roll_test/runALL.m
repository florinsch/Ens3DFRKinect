%$ ----------------------- runs all experiments ---------------------------------- %%
% use parallel computing
matlabpool; %open 2
ms = MultiStart('UseParallel', 'always');
opts = statset('UseParallel', true);
%% load data
clear all; clc;
cd /home/flsk/Documents/uni/thesis/facerecord/dataset/1roll_test/

if exist('dataset-8classes-roll.mat', 'file')
  load dataset-8classes-roll.mat;
  disp('Loaded 8 Class Dataset file');
else
  disp('Could not find file... Importing data');
  importData;
end
%% start running

tic;
fName = 'results-5cm-SHOT.csv';
som = [2 3; 3 3; 3 4]; 
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = SHOTdata5cm;
    wrkLbls = SHOTlbls5cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nSHOT 5cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-5cm-ESF.csv';
som = [2 3; 3 3; 3 4]; 
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = ESFdata5cm;
    wrkLbls = ESFlbls5cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nESF 5cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-5cm-PFH.csv';
som = [2 3; 3 3; 3 4]; % 8.61
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = PFHdata5cm;
    wrkLbls = PFHlbls5cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nPFH 5cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;
fprintf('\n ---------------- Finished 5cm\n');
tic;
fName = 'results-4cm-SHOT.csv';
som = [3 3; 3 4; 4 4]; % 12.17
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = SHOTdata4cm;
    wrkLbls = SHOTlbls4cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nSHOT 4cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-4cm-ESF.csv';
som = [3 3; 3 4; 4 4]; % 12.17
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = ESFdata4cm;
    wrkLbls = ESFlbls4cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nESF 4cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-4cm-PFH.csv';
som = [3 3; 3 4; 4 4]; % 12.17
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = PFHdata4cm;
    wrkLbls = PFHlbls4cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nPFH 4cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;
fprintf('\n ---------------- Finished 4cm\n');
tic;
fName = 'results-3cm-SHOT.csv';
som = [4 4; 4 5; 5 5]; % 21.41
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = SHOTdata3cm;
    wrkLbls = SHOTlbls3cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nSHOT 3cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-3cm-ESF.csv';
som = [4 4; 4 5; 5 5]; % 21.41
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = ESFdata3cm;
    wrkLbls = ESFlbls3cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nESF 3cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-3cm-PFH.csv';
som = [4 4; 4 5; 5 5]; % 21.41
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = PFHdata3cm;
    wrkLbls = PFHlbls3cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nPFH 3cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;
fprintf('\n ---------------- Finished 3cm\n');
tic;
fName = 'results-2cm-SHOT.csv';
som = [6 6; 6 7; 7 7]; % 46.03
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = SHOTdata2cm;
    wrkLbls = SHOTlbls2cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nSHOT 2cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-2cm-ESF.csv';
som = [6 6; 6 7; 7 7]; % 46.03
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = ESFdata2cm;
    wrkLbls = ESFlbls2cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nESF 2cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;

tic;
fName = 'results-2cm-PFH.csv';
som = [6 6; 6 7; 7 7]; % 46.03
for i = 1:3
    somW = som(i, 1);
    somH = som(i, 2);
    
    neighborsK = 1:2:7;
    
    % get working data
    wrkData = PFHdata2cm;
    wrkLbls = PFHlbls2cm;
    fid = fopen(fName,'a'); fprintf(fid,'\nPFH 2cm\n'); fclose(fid);
    
    runAllClusterTypes(wrkData, wrkLbls, neighborsK, somW, somH, fName);
end
toc;
fprintf('\n ---------------- Finished 2cm\n');
clear fName fid i neighborsK som somH somW;