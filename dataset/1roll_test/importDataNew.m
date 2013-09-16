clc;
fprintf('Recursively iterating folders to get file list...');
list = findAllFiles('.', '2_roll.csv');

fprintf('done\nGenerating individual lists of ESF, PFH and SHOT files...');
ESFfiles = list(find(~cellfun('isempty', strfind(list, 'ESF'))), :);
noESF = size(ESFfiles, 1);
PFHfiles = list(find(~cellfun('isempty', strfind(list, 'PFH'))), :);
noPFH = size(PFHfiles, 1);
SHOTfiles = list(find(~cellfun('isempty', strfind(list, 'SHOT'))), :);
noSHOT = size(SHOTfiles, 1);
%%
fprintf('done\nClearing previous data...');
ESFdata = []; ESFlbls = [];
PFHdata = []; PFHlbls = [];
SHOTdata = []; SHOTlbls = [];

totalfiles = noESF + noPFH + noSHOT; progress = 0;
fprintf('done\nImporting data:        \n');
for i = 1:noESF
    ESFdata = [ESFdata ; importESFdata(ESFfiles{i}) ];
    ESFlbls = [ESFlbls ; importESFlabels(ESFfiles{i}) ];
    progress = progress + 1;
    fprintf('\b\b\b\b\b\b\b\b%05.2f %%\n', progress/totalfiles*100);
end

for i = 1:noPFH
    PFHdata = [PFHdata ; importPFHdata(PFHfiles{i})];
    PFHlbls = [PFHlbls ; importPFHlabels(PFHfiles{i})];
    progress = progress + 1;
    fprintf('\b\b\b\b\b\b\b\b%05.2f %%\n', progress/totalfiles*100);
end

for i = 1:noSHOT
    SHOTdata = [SHOTdata ; importSHOTdata(SHOTfiles{i})];
    SHOTlbls = [SHOTlbls ; importSHOTlabels(SHOTfiles{i})];
    progress = progress + 1;
    fprintf('\b\b\b\b\b\b\b\b%05.2f %%\n', progress/totalfiles*100);
end
%%
fprintf('done\nNormalizing each row using L2 norm...');
noitems = size(ESFdata, 1);
ESFdata = spdiags(arrayfun(@(idx) norm(ESFdata(idx,:)), 1:noitems)', 0, noitems, noitems) \ ESFdata;
noitems = size(PFHdata, 1);
PFHdata = spdiags(arrayfun(@(idx) norm(PFHdata(idx,:)), 1:noitems)', 0, noitems, noitems) \ PFHdata;
noitems = size(SHOTdata, 1);
SHOTdata = spdiags(arrayfun(@(idx) norm(SHOTdata(idx,:)), 1:noitems)', 0, noitems, noitems) \ SHOTdata;
clear items;
%% 
fprintf('done\nConverting NaNs and Infs to 0s...');
ESFdata(~isfinite(ESFdata)) = 0;
PFHdata(~isfinite(PFHdata)) = 0;
SHOTdata(~isfinite(SHOTdata)) = 0;
%%
fprintf('done\nGenerating separate datasets for each patch size...');
ESFdataRoll = ESFdata(find(~cellfun('isempty', strfind(ESFlbls(:,3), 'roll'))), :);
ESFlblsRoll = ESFlbls(find(~cellfun('isempty', strfind(ESFlbls(:,3), 'roll'))), :);

PFHdataRoll = PFHdata(find(~cellfun('isempty', strfind(PFHlbls(:,3), 'roll'))), :);
PFHlblsRoll = PFHlbls(find(~cellfun('isempty', strfind(PFHlbls(:,3), 'roll'))), :);

SHOTdataRoll = SHOTdata(find(~cellfun('isempty', strfind(SHOTlbls(:,3), 'roll'))), :);
SHOTlblsRoll = SHOTlbls(find(~cellfun('isempty', strfind(SHOTlbls(:,3), 'roll'))), :);

clear ESFdata ESFlbls PFHdata PFHlbls SHOTdata SHOTlbls;
%% clean class labels (remove dashes, yaw, pitch, roll, etc)

ESFlblsRoll(:, 3) = strrep(ESFlblsRoll(:,3), '-', '');
ESFlblsRoll(:, 3) = strrep(ESFlblsRoll(:,3), '2_roll', '');

PFHlblsRoll(:, 3) = strrep(PFHlblsRoll(:,3), '-', '');
PFHlblsRoll(:, 3) = strrep(PFHlblsRoll(:,3), '2_roll', '');

SHOTlblsRoll(:, 3) = strrep(SHOTlblsRoll(:,3), '-', '');
SHOTlblsRoll(:, 3) = strrep(SHOTlblsRoll(:,3), '2_roll', '');

fprintf('done\nCleaning up...done\n');
clear progress totalfiles i rows list ESFfiles PFHfiles SHOTfiles noESF noPFH noSHOT;
%%
% fprintf('Transpose rows to columns...');
% ESFdata = ESFdata'; ESFlbls = ESFlbls';
% PFHdata = PFHdata'; PFHlbls = PFHlbls';
% SHOTdata = SHOTdata' ; SHOTlbls = SHOTlbls';