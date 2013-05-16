clc;
fprintf('Recursively iterating folders to get file list...');
list = findAllFiles('.', '.csv');

fprintf('done\nGenerating individual lists of ESF, PFH and SHOT files...');
ESFfiles = list(find(~cellfun('isempty', strfind(list, 'ESF'))));
noESF = size(ESFfiles, 1);
PFHfiles = list(find(~cellfun('isempty', strfind(list, 'PFH'))));
noPFH = size(PFHfiles, 1);
SHOTfiles = list(find(~cellfun('isempty', strfind(list, 'SHOT'))));
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
fprintf('Transpose rows to columns...');
ESFdata = ESFdata'; ESFlbls = ESFlbls';
PFHdata = PFHdata'; PFHlbls = PFHlbls';
SHOTdata = SHOTdata' ; SHOTlbls = SHOTlbls';
%%
% fprintf('done\nShuffling dataset entries...');
% [ESFdata, ESFlbls] = shuffleDatasetCols(ESFdata, ESFlbls);
% [PFHdata, PFHlbls] = shuffleDatasetCols(ESFdata, ESFlbls);
% [SHOTdata, ESFlbls] = shuffleDatasetCols(ESFdata, ESFlbls);
%%
fprintf('done\nGenerating separate datasets for each patch size...');
ESFdata2cm = ESFdata(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '2cm'))));
ESFlbls2cm = ESFlbls(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '2cm'))));
ESFdata3cm = ESFdata(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '3cm'))));
ESFlbls3cm = ESFlbls(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '3cm'))));
ESFdata4cm = ESFdata(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '4cm'))));
ESFlbls4cm = ESFlbls(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '4cm'))));
ESFdata5cm = ESFdata(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '5cm'))));
ESFlbls5cm = ESFlbls(:,find(~cellfun('isempty', strfind(ESFlbls(3,:), '5cm'))));

PFHdata2cm = PFHdata(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '2cm'))));
PFHlbls2cm = PFHlbls(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '2cm'))));
PFHdata3cm = PFHdata(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '3cm'))));
PFHlbls3cm = PFHlbls(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '3cm'))));
PFHdata4cm = PFHdata(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '4cm'))));
PFHlbls4cm = PFHlbls(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '4cm'))));
PFHdata5cm = PFHdata(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '5cm'))));
PFHlbls5cm = PFHlbls(:,find(~cellfun('isempty', strfind(PFHlbls(3,:), '5cm'))));

SHOTdata2cm = SHOTdata(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '2cm'))));
SHOTlbls2cm = SHOTlbls(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '2cm'))));
SHOTdata3cm = SHOTdata(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '3cm'))));
SHOTlbls3cm = SHOTlbls(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '3cm'))));
SHOTdata4cm = SHOTdata(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '4cm'))));
SHOTlbls4cm = SHOTlbls(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '4cm'))));
SHOTdata5cm = SHOTdata(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '5cm'))));
SHOTlbls5cm = SHOTlbls(:,find(~cellfun('isempty', strfind(SHOTlbls(3,:), '5cm'))));

fprintf('done\nCleaning up...done\n');
clear progress totalfiles i rows list ESFfiles PFHfiles SHOTfiles noESF noPFH noSHOT;