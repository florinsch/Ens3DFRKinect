clear all; clc;
cd /home/flsk/Documents/uni/thesis/facerecord/dataset/1roll_test/
load dataset-8classes-roll.mat
% importData;

%% choose dataset and parameters
wrkData = SHOTdata2cm;
wrkLbls = SHOTlbls2cm;

somW = 4; % number of cells WxH in SOM
somH = 4; 

noNeighbors = 1; % number of neighbors for KNNs

%% some precomputations

% count #files of type pcd NOT in patches subfolders (# frame files)
% > find . -path '*patches*' -prune -o -print | grep pcd | wc -l
% folder count is 152 and here there are 148 => not unique names for frames
% generated from c++ => add random number after each frame counter !!! 

fprintf('\nNumber of PCD frame files in folder: ');
system('find . -path "*patches*" -prune -o -print | grep pcd | wc -l');

% hack to generate unique frame names (concat frame + class in frame)
wrkLbls(:, 1) = strcat(wrkLbls(:, 1), wrkLbls(:,3));

frames = unique(wrkLbls(:, 1));
noFrames = numel(frames);
fprintf('Number of frames (camera observations): %d\n', noFrames);

noObservations = numel(wrkLbls(:,1));
fprintf('Number of observations (total patches): %d\n', noObservations);

classes = unique(wrkLbls(:,3));
noClasses = numel(classes);
fprintf('Number of classes (people): %d\n', noClasses);

noClusters = somW * somH;
fprintf('Number of clusters in SOM: %d\n', noClusters);

% compute number of frames and average number of patches per frame
sums = 0;
for i = 1:noFrames
    sums = sums + sum(strcmp(wrkLbls(:, 1), frames(i)));
end

% average number of patches per frame
avgPatchesPerFrame = sums/noFrames;
fprintf('Average number of patches per frame: %f\n', avgPatchesPerFrame);

% test if enough clusters
if (noClusters) < ceil(avgPatchesPerFrame)
    fprintf('\nWarning!!!\nNumber of clusters for SOM %d should be > avg patches per frame %f\n', ...
    noClusters, avgPatchesPerFrame);
end
clear i sums;
%% get class labels from patch grid ordering
 gridLbls = cellfun(@(s) s(14:end), wrkLbls(:, 2), 'UniformOutput', false);
 
 gridLbls = str2double(gridLbls); % convert to double
 gridLbls = gridLbls + 1; % increment +1 (starts from 0)
 
 noClusters = max(gridLbls);
 
 for i = 1:noClusters
    noclsz(i) = sum(gridLbls == i);
 end
 
 % keep only clusters with enough observations
 idx = noclsz >= mean(noclsz) - std(noclsz);
 noClusters = sum(idx); % add 1 for stray observations
 
 % filter out classes with not enough examples
 for i = 1:numel(idx)
     if ~idx(i) 
         gridLbls(gridLbls == i) = 0;
     end
 end
 
 wrkLbls(:, 4) = num2cell(gridLbls);
%% train SOMs
% shuffle dataset
[wrkData, wrkLbls] = shuffleObservations(wrkData, wrkLbls);
fprintf('\nTraining SOM...'); tic;
% create & train SOM
SOM = selforgmap([somW, somH]);
SOM.trainParam.showWindow = 0; % don't display GUI
SOM = train(SOM, wrkData', 'useParallel', 'yes');

% assign cluster patchLabels to each patch !!!! this should be done just before
% taking one frame, splitting in patches then on-line train SOM then get
% resulting cluster which it belongs to ...
wrkLbls(:, 4) = num2cell(vec2ind(SOM(wrkData')))';
fprintf('done\n'); toc;
%% split dataset into training / test set
noNeighbors = 1; 
accuracy = zeros(10, 1);
for x = 1:10

% remove clusters with 0 as index
valid = cell2mat(wrkLbls(:, 4)) ~= 0;
wrkLbls = wrkLbls(valid, :);
wrkData = wrkData(valid, :);
    
split = cvpartition(wrkLbls(:, 3), 'k', 10);
acc = zeros(split.NumTestSets, 1);

clc; tic;
for i = 1:split.NumTestSets
    trainIdx = split.training(i);
    testIdx = split.test(i);
    
    trainData = wrkData(trainIdx, :);
    trainLbls = wrkLbls(trainIdx, :);
    testData = wrkData(testIdx, :);
    testLbls = wrkLbls(testIdx, :);
    
    classes = unique(trainLbls(:, 3));
    
%     [wrkData, wrkLbls] = shuffleObservations(trainData, trainLbls);
%     SOM = selforgmap([somW, somH]);
%     SOM.trainParam.showWindow=0; % don't display GUI
%     SOM = train(SOM, wrkData', 'useParallel', 'yes');
%     
%     trainLbls(:, 4) = num2cell(vec2ind(SOM(trainData')))';

    % compute valid clusters
    clusters = unique(cell2mat(trainLbls(:, 4)));

    KNNs = trainKNNs(trainData, trainLbls, noNeighbors, clusters);
    acc(i) = testKNNVotes(KNNs, testData, testLbls, classes);


%     KNN = ClassificationKNN.fit(trainData, trainLbls(:, 3), ...
%         'DistanceWeight', 'squaredinverse', 'NumNeighbors', noNeighbors);
%     answ = predict(KNN, testData);
%     acc(i) = sum(strcmpi(answ, testLbls(:, 3)))/split.TestSize(i)
    
end
toc;
fprintf('\nMean Accuracy for 10-fold crossvalidation: %6.4f\n', mean(acc));
    accuracy(x) = mean(acc);
end
fprintf('\n10 runs accuracy: %6.4f\n', mean(accuracy));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% train somN x somN KNNs based on SOM's clustering

fprintf('\nTraining %d KNNs...', noClusters); tic;
KNNs{noClusters} = []; % preallocate

% for each cluster
for i = 1:noClusters
    % select observations
    idx = cell2mat(wrkLbls(:, 4)) == i;
    KNNdata = wrkData(idx, :);
    KNNlbls = wrkLbls(idx, 3);
    % train KNN
    KNNs{i} = ClassificationKNN.fit(KNNdata, KNNlbls);
    KNNs{i}.NumNeighbors = noNeighbors;
end
fprintf('done\n'); toc;
clear i idx KNNdata KNNlbls;
%% for each frame, measure prediction of majority voting

fprintf('\nTesting majority voting for each patch...'); tic;
results = zeros(noFrames, 1);

% for each frame
for i = 1:noFrames 
    % select patches of this frame
    idx = strcmp(wrkLbls(:, 1), frames(i));
    frameData = wrkData(idx, :);
    frameLbls = wrkLbls(idx, :);
    trueClass = frameLbls(1, 3);
    
    % this should be evaluated after each frame is captured from sensor
    patchLabels = cell2mat(wrkLbls(idx, 4)); 
    noPatches = numel(patchLabels);
    
    % take each patch and evaluate prediction
    countVotes = zeros(noClasses, 1);
    for j = 1:noPatches
        % select classifier for estimated cluster and evaluate patch
        vote = predict(KNNs{patchLabels(j)}, frameData(j, :));
        % add 1 vote for predicted class
        for k = 1:noClasses
            countVotes(k) = countVotes(k) + strcmpi(classes{k}, vote);
        end
    end
    [~, ind] = max(countVotes);
    results(i) = strcmpi(classes(ind), trueClass);
end
error = sum(results) / noFrames;
fprintf('done\nAccuracy: %6.4f <--------\n', error); toc;
clear i j k idx frameData frameLbls patchLabels triggerKNNs countVotes;
clear voteFrom clusterData vote ind trueClass;

%%
% clsz = unique(ESFlbls5cm(:, 3));
% lblz = zeros(numel(ESFlbls5cm(:, 3)), 1);
% for i = 1:numel(clsz)
%     lblz = lblz + strcmp(classes(i), ESFlbls5cm(:, 3)) * i;
% end
% csvwrite('ESF5cm8classes.csv', [ESFdata5cm, lblz] )