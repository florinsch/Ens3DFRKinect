%% use parallel computing
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
%% choose dataset and parameters
wrkData = PFHdata2cm;
wrkLbls = PFHlbls2cm;

% number of cells WxH in SOM
somW = 6;
somH = 6;
% number of clusters
noClusters = somW * somH;
fprintf('Number of clusters: %d\n', noClusters);
% -------------------------------------------------------------------------
% sanity check
% number of complete (NOT patches) files per directory
% > for D in `find . -maxdepth 2 -mindepth 2 -type d`; \
% do echo $D ; find $D -path '*patches*' -prune -o -print | grep pcd | wc -l; done

% number of examples per patch size X per class
% > for D in `find . -mindepth 3 -type d -path '*2cm'`; \
% do echo $D ; find $D  -print | grep pcd | wc -l; done

% count #files of type pcd NOT in patches subfolders (# frame files)
% > find . -path '*patches*' -prune -o -print | grep pcd | wc -l

fprintf('\nNumber of PCD frame files in folder: ');
system('find . -path "*patches*" -prune -o -print | grep pcd | wc -l');

% folder count is 152 and here there are 148 => not unique names for frames
% generated from c++ => add random number after each frame counter in c++ ! 
% hack to generate unique frame names (concat frame + class in frame)
wrkLbls(:, 1) = strcat(wrkLbls(:, 1), wrkLbls(:,3));
% -------------------------------------------------------------------------
% get stats for current dataset

[observations, noObservations, classes, noClasses, ...
    noPatches, avgPatchesPerObs] = getDatasetStats(wrkLbls);
%% remove columns with sum = 0
wrkData = wrkData(:, (abs(sum(wrkData)) ~= 0) );
fprintf('Kept %i features\n', size(wrkData,2));
%% z-score
figure, plot(wrkData(1,:), 'r'); hold on;
wrkData = zscore(wrkData);
plot(wrkData(1,:));
%% smooth filter
figure, plot(wrkData(1,:), 'r'); hold on;
wrkData = smoothFilter(wrkData);
plot(wrkData(1,:));
%% shuffle dataset
[wrkData, wrkLbls] = shuffleObservations(wrkData, wrkLbls);
%% train SOMs
wrkLbls = patchLabelsFromSOM(wrkData, wrkLbls, somW, somH);
%%  K-Means
wrkLbls = patchLabelsFromKmeans(wrkData, wrkLbls, noClusters);
%% get class labels from patch grid ordering
[wrkData, wrkLbls] = patchLabelsFromGrid(wrkData, wrkLbls);
%% patch labels from uniform random distribution of cluster labels
fprintf('\nRandom uniform distribution of cluster labels...'); tic;
wrkLbls(:, 4) = num2cell(unidrnd(noClusters, noPatches, 1));
fprintf('done\n'); toc;
%% Clustering plots ...
noHits = zeros(1, noClusters); knownClasses = noHits;
for c = 1:noClusters
    idx = cell2mat(wrkLbls(:, 4)) == c;
    noHits(c) = sum(idx);
    knownClasses(c) = numel(unique(wrkLbls(idx, 3)));
end
[noHits,idz] = sort(noHits / norm(noHits));
% for each frame, count the number of hits per cluster
patchHits = zeros(noObservations, noClusters);
for i = 1:noObservations 
        % select patch data of this frame
        idx = strcmp(wrkLbls(:, 1), observations(i));
        votesFrom = cell2mat(wrkLbls(idx, 4));
        patchHits(i,:) = histc(votesFrom, 1:noClusters);
end
% plot
patchHits = patchHits(:,idz);
figure;
bar(noHits, 0.5, 'y'), hold on;
plot (1:noClusters, zeros(1, noClusters)+mean(noHits), '--r'), hold on;
for i = 1:noClusters
    text(i-0.1, noHits(i)+0.04, num2str(knownClasses(i)));
end
% two = errorbar(mean(patchHits), var(patchHits), '--o'); hold on;
two = plot(mean(patchHits), '--o'); hold on;
uistack(two, 'top');
plot (1:noClusters, ones(1, noClusters), ':r'), hold on;
xlabel('Cluster'), ylabel('Hits'), xlim([0.5 noClusters+0.5]);
%% visualize clusters using PCA on 3 feature dimensions
[~, score] = princomp(wrkData);
vis = score(:, 1:3);
clr = distinguishable_colors(noClusters, 'w');
figure;
for i = 1:noClusters
    clust = find(cell2mat(wrkLbls(:,4))==i);
    % plot(data(clust,1),data(clust,2), 'color',cc(i,:), 'marker', 'x', 'linestyle', 'none');
    plot3(vis(clust,1), vis(clust,2), vis(clust,3), ...
        'color', clr(i,:), 'marker', '*', 'linestyle', 'none');
    plot3(mean(vis(clust,1)), mean(vis(clust,2)), mean(vis(clust,3)), ...
        'color', clr(i,:), 'marker', 'o', 'linestyle', 'none', 'MarkerSize', 10, 'LineWidth',4);
    plot3(mean(vis(clust,1)), mean(vis(clust,2)), mean(vis(clust,3)), ...
        'color', [0 0 0], 'marker', 'x', 'linestyle', 'none', 'MarkerSize', 10, 'LineWidth',4);
    hold on;
end
%%  LOOCV

predictEns = [];
predictOne = [];
split = cvpartition(observations, 'leaveout');

noNeighbors = 3;

fprintf('\nPerforming LOOCV K_nn = %i ...', noNeighbors); tic;
parfor i = 1:split.NumTestSets
    % select patches from the testing frame
    testIdx = strcmp(wrkLbls(:,1), observations(split.test(i)));
    % use everything else for training
    trainIdx = ~testIdx;
    
    % get the data
    trainData = wrkData(trainIdx, :);
    trainLbls = wrkLbls(trainIdx, :);
    testData = wrkData(testIdx, :);
    testLbls = wrkLbls(testIdx, :);
    
    % valid classes
    classes = unique(trainLbls(:, 3));
    % valid clusters
    clusters = unique(cell2mat(trainLbls(:, 4)));

    % train classifiers
    KNNs = ensembleTrainKNN(trainData, trainLbls, noNeighbors, clusters);
    % add predictions [trueClass, voteClass, sumClass]
    predictEns = [predictEns; ensemblePredictKNN(KNNs, testData, testLbls, classes)];
    
    % train one classifier
    oneKNN = ClassificationKNN.fit(trainData, trainLbls(:, 3), ...
        'NumNeighbors', noNeighbors, ...    
        'DistanceWeight', 'squaredinverse');
    predictOne = [predictOne ;ensembleOnePredictKNN(oneKNN, testData, testLbls, classes)];

end
fprintf('done\n'); toc;

[confEns] = confusionmat(predictEns(:, 1), predictEns(:, 2));
[f1Ens, precEns, recEns] = statsPerClass(confEns);
F_MacroEns = mean(f1Ens);
F_MicroEns = 2 * (precEns .* recEns) / (precEns + recEns);

[confOne] = confusionmat(predictOne(:, 1), predictOne(:, 2));
[f1One, precOne, recOne] = statsPerClass(confOne);
F_MacroOne = mean(f1One);
F_MicroOne = 2 * (precOne .* recOne) / (precOne + recOne);
fprintf('Leave-one-out ENS Sum F1_mu:%6.4f F1_M:%6.4f\n', F_MicroEns, F_MacroEns);
fprintf('Leave-one-out ONE Sum F1_mu:%6.4f F1_M:%6.4f\n', F_MicroOne, F_MacroOne);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ensembles
%cv = fitensemble(wrkData, wrkLbls(:,3),'Bag',200,'Tree', 'type','classification','kfold', 10);

bag = fitensemble(wrkData, wrkLbls(:,3),'Bag', 50,'Tree', 'type','classification');
resbag = strcmp(predict(bag, wrkData), wrkLbls(:, 3));

disp( sum(resbag) / numel(resbag) );
%%
% clsz = unique(ESFlbls5cm(:, 3));
% lblz = zeros(numel(ESFlbls5cm(:, 3)), 1);
% for i = 1:numel(clsz)
%     lblz = lblz + strcmp(classes(i), ESFlbls5cm(:, 3)) * i;
% end
% csvwrite('ESF5cm8classes.csv', [ESFdata5cm, lblz] )