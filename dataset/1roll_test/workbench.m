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
wrkData = SHOTdata5cm;
wrkLbls = SHOTlbls5cm;

% number of cells WxH in SOM
somW = 3;
somH = 3;
% number of clusters
noClusters = somW * somH;
fprintf('Number of clusters: %d\n', noClusters);
%% sanity check
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
%% get stats for current dataset

[observations, noObservations, classes, noClasses, ...
    noPatches, avgPatchesPerObs] = getDatasetStats(wrkLbls);
%% throw out features from the data which have eigenvalues == 0
covMatWrk = cov(wrkData);
[~, eigVal] = eig(covMatWrk);
wrkIdxKeep = (diag(eigVal) <= 0);
wrkData = wrkData(:, wrkIdxKeep);
%% smooth filter
plot(wrkData(1,:), 'r'); hold on;
wrkData = smoothFilter(wrkData);
plot(wrkData(1,:))
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
%% List number of hits and seen classes per cluster labeling
noHits = zeros(1, noClusters); knownClases = noHits;
%[noHits, bins] = histc(cell2mat(wrkLbls(:, 4)), 1:noClusters);
for c = 1:noClusters
    idxKeep = cell2mat(wrkLbls(:, 4)) == c;
    noHits(c) = sum(idxKeep);
    knownClases(c) = numel(unique(wrkLbls(idxKeep, 3)));
end
fprintf('\nNumber of HITS per cluster\n'); disp(noHits);
fprintf('Number of DISTINCT classes per cluster\n'); disp(knownClases);
%% for each frame, count the number of hits per cluster
patchHits = zeros(noObservations, noClusters);
for i = 1:noObservations 
        % select patch data of this frame
        idx = strcmp(wrkLbls(:, 1), observations(i));
        votesFrom = cell2mat(wrkLbls(idx, 4));
        patchHits(i,:) = histc(votesFrom, 1:noClusters);
end
figure, plot (1:noClusters, ones(1, noClusters), '.--r');
hold on, errorbar(mean(patchHits), std(patchHits), '--o');
xlabel('Cluster #'), ylabel('Mean + Std'), xlim([0.75 9.25]);
%%  LOOCV

predictEns = [];
predictOne = [];
split = cvpartition(observations, 'leaveout');

noNeighbors = 7;

fprintf('\nPerforming LOOCV...'); tic;
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
%F_MacroEns = mean(f1Ens);
F_MicroEns = 2 * (precEns .* recEns) / (precEns + recEns);

[confOne] = confusionmat(predictOne(:, 1), predictOne(:, 2));
[f1One, precOne, recOne] = statsPerClass(confOne);

F_MicroOne = 2 * (precOne .* recOne) / (precOne + recOne);
fprintf('Leave-one-out ENS Sum F1_mu:%6.4f\n', F_MicroEns);
fprintf('Leave-one-out ONE Sum F1_mu:%6.4f\n', F_MicroOne);
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