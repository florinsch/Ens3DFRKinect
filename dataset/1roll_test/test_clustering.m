hitsS = []; clszS = []; hitsK = []; clszK = [];
%% shuffle dataset
somW = 3;
somH = 3;
noClusters = somW * somH;

[wrkData, wrkLbls] = shuffleObservations(wrkData, wrkLbls);
%% train SOMs
fprintf('\nTraining SOM...'); tic;
wrkLbls = patchLabelsFromSOM(wrkData, wrkLbls, somW, somH);
fprintf('done\n'); toc;

%% List number of hits and seen classes per cluster labeling
noHits = zeros(1, noClusters); knownClases = noHits;
for c = 1:noClusters
    idx = cell2mat(wrkLbls(:, 4)) == c;
    noHits(c) = sum(idx);
    knownClases(c) = numel(unique(wrkLbls(idx, 3)));
end

hitsS = [hitsS ; noHits];
clszS = [clszS ; knownClases];
%%  K-Means
% shuffle dataset
fprintf('\nTraining K-Means...'); tic;
wrkLbls = patchLabelsFromKmeans(wrkData, wrkLbls, noClusters);
fprintf('done\n'); toc;

%% List number of hits and seen classes per cluster labeling
noHits = zeros(1, noClusters); knownClases = noHits;
for c = 1:noClusters
    idx = cell2mat(wrkLbls(:, 4)) == c;
    noHits(c) = sum(idx);
    knownClases(c) = numel(unique(wrkLbls(idx, 3)));
end
hitsK = [hitsK ; noHits];
clszK = [clszK ; knownClases];
%% for each frame, count the number of hits per cluster

fprintf('\nTraining K-Means...'); tic;
wrkLbls = patchLabelsFromKmeans(wrkData, wrkLbls, noClusters);
fprintf('done\n'); toc;

patchHits = zeros(noObservations, noClusters);
for i = 1:noObservations 
        % select patch data of this frame
        idx = strcmp(wrkLbls(:, 1), observations(i));
        votesFrom = cell2mat(wrkLbls(idx, 4));
        patchHits(i,:) = histc(votesFrom, 1:noClusters);
end
%%
figure, plot (1:noClusters, ones(1, noClusters), '.--r');
hold on, errorbar(mean(patchHits), std(patchHits), '--o');
% hold on, errorbar(mode(patchHits), std(patchHits), '--x');
xlabel('Cluster #'), ylabel('Mean + Std'), xlim([0.75 9.25]);
hold on;

% fprintf('\nTraining SOM...'); tic;
% wrkLbls = patchLabelsFromSOM(wrkData, wrkLbls, somW, somH);
% fprintf('done\n'); toc;
% patchHits = zeros(noObservations, noClusters);
% for i = 1:noObservations 
%         % select patch data of this frame
%         idx = strcmp(wrkLbls(:, 1), observations(i));
%         votesFrom = cell2mat(wrkLbls(idx, 4));
%         patchHits(i,:) = histc(votesFrom, 1:noClusters);
% end
% errorbar(mean(patchHits), std(patchHits), '--og'); hold on
% errorbar(mode(patchHits), std(patchHits), '--xg'); hold on;

plot((noHits / norm(noHits)), '-m'); hold on;
for i = 1:noClusters
    s = std(patchHits) + mean(patchHits) + 0.05;
    text(i-0.05, s(i), num2str(knownClasses(i)));
end