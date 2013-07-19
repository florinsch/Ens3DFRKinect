%% --------------------- IMPORT ALL RESULTS -------------------------------
results5cmSHOT = importResultFile('results-5cm-SHOT.csv');
results5cmESF = importResultFile('results-5cm-ESF.csv');
results5cmPFH = importResultFile('results-5cm-PFH.csv');
results4cmSHOT = importResultFile('results-4cm-SHOT.csv');
results4cmESF = importResultFile('results-4cm-ESF.csv');
results4cmPFH = importResultFile('results-4cm-PFH.csv');
results3cmSHOT = importResultFile('results-3cm-SHOT.csv');
results3cmPFH = importResultFile('results-3cm-PFH.csv');
results3cmESF = importResultFile('results-3cm-ESF.csv');
results2cmSHOT = importResultFile('results-2cm-SHOT.csv');
results2cmESF = importResultFile('results-2cm-ESF.csv');
results2cmPFH = importResultFile('results-2cm-PFH.csv');
%% ------------------------------- PLOTTING -------------------------------
%     SOM, Kmean, ONE-KNN (x, 1) = Macro, (x, 2) = Micro
% <   1-4,   5-8,   9-12,
% = 13-16, 17-20,  21-24,
% > 25-28, 29-32,  33-36,
%% set F-Micro or F-Macro
F1 = 1; % Macro
% F1 = 2; % Micro

%% SOM all features per cluster size
% --- 5cm
legendNamesSize = {'SHOT <', 'SHOT =', 'SHOT >', ...
               'ESF <', 'ESF =', 'ESF >', ...
               'PFH <', 'PFH =', 'PFH >'};
som5cmSHOT = [results5cmSHOT(1:4, F1), results5cmSHOT(13:16, F1), results5cmSHOT(25:28, F1)];
som5cmESF = [results5cmESF(1:4, F1), results5cmESF(13:16, F1), results5cmESF(25:28, F1)];
som5cmPFH = [results5cmPFH(1:4, F1), results5cmPFH(13:16, F1), results5cmPFH(25:28, F1)];
som5cm = [som5cmSHOT, som5cmESF, som5cmPFH];
plotFeaturesPerCluster(som5cm, '5cm SOM', legendNamesSize);
% 5CM KMEAN all features per cluster size
kmean5cmSHOT = [results5cmSHOT(5:8, F1), results5cmSHOT(17:20, F1), results5cmSHOT(29:32, F1)];
kmean5cmESF = [results5cmESF(5:8, F1), results5cmESF(17:20, F1), results5cmESF(29:32, F1) ];
kmean5cmPFH = [results5cmPFH(5:8, F1), results5cmPFH(17:20, F1), results5cmPFH(29:32, F1) ];
kmean5cm = [kmean5cmSHOT , kmean5cmESF, kmean5cmPFH];
plotFeaturesPerCluster(kmean5cm, '5cm K-Means', legendNamesSize);
% 5CM oneKNN all features per cluster size
oneknn5cmSHOT = [results5cmSHOT(9:12, F1), results5cmSHOT(21:24, F1), results5cmSHOT(33:36, F1)];
oneknn5cmESF = [results5cmESF(9:12, F1), results5cmESF(21:24, F1), results5cmESF(33:36, F1)];
oneknn5cmPFH = [results5cmPFH(9:12, F1), results5cmPFH(21:24, F1), results5cmPFH(33:36, F1)];
oneknn5cm = [oneknn5cmSHOT, oneknn5cmESF, oneknn5cmPFH];
plotFeaturesPerCluster(oneknn5cm, '5cm No Clustering', legendNamesSize);
%% --- 4cm
som4cmSHOT = [results4cmSHOT(1:4, F1), results4cmSHOT(13:16, F1), results4cmSHOT(25:28, F1)];
som4cmESF = [results4cmESF(1:4, F1), results4cmESF(13:16, F1), results4cmESF(25:28, F1)];
som4cmPFH = [results4cmPFH(1:4, F1), results4cmPFH(13:16, F1), results4cmPFH(25:28, F1)];
som4cm = [som4cmSHOT, som4cmESF, som4cmPFH];
plotFeaturesPerCluster(som4cm, '4cm SOM', legendNamesSize);
% 4cm KMEAN all features per cluster size
kmean4cmSHOT = [results4cmSHOT(5:8, F1), results4cmSHOT(17:20, F1), results4cmSHOT(29:32, F1)];
kmean4cmESF = [results4cmESF(5:8, F1), results4cmESF(17:20, F1), results4cmESF(29:32, F1) ];
kmean4cmPFH = [results4cmPFH(5:8, F1), results4cmPFH(17:20, F1), results4cmPFH(29:32, F1) ];
kmean4cm = [kmean4cmSHOT , kmean4cmESF, kmean4cmPFH];
plotFeaturesPerCluster(kmean4cm, '4cm K-Means', legendNamesSize);
% 4cm oneKNN all features per cluster size
oneknn4cmSHOT = [results4cmSHOT(9:12, F1), results4cmSHOT(21:24, F1), results4cmSHOT(33:36, F1)];
oneknn4cmESF = [results4cmESF(9:12, F1), results4cmESF(21:24, F1), results4cmESF(33:36, F1)];
oneknn4cmPFH = [results4cmPFH(9:12, F1), results4cmPFH(21:24, F1), results4cmPFH(33:36, F1)];
oneknn4cm = [oneknn4cmSHOT, oneknn4cmESF, oneknn4cmPFH];
plotFeaturesPerCluster(oneknn4cm, '4cm No Clustering', legendNamesSize);
%% --- 3cm
som3cmSHOT = [results3cmSHOT(1:4, F1), results3cmSHOT(13:16, F1), results3cmSHOT(25:28, F1)];
som3cmESF = [results3cmESF(1:4, F1), results3cmESF(13:16, F1), results3cmESF(25:28, F1)];
som3cmPFH = [results3cmPFH(1:4, F1), results3cmPFH(13:16, F1), results3cmPFH(25:28, F1)];
som3cm = [som3cmSHOT, som3cmESF, som3cmPFH];
plotFeaturesPerCluster(som3cm, '3cm SOM', legendNamesSize);
% 3cm KMEAN all features per cluster size
kmean3cmSHOT = [results3cmSHOT(5:8, F1), results3cmSHOT(17:20, F1), results3cmSHOT(29:32, F1)];
kmean3cmESF = [results3cmESF(5:8, F1), results3cmESF(17:20, F1), results3cmESF(29:32, F1) ];
kmean3cmPFH = [results3cmPFH(5:8, F1), results3cmPFH(17:20, F1), results3cmPFH(29:32, F1) ];
kmean3cm = [kmean3cmSHOT , kmean3cmESF, kmean3cmPFH];
plotFeaturesPerCluster(kmean3cm, '3cm K-Means', legendNamesSize);
% 3cm oneKNN all features per cluster size
oneknn3cmSHOT = [results3cmSHOT(9:12, F1), results3cmSHOT(21:24, F1), results3cmSHOT(33:36, F1)];
oneknn3cmESF = [results3cmESF(9:12, F1), results3cmESF(21:24, F1), results3cmESF(33:36, F1)];
oneknn3cmPFH = [results3cmPFH(9:12, F1), results3cmPFH(21:24, F1), results3cmPFH(33:36, F1)];
oneknn3cm = [oneknn3cmSHOT, oneknn3cmESF, oneknn3cmPFH];
plotFeaturesPerCluster(oneknn3cm, '3cm No Clustering', legendNamesSize);
%% --- 2cm
som2cmSHOT = [results2cmSHOT(1:4, F1), results2cmSHOT(13:16, F1), results2cmSHOT(25:28, F1)];
som2cmESF = [results2cmESF(1:4, F1), results2cmESF(13:16, F1), results2cmESF(25:28, F1)];
som2cmPFH = [results2cmPFH(1:4, F1), results2cmPFH(13:16, F1), results2cmPFH(25:28, F1)];
som2cm = [som2cmSHOT, som2cmESF, som2cmPFH];
plotFeaturesPerCluster(som2cm, '2cm SOM', legendNamesSize);
% 2cm KMEAN all features per cluster size
kmean2cmSHOT = [results2cmSHOT(5:8, F1), results2cmSHOT(17:20, F1), results2cmSHOT(29:32, F1)];
kmean2cmESF = [results2cmESF(5:8, F1), results2cmESF(17:20, F1), results2cmESF(29:32, F1) ];
kmean2cmPFH = [results2cmPFH(5:8, F1), results2cmPFH(17:20, F1), results2cmPFH(29:32, F1) ];
kmean2cm = [kmean2cmSHOT , kmean2cmESF, kmean2cmPFH];
plotFeaturesPerCluster(kmean2cm, '2cm K-Means', legendNamesSize);
% 2cm oneKNN all features per cluster size
oneknn2cmSHOT = [results2cmSHOT(9:12, F1), results2cmSHOT(21:24, F1), results2cmSHOT(33:36, F1)];
oneknn2cmESF = [results2cmESF(9:12, F1), results2cmESF(21:24, F1), results2cmESF(33:36, F1)];
oneknn2cmPFH = [results2cmPFH(9:12, F1), results2cmPFH(21:24, F1), results2cmPFH(33:36, F1)];
oneknn2cm = [oneknn2cmSHOT, oneknn2cmESF, oneknn2cmPFH];
plotFeaturesPerCluster(oneknn2cm, '2cm No Clustering', legendNamesSize);
%% score per grid size (need to run previous plots)
meanSom5cmSHOT = mean(som5cmSHOT(:)); meanSom4cmSHOT = mean(som4cmSHOT(:));
meanSom3cmSHOT = mean(som3cmSHOT(:)); meanSom2cmSHOT = mean(som2cmSHOT(:));
meanSomSHOT = [meanSom5cmSHOT, meanSom4cmSHOT, meanSom3cmSHOT, meanSom2cmSHOT];
meanSom5cmESF = mean(som5cmESF(:)); meanSom4cmESF = mean(som4cmESF(:));
meanSom3cmESF = mean(som3cmESF(:)); meanSom2cmESF = mean(som2cmESF(:));
meanSomESF = [meanSom5cmESF, meanSom4cmESF, meanSom3cmESF, meanSom2cmESF];
meanSom5cmPFH = mean(som5cmPFH(:)); meanSom4cmPFH = mean(som4cmPFH(:));
meanSom3cmPFH = mean(som3cmPFH(:)); meanSom2cmPFH = mean(som2cmPFH(:));
meanSomPFH = [meanSom5cmPFH, meanSom4cmPFH, meanSom3cmPFH, meanSom2cmPFH];

meanKmean5cmSHOT = mean(kmean5cmSHOT(:)); meanKmean4cmSHOT = mean(kmean4cmSHOT(:));
meanKmean3cmSHOT = mean(kmean3cmSHOT(:)); meanKmean2cmSHOT = mean(kmean2cmSHOT(:));
meanKmeanSHOT = [meanKmean5cmSHOT, meanKmean4cmSHOT, meanKmean3cmSHOT, meanKmean2cmSHOT];
meanKmean5cmESF = mean(kmean5cmESF(:)); meanKmean4cmESF = mean(kmean4cmESF(:));
meanKmean3cmESF = mean(kmean3cmESF(:)); meanKmean2cmESF = mean(kmean2cmESF(:));
meanKmeanESF = [meanKmean5cmESF, meanKmean4cmESF, meanKmean3cmESF, meanKmean2cmESF];
meanKmean5cmPFH = mean(kmean5cmPFH(:)); meanKmean4cmPFH = mean(kmean4cmPFH(:));
meanKmean3cmPFH = mean(kmean3cmPFH(:)); meanKmean2cmPFH = mean(kmean2cmPFH(:));
meanKmeanPFH = [meanKmean5cmESF, meanKmean4cmPFH, meanKmean3cmPFH, meanKmean2cmPFH];

meanOneKnn5cmSHOT = mean(oneknn5cmSHOT(:)); meanOneKnn4cmSHOT = mean(oneknn4cmSHOT(:));
meanOneKnn3cmSHOT = mean(oneknn3cmSHOT(:)); meanOneKnn2cmSHOT = mean(oneknn2cmSHOT(:));
meanOneKnnSHOT = [meanOneKnn5cmSHOT, meanOneKnn4cmSHOT, meanOneKnn3cmSHOT, meanOneKnn2cmSHOT];
meanOneKnn5cmESF = mean(oneknn5cmESF(:)); meanOneKnn4cmESF = mean(oneknn4cmESF(:));
meanOneKnn3cmESF = mean(oneknn3cmESF(:)); meanOneKnn2cmESF = mean(oneknn2cmESF(:));
meanOneKnnESF = [meanOneKnn5cmESF, meanOneKnn4cmESF, meanOneKnn3cmESF, meanOneKnn2cmESF];
meanOneKnn5cmPFH = mean(oneknn5cmPFH(:)); meanOneKnn4cmPFH = mean(oneknn4cmPFH(:));
meanOneKnn3cmPFH = mean(oneknn3cmPFH(:)); meanOneKnn2cmPFH = mean(oneknn2cmPFH(:));
meanOneKnnPFH = [meanOneKnn5cmPFH, meanOneKnn4cmPFH, meanOneKnn3cmPFH, meanOneKnn2cmPFH];

plot(meanSomSHOT, '--ro'); hold on; plot(meanSomESF, '--bs'); hold on; plot(meanSomPFH, '--md'); hold on;
plot(meanKmeanSHOT, ':ro'); hold on; plot(meanKmeanESF, ':bs'); hold on; plot(meanKmeanPFH, ':md'); hold on;
plot(meanOneKnnSHOT, '-ro'); hold on; plot(meanOneKnnESF, '-bs'); hold on; plot(meanOneKnnPFH, '-md');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%all features per clustering type
%% 5CM
figure1 = figure;
axes1 = axes('Parent',figure1,'XTickLabel',{'1','3','5','7'}, 'XTick',[1 2 3 4]);
xlim(axes1,[0.9 4.1]);
ylim(axes1,[0.5 1]);

box(axes1,'on');
hold(axes1,'all');

legendNamesCluster = {'SHOT SOM', 'SHOT KMEAN', 'SHOT 1KNN', ...
               'ESF SOM', 'ESF KMEAN', 'ESF 1KNN', ...
               'PFH SOM', 'PFH KMEAN', 'PFH 1KNN'};
low1 = [results5cmSHOT(1:4, F1), results5cmSHOT(5:8, F1), results5cmSHOT(9:12, F1)];
low2 = [results5cmESF(1:4, F1), results5cmESF(5:8, F1), results5cmESF(9:12, F1)];
low3 = [results5cmPFH(1:4, F1), results5cmPFH(5:8, F1), results5cmPFH(9:12, F1)];
low = [low1, low2, low3];
subplot 231;
plotFeaturesPerCluster(low, '6 Clusters - SOM 2 x 3', legendNamesCluster);
% clusters MED, all features per clustering type 
med1 = [results5cmSHOT(13:16, F1), results5cmSHOT(17:20, F1), results5cmSHOT(21:24, F1)];
med2 = [results5cmESF(13:16, F1), results5cmESF(17:20, F1), results5cmESF(21:24, F1)];
med3 = [results5cmPFH(13:16, F1), results5cmPFH(17:20, F1), results5cmPFH(21:24, F1)];
med = [med1, med2, med3]; 
subplot 232;
plotFeaturesPerCluster(med, '9 Clusters - SOM 3 x 3', legendNamesCluster);
% clusters HIGH, all features per clustering type 
high1 = [results5cmSHOT(25:28, F1), results5cmSHOT(29:32, F1), results5cmSHOT(33:36, F1)];
high2 = [results5cmESF(25:28, F1), results5cmESF(29:32, F1), results5cmESF(33:36, F1)];
high3 = [results5cmPFH(25:28, F1), results5cmPFH(29:32, F1), results5cmPFH(33:36, F1)];
subplot 233;
high = [high1, high2, high3]; 
plotFeaturesPerCluster(high, '12 Clusters - SOM 3 x 4', legendNamesCluster);
% 4cm
low1 = [results4cmSHOT(1:4, F1), results4cmSHOT(5:8, F1), results4cmSHOT(9:12, F1)];
low2 = [results4cmESF(1:4, F1), results4cmESF(5:8, F1), results4cmESF(9:12, F1)];
low3 = [results4cmPFH(1:4, F1), results4cmPFH(5:8, F1), results4cmPFH(9:12, F1)];
low = [low1, low2, low3];
subplot 234;
plotFeaturesPerCluster(low, '9 Clusters - SOM 3 x 3', legendNamesCluster);
% MED, all features per clustering type 
med1 = [results4cmSHOT(13:16, F1), results4cmSHOT(17:20, F1), results4cmSHOT(21:24, F1)];
med2 = [results4cmESF(13:16, F1), results4cmESF(17:20, F1), results4cmESF(21:24, F1)];
med3 = [results4cmPFH(13:16, F1), results4cmPFH(17:20, F1), results4cmPFH(21:24, F1)];
med = [med1, med2, med3];
subplot 235;
plotFeaturesPerCluster(med, '12 Clusters - SOM 3 x 4', legendNamesCluster);
% HIGH, all features per clustering type 
high1 = [results4cmSHOT(25:28, F1), results4cmSHOT(29:32, F1), results4cmSHOT(33:36, F1)];
high2 = [results4cmESF(25:28, F1), results4cmESF(29:32, F1), results4cmESF(33:36, F1)];
high3 = [results4cmPFH(25:28, F1), results4cmPFH(29:32, F1), results4cmPFH(33:36, F1)];
high = [high1, high2, high3];
subplot 236;
plotFeaturesPerCluster(high, '16 Clusters - SOM 4 x 4', legendNamesCluster);
%% 3cm
figure1 = figure;
axes1 = axes('Parent',figure1,'XTickLabel',{'1','3','5','7'}, 'XTick',[1 2 3 4]);
xlim(axes1,[0.9 4.1]);
ylim(axes1,[0.5 1]);

box(axes1,'on');
hold(axes1,'all');
low1 = [results3cmSHOT(1:4, F1), results3cmSHOT(5:8, F1), results3cmSHOT(9:12, F1)];
low2 = [results3cmESF(1:4, F1), results3cmESF(5:8, F1), results3cmESF(9:12, F1)];
low3 = [results3cmPFH(1:4, F1), results3cmPFH(5:8, F1), results3cmPFH(9:12, F1)];
low = [low1, low2, low3];
subplot 231;
plotFeaturesPerCluster(low, '16 Clusters - SOM 4 x 4', legendNamesCluster);
% clusters MED, all features per clustering type 
med1 = [results3cmSHOT(13:16, F1), results3cmSHOT(17:20, F1), results3cmSHOT(21:24, F1)];
med2 = [results3cmESF(13:16, F1), results3cmESF(17:20, F1), results3cmESF(21:24, F1)];
med3 = [results3cmPFH(13:16, F1), results3cmPFH(17:20, F1), results3cmPFH(21:24, F1)];
med = [med1, med2, med3]; 
subplot 232;
plotFeaturesPerCluster(med, '20 Clusters - SOM 4 x 5', legendNamesCluster);
% clusters HIGH, all features per clustering type 
high1 = [results3cmSHOT(25:28, F1), results3cmSHOT(29:32, F1), results3cmSHOT(33:36, F1)];
high2 = [results3cmESF(25:28, F1), results3cmESF(29:32, F1), results3cmESF(33:36, F1)];
high3 = [results3cmPFH(25:28, F1), results3cmPFH(29:32, F1), results3cmPFH(33:36, F1)];
high = [high1, high2, high3]; 
subplot 233;
plotFeaturesPerCluster(high, '25 Clusters - SOM 5 x 5', legendNamesCluster);
% 2cm
low1 = [results2cmSHOT(1:4, F1), results2cmSHOT(5:8, F1), results2cmSHOT(9:12, F1)];
low2 = [results2cmESF(1:4, F1), results2cmESF(5:8, F1), results2cmESF(9:12, F1)];
low3 = [results2cmPFH(1:4, F1), results2cmPFH(5:8, F1), results2cmPFH(9:12, F1)];
low = [low1, low2, low3];
subplot 234;
plotFeaturesPerCluster(low, '36 Clusters - SOM 6 x 6', legendNamesCluster);
% MED, all features per clustering type 
med1 = [results2cmSHOT(13:16, F1), results2cmSHOT(17:20, F1), results2cmSHOT(21:24, F1)];
med2 = [results2cmESF(13:16, F1), results2cmESF(17:20, F1), results2cmESF(21:24, F1)];
med3 = [results2cmPFH(13:16, F1), results2cmPFH(17:20, F1), results2cmPFH(21:24, F1)];
med = [med1, med2, med3];
subplot 235;
plotFeaturesPerCluster(med, '42 Clusters - SOM 6 x 7', legendNamesCluster);
% HIGH, all features per clustering type 
high1 = [results2cmSHOT(25:28, F1), results2cmSHOT(29:32, F1), results2cmSHOT(33:36, F1)];
high2 = [results2cmESF(25:28, F1), results2cmESF(29:32, F1), results2cmESF(33:36, F1)];
high3 = [results2cmPFH(25:28, F1), results2cmPFH(29:32, F1), results2cmPFH(33:36, F1)];
high = [high1, high2, high3];
subplot 236;
plotFeaturesPerCluster(high, '49 Clusters - SOM 7 x 7', legendNamesCluster);