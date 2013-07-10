%% --------------------- IMPORT ALL RESULTS -------------------------------
results5cmSHOT = importAllResults('results-5cm-SHOT.csv');
results5cmESF = importAllResults('results-5cm-ESF.csv');
results5cmPFH = importAllResults('results-5cm-PFH.csv');
results4cmSHOT = importAllResults('results-4cm-SHOT.csv');
results4cmESF = importAllResults('results-4cm-ESF.csv');
results4cmPFH = importAllResults('results-4cm-PFH.csv');
results3cmSHOT = importAllResults('results-3cm-SHOT.csv');
results3cmPFH = importAllResults('results-3cm-PFH.csv');
results3cmESF = importAllResults('results-3cm-ESF.csv');
results2cmSHOT = importAllResults('results-2cm-SHOT.csv');
results2cmESF = importAllResults('results-2cm-ESF.csv');
results2cmPFH = importAllResults('results-2cm-PFH.csv');
%% ------------------------------- PLOTTING -------------------------------
%     SOM, kmean,   RND,   ONE-KNN -> (1-4) = (5-8) = (9-12)
% <   1-4,   5-8,  9-12,   1-4 | 2 
% = 13-16, 17-20, 21-24, 13-16 | 2
% > 25-28, 29-32, 33-36, 25-28 | 2
%% SOM all features per cluster size
% --- 5cm
legendNamesSize = {'SHOT <', 'SHOT =', 'SHOT >', ...
               'ESF <', 'ESF =', 'ESF >', ...
               'PFH <', 'PFH =', 'PFH >'};
som5cmSHOT = [results5cmSHOT(1:4, 1), results5cmSHOT(13:16, 1), results5cmSHOT(25:28, 1)];
som5cmESF = [results5cmESF(1:4, 1), results5cmESF(13:16, 1), results5cmESF(25:28, 1)];
som5cmPFH = [results5cmPFH(1:4, 1), results5cmPFH(13:16, 1), results5cmPFH(25:28, 1)];
som5cm = [som5cmSHOT, som5cmESF, som5cmPFH];
plotFeaturesPerCluster(som5cm, '5cm SOM', legendNamesSize);
% 5CM KMEAN all features per cluster size
kmean5cmSHOT = [results5cmSHOT(5:8, 1), results5cmSHOT(17:20, 1), results5cmSHOT(29:32, 1)];
kmean5cmESF = [results5cmESF(5:8, 1), results5cmESF(17:20, 1), results5cmESF(29:32, 1) ];
kmean5cmPFH = [results5cmPFH(5:8, 1), results5cmPFH(17:20, 1), results5cmPFH(29:32, 1) ];
kmean5cm = [kmean5cmSHOT , kmean5cmESF, kmean5cmPFH];
plotFeaturesPerCluster(kmean5cm, '5cm K-Means', legendNamesSize);
% 5CM oneKNN all features per cluster size
oneknn5cmSHOT = [results5cmSHOT(1:4, 2), results5cmSHOT(13:16, 2), results5cmSHOT(25:28, 2)];
oneknn5cmESF = [results5cmESF(1:4, 2), results5cmESF(13:16, 2), results5cmESF(25:28, 2)];
oneknn5cmPFH = [results5cmPFH(1:4, 2), results5cmPFH(13:16, 2), results5cmPFH(25:28, 2)];
oneknn5cm = [oneknn5cmSHOT, oneknn5cmESF, oneknn5cmPFH];
plotFeaturesPerCluster(oneknn5cm, '5cm No Clustering', legendNamesSize);
%% --- 4cm
som4cmSHOT = [results4cmSHOT(1:4, 1), results4cmSHOT(13:16, 1), results4cmSHOT(25:28, 1)];
som4cmESF = [results4cmESF(1:4, 1), results4cmESF(13:16, 1), results4cmESF(25:28, 1)];
som4cmPFH = [results4cmPFH(1:4, 1), results4cmPFH(13:16, 1), results4cmPFH(25:28, 1)];
som4cm = [som4cmSHOT, som4cmESF, som4cmPFH];
plotFeaturesPerCluster(som4cm, '4cm SOM', legendNamesSize);
% 4cm KMEAN all features per cluster size
kmean4cmSHOT = [results4cmSHOT(5:8, 1), results4cmSHOT(17:20, 1), results4cmSHOT(29:32, 1)];
kmean4cmESF = [results4cmESF(5:8, 1), results4cmESF(17:20, 1), results4cmESF(29:32, 1) ];
kmean4cmPFH = [results4cmPFH(5:8, 1), results4cmPFH(17:20, 1), results4cmPFH(29:32, 1) ];
kmean4cm = [kmean4cmSHOT , kmean4cmESF, kmean4cmPFH];
plotFeaturesPerCluster(kmean4cm, '4cm K-Means', legendNamesSize);
% 4cm oneKNN all features per cluster size
oneknn4cmSHOT = [results4cmSHOT(1:4, 2), results4cmSHOT(13:16, 2), results4cmSHOT(25:28, 2)];
oneknn4cmESF = [results4cmESF(1:4, 2), results4cmESF(13:16, 2), results4cmESF(25:28, 2)];
oneknn4cmPFH = [results4cmPFH(1:4, 2), results4cmPFH(13:16, 2), results4cmPFH(25:28, 2)];
oneknn4cm = [oneknn4cmSHOT, oneknn4cmESF, oneknn4cmPFH];
plotFeaturesPerCluster(oneknn4cm, '4cm No Clustering', legendNamesSize);
%% --- 3cm
som3cmSHOT = [results3cmSHOT(1:4, 1), results3cmSHOT(13:16, 1), results3cmSHOT(25:28, 1)];
som3cmESF = [results3cmESF(1:4, 1), results3cmESF(13:16, 1), results3cmESF(25:28, 1)];
som3cmPFH = [results3cmPFH(1:4, 1), results3cmPFH(13:16, 1), results3cmPFH(25:28, 1)];
som3cm = [som3cmSHOT, som3cmESF, som3cmPFH];
plotFeaturesPerCluster(som3cm, '3cm SOM', legendNamesSize);
% 3cm KMEAN all features per cluster size
kmean3cmSHOT = [results3cmSHOT(5:8, 1), results3cmSHOT(17:20, 1), results3cmSHOT(29:32, 1)];
kmean3cmESF = [results3cmESF(5:8, 1), results3cmESF(17:20, 1), results3cmESF(29:32, 1) ];
kmean3cmPFH = [results3cmPFH(5:8, 1), results3cmPFH(17:20, 1), results3cmPFH(29:32, 1) ];
kmean3cm = [kmean3cmSHOT , kmean3cmESF, kmean3cmPFH];
plotFeaturesPerCluster(kmean3cm, '3cm K-Means', legendNamesSize);
% 3cm oneKNN all features per cluster size
oneknn3cmSHOT = [results3cmSHOT(1:4, 2), results3cmSHOT(13:16, 2), results3cmSHOT(25:28, 2)];
oneknn3cmESF = [results3cmESF(1:4, 2), results3cmESF(13:16, 2), results3cmESF(25:28, 2)];
oneknn3cmPFH = [results3cmPFH(1:4, 2), results3cmPFH(13:16, 2), results3cmPFH(25:28, 2)];
oneknn3cm = [oneknn3cmSHOT, oneknn3cmESF, oneknn3cmPFH];
plotFeaturesPerCluster(oneknn3cm, '3cm No Clustering', legendNamesSize);
%% --- 2cm
som2cmSHOT = [results2cmSHOT(1:4, 1), results2cmSHOT(13:16, 1), results2cmSHOT(25:28, 1)];
som2cmESF = [results2cmESF(1:4, 1), results2cmESF(13:16, 1), results2cmESF(25:28, 1)];
som2cmPFH = [results2cmPFH(1:4, 1), results2cmPFH(13:16, 1), results2cmPFH(25:28, 1)];
som2cm = [som2cmSHOT, som2cmESF, som2cmPFH];
plotFeaturesPerCluster(som2cm, '2cm SOM', legendNamesSize);
% 2cm KMEAN all features per cluster size
kmean2cmSHOT = [results2cmSHOT(5:8, 1), results2cmSHOT(17:20, 1), results2cmSHOT(29:32, 1)];
kmean2cmESF = [results2cmESF(5:8, 1), results2cmESF(17:20, 1), results2cmESF(29:32, 1) ];
kmean2cmPFH = [results2cmPFH(5:8, 1), results2cmPFH(17:20, 1), results2cmPFH(29:32, 1) ];
kmean2cm = [kmean2cmSHOT , kmean2cmESF, kmean2cmPFH];
plotFeaturesPerCluster(kmean2cm, '2cm K-Means', legendNamesSize);
% 2cm oneKNN all features per cluster size
oneknn2cmSHOT = [results2cmSHOT(1:4, 2), results2cmSHOT(13:16, 2), results2cmSHOT(25:28, 2)];
oneknn2cmESF = [results2cmESF(1:4, 2), results2cmESF(13:16, 2), results2cmESF(25:28, 2)];
oneknn2cmPFH = [results2cmPFH(1:4, 2), results2cmPFH(13:16, 2), results2cmPFH(25:28, 2)];
oneknn2cm = [oneknn2cmSHOT, oneknn2cmESF, oneknn2cmPFH];
plotFeaturesPerCluster(oneknn2cm, '2cm No Clustering', legendNamesSize);
%% score per grid size (need to run previous plots)
%     SOM, kmean,   RND,   ONE-KNN -> (1-4) = (5-8) = (9-12)
% <   1-4,   5-8,  9-12,   1-4 | 2 
% = 13-16, 17-20, 21-24, 13-16 | 2
% > 25-28, 29-32, 33-36, 25-28 | 2
maxSom5cmSHOT = max(som5cmSHOT(:)); maxSom4cmSHOT = max(som4cmSHOT(:));
maxSom3cmSHOT = max(som3cmSHOT(:)); maxSom2cmSHOT = max(som2cmSHOT(:));
maxSomSHOT = [maxSom5cmSHOT, maxSom4cmSHOT, maxSom3cmSHOT, maxSom2cmSHOT];
maxSom5cmESF = max(som5cmESF(:)); maxSom4cmESF = max(som4cmESF(:));
maxSom3cmESF = max(som3cmESF(:)); maxSom2cmESF = max(som2cmESF(:));
maxSomESF = [maxSom5cmESF, maxSom4cmESF, maxSom3cmESF, maxSom2cmESF];
maxSom5cmPFH = max(som5cmPFH(:)); maxSom4cmPFH = max(som4cmPFH(:));
maxSom3cmPFH = max(som3cmPFH(:)); maxSom2cmPFH = max(som2cmPFH(:));
maxSomPFH = [maxSom5cmPFH, maxSom4cmPFH, maxSom3cmPFH, maxSom2cmPFH];

maxKmean5cmSHOT = max(kmean5cmSHOT(:)); maxKmean4cmSHOT = max(kmean4cmSHOT(:));
maxKmean3cmSHOT = max(kmean3cmSHOT(:)); maxKmean2cmSHOT = max(kmean2cmSHOT(:));
maxKmeanSHOT = [maxKmean5cmSHOT, maxKmean4cmSHOT, maxKmean3cmSHOT, maxKmean2cmSHOT];
maxKmean5cmESF = max(kmean5cmESF(:)); maxKmean4cmESF = max(kmean4cmESF(:));
maxKmean3cmESF = max(kmean3cmESF(:)); maxKmean2cmESF = max(kmean2cmESF(:));
maxKmeanESF = [maxKmean5cmESF, maxKmean4cmESF, maxKmean3cmESF, maxKmean2cmESF];
maxKmean5cmPFH = max(kmean5cmPFH(:)); maxKmean4cmPFH = max(kmean4cmPFH(:));
maxKmean3cmPFH = max(kmean3cmPFH(:)); maxKmean2cmPFH = max(kmean2cmPFH(:));
maxKmeanPFH = [maxKmean5cmESF, maxKmean4cmPFH, maxKmean3cmPFH, maxKmean2cmPFH];

maxOneKnn5cmSHOT = max(oneknn5cmSHOT(:)); maxOneKnn4cmSHOT = max(oneknn4cmSHOT(:));
maxOneKnn3cmSHOT = max(oneknn3cmSHOT(:)); maxOneKnn2cmSHOT = max(oneknn2cmSHOT(:));
maxOneKnnSHOT = [maxOneKnn5cmSHOT, maxOneKnn4cmSHOT, maxOneKnn3cmSHOT, maxOneKnn2cmSHOT];
maxOneKnn5cmESF = max(oneknn5cmESF(:)); maxOneKnn4cmESF = max(oneknn4cmESF(:));
maxOneKnn3cmESF = max(oneknn3cmESF(:)); maxOneKnn2cmESF = max(oneknn2cmESF(:));
maxOneKnnESF = [maxOneKnn5cmESF, maxOneKnn4cmESF, maxOneKnn3cmESF, maxOneKnn2cmESF];
maxOneKnn5cmPFH = max(oneknn5cmPFH(:)); maxOneKnn4cmPFH = max(oneknn4cmPFH(:));
maxOneKnn3cmPFH = max(oneknn3cmPFH(:)); maxOneKnn2cmPFH = max(oneknn2cmPFH(:));
maxOneKnnPFH = [maxOneKnn5cmPFH, maxOneKnn4cmPFH, maxOneKnn3cmPFH, maxOneKnn2cmPFH];

plot(maxSomSHOT, '--ro'); hold on; plot(maxSomESF, '--bs'); hold on; plot(maxSomPFH, '--md'); hold on;
plot(maxKmeanSHOT, ':ro'); hold on; plot(maxKmeanESF, ':bs'); hold on; plot(maxKmeanPFH, ':md'); hold on;
plot(maxOneKnnSHOT, '-ro'); hold on; plot(maxOneKnnESF, '-bs'); hold on; plot(maxOneKnnPFH, '-md');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% THE LINE PATTERNS SHOULD BE THE SAME REGARLDESS OF CLUSTERING, WHY NOT?
figure, plot(results5cmESF(:, 2),'-b'); hold on; plot(results5cmSHOT(:, 2),'-g'); hold on; plot(results5cmPFH(:, 2), '-r');
figure, plot(results4cmESF(:, 2),'-b'); hold on; plot(results4cmSHOT(:, 2),'-g'); hold on; plot(results4cmPFH(:, 2), '-r');
figure, plot(results3cmESF(:, 2),'-b'); hold on; plot(results3cmSHOT(:, 2),'-g'); hold on; plot(results3cmPFH(:, 2), '-r');
figure, plot(results2cmESF(:, 2),'-b'); hold on; plot(results2cmSHOT(:, 2),'-g'); hold on; plot(results2cmPFH(:, 2), '-r');
%% TESTING -- 5CM clusters LOW, all features per clustering type
legendNamesCluster = {'SHOT SOM', 'SHOT KMEAN', 'SHOT 1KNN', ...
               'ESF SOM', 'ESF KMEAN', 'ESF 1KNN', ...
               'PFH SOM', 'PFH KMEAN', 'PFH 1KNN'};
low1 = [results5cmSHOT(1:4, 1), results5cmSHOT(5:8, 1), results5cmSHOT(1:4, 2)];
low2 = [results5cmESF(1:4, 1), results5cmESF(5:8, 1), results5cmESF(1:4, 2)];
low3 = [results5cmPFH(1:4, 1), results5cmPFH(5:8, 1), results5cmPFH(1:4, 2)];
low = [low1, low2, low3]; 
plotFeaturesPerCluster(low, '6 Clusters - SOM 2 x 3', legendNamesCluster);
% 5CM clusters MED, all features per clustering type 
med1 = [results5cmSHOT(13:16, 1), results5cmSHOT(17:20, 1), results5cmSHOT(13:16, 2)];
med2 = [results5cmESF(13:16, 1), results5cmESF(17:20, 1), results5cmESF(13:16, 2)];
med3 = [results5cmPFH(13:16, 1), results5cmPFH(17:20, 1), results5cmPFH(13:16, 2)];
med = [med1, med2, med3]; 
plotFeaturesPerCluster(med, '9 Clusters - SOM 3 x 3', legendNamesCluster);
% 5CM clusters HIGH, all features per clustering type 
high1 = [results5cmSHOT(25:28, 1), results5cmSHOT(29:32, 1), results5cmSHOT(25:28, 2)];
high2 = [results5cmESF(25:28, 1), results5cmESF(29:32, 1), results5cmESF(25:28, 2)];
high3 = [results5cmPFH(25:28, 1), results5cmPFH(29:32, 1), results5cmPFH(25:28, 2)];
high = [high1, high2, high3]; 
plotFeaturesPerCluster(high, '12 Clusters - SOM 3 x 4', legendNamesCluster);
