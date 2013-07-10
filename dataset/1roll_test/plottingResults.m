%% ------------------------------- PLOTTING -------------------------------
%     SOM, kmean,   RND,   ONE-KNN
% <   1-4,   5-8,  9-12,   1-4 | 2
% = 13-16, 17-20, 21-24, 13-16 | 2
% > 25-28, 29-32, 33-36, 25-28 | 2
%% 5CM clusters LOW, all features per clustering type
legendNames = {'SHOT SOM', 'SHOT KMEAN', 'SHOT 1KNN', ...
               'ESF SOM', 'ESF KMEAN', 'ESF 1KNN', ...
               'PFH SOM', 'PFH KMEAN', 'PFH 1KNN'};
low1 = [results5cmSHOT(1:4, 1), results5cmSHOT(5:8, 1), results5cmSHOT(1:4, 2)];
low2 = [results5cmESF(1:4, 1), results5cmESF(5:8, 1), results5cmESF(1:4, 2)];
low3 = [results5cmPFH(1:4, 1), results5cmPFH(5:8, 1), results5cmPFH(1:4, 2)];
low = [low1, low2, low3]; 
plotFeaturesPerCluster(low, '6 Clusters - SOM 2 x 3', legendNames);
% 5CM clusters MED, all features per clustering type 
med1 = [results5cmSHOT(13:16, 1), results5cmSHOT(17:20, 1), results5cmSHOT(13:16, 2)];
med2 = [results5cmESF(13:16, 1), results5cmESF(17:20, 1), results5cmESF(13:16, 2)];
med3 = [results5cmPFH(13:16, 1), results5cmPFH(17:20, 1), results5cmPFH(13:16, 2)];
med = [med1, med2, med3]; 
plotFeaturesPerCluster(med, '9 Clusters - SOM 3 x 3', legendNames);
% 5CM clusters HIGH, all features per clustering type 
high1 = [results5cmSHOT(25:28, 1), results5cmSHOT(29:32, 1), results5cmSHOT(25:28, 2)];
high2 = [results5cmESF(25:28, 1), results5cmESF(29:32, 1), results5cmESF(25:28, 2)];
high3 = [results5cmPFH(25:28, 1), results5cmPFH(29:32, 1), results5cmPFH(25:28, 2)];
high = [high1, high2, high3]; 
plotFeaturesPerCluster(high, '12 Clusters - SOM 3 x 4', legendNames);

%% 5CM SOM all features per cluster size
legendNames = {'SHOT <', 'SHOT =', 'SHOT >', ...
               'ESF <', 'ESF =', 'ESF >', ...
               'PFH <', 'PFH =', 'PFH >'};
som1 = [results5cmSHOT(1:4, 1), results5cmSHOT(13:16, 1), results5cmSHOT(25:28, 1)];
som2 = [results5cmESF(1:4, 1), results5cmESF(13:16, 1), results5cmESF(25:28, 1)];
som3 = [results5cmPFH(1:4, 1), results5cmPFH(13:16, 1), results5cmPFH(25:28, 1)];
som = [som1, som2, som3];
plotFeaturesPerCluster(som, 'SOM', legendNames);

% 5CM KMEAN all features per cluster size
kmean1 = [results5cmSHOT(5:8, 1), results5cmSHOT(17:20, 1), results5cmSHOT(29:32, 1)];
kmean2 = [results5cmESF(5:8, 1), results5cmESF(17:20, 1), results5cmESF(29:32, 1) ];
kmean3 = [results5cmPFH(5:8, 1), results5cmPFH(17:20, 1), results5cmPFH(29:32, 1) ];
kmean = [kmean1 , kmean2, kmean3];
plotFeaturesPerCluster(kmean, 'K-Means', legendNames);

% 5CM oneKNN all features per cluster size
oneknn1 = [results5cmSHOT(1:4, 2), results5cmSHOT(13:16, 2), results5cmSHOT(25:28, 2)];
oneknn2 = [results5cmESF(1:4, 2), results5cmESF(13:16, 2), results5cmESF(25:28, 2)];
oneknn3 = [results5cmPFH(1:4, 2), results5cmPFH(13:16, 2), results5cmPFH(25:28, 2)];
oneknn = [oneknn1, oneknn2, oneknn3];
plotFeaturesPerCluster(oneknn, 'One KNN (No Clustering)', legendNames);