function plotFeaturesPerCluster(dataMatrix, figTitle, legendNames)
figure1 = figure;
axes1 = axes('Parent',figure1,'XTickLabel',{'1','3','5','7'}, 'XTick',[1 2 3 4]);
xlim(axes1,[0.9 4.1]);
ylim(axes1,[0.75 1]);

box(axes1,'off');
hold(axes1,'all');

% Create multiple lines using matrix input to plot
plot1 = plot(dataMatrix,'Parent',axes1);
% grid on;
% set(gca, 'GridLineStyle', ':');
% set(axes1,'XGrid','on','YGrid','off','ZGrid','off');

% set(plot1, 'LineWidth',1.5);
% set(plot1, 'MarkerFaceColor',[1 1 0]);
% set(plot1, 'MarkerEdgeColor','k');

set(plot1(1),'Marker','o','LineStyle','-','Color',[1 0 0],'DisplayName',legendNames{1});
set(plot1(2),'Marker','o','LineStyle','--','Color',[1 0 0],'DisplayName',legendNames{2});
set(plot1(3),'Marker','o','LineStyle',':','Color',[1 0 0], 'DisplayName',legendNames{3});
set(plot1(4),'Marker','square','LineStyle','-','Color',[0 0 1],'DisplayName',legendNames{4});
set(plot1(5),'Marker','square','LineStyle','--','Color',[0 0 1],'DisplayName',legendNames{5});
set(plot1(6),'Marker','square','LineStyle',':','Color',[0 0 1],'DisplayName',legendNames{6});
set(plot1(7),'Marker','diamond','LineStyle','-','Color',[0.17 0.51 0.34],'DisplayName',legendNames{7});
set(plot1(8),'Marker','diamond','LineStyle','--','Color',[0.17 0.51 0.34],'DisplayName',legendNames{8});
set(plot1(9),'Marker','diamond','LineStyle',':','Color',[0.17 0.51 0.34], 'DisplayName',legendNames{9});

% Create xlabel
xlabel('# Neighbors K');
ylabel('Micro F1-Score');

if nargin > 2
    legend1 = legend(axes1,'show');
    set(legend1,'Location','NorthEastOutside');
end
title(figTitle);