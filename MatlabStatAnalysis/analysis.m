clear all, close all, clc;
%%
load('planningData.mat');

figure(1);
subplot(3,5,1)
histogram(Eps001.nodes);
str = '$$ \epsilon = 0.01 $$';
title(str,'Interpreter','latex');
xlabel('No. of steps');
ylabel('Counts');

subplot(3,5,6)
histogram(Eps001.Time);
str = '$$ \epsilon = 0.01 $$';
title(str,'Interpreter','latex');
xlabel('Time of planning[s]');
ylabel('Counts');

subplot(3,5,11)
histogram(Eps001.PathL);
str = '$$ \epsilon = 0.01 $$';
title(str,'Interpreter','latex');
xlabel('Path Lengths');
ylabel('Counts');

subplot(3,5,2)
histogram(Eps01.nodes);
str = '$$ \epsilon = 0.1 $$';
title(str,'Interpreter','latex');
xlabel('No. of steps');
ylabel('Counts');

subplot(3,5,7)
histogram(Eps01.Time);
str = '$$ \epsilon = 0.1 $$';
title(str,'Interpreter','latex');
xlabel('Time of planning[s]');
ylabel('Counts');

subplot(3,5,12)
histogram(Eps01.PathL);
str = '$$ \epsilon = 0.1 $$';
title(str,'Interpreter','latex');
xlabel('Path Lengths');
ylabel('Counts');

subplot(3,5,3)
histogram(Eps03.nodes);
str = '$$ \epsilon = 0.3 $$';
title(str,'Interpreter','latex');
xlabel('No. of steps');
ylabel('Counts');

subplot(3,5,8)
histogram(Eps03.Time);
str = '$$ \epsilon = 0.3 $$';
title(str,'Interpreter','latex');
xlabel('Time of planning[s]');
ylabel('Counts');

subplot(3,5,13)
histogram(Eps03.PathL);
str = '$$ \epsilon = 0.3 $$';
title(str,'Interpreter','latex');
xlabel('Path Lengths');
ylabel('Counts');

subplot(3,5,4)
histogram(Eps05.nodes);
str = '$$ \epsilon = 0.5 $$';
title(str,'Interpreter','latex');
xlabel('No. of steps');
ylabel('Counts');

subplot(3,5,9)
histogram(Eps05.Time);
str = '$$ \epsilon = 0.5 $$';
title(str,'Interpreter','latex');
xlabel('Time of planning');
ylabel('Counts');

subplot(3,5,14)
histogram(Eps05.PathL);
str = '$$ \epsilon = 0.5 $$';
title(str,'Interpreter','latex');
xlabel('Path Lengths');
ylabel('Counts');

subplot(3,5,5)
histogram(Eps1.nodes);
str = '$$ \epsilon = 1 $$';
title(str,'Interpreter','latex');
xlabel('No. of steps');
ylabel('Counts');

subplot(3,5,10)
histogram(Eps1.Time);
str = '$$ \epsilon = 1 $$';
title(str,'Interpreter','latex');
xlabel('Time of planning[s]');
ylabel('Counts');

subplot(3,5,15)
histogram(Eps1.PathL);
str = '$$ \epsilon = 1 $$';
title(str,'Interpreter','latex');
xlabel('Path Lengths');
ylabel('Counts');


%% Computations
nodes = [Eps005.nodes, Eps01.nodes, Eps02.nodes, Eps03.nodes, Eps05.nodes, Eps08.nodes, Eps1.nodes];
meanNodes = mean(nodes);
Eps001.meanNodes = mean(Eps001.nodes);

PathLs = [Eps005.PathL, Eps01.PathL, Eps02.PathL, Eps03.PathL, Eps05.PathL, Eps08.PathL, Eps1.PathL];
meanLs = mean(PathLs);
Eps001.meanL = mean(Eps001.PathL);

Times = [Eps005.Time, Eps01.Time, Eps02.Time, Eps03.Time, Eps05.Time, Eps08.Time, Eps1.Time];
meanTimes = mean(Times);
Eps001.meanTime = mean(Eps001.Time);

%% Plot correlation between epsilon size and average No. of nodes
figure(2);
plot([0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1] , [Eps001.meanNodes, meanNodes], 'x');
tit = '$$ Dependence\; of\; average \; No. \; of\; Nodes\; on\; size\; of\; \epsilon $$';
title(tit, 'Interpreter', 'latex');
str = '$$ size\; of\; \epsilon $$';
xlabel(str,'Interpreter','latex');
ylabel('Avg. No. of Nodes','Interpreter', 'latex');
%% Plot correlation between epsilon size and average computation time
figure(3);
plot([0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1] , [Eps001.meanTime, meanTimes], 'x');
tit = '$$ Dependence\; of\; average \; Computation\; Time\; on\; size\; of\; \epsilon $$';
title(tit, 'Interpreter', 'latex');
str = '$$ size\; of\; \epsilon $$';
xlabel(str,'Interpreter','latex');
ylabel('Avg. Computation Time[s]','Interpreter', 'latex');

%% Plot correlation between epsilon size and average Path Length
figure(4);
plot([0.01, 0.05, 0.1, 0.2, 0.3, 0.5, 0.8, 1] , [Eps001.meanL, meanLs], 'x');
tit = '$$ Dependence\; of\; average \; Path\; Length\; on\; size\; of\; \epsilon $$';
title(tit, 'Interpreter', 'latex');
str = '$$ size\; of\; \epsilon $$';
xlabel(str,'Interpreter','latex');
ylabel('Avg. Path Length','Interpreter', 'latex');

%% Plot correlation between path length and average computation time
figure(5);
plot( [Eps001.meanL, meanLs] , [Eps001.meanTime, meanTimes], 'x');
tit = 'Correlation of avg. Path Length on avg. Computation Time';
title(tit);

xlabel('Avg. Path Length');
ylabel('Avg. Computation Time[s]');
