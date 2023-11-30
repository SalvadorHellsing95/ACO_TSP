%Ant Colony Optimization

%Declare the random numbers

%OCTAVE PACKAGE LOAD
pkg load statistics;

clc; clear all; close all;

startNode = 1;

numIt = 100;
numAnts = 20;%Number of ants
numNodes = 100; %Number of nodes
alpha = 1; %constant of phermone weight
beta = 1.5; %constance of distance weight
evRate = 0.1; %Evaporation Rate

bestCost = inf;

% TEST FOR CONSISTENCY
%x=[82 91 12 92 63 9 28 55 96 97 15 98 96 49 80 14 42 92 80 96];

%y=[66 3 85 94 68 76 75 39 66 17 71 3 27 4 9 83 70 32 95 3];

%nodes(:, 1) = x;
%nodes(:, 2) = y;

%Random Nodes
x1 = -5; x2 = 5;
y1 = -5; y2 = 5;

nodes(:, 1) = x1 + rand(numNodes,1)*(x2-x1);
nodes(:, 2) = y1 + rand(numNodes,1)*(y2-y1);

bestPath = [1:numNodes]; % JUST FOR THE GRAPH
xBest = nodes([bestPath, bestPath(1)], 1);
yBest = nodes([bestPath, bestPath(1)], 2);

figure()
plot(nodes(:,1),nodes(:,2),'*');
hold on;

%OCTAVE
plot(xBest, yBest, '-o','xdatasource', 'xBest', 'ydatasource', 'yBest');


%MATLAB
%bestPathPlot = plot(xBest, yBest, '-o');
%bestPathPlot = plot(xBest, yBest, '-o');
%bestPathPlot.XDataSource = 'xBest';
%bestPathPlot.YDataSource = 'yBest';

%Cost matrix (distance between points)
costMatrix = zeros(numNodes, numNodes);


%%phermone matrix
phermones = zeros(numNodes, numNodes) + 0.2;

%Calculate cost matrix
for i = 1:numNodes
  for j = i+1:numNodes
    costMatrix(i,j) = pdist([nodes(i,:);nodes(j,:)], 'euclidean');
    costMatrix(j,i) = costMatrix(i,j);
  endfor
end


%START GLOBAL ITERATIONS
for iteration = 1:numIt
  antPaths = zeros(numAnts, numNodes);
  antTotalCost = zeros(1, numAnts);
  %%START ITERATION PER ANT

  for currentAnt = 1 : numAnts

    currentNode = startNode;

    visitedNodes = currentNode;

    %Nodes Iteraion
    for i = 1:numNodes-1
      %%Calculate the next node
      %first, calculate all the prob
      unvisitedNodes = setdiff(1:numNodes, visitedNodes);

       %Calculate the probability of the unvisited nodes

      probabilities = (phermones(currentNode, unvisitedNodes).^alpha) .* ...
                    (1./costMatrix(currentNode, unvisitedNodes, :).^beta);

                    %% costMatrix(currentNode, unvisitedNodes, :)

      probabilities = probabilities / sum(probabilities);

      %get the random index of the unvisited nodes based on their probabilities
      [~, index] = histc(rand(), [0, cumsum(probabilities)]);
      nextNode = unvisitedNodes(index);


      %update the ant path
      antPaths(currentAnt, i) = currentNode;
      antTotalCost(currentAnt) = antTotalCost(currentAnt) + costMatrix(currentNode, nextNode, :);

      %update the unvisitedNodes
      currentNode = nextNode;
      visitedNodes = [ visitedNodes, currentNode];

    endfor
    antPaths(currentAnt, numNodes) = currentNode; % put also the last node

  endfor

  %UPDATE THE PHEROMONE
  phermones = (1-evRate) * phermones;

  %UPDATE BASES ON PATH
  for currentAnt = 1:numAnts
    for i = 1:numNodes-1
      phermones(antPaths(currentAnt, i), antPaths(currentAnt, i+1)) = ...
                  phermones(antPaths(currentAnt, i), antPaths(currentAnt, i+1)) + 1 / antTotalCost(currentAnt);

      phermones(antPaths(currentAnt, i+1), antPaths(currentAnt, i)) = phermones(antPaths(currentAnt, i), antPaths(currentAnt, i+1));
    endfor
  end

  [minLength, minIndex] = min(antTotalCost);
  if minLength < bestCost
      bestPath = antPaths(minIndex, :);
      bestCost = minLength;
  end
  startNode = antPaths(minIndex, numNodes);
  disp(['Iteration ', num2str(iteration), ': Best Cost = ', num2str(bestCost), ': Start Node =', num2str(startNode)]);

  xBest = nodes([bestPath, bestPath(1)], 1);
  yBest = nodes([bestPath, bestPath(1)], 2);

  refreshdata;

  pause(0.001)

end

disp(['Best cost =', num2str(bestCost)]);
disp(['Best Path = [', num2str(bestPath),']']);

hold off;



