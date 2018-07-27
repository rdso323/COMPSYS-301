%% This is a shell that you will have to follow strictly.
% You will use the plotmap() and viewmap() to display the outcome of your algorithm.

% Load sample_data_map_8, three variables will be created in your workspace. These were created as a
% result of [m,v,s]=dfs('map_8.txt',[14,1],[1,18]);
% The solution can be viewed using
% plotmap(m,s)

% write your own function for the DFS algorithm.


% This code was written after looking through the infomation provided on
% https://en.wikipedia.org/wiki/A*_search_algorithm
function [retmap,retvisited,retsteps] = dfs( mapfile,startlocation,targetlocation)

%Convert the map into a 2D array
retmap = map_convert(mapfile);
%initialising the step array to empty
retsteps = [];
%get the numer of rows and colloumns of the map
[rows, cols] = size(retmap);
%set all nodes to 1 to symbolize that they have not been evaluated
retvisited = ones(rows,cols);
% create a cell array for nodes that are on the path
path = cell(1,1);
count = 1;
%setting variables for wall and path


% Going throgh the map and if a path is found i.e. 0 then we add the node
% to the the path array which keeps track of the path
for i = 1:rows
    for j = 1:cols
        if(retmap(i,j) == 0)
            path{count,1} = [i,j];
            count = count + 1;
        end
    end
end

% These set of nodes have already been evaluated
closedSet = cell(1,1);
closeSetCount = 1;

% A set of discovered nodes that are not yet evaluated
% These set of nodes have been discovered but have not been evaluated
openSet = cell(1,1);
openSetCount = 2;

% Initially only the start node is know there fore we set it to the
% desired starting location
openSet{1} = startlocation;

% Cell array which tells us where the neighbour node i.e. the previuos node
% it was at and hence where it came from
cameFrom = cell(1,1);

% Setting all values of gScore to infinity as described in the algorithm
gScore = Inf(rows, cols);
% Setting the distance/cost of goint to the start node is 0 as you start
% there.
gScore(startlocation(1),startlocation(2)) = 0;

fScore = Inf(rows, cols);
fScore(startlocation(1),startlocation(2)) = heuristic_cost_estimate(startlocation, targetlocation);

while(~cellfun('isempty', openSet))
    current = lowestIndex(targetlocation, openSet,fScore);
    
    %if the target node is equal to the current node then exit the loop
    if(current == targetlocation)
        retvisited(targetlocation(1),targetlocation(2)) = 0;
        closedSet{closeSetCount + 1} = current;
        break;
    end
    % Removing current from openSet by looping through open set, finding
    % current and removing it
    for l= 1: numel(openSet)
        if(openSet{l} == current)
            openSet(:,l) = [];
            openSetCount = openSetCount - 1;
            break;
        end
    end
    % Add the current node to closedSet
    closedSet{closeSetCount} = current;
    % set current node to 0
    retvisited(current(1), current(2)) = 0;
    closeSetCount = closeSetCount + 1;
    % Get the neighbour nodes
    neighbours = neighbourNodes(path,current);
    % Get the number of neighbours
    [rowNodes, ~] = size(neighbours);
    for x = 1:rowNodes
        % if neighbour node is evaluated then ignore it and move on
        if(nodeFound(neighbours(x,:),closedSet))
            continue;
        end
        % The distance from start to a neighbor. 1 signifies that the
        % distance bewteen each node is going to be one
        tentitative_gScore = gScore(current(1), current(2)) + 1;
        
        % if new node is found then add it to openSet
        if(~nodeFound(neighbours(x,:),openSet))
            % Add the node to the set since it has been discovered
            openSet{openSetCount} = neighbours(x,:);
            openSetCount = openSetCount + 1;
        elseif(tentitative_gScore >= gScore(neighbours(x,1),neighbours(x,2)))
            continue;
        end
        % This is the optimal path as of now. Storing the where the node
        % camefrom, gscore and fScore
        cameFrom{neighbours(x,1),neighbours(x,2)} = current;
        gScore(neighbours(x,1),neighbours(x,2)) = tentitative_gScore;
        fScore(neighbours(x,1),neighbours(x,2)) = gScore(neighbours(x,1),neighbours(x,2)) + heuristic_cost_estimate(neighbours(x,:), targetlocation);
    end
end
% If the targetloction is reached then get the steps to best path
if(current == targetlocation)
    % Gets the steps of the best path found
    tempCurrent = current;
    while(~isequal(current, startlocation))
        current = cell2mat(cameFrom(current(1),current(2)));
        % Add the current node to the step array
        tempCurrent = [tempCurrent;current];
    end
    retsteps = flipud(tempCurrent);
end
end





% This function return the manhattan distance between 2 nodes
function [cost] = heuristic_cost_estimate(start, goal)
cost = (abs(start(1) - goal(1)) + abs(start(2) - goal(2)));
end




% This the node(index) with the lowest fScore
function [index] = lowestIndex(targetlocation, openSet, fScore)
[~, m] = size(openSet);
% set first node to lowest and set the first node as index
lowestScore = fScore(openSet{1}(1),openSet{1}(2));
index = openSet{1};
% Loop through all nodes
for i = 2:m
    % If lower fScore is found then that is set to the new lowestScore and
    % new index is set
    if(fScore(openSet{i}(1),openSet{i}(2)) < lowestScore)
        lowestScore = fScore(openSet{i}(1),openSet{i}(2));
        index = openSet{i};
        % If fScore is == to the lowest score then node that is closer to the
        % end node becomes the new lowest and new index is set
    elseif(fScore(openSet{i}(1),openSet{i}(2)) == lowestScore)
        if(heuristic_cost_estimate(openSet{i}, targetlocation) <= heuristic_cost_estimate(index, targetlocation))
            lowestScore = fScore(openSet{i}(1),openSet{i}(2));
            index = openSet{i};
        end
    end
end
end



% This function returns an array of neighbouring nodes to the current node
function [neighbourNodes] = neighbourNodes(path,current)
[row,~] = size(path);
neighbourNodes = [];
for n =1:row
    % Checking all neighbouring nodes in all derictions of the current node
    % and adding them to the array
    if(path{n,1}(1) + 1 == current(1) && path{n,1}(2) == current(2)...
            || path{n,1}(1) - 1 == current(1) && path{n,1}(2) == current(2)...
            || path{n,1}(1) == current(1) && path{n,1}(2) + 1 == current(2)...
            || path{n,1}(1) == current(1) && path{n,1}(2) - 1 == current(2))
        neighbourNodes = [neighbourNodes;path{n,1}];
    end
end
end

% This function returns a true if a node is found in a set else false
function [logic] = nodeFound(node, closedSet)
logic = false;
for i = 1:numel(closedSet)
    if(node == closedSet{i})
        logic = true;
    end
end
end












function placestep(position,i)
% This function will plot a insert yellow rectangle and also print a number in this rectangle. Use with plotmap/viewmap.
position = [16-position(1) position(2)];
position=[position(2)+0.1 position(1)+0.1];
rectangle('Position',[position,0.8,0.8],'FaceColor','y');
c=sprintf('%d',i);
text(position(1)+0.2,position(2)+0.2,c,'FontSize',10);
end
