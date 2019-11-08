% Xbad = [31787,12444,12493.1682009674,31909]
% Xg1 = [11856,4477,4530.54095802777,11990]
Xg2 = [10408,3898,3952.50195038227,10544]
% mg2 = moment(Xg1,1)
mg2 = mean(moment(Xg2,1))
% mb = moment(Xbad,1)


% aMatrix = magic(5);
% % expression = input('Enter the name of a matrix: ','s');
% % if (exist(expression,'var'))
% %     mesh(eval(expression))
% % end
% % 
% % trial = ['s', '1', 's', '2','Time'];
% % % % Assign ambulatory task vector to variable
% % ambTask = eval(trial);
% 
% % Define subject:
% subject = input('Which subject (input sub. number)? ');
% % Define trial:
% setting = input('Which setting (enter 1, 2, or 3)? '); % 1(softest), 2, 3(stiffest)
% % Create variable name for this trial:
% trial = ['s', num2str(subject), 's', num2str(setting),'Time'];
% % Assign ambulatory task vector to variable
% ambTask = eval(trial);



