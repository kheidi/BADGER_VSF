%% OVERALL CODE DESCRIPTION

% This code is intended to calculate the EMAMA values for trials using the
% XSens and iPecs data collection systems.

%% SECTION 1: IMPORT DIRECTIONS AND SET UP
% This section defines the trial number and the task time points

% Import "ambulationTaskTimes.xlsx" file with ambulation task time stamps

% Define subject:
subject = input('Which subject (input sub. number)? ');
% Define trial:
setting = input('Which setting (enter 1, 2, or 3)? '); % 1(softest), 2, 3(stiffest)
% Create variable name for this trial:
trial = ['s', num2str(subject), 's', num2str(setting),'Time'];
% Assign ambulatory task vector to variable
ambTask = eval(trial);
% Amp foot input
ampSide = input('Which is amputee side (R or L)? ','s');

% Import iPecs trial
filename = ['/Users/jenniferleestma/Desktop/EMAMA/iPecs_processing/Processed/IP',num2str(subject),num2str(setting),'.xlsx'];
iPecsData = xlsread(filename);

%% SECTION 2: GRAPH INITIAL DATA AND THRESHOLDS
% Cut data to match up

% FIGURE 1: Raw iPecs Data
figure
subplot(2,1,1)
hold on
[a,b] = size(iPecsData);
ipLength = a;
plot(1:ipLength, iPecsData(:,3), 'r-')
plot(1:ipLength, iPecsData(:,4), 'k-')
legend('Y iPecs Force','Z iPecs Force')
xlabel('iPecs Windows')
ylabel('Force (N)')
title('Raw iPecs Data (No Zeroing)')
hold off

% These are the cuts for the iPecs data in order to zero them
% Row is subject, then y & z for each setting
cuts = [-70, 335,0,0,0,0;
              -5,-25,-15,-25,-15,-25;
              0,0,0,0,0,0;
              -3,-50,-15,-50,-15,-25];
% This actually cuts the data off and creates a vector          
ipFy = iPecsData(:,3) - cuts(subject, setting*2-1);
ipFz = iPecsData(:,4) - cuts(subject, setting*2);
sagForce = (ipFy.^2 + ipFz.^2).^(1/2);
ipWindows = 1:length(ipFy);

% These are the threshold for each data set
thresholds = [10, 20,0,0,0,0;
              10,20,10,20,10,20;
              10,20,0,20,0,20;
              0,20,0,20,0,20];          
% This creates vectors that can be graphed to show the thresholds         
thresholdFy(1:length(ipWindows),1) = thresholds(subject, setting*2-1);
thresholdFz(1:length(ipWindows),1) = thresholds(subject, setting*2);

% FIGURE 2: Zeroed iPecs Data
subplot(2,1,2)
hold on
plot(ipWindows, ipFy, 'r-')
plot(ipWindows, ipFz, 'k-')
legend('Y iPecs Force','Z iPecs Force')
xlabel('iPecs Windows')
ylabel('Force (N)')
title('Zeroed iPecs Data')          
plot(ipWindows, thresholdFy, 'r:', 'LineWidth', 2)
plot(ipWindows, thresholdFz, 'k:', 'LineWidth', 2)          
legend('Fy','Fz', 'Fy Threshold', 'Fz Threshold')
hold off

%% SECTION 3: IPECS - ID HEEL CONTACT AND TOE-OFF
% This will identify the HC and TO for both the iPecs and Xsens data

ipHCValues = []; % Sets a blank matrix to be filled by the following loops
ipTOValues = []; 

% Loops that create matrices of all heel contact and toe-off points of the
% iPecs data
for all = 1:length(ipFz)-1
    if ipFz(all,1) < thresholdFz(1) && ipFz(all+1,1) >= thresholdFz(1)
        [a,b] = size(ipHCValues);
        ipHCValues(a+1,1) = all;
    end
    if ipFz(all,1) > thresholdFz(1) && ipFz(all+1,1) <= thresholdFz(1)
        [c,d] = size(ipTOValues);
        ipTOValues(c+1,1) = all+1;
    end
end

% FIGURE 3: Graph Z Ground Reaction Force, HC & TO
figure
subplot(2,1,1)
hold on
plot(ipWindows, ipFz, 'k-')
plot(ipHCValues, ipFz(ipHCValues), 'ko', 'LineWidth',2)
plot(ipTOValues, ipFz(ipTOValues), 'ro', 'LineWidth',2)
legend('Z Force - iPecs', 'HC','TO')
xlabel('iPecs Windows')
ylabel('Force (N)')
title('iPecs Forces with Identified HC and TO')

%% SECTION 4: BRING IN XSENS FOOT POSITION DATA
% This will be used to align the heel conact point using the position in
% the z-axis of the proximal foot (ankle)

filename = ['/Users/jenniferleestma/Desktop/EMAMA/all_files/SUB_',num2str(subject),'/SET_',num2str(setting),'/proxPos',ampSide,'FT.txt'];
tempFile = importdata(filename);
heelZPos = tempFile.data;

filename = ['/Users/jenniferleestma/Desktop/EMAMA/all_files/SUB_',num2str(subject),'/SET_',num2str(setting),'/distPos',ampSide,'FT.txt'];
tempFile = importdata(filename);
toeZPos = tempFile.data;

%% SECTION 5: XSENS - ID HEEL CONTACT AND TOE-OFF

% ID heel contact using heel/ankle position data
xsensHCValues = [];
checkRange = 40;
[a,b] = size(heelZPos(:,4));
for len = checkRange + 1:a-checkRange
    counter = 0;
    for checkThese = 1:checkRange
        if heelZPos(len,4) < heelZPos(len+checkThese,4) && heelZPos(len,4) < heelZPos(len-checkThese,4)
            counter = counter + 1;
        end
    end
    if counter == checkRange;
        [a,b] = size(xsensHCValues);
        xsensHCValues(a+1,1) = len;
    end
end

% ID toe-off using toe position data
xsensTOValues = [];
checkRange = 40;
[a,b] = size(toeZPos(:,4));
for len = checkRange + 1:a-checkRange
    counter = 0;
    for checkThese = 1:checkRange
        if toeZPos(len,4) < toeZPos(len+checkThese,4) && toeZPos(len,4) < toeZPos(len-checkThese,4)
            counter = counter + 1;
        end
    end
    if counter == checkRange;
        [a,b] = size(xsensTOValues);
        xsensTOValues(a+1,1) = len;
    end
end

subplot(2,1,2)
hold on
plot(toeZPos(:,1), toeZPos(:,4), 'r-')
plot(xsensTOValues, toeZPos(xsensTOValues,4), 'ro', 'LineWidth', 2)
plot(heelZPos(:,1), heelZPos(:,4), 'k-')
plot(xsensHCValues, heelZPos(xsensHCValues,4), 'ko', 'LineWidth', 2)
legend('Toe Z Pos.', 'TO','Heel/Ankle Z Pos.', 'HC')
xlabel('iPecs Window')
ylabel('Height (m)')
title('Toe and Heel Data with Identified HC and TO Points')

%% SECTION 6: MANUALLY TRIM DATA
% These are the manual cuts that 

% iPecs starting and end values to subtract off data
ipStartValues = [2140,  0,      0;
              1045,     1860,   3728;
              0,        0,      0;
              3280,     1488,   2332];
          
ipEndValues = [34185,   0,      0;
              25986,    27943,  33040;
              0,        0,      0;
              24003,    21688,  22758];
          
% Xsens starting and end values to subtract off data          
xsensStartValues = [593,    0,      0;
                    400,    489,    609;
                    0,      0,      0;
                    494,    448,    892];
                
xsensEndValues = [  13403,  0,      0;
                    10371,  10916,  12330;
                    0,      0,      0;
                    8779,   8528,   9060];          

ipStart = ipStartValues(subject,setting);
ipEnd = ipEndValues(subject,setting);
xsensStart = xsensStartValues(subject,setting);
xsensEnd = xsensEndValues(subject,setting);

%% SECTION 7: ALIGN DATA & DEFINE TASK TIMES

% Create multiplier for stretching xsens data to iPecs data length
ipLength = ipEnd - ipStart;
xsensLength = xsensEnd - xsensStart;

% Create iPecs multipler
% Doing this because time of events is marker in xsens so want to leave
% that as is
ipMultiplier = xsensLength/ipLength;
ipAdjustedWindows = (xsensStart:ipMultiplier:xsensEnd);
ipTONew = xsensStart+(ipTOValues-ipStart)*ipMultiplier;
ipHCNew = xsensStart+(ipHCValues-ipStart)*ipMultiplier;

% FIGURE: aligned data for toe-off
figure
hold on
plot(ipAdjustedWindows, ipFz(ipStart:ipEnd), 'k-')
plot(toeZPos(xsensStart:xsensEnd,1), toeZPos(xsensStart:xsensEnd,4)*250, 'r-')
plot(ipTONew, ipFz(ipTOValues), 'ko', 'LineWidth',2)
plot(xsensTOValues, toeZPos(xsensTOValues,4), 'ro', 'LineWidth', 2)
legend('Fz iPecs','Z pos. toe','ip TO','xsens TO')
xlabel('Windows')

%% SECTION 8: ATTEMPT AT DATA ALIGNMENT CODE

% % Create multiplier for stretching xsens data to iPecs data length
% ipLength = ipEnd - ipStart;
% xsensLength = xsensEnd - xsensStart;

% Create iPecs multipler
% Doing this because time of events is marker in xsens so want to leave
% that as is

% Create multiplier for stretching xsens data to iPecs data length
ipLength = ipEnd - ipStart;
xsensLength = xsensEnd - xsensStart;

ipMultiplier = xsensLength/ipLength;
ipTONew = xsensStart+(ipTOValues-ipStart)*ipMultiplier;
ipHCNew = xsensStart+(ipHCValues-ipStart)*ipMultiplier;

% FIGURE: aligned data for toe-off
figure
hold on
plot(ipAdjustedWindows, ipFz(ipStart:ipEnd), 'k-') % plots a continuous line of the adjusted iPecs readings
plot(toeZPos(xsensStart:xsensEnd,1), toeZPos(xsensStart:xsensEnd,4)*250, 'r-') % matching xsens data lines
plot(ipTONew, ipFz(ipTOValues), 'ko', 'LineWidth',2) % plots iPecs toe-off data
plot(xsensTOValues, toeZPos(xsensTOValues,4), 'ro', 'LineWidth', 2) % plots xsens toe-off data
legend('Fz iPecs','Z pos. toe','ip TO','xsens TO')
xlabel('Windows')
title('Shifted Data Windows')
%plot(ambTask, toeZPos(ambTask,4), 'bo', 'LineWidth', 2)

%% SECTION 9: DEFINE TASK VARIABLES
% Define task variables:
urStart1 = ambTask(1); % ur: up ramp
urEnd1 = ambTask(2);
lgStart1 = ambTask(3); % lg: level ground
lgEnd1 = ambTask(4);
drStart1 = ambTask(5); % dr: down ramp
drEnd1 = ambTask(6);
urStart2 = ambTask(7);
urEnd2 = ambTask(8);
lgStart2 = ambTask(9);
lgEnd2 = ambTask(10);
usStart1 = ambTask(11); % us: up stairs
usEnd1 = ambTask(12);
usStart2 = ambTask(13); % ds: down stairs
usEnd2 = ambTask(14);
dsStart1 = ambTask(15);
dsEnd1 = ambTask(16);
dsStart2 = ambTask(17);
dsEnd2 = ambTask(18);
lgStart3 = ambTask(19);
lgEnd3 = ambTask(20);
drStart2 = ambTask(21);
drEnd2 = ambTask(22);

%% SECTION 10: CREATE NEW VECTORS TO CLEAN THINGS UP AND MAKE IT EASIER
% (xsensStart:xsensEnd) for all xsens data

filename = ['/Users/jenniferleestma/Desktop/EMAMA/all_files/SUB_', num2str(subject),'/SET_',num2str(setting),'/shankAngle',num2str(subject),num2str(setting),'.xlsx'];
shankAngle = xlsread(filename);
shankAngle = shankAngle * pi/180;
figure; hold on;
plot(1:length(shankAngle), shankAngle, 'k-')

finalXsensWindows(:,1) = xsensStart:xsensEnd;
finalXsensTO(:,1) = xsensTOValues;

finalIPWindows(:,1) = ipAdjustedWindows;
finalIPForce(:,1) = ipFy(ipStart:ipEnd);
finalIPForce(:,2) = ipFz(ipStart:ipEnd);
finalIPTO(:,1) = ipTONew;
finalIPHC(:,1) = ipHCNew;

% Create iPecs HC and TO vector for all phases being considered
finalIPHCTO = [];
for HC = 1:length(ipHCNew) % For all ip HC values
    if ipHCNew(HC,1) > urStart1 && ipHCNew(HC,1) < drEnd2 % If value is in range I'm considering
        [a,b] = size(finalIPHCTO);
        finalIPHCTO(a+1,1) = ipHCNew(HC,1); % Save relavent value in new table
        finalIPHCTO(a+1,2) = ipHCValues(HC,1); % Windows
    end
end
[a,b] = size(finalIPHCTO);
for TO = 1:length(ipTONew)
    if ipTONew(TO,1) > finalIPHCTO(1,1) && ipTONew(TO,1) < finalIPHCTO(2,1)
        starting = TO;
    end
    if ipTONew(TO,1) > finalIPHCTO(a-1,1) && ipTONew(TO,1) < finalIPHCTO(a,1)
        ending = TO+1;
    end
end
finalIPHCTO(:,4) = ipTONew(starting:ending,1);
finalIPHCTO(:,5) = ipTOValues(starting:ending,1);   

% Create xsens HC and TO vector for all phases being considered
% iPecs is more consistent so the official time values will be used from
% iPecs and approximated in xsens values
[a,b] = size(finalIPHCTO);
for len = 1:a
    num = mod(finalIPHCTO(len,1),1);
    if num > 0.5
        value = finalIPHCTO(len,1) + (1-num);
    else
        value = finalIPHCTO(len,1) - num;
    end
    finalIPHCTO(len,3) = value;
end

for len = 1:a
    num = mod(finalIPHCTO(len,4),1);
    if num > 0.5
        value = finalIPHCTO(len,4) + (1-num);
    else
        value = finalIPHCTO(len,4) - num;
    end
    finalIPHCTO(len,6) = value;
end

%% SECTION 11: MOMENT CALCULATOR

% Pre-determined variables
% Convention is position is tipped backwards
innateOffset2 = 0.1082;
innateOffset4 = 0.0647;
innateOffset = eval(['innateOffset',num2str(subject)]);

shankLength2 = 0.2516713; % In meters
shankLength4 = 0.24174289; % In meters
shankLength = eval(['shankLength',num2str(subject)]);

momentCalc(:,1) = finalIPWindows;
momentCalc(:,2:3) = finalIPForce; % Fy, Fz
momentCalc(:,4) = momentCalc(:,2)*cos(innateOffset); % component from Fy
momentCalc(:,5) = momentCalc(:,3)*sin(innateOffset)*-1; % component from Fz
momentCalc(:,6) = momentCalc(:,4) + momentCalc(:,5);
momentCalc(:,7) = momentCalc(:,6)*shankLength;

figure; hold on;
plot(momentCalc(:,1), momentCalc(:,6), 'k-')
plot(506,0, 'mo')
plot(552, 0, 'mo')

moment = momentCalc(:,7);




% 
% % Create shank angle vector approximation
% shankAngleScaled = [];
% shankAngleScaled(:,1) = ipStart+(xsensStart:xsensEnd)*1/ipMultiplier - xsensStart*1/ipMultiplier; % This is in iPecs time
% shankAngleScaled(:,2) = shankAngle(xsensStart:xsensEnd); % this is the shank angle
% [a,b] = size(shankAngleScaled);
% for len = 1:a
%     modNum = mod(shankAngleScaled(len,1),1); 
%     if modNum > 0.5
%         est = shankAngleScaled(len,1) + (1-modNum);
%     else
%         est = shankAngleScaled(len,1) - modNum;
%     end    
%     for allLarge = 1:length(ipFz)
%         shankAngleScaled(len,3) = ipFy(est); % Est of Y force
%         shankAngleScaled(len,4) = ipFz(est); % Est of Z force
%         shankAngleScaled(len,5) = est; % ipWindow
%         shankAngleScaled(len,6) = (shankAngleScaled(len,3)^2 + shankAngleScaled(len,4)^2)^0.5; % Mag of force
%     end
% end




% 
% % % Create shank angle vector approximation
% % shankAngleScaled = [];
% % shankAngleScaled(:,1) = ipStart+(xsensStart:xsensEnd)*1/ipMultiplier - xsensStart*1/ipMultiplier; % This is in iPecs time
% % shankAngleScaled(:,2) = shankAngle(xsensStart:xsensEnd); % this is the shank angle
% % [a,b] = size(shankAngleScaled);
% % for len = 1:a
% %     modNum = mod(shankAngleScaled(len,1),1); 
% %     if modNum > 0.5
% %         est = shankAngleScaled(len,1) + (1-modNum);
% %     else
% %         est = shankAngleScaled(len,1) - modNum;
% %     end    
% %     for allLarge = 1:length(ipFz)
% %         shankAngleScaled(len,3) = ipFy(est); % Est of Y force
% %         shankAngleScaled(len,4) = ipFz(est); % Est of Z force
% %         shankAngleScaled(len,5) = est; % ipWindow
% %     end
% % end
% 
% % perpForces = [];
% % for len = 1:a
% %     perpForces(len,1) =  shankAngleScaled(len,3)*cos(innateOffset); % Component from Y
% %     perpForces(len,2) =  shankAngleScaled(len,4)*sin(innateOffset)*-1;
% %     perpForces(len,3) = perpForces(len,1) + perpForces(len,2);
% %     perpForces(len,4) = perpForces(len,3) * shankLength;
% % end
% % 
% % figure; hold on;
% % plot(shankAngleScaled(:,1), perpForces(:,4), 'b-')
% % plot(1309,0, 'mo')
% % plot(1424, 0, 'mo')
% % 
% % figure; hold on;
% % plot(1:length(shankAngle), shankAngle, 'k-')
% 
% % Define R vector
% R = [];
% forceMag = [];
% nativeAngle = [];
% withInnateOffset = [];
% finalAngle = [];
% 
% angles = [];
% 
% [a,b] = size(shankAngleScaled);
% for len = 1:a
%     % 2 is Y, 3 is Z, 4 is magnitude FOR R VECTOR
%     R(len,1:3) = [0, shankLength*sin(shankAngleScaled(len,2)), shankLength*(cos(shankAngleScaled(len,2)))];
%     R(len,4) = (R(len,2)^2 + R(len,3)^2)^0.5;
%     % 1 is original, 2 is with innate, 3 is world
%     angle(len,1) = atan(shankAngleScaled(len,4)/ shankAngleScaled(len,3));
%     angle(len,2) = angle(len,1) + innateOffset;
%     angle(len,3) = angle(len,2) + shankAngleScaled(len,2);
%     
% %     nativeAngle(len,1) = atan(shankAngleScaled(len,4)/shankAngleScaled(len,3));
% %     withInnateOffset(len,1) = nativeAngle(len,1) + innateOffset;
% %     finalAngle(len,1) = withInnateOffset(len,1) +shankAngleScaled(len,2);
% end
% for len = 1:a
%     F(len,1:3) = [0, shankAngleScaled(len,6)*cos(angle(len,3)), -1*shankAngleScaled(len,6)*sin(angle(len,3))];
% end
% 
% crossRF = [];
% for len = 1:a
%     crossRF(len,1:3) = cross(R(len,1:3),F(len,1:3));
% end
% 
% figure; hold on;
% plot(shankAngleScaled(:,1), crossRF(:,1), 'r-')
% plot(1309,0, 'mo')
% plot(1424, 0, 'mo')

%% SECTION 12: UP RAMP ANALYSIS

% Define stance phases
urAnalyze = [];
[a,b] = size(finalIPHCTO);
for len = 1:a
    if finalIPHCTO(len,3) > urStart1 && finalIPHCTO(len,6) < urEnd1 || ...
            finalIPHCTO(len,3) > urStart2 && finalIPHCTO(len,6) < urEnd2
        [a,b] = size(urAnalyze);
        urAnalyze(a+1,:) = finalIPHCTO(len,:);
    end
end

% Calculate average ground reaction force over stance phase
% Column 7 is GRF
[a,b] = size(urAnalyze);
for len = 1:a
    urAnalyze(len,7) = mean(sagForce( urAnalyze(len,2):urAnalyze(len,5)));
end

% Calculate average moment over stance phase
[a,b] = size(urAnalyze);
for len = 1:a
    urAnalyze(len,8) = mean( moment(urAnalyze(len,2):urAnalyze(len,5),1));
end
urAnalyze(:,9) = urAnalyze(:,8)./urAnalyze(:,7);
urAnalyze(:,10) = urAnalyze(:,9)/0.24;

%% SECTION 13: DOWN RAMP ANALYSIS

% Define stance phases
drAnalyze = [];
[a,b] = size(finalIPHCTO);
for len = 1:a
    if finalIPHCTO(len,3) > drStart1 && finalIPHCTO(len,6) < drEnd1 || ...
            finalIPHCTO(len,3) > drStart2 && finalIPHCTO(len,6) < drEnd2
        [a,b] = size(drAnalyze);
        drAnalyze(a+1,:) = finalIPHCTO(len,:);
    end
end

% Calculate average ground reaction force over stance phase
% Column 7 is GRF
[a,b] = size(drAnalyze);
for len = 1:a
    drAnalyze(len,7) = mean(sagForce( drAnalyze(len,2):drAnalyze(len,5)));
end

% Calculate average moment over stance phase
[a,b] = size(drAnalyze);
for len = 1:a
    drAnalyze(len,8) = mean( moment(drAnalyze(len,2):drAnalyze(len,5),1));
end
drAnalyze(:,9) = drAnalyze(:,8)./drAnalyze(:,7);
drAnalyze(:,10) = drAnalyze(:,9)/0.24;

%% SECTION 14: UP STAIRS ANALYSIS

% Define stance phases
usAnalyze = [];
[a,b] = size(finalIPHCTO);
for len = 1:a
    if finalIPHCTO(len,3) > usStart1 && finalIPHCTO(len,6) < usEnd1 || ...
            finalIPHCTO(len,3) > usStart2 && finalIPHCTO(len,6) < usEnd2
        [a,b] = size(usAnalyze);
        usAnalyze(a+1,:) = finalIPHCTO(len,:);
    end
end

% Calculate average ground reaction force over stance phase
% Column 7 is GRF
[a,b] = size(usAnalyze);
for len = 1:a
    usAnalyze(len,7) = mean(sagForce( usAnalyze(len,2):usAnalyze(len,5)));
end

% Calculate average moment over stance phase
[a,b] = size(usAnalyze);
for len = 1:a
    usAnalyze(len,8) = mean( moment(usAnalyze(len,2):usAnalyze(len,5),1));
end
usAnalyze(:,9) = usAnalyze(:,8)./usAnalyze(:,7);
usAnalyze(:,10) = usAnalyze(:,9)/0.24;

%% SECTION 15: DOWN STAIRS ANALYSIS

% Define stance phases
dsAnalyze = [];
[a,b] = size(finalIPHCTO);
for len = 1:a
    if finalIPHCTO(len,3) > dsStart1 && finalIPHCTO(len,6) < dsEnd1 || ...
            finalIPHCTO(len,3) > dsStart2 && finalIPHCTO(len,6) < dsEnd2
        [a,b] = size(dsAnalyze);
        dsAnalyze(a+1,:) = finalIPHCTO(len,:);
    end
end

% Calculate average ground reaction force over stance phase
% Column 7 is GRF
[a,b] = size(dsAnalyze);
for len = 1:a
    dsAnalyze(len,7) = mean(sagForce( dsAnalyze(len,2):dsAnalyze(len,5)));
end

% Calculate average moment over stance phase
[a,b] = size(dsAnalyze);
for len = 1:a
    dsAnalyze(len,8) = mean( moment(dsAnalyze(len,2):dsAnalyze(len,5),1));
end
dsAnalyze(:,9) = dsAnalyze(:,8)./dsAnalyze(:,7);
dsAnalyze(:,10) = dsAnalyze(:,9)/0.24;

%% SECTION 16: LEVEL GROUND WALKING ANALYSIS

% Define stance phases
lgAnalyze = [];
[a,b] = size(finalIPHCTO);
for len = 1:a
    if finalIPHCTO(len,3) > lgStart1 && finalIPHCTO(len,6) < lgEnd1 || ...
            finalIPHCTO(len,3) > lgStart2 && finalIPHCTO(len,6) < lgEnd2 || ...
            finalIPHCTO(len,3) > lgStart3 && finalIPHCTO(len,6) < lgEnd3
        [a,b] = size(lgAnalyze);
        lgAnalyze(a+1,:) = finalIPHCTO(len,:);
    end
end

% Calculate average ground reaction force over stance phase
% Column 7 is GRF
[a,b] = size(lgAnalyze);
for len = 1:a
    lgAnalyze(len,7) = mean(sagForce( lgAnalyze(len,2):lgAnalyze(len,5)));
end

% Calculate average moment over stance phase
[a,b] = size(lgAnalyze);
for len = 1:a
    lgAnalyze(len,8) = mean( moment(lgAnalyze(len,2):lgAnalyze(len,5),1));
end
lgAnalyze(:,9) = lgAnalyze(:,8)./lgAnalyze(:,7);
lgAnalyze(:,10) = lgAnalyze(:,9)/0.24;
% CHECK THIS PROSTHETIC FOOT LENGTH!!!!!!
%%

FINALLG = lgAnalyze(:,10)
FINALUR = urAnalyze(:,10)
FINALDR = drAnalyze(:,10)
FINALUS = usAnalyze(:,10)
FINALDS = dsAnalyze(:,10)

