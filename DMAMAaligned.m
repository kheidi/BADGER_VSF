clear
%% TO-DO
%-The current error is that ipHC values are longer than moment_all? and for
%some reason momentStance appears to be only capturing TO to HC?!

%% OVERALL CODE DESCRIPTION

% This code is intended to calculate the EMAMA values for trials using the
% XSens and iPecs data collection systems.

% Author: Katherine Heidi Fehr
% Based on work by: Jennifer Leetsma

%% ** REPLACE THIS DATA WITH TRIAL INFO **

% Set subject and stiffness setting, this will find correct data in
%matrices of provided, known values
subject = 4;
setting = 1;
cd Data

allInnateOffsets = [0 0.1082 0 0.0647];
allShankLengths = [0 0.2516713 0 0.24174289]; %in meters
innateOffset = allInnateOffsets(subject);
shankLength = allShankLengths(subject);

% -----------------iPecs Data-----------------
% iPecs Filename, forces & moments in X, Y, Z
fileName=(['IP', num2str(subject), num2str(setting)]);
file_iPecs = fileName; 
% Add a row to this matrix for each subject. These are 'cuts' used to zero
%the iPecs data. If unsure of what cuts to use, run code until Figure 1 is
%generated.
%Row is subject, then y & z for each setting
iPecsCuts = [-70, 335,0,0,0,0;
              -5,-25,-15,-25,-15,-25;
              0,0,0,0,0,0;
              -3,-50,-15,-50,-15,-25];
               % 0,0,-15,-50,-15,-25];
%Similarily input iPecs Thresholds
iPecsThresholds = [10, 20,0,0,0,0;
              10,20,10,20,10,20;
              10,20,0,20,0,20;
              0,20,0,20,0,20];

% Add a row to the below matrices to manually trim the data to the correct
% stop and stopping point.
% iPecs starting and end values to subtract off data.
ipStartValues = [2140,  0,      0;
              1045,     1860,   3728;
              0,        0,      0;
              3280,     1488,   2332];
          
ipEndValues = [34185,   0,      0;
              25986,    27943,  33040;
              0,        0,      0;
              24003,    21688,  22758];

% -----------------timeTracking Data-----------------          
% timeTracking, file that holds time stamps for the different ambulation
%modes (Up ramp, level ground). These time stamps correspond to the XSENS
%time. Follow "timetracking.xlsx" formatting for correct column usage.
file_ambModeTiming = 'timeTracking';


% -----------------XSENS Data-----------------
% XSENS foot position data, used to align heel contact (and toe off) point
%using position in the z-axis of the proximal foot (ankle). REALLY ONLY
%USED FOR AMBULATION TASKS IDENTIFICATION. See 'proxPosRFT.txt' for format.
fileName=(['heel', num2str(subject), num2str(setting),'.txt']);
file_heelZPos = fileName;
fileName=(['toe', num2str(subject), num2str(setting),'.txt']);
file_toeZPos = fileName;

% Add a row (row per participant, column per setting) to the below matrices to manually trim the data to the correct
% stop and stopping point.
% XSENS starting and end values to subtract off data.
xsensStartValues = [593,    0,      0;
                    400,    489,    609;
                    0,      0,      0;
                    494,    448,    892];
                
xsensEndValues = [  13403,  0,      0;
                    10371,  10916,  12330;
                    0,      0,      0;
                    8779,   8528,   9060];  
%% SECTION 1: IMPORT DATA

% Imports the iPecs data, timetracking file, heel and toe Z position
iPecsData = xlsread(file_iPecs);
timeTrack = xlsread(file_ambModeTiming);
tempFile = importdata(file_heelZPos);
heelZPos = tempFile.data;
tempFile = importdata(file_toeZPos);
toeZPos = tempFile.data;
cd ..

%% SECTION 2: INITIAL GRAPHS
% These graphs are not trimmed in length.

% Figure 1: Raw iPecs Data
figure
subplot(2,1,1)
hold on
[a,b] = size(iPecsData);
ipLength = a;
plot(1:ipLength, iPecsData(:,3), 'r-')
plot(1:ipLength, iPecsData(:,4), 'k-')
legend('Y iPecs Force','Z iPecs Force')
xlabel('iPecs Time')
ylabel('Force (N)')
title('Raw iPecs Data (No Zeroing)')
hold off


% Apply cuts and thresholds
% This actually cuts the data off and creates a vector          
ipFy = iPecsData(:,3) - iPecsCuts(subject, setting*2-1);
ipFz = iPecsData(:,4) - iPecsCuts(subject, setting*2);
ipTime = 1:length(ipFy);

%Calculate vector with all resultant force
sagForce = (ipFy.^2 + ipFz.^2).^(1/2);

% This creates vectors that can be graphed to show the thresholds         
thresholdFy(1:length(ipTime),1) = iPecsThresholds(subject, setting*2-1);
thresholdFz(1:length(ipTime),1) = iPecsThresholds(subject, setting*2);

% Figure 2: Zeroed iPecs Data
subplot(2,1,2)
hold on
plot(ipTime, ipFy, 'r-')
plot(ipTime, ipFz, 'k-')
legend('Y iPecs Force','Z iPecs Force')
xlabel('iPecs Time')
ylabel('Force (N)')
title('Zeroed iPecs Data')          
plot(ipTime, thresholdFy, 'r:', 'LineWidth', 2)
plot(ipTime, thresholdFz, 'k:', 'LineWidth', 2)          
legend('Fy','Fz', 'Fy Threshold', 'Fz Threshold')
hold off

figure
hold on 
plot(1:ipLength, iPecsData(:,5), 'b-')
xlabel('iPecs Time')
ylabel('Moment')
title('Moment Around Knee (Mx)')
hold off
MxKnee = iPecsData(:,5);

% Figure 3: Resultant Force
figure
hold on 
plot(ipTime, sagForce);
xlabel('iPecs Time')
ylabel('Resultant Force (N)')
title('Resultant/Sagital Force over Time')
hold off

%% SECTION 4: IPECS - ID HEEL CONTACT AND TOE-OFF
% This will find the HC and TO for the iPecs based on the iPecsThresholds
%information.

ipHCValues = []; % Sets a blank matrix to be filled by the following loops
ipTOValues = []; 

% Loops that create matrices of all heel contact and toe-off points of the
% iPecs data.

% Points are determined to be HC if the point is less than the
% threshold and the next point is greater than the threshold.
% Point is TO if it is greater than the threshold and the next point is
% less than the threshold.
% These points are placed in a vector called ipHCValues or ipTOValues
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

% Figure 4: Graph Z Ground Reaction Force, HC & TO
figure
subplot(2,1,1)
hold on
plot(ipTime, ipFz, 'k-')
plot(ipHCValues, ipFz(ipHCValues), 'ko', 'LineWidth',2)
plot(ipTOValues, ipFz(ipTOValues), 'ro', 'LineWidth',2)
legend('Z Force - iPecs', 'HC','TO')
xlabel('iPecs Frame/Time')
ylabel('Force (N)')
title('iPecs Forces with Identified HC and TO')
hold off


%% SECTION 5: XSENS - ID HEEL CONTACT AND TOE-OFF 

% To detect HC and To from XSENS data this section looks for point in the
% xsens data where the checkRange value of points before and after it are
% greater than the point you are at. For example, if check range was 4:
% '555515555'. Essentially this is finding a substantial minimum. Similar
% technique is used to find TO.
xsensHCValues = [];
%The check range number might have to be adjusted, look at the XSENS Toe
%and Heel position graph to see if there are obvious points missing or if
%there are too many points. Increase the number to get rid of extra points,
%decrease to accept more points.
checkRange = 30;
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
        %This variable holds the index of each time heel contact is made
        xsensHCValues(a+1,1) = len; 
    end
end

% ID toe-off using toe position data
xsensTOValues = [];
%Also may need to be adjusted
checkRange = 22;
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
xlabel('XSENS Frame/Time')
ylabel('Height (m)')
title('XSENS Toe and Heel Data with Identified HC and TO Points')

%% SECTION 6: XSENS - GRAPH AMB MODES

%First we need to define at which point in the XSENS data the modes begin
%and end based on the info in the timeTrack file
%**Note these need to be fixed to work with different subjects files***

if subject == 1
    if setting == 1
        tTY = 2;
    elseif setting == 2
        tTY = 1;
    elseif setting == 3
        tTY = 3;
    end
end

if subject == 2
    if setting == 1
        tTY = 6;
    elseif setting == 2
        tTY = 5;
    elseif setting == 3
        tTY = 4;
    end
end

if subject == 3
    if setting == 1
        tTY = 7;
    elseif setting == 2
        tTY = 9;
    elseif setting == 3
        tTY = 8;
    end
end

if subject == 4
    if setting == 1
        tTY = 10;
    elseif setting == 2
        tTY = 12;
    elseif setting == 3
        tTY = 11;
    end
end

urStart1X = timeTrack(tTY,7); % ur: up ramp
urEnd1X = timeTrack(tTY,8);
lgStart1X = timeTrack(tTY,9); % lg: level ground
lgEnd1X = timeTrack(tTY,10);
drStart1X = timeTrack(tTY,11); % dr: down ramp
drEnd1X = timeTrack(tTY,12);
urStart2X = timeTrack(tTY,13);
urEnd2X = timeTrack(tTY,14);
lgStart2X = timeTrack(tTY,15);
lgEnd2X = timeTrack(tTY,16);
usStart1X = timeTrack(tTY,17); % us: up stairs
usEnd1X = timeTrack(tTY,18);
usStart2X = timeTrack(tTY,19); % ds: down stairs
usEnd2X = timeTrack(tTY,20);
dsStart1X = timeTrack(tTY,21);
dsEnd1X = timeTrack(tTY,22);
dsStart2X = timeTrack(tTY,23);
dsEnd2X = timeTrack(tTY,24);
lgStart3X = timeTrack(tTY,25);
lgEnd3X = timeTrack(tTY,26);
drStart2X = timeTrack(tTY,27);
drEnd2X = timeTrack(tTY,28);

% Graph Heel and Toe Position from XSENS with lines showing where each mode
%begins
figure
plot(toeZPos(:,1), toeZPos(:,4), 'r-')
hold on
plot(heelZPos(:,1), heelZPos(:,4), 'k-')
hold on
plot(xsensHCValues, heelZPos(xsensHCValues,4), 'ko', 'LineWidth', 2)
plot(xsensTOValues, toeZPos(xsensTOValues,4), 'ro', 'LineWidth', 2)
legend('Toe Z Pos.', 'Heel/Ankle Z Pos.', 'HC', 'TO')
xline(urStart1X, ':b', 'UR1 Start','HandleVisibility','off');
xline(urEnd1X, ':r', 'UR1 End','HandleVisibility','off');
xline(lgStart1X, ':b', 'LG1 Start','HandleVisibility','off');
xline(lgEnd1X, ':r', 'LG1 End','HandleVisibility','off');
xline(drStart1X, ':b', 'DR1 Start','HandleVisibility','off');
xline(urStart2X, ':b', 'UR2 Start','HandleVisibility','off');
xline(lgStart2X, ':b', 'LG2 Start','HandleVisibility','off');
xline(usStart1X, ':b', 'US1 Start','HandleVisibility','off');
xline(usStart2X, ':b', 'US2 Start','HandleVisibility','off');
xline(dsStart1X, ':b', 'DS1 Start','HandleVisibility','off');
xline(dsStart2X, ':b', 'DS2 Start','HandleVisibility','off');
xline(lgStart3X, ':b', 'LG3 Start','HandleVisibility','off');
xline(drStart2X, ':b', 'DR2 Start','HandleVisibility','off');
title('XSENS Toe and Heel Position with Ambulation Modes')

%% ALIGNMENT METHOD

%% SECTION 8: DATA ALIGNMENT CODE

% Create iPecs multipler:
% Doing this because time of events is marker in xsens so want to leave
% that as is

ipStart = ipStartValues(subject,setting);
ipEnd = ipEndValues(subject,setting);
xsensStart = xsensStartValues(subject,setting);
xsensEnd = xsensEndValues(subject,setting);

% Create multiplier for stretching xsens data to iPecs data length
ipLength = ipEnd - ipStart;
xsensLength = xsensEnd - xsensStart;

ipMultiplier = xsensLength/ipLength;
newTime = (xsensStart:ipMultiplier:xsensEnd);

% FIGURE: aligned data for toe-off
figure
hold on
plot(newTime, ipFz(ipStart:ipEnd), 'k-') % plots a continuous line of the adjusted iPecs readings
plot(toeZPos(xsensStart:xsensEnd,1), toeZPos(xsensStart:xsensEnd,4)*250, 'r-') % matching xsens data lines
plot(xsensTOValues, toeZPos(xsensTOValues,4), 'ro', 'LineWidth', 2) % plots xsens toe-off data
xlabel('Windows')
%Trying to multiply the amb mode?
urStart1X
urStart1stretch = urStart1X * ipMultiplier
xline(urStart1X, ':b', 'UR1 Start','HandleVisibility','off');
xline(urEnd1X, ':r', 'UR1 End','HandleVisibility','off');
xline(lgStart1X, ':b', 'LG1 Start','HandleVisibility','off');
xline(lgEnd1X, ':r', 'LG1 End','HandleVisibility','off');
xline(drStart1X, ':b', 'DR1 Start','HandleVisibility','off');
xline(urStart2X, ':b', 'UR2 Start','HandleVisibility','off');
xline(lgStart2X, ':b', 'LG2 Start','HandleVisibility','off');
xline(usStart1X, ':b', 'US1 Start','HandleVisibility','off');
xline(usStart2X, ':b', 'US2 Start','HandleVisibility','off');
xline(dsStart1X, ':b', 'DS1 Start','HandleVisibility','off');
xline(dsStart2X, ':b', 'DS2 Start','HandleVisibility','off');
xline(lgStart3X, ':b', 'LG3 Start','HandleVisibility','off');
xline(drStart2X, ':b', 'DR2 Start','HandleVisibility','off');
title('Shifted Data Windows')
%plot(ambTask, toeZPos(ambTask,4), 'bo', 'LineWidth', 2)

%% SECTION ?: APPLY CUTS

%Apply cuts to Data so that it aligns nicely with XSENS
ipFy = ipFy(ipStart:ipEnd);
ipFz = ipFz(ipStart:ipEnd);
MxKnee = MxKnee(ipStart:ipEnd);
clear sagForce;
sagForce = (ipFy.^2 + ipFz.^2).^(1/2);

%Get rid of HC and TO that are outside of the designated cuts

HCStart = find(ipHCValues>=ipStart,1,'first');
HCEnd = find(ipHCValues>=ipEnd,1,'first');
ipHCValues = ipHCValues(HCStart:HCEnd);

TOStart = find(ipTOValues>=ipStart,1,'first');
TOEnd = find(ipTOValues>=ipEnd,1,'first');
ipTOValues = ipTOValues(TOStart:TOEnd);

%% SECTION 3: FIND MOMENT VECTOR

fyComponent = ipFy * cos(innateOffset);
fzComponent = ipFz * sin(innateOffset)*-1;
fSum = fyComponent + fzComponent;

moment_all = fSum * shankLength - MxKnee;
moment_withXsensTime(:,1) = newTime;
moment_withXsensTime(:,2) = moment_all;

figure
yyaxis left
plot(newTime, moment_all, 'b-')
ylim([-120,120])
xlabel('iPecs Time')
ylabel('Moment')
yyaxis right
plot(toeZPos(xsensStart:xsensEnd,1), toeZPos(xsensStart:xsensEnd,4)*250, 'r-')
ylim([-1200,1200])
title('iPecs Only - Moment + Knee Moment')

%% Find Amb tasks in iPecs time

%Find function: find(condition, number or results to return, direction it 
%is searching in)

urStart1 = find(moment_withXsensTime(:,1)>=urStart1X,1,'first');
urEnd1 = find(moment_withXsensTime(:,1)>=urEnd1X,1,'first');
lgStart1 = find(moment_withXsensTime(:,1)>=lgStart1X,1,'first');
lgEnd1 = find(moment_withXsensTime(:,1)>=lgEnd1X,1,'first');
drStart1 = find(moment_withXsensTime(:,1)>=drStart1X,1,'first');
drEnd1 = find(moment_withXsensTime(:,1)>=drEnd1X,1,'first');
urStart2 = find(moment_withXsensTime(:,1)>=urStart2X,1,'first');
urEnd2 = find(moment_withXsensTime(:,1)>=urEnd2X,1,'first');
lgStart2 = find(moment_withXsensTime(:,1)>=lgStart2X,1,'first');
lgEnd2 = find(moment_withXsensTime(:,1)>=lgEnd2X,1,'first');
usStart1 = find(moment_withXsensTime(:,1)>=usStart1X,1,'first');
usEnd1 = find(moment_withXsensTime(:,1)>=usEnd1X,1,'first');
usStart2 = find(moment_withXsensTime(:,1)>=usStart2X,1,'first');
usEnd2 = find(moment_withXsensTime(:,1)>=usEnd2X,1,'first');
dsStart1 = find(moment_withXsensTime(:,1)>=dsStart1X,1,'first');
dsEnd1 = find(moment_withXsensTime(:,1)>=dsEnd1X,1,'first');
dsStart2 = find(moment_withXsensTime(:,1)>=dsStart2X,1,'first');
dsEnd2 = find(moment_withXsensTime(:,1)>=dsEnd2X,1,'first');
lgStart3 = find(moment_withXsensTime(:,1)>=lgStart3X,1,'first');
lgEnd3 = find(moment_withXsensTime(:,1)>=lgEnd3X,1,'first');
drStart2 = find(moment_withXsensTime(:,1)>=drStart2X,1,'first');
drEnd2 = find(moment_withXsensTime(:,1)>=drEnd2X,1,'first');

% Figure 4: Graph Z Ground Reaction Force, HC & TO
titleV=(['Moment with Amb Tasks: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
figure
subplot(2,1,1)
hold on
plot(moment_all, 'k-')
xline(urStart1, ':b', 'UR1 Start','HandleVisibility','off');
xline(urEnd1, ':r', 'UR1 End','HandleVisibility','off');
xline(lgStart1, ':b', 'LG1 Start','HandleVisibility','off');
xline(lgEnd1, ':r', 'LG1 End','HandleVisibility','off');
xline(drStart1, ':b', 'DR1 Start','HandleVisibility','off');
xline(urStart2, ':b', 'UR2 Start','HandleVisibility','off');
xline(lgStart2, ':b', 'LG2 Start','HandleVisibility','off');
xline(usStart1, ':b', 'US1 Start','HandleVisibility','off');
xline(usStart2, ':b', 'US2 Start','HandleVisibility','off');
xline(dsStart1, ':b', 'DS1 Start','HandleVisibility','off');
xline(dsStart2, ':b', 'DS2 Start','HandleVisibility','off');
xline(lgStart3, ':b', 'LG3 Start','HandleVisibility','off');
xline(drStart2, ':b', 'DR2 Start','HandleVisibility','off');
xlim([0 length(moment_all)])
xlabel('iPecs Frame/Time')
ylabel('Moment (Nm)')
title(titleV)
hold off

titleV=(['Toe Position in Z: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
subplot(2,1,2)
hold on
plot(toeZPos(xsensStart:xsensEnd,1), toeZPos(xsensStart:xsensEnd,4), 'r-')
xlim([xsensStart xsensEnd])
xline(urStart1X, ':b', 'UR1 Start','HandleVisibility','off');
xline(urEnd1X, ':r', 'UR1 End','HandleVisibility','off');
xline(lgStart1X, ':b', 'LG1 Start','HandleVisibility','off');
xline(lgEnd1X, ':r', 'LG1 End','HandleVisibility','off');
xline(drStart1X, ':b', 'DR1 Start','HandleVisibility','off');
xline(urStart2X, ':b', 'UR2 Start','HandleVisibility','off');
xline(lgStart2X, ':b', 'LG2 Start','HandleVisibility','off');
xline(usStart1X, ':b', 'US1 Start','HandleVisibility','off');
xline(usStart2X, ':b', 'US2 Start','HandleVisibility','off');
xline(dsStart1X, ':b', 'DS1 Start','HandleVisibility','off');
xline(dsStart2X, ':b', 'DS2 Start','HandleVisibility','off');
xline(lgStart3X, ':b', 'LG3 Start','HandleVisibility','off');
xline(drStart2X, ':b', 'DR2 Start','HandleVisibility','off');
xlabel('XSENS Frame/Time')
ylabel('Height (m)')
title(titleV)

%% SECTION NEW 5: FIND MOMENT & FORCE OF STANCE PHASE ONLY
% This creates a table where the first column is the moment value and the
% second column is the number of the step. Only the values from HC to TO
% will be here ("ignoring" the values from TO back to HC where the foot is
% in the air)
k = 1;
if ipTOValues(1)<ipHCValues(1)
    check = 1
else
    check = 0
end
%momentStance = zeros(length(moment_all),2);
%forceStance = zeros(length(sagForce),2);
for j = 1:(length(ipHCValues)-1)
    for i = 1:length(moment_all)
        if i > ipHCValues(j,1) && i < ipTOValues(j+check,1)
            momentStance(k,1) = i;
            momentStance(k,2) = moment_all(i,1);
            forceStance(k,1) = i;
            forceStance(k,2) = sagForce(i,1);
            k = k + 1;
        end
    end
end

%% SECTION 9: FIND MEAN OF MOMENT AND SAG FORCE FOR EACH AMB TASK
%These next sections will be broken up into each amb task

%% SECTION 9A: UP RAMP ANALYSIS
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(momentStance)
    if momentStance(i,1) >= urStart1 && momentStance(i,1) < urEnd1 + 1 || ...
            momentStance(i,1) >= urStart2 && momentStance(i,1) < urEnd2 + 1
        count = count + 1;
        urForce(count,1) = forceStance(i,2);
        urMoment(count,1) = momentStance(i,2);
        sumForce = sumForce + forceStance(i,2);
        sumMoment = sumMoment + momentStance(i,2);
    end
end
%urStDev = std(urMomentArmPercentFoot)
urForceMean = sumForce/count;
urMomentMean = sumMoment/count;
urMomentArmMean = urMomentMean / urForceMean;
urMomentArmPercentFootMean = (urMomentArmMean / 0.24)*100

%%Checking some graphs
titleV=(['Moment with Amb Tasks and HC and TO: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
figure
plot(moment_all, 'k-')
plot(ipHCValues, moment_all(ipHCValues), 'ko', 'LineWidth',2)
plot(ipTOValues, moment_all(ipTOValues), 'ro', 'LineWidth',2)
xline(urStart1, ':b', 'UR1 Start','HandleVisibility','off');
xline(urEnd1, ':r', 'UR1 End','HandleVisibility','off');
xline(lgStart1, ':b', 'LG1 Start','HandleVisibility','off');
xline(lgEnd1, ':r', 'LG1 End','HandleVisibility','off');
xline(drStart1, ':b', 'DR1 Start','HandleVisibility','off');
xline(urStart2, ':b', 'UR2 Start','HandleVisibility','off');
xline(lgStart2, ':b', 'LG2 Start','HandleVisibility','off');
xline(usStart1, ':b', 'US1 Start','HandleVisibility','off');
xline(usStart2, ':b', 'US2 Start','HandleVisibility','off');
xline(dsStart1, ':b', 'DS1 Start','HandleVisibility','off');
xline(dsStart2, ':b', 'DS2 Start','HandleVisibility','off');
xline(lgStart3, ':b', 'LG3 Start','HandleVisibility','off');
xline(drStart2, ':b', 'DR2 Start','HandleVisibility','off');
xlim([0 length(moment_all)])
xlabel('iPecs Frame/Time')
ylabel('Moment (Nm)')
title(titleV)




