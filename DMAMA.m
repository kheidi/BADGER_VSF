clear
%% TO-DO
%-Issue with DS
%-Make presentation
%   -describing new method
%   -discussing big standard deviation
%   -Showing excel proof
%   -rounding error


%% OVERALL CODE DESCRIPTION

% This code is intended to calculate the EMAMA values for trials using the
% XSens and iPecs data collection systems.

% Author: Katherine Heidi Fehr
% Based on work by: Jennifer Leetsma

%% ** REPLACE THIS DATA WITH TRIAL INFO **

% Set subject and stiffness setting, this will find correct data in
%matrices of provided, known values
subject = 2;
setting = 2;
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

%% SECTION 3: FIND MOMENT VECTOR


fyComponent = ipFy * cos(innateOffset);
fzComponent = ipFz * sin(innateOffset)*-1;
fSum = fyComponent + fzComponent;
moment_h = fSum * shankLength ;


moment_all = fSum * shankLength - MxKnee;

figure
subplot(2,1,1)
hold on 
plot(1:ipLength, moment_h, 'b-')
xlabel('iPecs Time')
ylabel('Moment')
ylim([-60 120])
title('iPecs Only - Moment, Just Force')

subplot(2,1,2)
plot(1:ipLength, moment_all, 'b-')
ylim([-60 120])
xlabel('iPecs Time')
ylabel('Moment')
title('iPecs Only - Moment + Knee Moment')
hold off
%Need to add moment about x to moment_h, not sure what cuts to use if any.


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

urStart1 = timeTrack(tTY,7); % ur: up ramp
urEnd1 = timeTrack(tTY,8);
lgStart1 = timeTrack(tTY,9); % lg: level ground
lgEnd1 = timeTrack(tTY,10);
drStart1 = timeTrack(tTY,11); % dr: down ramp
drEnd1 = timeTrack(tTY,12);
urStart2 = timeTrack(tTY,13);
urEnd2 = timeTrack(tTY,14);
lgStart2 = timeTrack(tTY,15);
lgEnd2 = timeTrack(tTY,16);
usStart1 = timeTrack(tTY,17); % us: up stairs
usEnd1 = timeTrack(tTY,18);
usStart2 = timeTrack(tTY,19); % ds: down stairs
usEnd2 = timeTrack(tTY,20);
dsStart1 = timeTrack(tTY,21);
dsEnd1 = timeTrack(tTY,22);
dsStart2 = timeTrack(tTY,23);
dsEnd2 = timeTrack(tTY,24);
lgStart3 = timeTrack(tTY,25);
lgEnd3 = timeTrack(tTY,26);
drStart2 = timeTrack(tTY,27);
drEnd2 = timeTrack(tTY,28);

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
title('XSENS Toe and Heel Position with Ambulation Modes')

%% ALIGNMENT METHOD

%% SECTION 8: ATTEMPT AT DATA ALIGNMENT CODE

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
ipAdjustedWindows = (xsensStart:ipMultiplier:xsensEnd);
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
%Trying to multiply the amb mode?
urStart1
urStart1stretch = urStart1 * ipMultiplier
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
title('Shifted Data Windows')
%plot(ambTask, toeZPos(ambTask,4), 'bo', 'LineWidth', 2)



%% SECTION 7: COUNTS THE NUMBER OF STEPS (HC TO HC) IN EACH AMB MODE BASED ON XSENS

%To find the different ambulation mode sections in the iPecs data we will
%count the number of steps in each mode in the xsens data and then count
%the steps in the ipecs data. The steps will be counted based on heel
%contact.

%Sometimes there are gaps with steps that dont correspond to an ambulation
%mode, because of this there will be a counter that counts all steps and
%each Amb mode will get a varaible that commmunicates on which step they
%start.

allSteps = 1;
ur1Steps = 0;
lg1Steps = 0;
dr1Steps = 0;
ur2Steps = 0;
lg2Steps = 0;
us1Steps = 0;
us2Steps = 0;
ds1Steps = 0;
ds2Steps = 0;
lg3Steps = 0;
dr2Steps = 0;

% This counts HC's in each amb mode based only on XSENS data

for i = 1:length(xsensHCValues)
    
    if (xsensHCValues(i) > urStart1) && (xsensHCValues(i) < urEnd1)
        ur1LastStep = allSteps;
        ur1Steps = ur1Steps + 1;
    elseif (xsensHCValues(i) > lgStart1) && (xsensHCValues(i) < lgEnd1)
        lg1LastStep = allSteps;
        lg1Steps = lg1Steps + 1;
    elseif (xsensHCValues(i) > drStart1) && (xsensHCValues(i) < drEnd1)
        dr1LastStep = allSteps;
        dr1Steps = dr1Steps + 1;
    elseif (xsensHCValues(i) > urStart2) && (xsensHCValues(i) < urEnd2)
        ur2LastStep = allSteps;
        ur2Steps = ur2Steps + 1;
    elseif (xsensHCValues(i) > lgStart2) && (xsensHCValues(i) < lgEnd2)
        lg2LastStep = allSteps;
        lg2Steps = lg2Steps + 1;
    elseif (xsensHCValues(i) > usStart1) && (xsensHCValues(i) < usEnd1)
        us1LastStep = allSteps;
        us1Steps = us1Steps + 1;
    elseif (xsensHCValues(i) > usStart2) && (xsensHCValues(i) < usEnd2)
        us2LastStep = allSteps;
        us2Steps = us2Steps + 1;
    elseif (xsensHCValues(i) > dsStart1) && (xsensHCValues(i) < dsEnd1)
        ds1LastStep = allSteps;
        ds1Steps = ds1Steps + 1;
    elseif (xsensHCValues(i) > dsStart2) && (xsensHCValues(i) < dsEnd2)
        ds2LastStep = allSteps;
        ds2Steps = ds2Steps + 1;
    elseif (xsensHCValues(i) > lgStart3) && (xsensHCValues(i) < lgEnd3)
        lg3LastStep = allSteps;
        lg3Steps = lg3Steps + 1;
    elseif (xsensHCValues(i) > drStart2) && (xsensHCValues(i) < drEnd2)
        dr2LastStep = allSteps;
        dr2Steps = dr2Steps + 1;
    end
    
    allSteps = allSteps + 1;
end

%Find the first HC of each mode: (Like you're assigning a number to each
%HC)

ur1FirstStep = ur1LastStep - ur1Steps + 1;
lg1FirstStep = lg1LastStep - lg1Steps + 1;
dr1FirstStep = dr1LastStep - dr1Steps + 1;
ur2FirstStep = ur2LastStep - ur2Steps + 1;
lg2FirstStep = lg2LastStep - lg2Steps + 1;
us1FirstStep = us1LastStep - us1Steps + 1;
us2FirstStep = us2LastStep - us2Steps + 1;
ds1FirstStep = ds1LastStep - ds1Steps + 1;
ds2FirstStep = ds2LastStep - ds2Steps + 1;
lg3FirstStep = lg3LastStep - lg3Steps + 1;
dr2FirstStep = dr2LastStep - dr2Steps + 1;

%% SECTION 8: NUMBERS EACH IPECS STEP

%The following for loop looks at the iPecs heel contact values. With this
%information it writes the step number alongside the moment value in the
%moment_all matrix. For example "step 1" is from the first HC value to the
%second HC value. On the moment array every data point between these two
%values will have a "1" in the second column

for i = 1:(length(ipHCValues)-1)
    for j = 1:length(moment_all)
        if ipHCValues(i) <= j && j < ipHCValues(i+1)
            moment_all(j,2) = i;
            sagForce(j,2) = i;
        elseif j >= ipHCValues(end)
            moment_all(j,2) = length(ipHCValues);
            sagForce(j,2) = length(ipHCValues);
        end
    end
end

%% SECTION NEW 5: FIND MOMENT & FORCE OF STANCE PHASE ONLY
% This creates a table where the first column is the moment value and the
% second column is the number of the step. Only the values from HC to TO
% will be here ("ignoring" the values from TO back to HC where the foot is
% in the air)
k = 1;
%momentStance = zeros(length(moment_all),2);
%forceStance = zeros(length(sagForce),2);
for j = 1:(length(ipTOValues)-1)
    for i = 1:length(moment_all)
        if i > ipHCValues(j,1) && i < ipTOValues(j+1,1)
            momentStance(k,1) = moment_all(i,1);
            momentStance(k,2) = moment_all(i,2);
            forceStance(k,1) = sagForce(i,1);
            forceStance(k,2) = sagForce(i,2);
            k = k + 1;
        end
    end
end

%momentStance(k:end, 1:2) = []
    

%% SECTION 9: FIND MEAN OF MOMENT AND SAG FORCE FOR EACH AMB TASK
%These next sections will be broken up into each amb task

%% SECTION 9A: UP RAMP ANALYSIS
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(momentStance)
    if momentStance(i,2) >= ur1FirstStep && momentStance(i,2) < ur1LastStep + 1 || ...
            momentStance(i,2) >= ur2FirstStep && momentStance(i,2) < ur2LastStep + 1
        count = count + 1;
        urForce(count,1) = forceStance(i,1);
        urMoment(count,1) = momentStance(i,1);
        sumForce = sumForce + forceStance(i,1);
        sumMoment = sumMoment + momentStance(i,1);
    end
end
%urStDev = std(urMomentArmPercentFoot)
urForceMean = sumForce/count;
urMomentMean = sumMoment/count;
urMomentArmMean = urMomentMean / urForceMean;
urMomentArmPercentFootMean = (urMomentArmMean / 0.24)*100


%% SECTION 9B: LEVEL GROUND ANALYSIS
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(momentStance)
    if momentStance(i,2) >= lg1FirstStep && momentStance(i,2) < lg1LastStep + 1 || ...
            momentStance(i,2) >= lg2FirstStep && momentStance(i,2) < lg2LastStep + 1 || ...
            momentStance(i,2) >= lg3FirstStep && momentStance(i,2) < lg3LastStep + 1
        count = count + 1;
        lgForce(count,1) = forceStance(i,1);
        lgMoment(count,1) = momentStance(i,1);
        sumForce = sumForce + forceStance(i,1);
        sumMoment = sumMoment + momentStance(i,1);
    end
end

%lgStDev = std(lgMomentArmPercentFoot)
lgForceMean = sumForce/count;
lgMomentMean = sumMoment/count;
lgMomentArmMean = lgMomentMean / lgForceMean;
lgMomentArmPercentFootMean = (lgMomentArmMean / 0.24)*100

%% SECTION 9C: DOWN RAMP ANALYSIS
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(momentStance)
    if momentStance(i,2) >= dr1FirstStep && momentStance(i,2) < dr1LastStep + 1 || ...
            momentStance(i,2) >= dr2FirstStep && momentStance(i,2) < dr2LastStep + 1
        count = count + 1;
        drForce(count,1) = forceStance(i,1);
        drMoment(count,1) = momentStance(i,1);
        sumForce = sumForce + forceStance(i,1);
        sumMoment = sumMoment + momentStance(i,1);
    end
end

%drStDev = std(drMomentArmPercentFoot)
drForceMean = sumForce/count;
drMomentMean = sumMoment/count;
drMomentArmMean = drMomentMean / drForceMean;
drMomentArmPercentFootMean = (drMomentArmMean / 0.24)*100

%% SECTION 9D: UP STAIRS ANALYSIS
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(momentStance)
    if momentStance(i,2) >= us1FirstStep && momentStance(i,2) < us1LastStep + 1 || ...
            momentStance(i,2) >= us2FirstStep && momentStance(i,2) < us2LastStep + 1
        count = count + 1;
        usForce(count,1) = forceStance(i,1);
        usMoment(count,1) = momentStance(i,1);

        sumForce = sumForce + forceStance(i,1);
        sumMoment = sumMoment + momentStance(i,1);
    end
end

%usStDev = std(usMomentArmPercentFoot)
usForceMean = sumForce/count;
usMomentMean = sumMoment/count;
usMomentArmMean = usMomentMean / usForceMean;
usMomentArmPercentFootMean = (usMomentArmMean / 0.24)*100


%% SECTION 9E: DOWN STAIRS ANALYSIS
%something is not right
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(momentStance)
    if momentStance(i,2) >= ds1FirstStep && momentStance(i,2) < ds1LastStep + 1 || ...
            moment_all(i,2) >= ds2FirstStep && momentStance(i,2) < ds2LastStep + 1
        count = count + 1;
        dsForce(count,1) = forceStance(i,1);
        dsMoment(count,1) = momentStance(i,1);
        sumForce = sumForce + forceStance(i,1);
        sumMoment = sumMoment + momentStance(i,1);
    end
end

%dsStDev = std(dsMomentArmPercentFoot)
dsForceMean = sumForce/count;
dsMomentMean = sumMoment/count;
dsMomentArmMean = dsMomentMean / dsForceMean;
dsMomentArmPercentFootMean = (dsMomentArmMean / 0.24)*100


%% SECTION 10: GRAPH DMAMA

X = categorical({'Level Ground','Up Ramp','Down Ramp','Up Stairs','Down Stairs'});
X = reordercats(X,{'Level Ground','Up Ramp','Down Ramp','Up Stairs','Down Stairs'});
Y = [lgMomentArmPercentFootMean urMomentArmPercentFootMean drMomentArmPercentFootMean ...
    usMomentArmPercentFootMean dsMomentArmPercentFootMean];
%error = [lgStDev urStDev drStDev usStDev dsStDev];


titleV=(['Division of Means: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);

figure
bar(X,Y)
hold on
%errorbar(Y, error, '.')
%ylim([0 50])
title(titleV)
ylabel('DMAMA (% of foot length)')
text(1:length(Y),Y,num2str(Y'),'vert','bottom','horiz','center');
hold off

%% SECTION 11: EXTRA INFO GRAPHS
%Show the moment and force graphs, bad graph because its just showing
%stance phase, but good for seeing outliers and trends.

%Force
titleV=(['Force Plot of Each Ambulation Mode for: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
figure
subplot(5,1,1)
plot(lgForce)
title('Level Ground')
ylabel('Force (N)')

subplot(5,1,2)
plot(urForce)
title('Up Ramp')
ylabel('Force (N)')

subplot(5,1,3)
plot(drForce)
title('Down Ramp')
ylabel('Force (N)')

subplot(5,1,4)
plot(usForce)
title('Up Stairs')
ylabel('Force (N)')

subplot(5,1,5)
plot(dsForce)
title('Down Stairs')
ylabel('Force (N)')

sgtitle(titleV)

%Moment
titleV=(['Moment Plot of Each Ambulation Mode for: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
figure
subplot(5,1,1)
plot(lgMoment)
title('Level Ground')
ylabel('Moment (Nm)')

subplot(5,1,2)
plot(urMoment)
title('Up Ramp')
ylabel('Moment (Nm)')

subplot(5,1,3)
plot(drMoment)
title('Down Ramp')
ylabel('Moment (Nm)')

subplot(5,1,4)
plot(usMoment)
title('Up Stairs')
ylabel('Moment (Nm)')

subplot(5,1,5)
plot(dsMoment)
title('Down Stairs')
ylabel('Moment (Nm)')

sgtitle(titleV)

%Both
titleV=(['Force Plot of Each Ambulation Mode for: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
figure
subplot(5,1,1)
yyaxis left
plot(lgForce)
ylabel('Force (N)')
yyaxis right
plot(lgMoment)
ylabel('Moment (Nm)')
title('Level Ground')

subplot(5,1,2)
yyaxis left
plot(urForce)
ylabel('Force (N)')
yyaxis right
plot(urMoment)
ylabel('Moment (Nm)')
title('Up Ramp')

subplot(5,1,3)
yyaxis left
plot(drForce)
ylabel('Force (N)')
yyaxis right
plot(drMoment)
ylabel('Moment (Nm)')
title('Down Ramp')

subplot(5,1,4)
yyaxis left
plot(usForce)
ylabel('Force (N)')
yyaxis right
plot(usMoment)
ylabel('Moment (Nm)')
title('Up Stairs')

subplot(5,1,5)
yyaxis left
plot(dsForce)
ylabel('Force (N)')
yyaxis right
plot(dsMoment)
ylabel('Moment (Nm)')
title('Down Stairs')

sgtitle(titleV)

%Print out step count
lgTotalSteps = lg1Steps++lg2Steps+lg3Steps
urTotalSteps = ur1Steps+ur2Steps
drTotalSteps = dr1Steps+dr2Steps
usTotalSteps = us1Steps+us2Steps
dsTotalSteps = ds1Steps+ds2Steps
 