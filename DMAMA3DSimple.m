% Set subject and stiffness setting, this will find correct data in
%matrices of provided, known values

clearvars -except collectF
subject = 11;
setting = 3;
cd Data
save = 0;
saveknee = 0;
saveJ = 0;
smooth = 0; 
%Using vectorCalc3D file find the average A matrix and r vector for each
%subject  
collectF = figure()

%%%Check

% New Rotation Matrices
%% Start
if subject == 2
   A = [0.993377890497891,0.114654960939642,0.00738962799396889;
       0.112619913798300,-0.984439829158985,0.134888760767871;
       0.0227403097169682,-0.133163293355780,-0.990833192629627];
    %     A = A + 2*[0.000339240369236025,0.0131392759222300,0.00915101541532063;
%           0.0127166208301486,0.00102537968856321,0.0108836250133737;
%           0.00976049816795206,0.0108105566519519,0.000976925671012597];


    r = [-0.00195556783467125,0.00335859363830944,0.201377591270110];
%     r = r + 2*[0.0111796678450822,0.0245802172837022,0.00949139156619026];
    rknee = [0.00535509835381191,0.0103195727097248,-0.232692698143401];
end

if subject == 4
    
    A = [0.945815065985516,0.319246409425497,-0.0592924196145092;
        0.322596917275835,-0.944650968801216,0.0597141198298429;
        -0.0369471232837331,-0.0756060659725957,-0.996453025922047];
   
% A = [1,0,0;0 1 0; 0 0 1]

    r = [0.00336289598446481,0.00272672758618676,0.201686384318682];
%     r = r + 2*[0.0129213278965632,0.0481979683284418,0.00524388679407593];
    rknee = [-0.0113741964148886,0.0190419906687991,-0.273751753374518];
end

if subject == 12 %Using subject 4 in the meantime! While waiting for Visual3D data to get prelim results
   A =  [0.784928369608924,0.616552889665954,-0.0612371523475584;
       0.618997314795591,-0.784651706984789,0.0341177813127678;
       -0.0270144194630212,-0.0646856473293718,-0.997539968207117];

    %Average-ish from previous trials
    r = [0.0361009496053564,0.0118603205904783,0.171858821308886];
%     r = r + 2*[0.0111796678450822,0.0245802172837022,0.00949139156619026];
    rknee = [0.0129992729454744,0.0104358210051803,-0.342698552065082];
    
end

if subject == 11 %Using subject 4 in the meantime! While waiting for Visual3D data to get prelim results

A = [0.999624911047254,-0.0246886788528556,-0.0118535374580907;
    0.0245994980693051,0.999668411130790,-0.00761133903994592;
    0.0120375208622063,0.00731689303894165,0.999900775661140]*[1,0,0;0 -1 0; 0 0 -1];
%          A = [
%         1,0,0;
%         0,-1,0;
%         0,0,-1
%         ];

    r = [0.00775120131537105,-0.00344310279541798,0.153703323156651];
    rknee = [0.00719117569484395,-0.00344310181265697,-0.274829208038666];
%     
end

% -----------------iPecs Data-----------------
% iPecs Filename, forces & moments in X, Y, Z
fileName=(['IP', num2str(subject), num2str(setting)]);
file_iPecs = fileName; 
% Add a row to this matrix for each subject. These are 'cuts' used to zero
%the iPecs data. If unsure of what cuts to use, run code until Figure 1 is
%generated.
%Row is subject, then y & z for each setting
% These were Jenny's cuts, I will attempt my own below
% iPecsCuts = [0,-70, 335, 0,0,0 0,0,0;
%               0,-5,-25,0,-15,-25,0,-15,-25;
%               0,0,0,0,0,0,0,0,0;
%               0,-3,-50, 0, -15,-50, 0, -15,-25];
% cuts = [iPecsCuts(subject, setting*3-2);iPecsCuts(subject, setting*3-1);iPecsCuts(subject, setting*3)];
% cuts = transpose(A*cuts);
        
% Current below
iPecsCuts = [0,-70, 335, 0,0,0 0,0,0;
              7.5,-5,-25,7.5,-5,-25,7.5,-5,-25;
              0,0,0,0,0,0,0,0,0;
              1.5,-3,-50,-5,-15,-50,10,-20,-40;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             -25,-5,-610,-25,-15,-610,-25,-15,-610;
             0,0,-660,0,0,-656,0,0,-656];
         
         iPecsCuts = [0,-70, 335, 0,0,0 0,0,0;
              7.5,-5,-25,7.5,-5,-25,7.5,-5,-25;
              0,0,0,0,0,0,0,0,0;
              1.5,-3,-50,-5,-15,-50,10,-20,-40;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             0,0,0,0,0,0,0,0,0;
             -30,0,610,-25,-15,610,-25,-15,610;
             0,0,660,0,0,660,0,0,660];


cuts = [iPecsCuts(subject, setting*3-2);iPecsCuts(subject, setting*3-1);iPecsCuts(subject, setting*3)];
% cuts = transpose(A*cuts);

%Similarily input iPecs Thresholds
iPecsThresholds = [0,10, 20,0,0,0,0,0,0;
              0,10,20,0,10,20,0,10,20;
              0,10,20,0,0,20,0,0,20;
              0,0,20,0,0,20,0,0,20;
              0,0,0,0,0,0,0,0,0;
              0,0,0,0,0,0,0,0,0;
              0,0,0,0,0,0,0,0,0;
              0,0,0,0,0,0,0,0,0;
              0,0,0,0,0,0,0,0,0;
              0,0,0,0,0,0,0,0,0;
              0,0,50,0,0,50,0,0,50;
              0,0,40,0,0,40,0,0,40];

% Add a row to the below matrices to manually trim the data to the correct
% stop and stopping point.
% iPecs starting and end values to subtract off data.
ipStartValues = [2140,  0,      0;
              1045,     1860,   3728;
              0,        0,      0;
              3280,     1488,   2332;
              0,0,0;
              0,0,0;
              0,0,0;
              0,0,0;
              0,0,0;
              0,0,0;
              6789,2695,2005;
              35730,6394,6420];
          
ipEndValues = [34185,   0,      0;
              25986,    27943,  33040;
              0,        0,      0;
              24003,    21688,  22758;
              0,0,0;
              0,0,0;
              0,0,0;
              0,0,0;
              0,0,0;
              0,0,0;
              25840,23105,22957;
              57240,27403,26390];

% -----------------timeTracking Data-----------------          
% timeTracking, file that holds time stamps for the different ambulation
%modes (Up ramp, level ground). These time stamps correspond to the XSENS
%time. Follow "timetracking.xlsx" formatting for correct column usage.
file_ambModeTiming = 'timeTracking';

% -----------------XSENS Data-----------------
% XSENS foot position data, used to align heel contact (and toe off) point
%using position in the z-axis of the proximal foot (ankle). REALLY ONLY
%USED FOR AMBULATION TASKS IDENTIFICATION. See 'proxPosRFT.txt' for format.
% fileName=(['heel', num2str(subject), num2str(setting),'.txt']);
% file_heelZPos = fileName;
fileName=(['toe', num2str(subject), num2str(setting),'.txt']);
file_toeZPos = fileName;

% Add a row (row per participant, column per setting) to the below matrices to manually trim the data to the correct
% stop and stopping point.
% XSENS starting and end values to subtract off data.
xsensStartValues = [593,    0,      0;
                    400,    489,    609;
                    0,      0,      0;
                    494,    448,    892;
                      0,0,0;
                      0,0,0;
                      0,0,0;
                      0,0,0;
                      0,0,0;
                      0,0,0;
                      226,1,1;
                      808,151,144];
                
xsensEndValues = [  13403,  0,      0;
                    10371,  10916,  12330;
                    0,      0,      0;
                    8779,   8528,   9060;
                    0,0,0;
                      0,0,0;
                      0,0,0;
                      0,0,0;
                      0,0,0;
                      0,0,0;
                      7849,8167,8376;
                      9412,8553,8791]; 
                
%% SECTION 1: IMPORT DATA

% Imports the iPecs data, timetracking file, heel and toe Z position
iPecsData = xlsread(file_iPecs);
timeTrack = xlsread(file_ambModeTiming);
% tempFile = importdata(file_heelZPos);
% heelZPos = tempFile.data;
tempFile = importdata(file_toeZPos);
if subject < 5
    toePos = tempFile.data;
end
% toePos = tempFile;
cd ..

% i = toePos(:,1);
% toePosZ = toePos(:,4);



%% Initial Graphs

% Figure 1: Raw iPecs Data
figure
subplot(2,1,1)
hold on
[a,b] = size(iPecsData);
ipLength = a;
plot(1:ipLength, iPecsData(:,2), 'b-')
plot(1:ipLength, iPecsData(:,3), 'r-')
plot(1:ipLength, iPecsData(:,4), 'k-')
legend('X iPecs Force','Y iPecs Force','Z iPecs Force')
xlabel('iPecs Time')
ylabel('Force (N)')
title('Raw iPecs Data (No Zeroing)')

subplot(2,1,2)
hold on
[a,b] = size(iPecsData);
ipLength = a;
plot(1:ipLength, iPecsData(:,5), 'b-')
plot(1:ipLength, iPecsData(:,6), 'r-')
plot(1:ipLength, iPecsData(:,7), 'k-')
% plot(i,toePosZ)
%% Smooth

if smooth ==1
    collectionFrequency = 150;
    [transformedData,frequency,t] = dataProcessing.fastFourier(iPecsData(:,3), collectionFrequency);
    cutoffFrequency = 10;
    iPecsData = dataProcessing.apply4OButter(iPecsData, collectionFrequency, cutoffFrequency);
end

%% Apply Cuts and Trim Length
% Apply cuts and thresholds
% This actually cuts the data off and creates a vector
ipFx = iPecsData(:,2) - cuts(1);
ipFy = iPecsData(:,3) - cuts(2);
ipFz = iPecsData(:,4) - cuts(3);
MxiPecs = iPecsData(:,5);
MyiPecs = iPecsData(:,6);
MziPecs = iPecsData(:,7);
ipFrameTime = 1:length(ipFy);
ipRealTime = iPecsData(:,1);

%Trim length of iPecs Data so that it aligns nicely with XSENS
ipStart = ipStartValues(subject,setting);
ipEnd = ipEndValues(subject,setting);
xsensStart = xsensStartValues(subject,setting);
xsensEnd = xsensEndValues(subject,setting);

ipRealTime = ipRealTime(ipStart:ipEnd);

ipFx = ipFx(ipStart:ipEnd);
ipFy = ipFy(ipStart:ipEnd);
ipFz = ipFz(ipStart:ipEnd);


MxiPecs = MxiPecs(ipStart:ipEnd);
MyiPecs = MyiPecs(ipStart:ipEnd);
MziPecs = MziPecs(ipStart:ipEnd);

figure
hold on
[a,b] = size(iPecsData);
ipLength = a;
plot(ipFx, 'b-')
plot(ipFy, 'r-')
plot(ipFz, 'k-')
yline(0)
legend('X iPecs Force','Y iPecs Force','Z iPecs Force')
xlabel('iPecs Time')
ylabel('Force (N)')
title('Raw iPecs Data Zeroed')



%% SECTION 8: DATA ALIGNMENT CODE

% Create iPecs multipler:
% Doing this because time of events is marker in xsens so want to leave
% that as is

% Create multiplier for stretching xsens data to iPecs data length
ipLength = ipEnd - ipStart;
xsensLength = xsensEnd - xsensStart;

ipMultiplier = xsensLength/ipLength;
newTime = (xsensStart:ipMultiplier:xsensEnd);

%% SECTION ?: APPLY CUTS & ROTATION MATRICES
figure
plot(ipFx)
hold on 
plot(ipFy)
plot(ipFz)
hold off
F_shankframe = zeros(length(ipFx),3);
for i = 1:length(ipFz)
    F_shankframe(i,:) = transpose(A * [ipFx(i); ipFy(i); ipFz(i)]);
end

iPecsMomentV = zeros(length(MxiPecs),3);
for i = 1:length(ipFz)
    iPecsMomentV(i,:) = transpose(A * [MxiPecs(i); MyiPecs(i); MziPecs(i)]);
end

figure
plot(ipFx)
hold on 
plot(ipFy)
plot(ipFz)
hold off

ogMxiP = MxiPecs;
ogMyiP = MyiPecs;
ogMziP = MziPecs;


MxiPecs = iPecsMomentV(:,1);
MyiPecs = iPecsMomentV(:,2);
MziPecs = iPecsMomentV(:,3);

figure
plot(MxiPecs)
hold on 
plot(MyiPecs)
plot(MziPecs)
hold off 

% Compute Saggital Force
sagForce = (F_shankframe(:,2).^2 + F_shankframe(:,3).^2).^(1/2);

% Moment r X f
clear moment_all
moment_ankle_computed = zeros(length(F_shankframe),3);
for i = 1:length(F_shankframe)
    moment_ankle_computed(i,:) = cross(r, F_shankframe(i,:));
    moment_knee_computed(i,:) = cross(rknee, F_shankframe(i,:));
end
figure
plot(moment_ankle_computed(:,1),'-r')
hold on
plot(MxiPecs(:,1),'-b')
moment_ankle(:,1) = moment_ankle_computed(:,1) - MxiPecs(:,1);
plot(moment_ankle(:,1),'-g')
hold off
moment_ankle(:,2) = moment_ankle_computed(:,2) - MyiPecs(:,1);
moment_ankle(:,3) = moment_ankle_computed(:,3) - MziPecs(:,1);

figure()
plot(moment_knee_computed(:,1),'-r')
hold on
plot(MxiPecs,'-b')


moment_knee(:,1) = -moment_knee_computed(:,1) - MxiPecs(:,1);
plot(moment_knee(:,1),'-g')
hold off
moment_knee(:,2) = -moment_knee_computed(:,2) - MyiPecs(:,1);
moment_knee(:,3) = -moment_knee_computed(:,3) - MziPecs(:,1);

clear moment_withXsensTime
moment_withXsensTime(:,1) = newTime;
moment_withXsensTime(:,2:4) = moment_ankle;

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
if subject == 11
    if setting == 1
        tTY = 25;
    elseif setting == 2
        tTY = 26;
    elseif setting == 3
        tTY = 27;
    end
end

if subject == 12
    if setting == 1
        tTY = 28;
    elseif setting == 2
        tTY = 29;
    elseif setting == 3
        tTY = 30;
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
if isempty(drEnd2) == 1
    drEnd2 = ipLength
end
allFinalTimings = [
    urStart1;urEnd1;lgStart1;lgEnd1;drStart1;drEnd1;urStart2;
    urEnd2;lgStart2;lgEnd2;usStart1;usEnd1;usStart2;usEnd2;
    dsStart1;dsEnd1;dsStart2;dsEnd2;lgStart3;lgEnd3;drStart2;
    drEnd2];

%% Create Moment Plots

%[kneeMo] = momentProcessing(moment_knee, ipFz, allFinalTimings,setting,collectF);
%[ankleMo] = momentProcessing(moment_ankle, ipFz, allFinalTimings,setting,collectF);


%% SECTION 9: FIND MEAN OF MOMENT AND SAG FORCE FOR EACH AMB TASK
%These next sections will be broken up into each amb task

% Here moment_all is set to moment arond the X since this is what we are
% using for DMAMA
moment_ankle = moment_ankle(:,1);

%-----------------------------------------------------------
% titleV=(['Toe Position in Z: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
% subplot(2,1,2)
% hold on
% plot(toeZPos(xsensStart:xsensEnd,1), toeZPos(xsensStart:xsensEnd,4), 'r-')
% xlim([xsensStart xsensEnd])
% xline(urStart1X, ':b', 'UR1 Start','HandleVisibility','off');
% xline(urEnd1X, ':r', 'UR1 End','HandleVisibility','off');
% xline(lgStart1X, ':b', 'LG1 Start','HandleVisibility','off');
% xline(lgEnd1X, ':r', 'LG1 End','HandleVisibility','off');
% xline(drStart1X, ':b', 'DR1 Start','HandleVisibility','off');
% xline(urStart2X, ':b', 'UR2 Start','HandleVisibility','off');
% xline(lgStart2X, ':b', 'LG2 Start','HandleVisibility','off');
% xline(usStart1X, ':b', 'US1 Start','HandleVisibility','off');
% xline(usStart2X, ':b', 'US2 Start','HandleVisibility','off');
% xline(dsStart1X, ':b', 'DS1 Start','HandleVisibility','off');
% xline(dsStart2X, ':b', 'DS2 Start','HandleVisibility','off');
% xline(lgStart3X, ':b', 'LG3 Start','HandleVisibility','off');
% xline(drStart2X, ':b', 'DR2 Start','HandleVisibility','off');
% xlabel('XSENS Frame/Time')
% ylabel('Height (m)')
% title(titleV)

titleV=(['Toe Position in Z: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
figure
%subplot(2,1,2)
yyaxis left
plot(moment_knee(:,1), ' -b')
ylabel('Knee Moment')
%plot((-1)*moment_all, '-b')
%ylabel('Moment')
%ylim([-40 900])
% yyaxis right
% plot(sagForce, '-r')
% ylabel('Force')
%ylim([-40 900])
xlim([-1000 length(moment_ankle)+1000])
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
%-----------------------------------------------------------

%% Wrong direction of moment?
moment_ankle = (moment_ankle);

ipMx = MxiPecs;
ipMy = MyiPecs;
ipMz = MziPecs;
%% SECTION 9A: UP RAMP ANALYSIS
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(moment_ankle)
    if i >= urStart1 && i < urEnd1 + 1 || ...
            i >= urStart2 && i < urEnd2 + 1
        count = count + 1;
        urSagForce(count,1) = sagForce(i,1);
        urAnkleMoment(count,1) = moment_ankle(i,1);
        urKneeMoment(count,1) = moment_knee(i,1);
        urFx(count,1) = ipFx(i,1);
        urFy(count,1) = ipFy(i,1);
        urFz(count,1) = ipFz(i,1);
        urMx(count,1) = ogMxiP(i,1);
        urMy(count,1) = ogMyiP(i,1);
        urMz(count,1) = ogMziP(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_ankle(i,1);
        urTime(count,1) = ipRealTime(i,1);
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
for i = 1:length(moment_ankle)
    if i >= lgStart1 && i < lgEnd1 + 1 || ...
            i >= lgStart2 && i < lgEnd2 + 1 || ...
            i >= lgStart3 && i < lgEnd3 + 1
        count = count + 1;
        lgSagForce(count,1) = sagForce(i,1);
        lgAnkleMoment(count,1) = moment_ankle(i,1);
        lgKneeMoment(count,1) = moment_knee(i,1);
        lgFx(count,1) = ipFx(i,1);
        lgFy(count,1) = ipFy(i,1);
        lgFz(count,1) = ipFz(i,1);
        lgMx(count,1) = ogMxiP(i,1);
        lgMy(count,1) = ogMyiP(i,1);
        lgMz(count,1) = ogMziP(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_ankle(i,1);
        lgTime(count,1) = ipRealTime(i,1);
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
for i = 1:length(moment_ankle)
    if i >= drStart1 && i < drEnd1 + 1 || ...
            i >= drStart2 && i < drEnd2 + 1
        count = count + 1;
        drSagForce(count,1) = sagForce(i,1);
        drAnkleMoment(count,1) = moment_ankle(i,1);
        drKneeMoment(count,1) = moment_knee(i,1);
        drFx(count,1) = ipFx(i,1);
        drFy(count,1) = ipFy(i,1);
        drFz(count,1) = ipFz(i,1);
        drMx(count,1) = ogMxiP(i,1);
        drMy(count,1) = ogMyiP(i,1);
        drMz(count,1) = ogMziP(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_ankle(i,1);
        drTime(count,1) = ipRealTime(i,1);
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
for i = 1:length(moment_ankle)
    if i >= usStart1 && i < usEnd1 + 1 || ...
            i >= usStart2 && i < usEnd2 + 1
        count = count + 1;
        usSagForce(count,1) = sagForce(i,1);
        usAnkleMoment(count,1) = moment_ankle(i,1);
        usKneeMoment(count,1) = moment_knee(i,1);
        usFx(count,1) = ipFx(i,1);
        usFy(count,1) = ipFy(i,1);
        usFz(count,1) = ipFz(i,1);
        usMx(count,1) = ogMxiP(i,1);
        usMy(count,1) = ogMyiP(i,1);
        usMz(count,1) = ogMziP(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_ankle(i,1);
        usTime(count,1) = ipRealTime(i,1);
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
for i = 1:length(moment_ankle)
    if i >= dsStart1 && i < dsEnd1 + 1 || ...
            i >= dsStart2 && i < dsEnd2 + 1
        count = count + 1;
        dsSagForce(count,1) = sagForce(i,1);
        dsAnkleMoment(count,1) = moment_ankle(i,1);
        dsKneeMoment(count,1) = moment_knee(i,1);
        dsFx(count,1) = ipFx(i,1);
        dsFy(count,1) = ipFy(i,1);
        dsFz(count,1) = ipFz(i,1);
        dsMx(count,1) = ogMxiP(i,1);
        dsMy(count,1) = ogMyiP(i,1);
        dsMz(count,1) = ogMziP(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_ankle(i,1);
        dsTime(count,1) = ipRealTime(i,1);
    end
end

%dsStDev = std(dsMomentArmPercentFoot)
dsForceMean = sumForce/count;
dsMomentMean = sumMoment/count;
dsMomentArmMean = dsMomentMean / dsForceMean;
dsMomentArmPercentFootMean = (dsMomentArmMean / 0.24)*100

%% Write everthing to .csv
if save == 1
    cd dataExport

    T = table(lgSagForce,lgAnkleMoment,lgKneeMoment,lgTime);
    writetable(T,['T_',num2str(subject),'-', num2str(setting),'_levelGround','.csv'])
    clear T

    T = table(urSagForce,urAnkleMoment,urKneeMoment,urTime);
    writetable(T,['T_',num2str(subject),'-', num2str(setting),'_upRamp','.csv'])
    clear T

    T = table(drSagForce,drAnkleMoment,drKneeMoment,drTime);
    writetable(T,['T_',num2str(subject),'-', num2str(setting),'_downRamp','.csv'])
    clear T

    T = table(usSagForce,usAnkleMoment,usKneeMoment,usTime);
    writetable(T,['T_',num2str(subject),'-', num2str(setting),'_upStairs','.csv'])
    clear T

    T = table(dsSagForce,dsAnkleMoment,dsKneeMoment,dsTime);
    writetable(T,['T_',num2str(subject),'-', num2str(setting),'_downStairs','.csv'])
    clear T

    cd ..
end
if saveknee == 1
    cd dataExport

    T = table(lgSagForce,lgAnkleMoment,lgKneeMoment);
    writetable(T,['KneeMo',num2str(subject),'-', num2str(setting),'_levelGround','.csv'])
    clear T

    T = table(urSagForce,urAnkleMoment,urKneeMoment);
    writetable(T,['KneeMo',num2str(subject),'-', num2str(setting),'_upRamp','.csv'])
    clear T

    T = table(drSagForce,drAnkleMoment,drKneeMoment);
    writetable(T,['KneeMo',num2str(subject),'-', num2str(setting),'_downRamp','.csv'])
    clear T

    T = table(usSagForce,usAnkleMoment,usKneeMoment);
    writetable(T,['KneeMo',num2str(subject),'-', num2str(setting),'_upStairs','.csv'])
    clear T

    T = table(dsSagForce,dsAnkleMoment,dsKneeMoment);
    writetable(T,['KneeMo',num2str(subject),'-', num2str(setting),'_downStairs','.csv'])
    clear T

    cd ..
end
    
if saveJ == 1
    cd dataExportJ

    T = table(lgTime,lgFx,lgFy,lgFz,lgMx,lgMy,lgMz,lgAnkleMoment,lgKneeMoment,lgSagForce);
    writetable(T,['CwTime_',num2str(subject),'-', num2str(setting),'_levelGround','.csv'])
    clear T

    T = table(urTime,urFx,urFy,urFz,urMx,urMy,urMz,urAnkleMoment,urKneeMoment,urSagForce);
    writetable(T,['CwTime_',num2str(subject),'-', num2str(setting),'_upRamp','.csv'])
    clear T

    T = table(drTime,drFx,drFy,drFz,drMx,drMy,drMz,drAnkleMoment,drKneeMoment,drSagForce);
    writetable(T,['CwTime_',num2str(subject),'-', num2str(setting),'_downRamp','.csv'])
    clear T

    T = table(usTime,usFx,usFy,usFz,usMx,usMy,usMz,usAnkleMoment,usKneeMoment,usSagForce);
    writetable(T,['CwTime_',num2str(subject),'-', num2str(setting),'_upStairs','.csv'])
    clear T

    T = table(dsTime,dsFx,dsFy,dsFz,dsMx,dsMy,dsMz,dsAnkleMoment,dsKneeMoment,dsSagForce);
    writetable(T,['CwTime_',num2str(subject),'-', num2str(setting),'_downStairs','.csv'])
    clear T

    cd ..
end
    

%% SECTION 10: GRAPH DMAMA

X = categorical({'Level Ground','Up Ramp','Down Ramp','Up Stairs','Down Stairs'});
X = reordercats(X,{'Level Ground','Up Ramp','Down Ramp','Up Stairs','Down Stairs'});
Y = [lgMomentArmPercentFootMean urMomentArmPercentFootMean drMomentArmPercentFootMean ...
    usMomentArmPercentFootMean dsMomentArmPercentFootMean];
%error = [lgStDev urStDev drStDev usStDev dsStDev];


titleV=(['DMAMA: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);

figure
bar(X,Y)
hold on
%errorbar(Y, error, '.')
ylim([0 77])
title(titleV)
ylabel('DMAMA (% of foot length)')
text(1:length(Y),Y,num2str(Y'),'vert','bottom','horiz','center');
set(gca,'FontSize',12)
hold off

%% SECTION 11: EXTRA INFO GRAPHS
%Show the moment and force graphs, bad graph because its just showing
%stance phase, but good for seeing outliers and trends.


%Force
titleV=(['Force Plot of Each Ambulation Mode for: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
forceFigure = figure;
subplot(5,1,1)
plot(lgSagForce)
xlim([0 length(lgSagForce)])
title('Level Ground')
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,2)
plot(urSagForce)
xlim([0 length(urSagForce)])
title('Up Ramp')
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,3)
plot(drSagForce)
title('Down Ramp')
xlim([0 length(drSagForce)])
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,4)
plot(usSagForce)
title('Up Stairs')
xlim([0 length(usSagForce)])
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,5)
plot(dsSagForce)
title('Down Stairs')
xlim([0 length(dsSagForce)])
ylabel('Force (N)')
set(gca,'FontSize',15)

sgtitle(titleV)
shortTitle = ([pwd,'\Figures\forceSub',num2str(subject), 'Set', num2str(setting),'.pdf']);
%set(gcf,'PaperPositionMode','auto')
print(forceFigure,shortTitle, '-dpdf','-fillpage')
%saveas(forceFigure, [pwd shortTitle]);

%Moment
titleV=(['Moment Plot of Each Ambulation Mode for: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
momentFigure = figure;
subplot(5,1,1)
plot(lgAnkleMoment, 'r')
xlim([0 length(lgSagForce)])
title('Level Ground')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,2)
plot(urAnkleMoment, 'r')
xlim([0 length(urSagForce)])
title('Up Ramp')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,3)
plot(drAnkleMoment, 'r')
xlim([0 length(drSagForce)])
title('Down Ramp')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,4)
plot(usAnkleMoment, 'r')
xlim([0 length(usSagForce)])
title('Up Stairs')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,5)
plot(dsAnkleMoment, 'r')
xlim([0 length(dsSagForce)])
title('Down Stairs')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

sgtitle(titleV)

shortTitle = ([pwd,'\Figures\momentSub',num2str(subject), 'Set', num2str(setting),'.pdf']);
print(momentFigure,shortTitle, '-dpdf','-fillpage')

%Both
titleV=(['Force Plot of Each Ambulation Mode for: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
allFigure = figure
subplot(5,1,1)
xlim([0 length(lgSagForce)])
yyaxis left
plot(lgSagForce)
ylabel('Force (N)')
yyaxis right
plot(lgAnkleMoment)
ylabel('Moment (Nm)')
title('Level Ground')
set(gca,'FontSize',15)

subplot(5,1,2)
xlim([0 length(urSagForce)])
yyaxis left
plot(urSagForce)
ylabel('Force (N)')
yyaxis right
plot(urAnkleMoment)
ylabel('Moment (Nm)')
title('Up Ramp')
set(gca,'FontSize',15)

subplot(5,1,3)
xlim([0 length(drSagForce)])
yyaxis left
plot(drSagForce)
ylabel('Force (N)')
yyaxis right
plot(drAnkleMoment)
ylabel('Moment (Nm)')
title('Down Ramp')
set(gca,'FontSize',15)

subplot(5,1,4)
xlim([0 length(usSagForce)])
yyaxis left
plot(usSagForce)
ylabel('Force (N)')
yyaxis right
plot(usAnkleMoment)
ylabel('Moment (Nm)')
title('Up Stairs')
set(gca,'FontSize',15)

subplot(5,1,5)
xlim([0 length(dsSagForce)])
yyaxis left
plot(dsSagForce)
ylabel('Force (N)')
yyaxis right
plot(dsAnkleMoment)
ylabel('Moment (Nm)')
title('Down Stairs')

sgtitle(titleV)
set(gca,'FontSize',15)

shortTitle = ([pwd,'\Figures\allSub',num2str(subject), 'Set', num2str(setting),'.pdf']);
print(allFigure,shortTitle, '-dpdf','-fillpage')


%% Whole thing
titleV=(['Force Plot of Each Ambulation Mode for: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);
allFigure = figure
yyaxis left
plot(sagForce)
ylim([-350 1400])
ylabel('Force (N)')
yyaxis right
plot(moment_ankle)
ylabel('Moment (Nm)')
title('All')
set(gca,'FontSize',15)

