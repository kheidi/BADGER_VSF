% Set subject and stiffness setting, this will find correct data in
%matrices of provided, known values
subject = 2;
setting = 3;
cd Data

%Using vectorCalc3D file find the average A matrix and r vector for each
%subject

if subject == 2
    A = [0.999421797371987,-0.0299351255331246,-0.00207784678786959;
        0.0296572581042341,0.996128500422157,-0.0810442961388628;
        0.00450334414414811,0.0808845308407729,0.996606759468490]
%     A = A + 2*[0.000339240369236025,0.0131392759222300,0.00915101541532063;
%           0.0127166208301486,0.00102537968856321,0.0108836250133737;
%           0.00976049816795206,0.0108105566519519,0.000976925671012597];
    
    r = [0.0430225062955562,-0.0128729273132021,0.209246247039647];
%     r = r + 2*[0.0111796678450822,0.0245802172837022,0.00949139156619026];
end

if subject == 4
    A = [0.943071963631529,-0.330951810333087,0.0324486338337264;
        0.331480753040735,0.943349641245414,-0.0123907118118581;
        -0.0265197758803160,0.0224446947434716,0.999378277248041];
%     A = A + 2*[0.00174727427656611,0.00499215883793746,0.00232237406212035;
%         0.00493591903103956,0.00171367888347061,0.00559735913011915;
%         0.00227500298354999,0.00556119437295477,0.000118308210783904];

    r = [-0.0167216308672882,0.0115755375014926,0.196382747541239];
%     r = r + 2*[0.0129213278965632,0.0481979683284418,0.00524388679407593];
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
        
iPecsCuts = [0,-70, 335, 0,0,0 0,0,0;
              7.5,-5,-25,7.5,-5,-25,7.5,-5,-25;
              0,0,0,0,0,0,0,0,0;
             -5, 0.3, 25,-5, 0.3, 25,-5, 0.3, 25];

cuts = [iPecsCuts(subject, setting*3-2);iPecsCuts(subject, setting*3-1);iPecsCuts(subject, setting*3)];
cuts = transpose(A*cuts);

%Similarily input iPecs Thresholds
iPecsThresholds = [0,10, 20,0,0,0,0,0,0;
              0,10,20,0,10,20,0,10,20;
              0,10,20,0,0,20,0,0,20;
              0,0,20,0,0,20,0,0,20];

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

%% Apply Cuts and Trim Length
% Apply cuts and thresholds
% This actually cuts the data off and creates a vector
ipFx = iPecsData(:,2) - cuts(1);
ipFy = iPecsData(:,3) - cuts(2);
ipFz = iPecsData(:,4) - cuts(3);
MxiPecs = iPecsData(:,5);
MyiPecs = iPecsData(:,6);
MziPecs = iPecsData(:,7);
ipTime = 1:length(ipFy);

%Trim length of iPecs Data so that it aligns nicely with XSENS
ipStart = ipStartValues(subject,setting);
ipEnd = ipEndValues(subject,setting);
xsensStart = xsensStartValues(subject,setting);
xsensEnd = xsensEndValues(subject,setting);

ipFx = ipFx(ipStart:ipEnd);
ipFy = ipFy(ipStart:ipEnd);
ipFz = ipFz(ipStart:ipEnd);

MxiPecs = MxiPecs(ipStart:ipEnd);
MyiPecs = MyiPecs(ipStart:ipEnd);
MziPecs = MziPecs(ipStart:ipEnd);


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
F_shankframe = zeros(length(ipFx),3);
for i = 1:length(ipFz)
    F_shankframe(i,:) = transpose(A * [ipFx(i); ipFy(i); ipFz(i)]);
end

iPecsMomentV = zeros(length(MxiPecs),3);
for i = 1:length(ipFz)
    iPecsMomentV(i,:) = transpose(A * [MxiPecs(i); MyiPecs(i); MziPecs(i)]);
end

MxiPecs = iPecsMomentV(:,1);
MyiPecs = iPecsMomentV(:,2);
MziPecs = iPecsMomentV(:,3);

% Compute Saggital Force
sagForce = (F_shankframe(:,2).^2 + F_shankframe(:,3).^2).^(1/2);

% Moment r X f
 clear moment_all
moment_computed = zeros(length(F_shankframe),3);
for i = 1:length(F_shankframe)
    moment_computed(i,:) = cross(r, F_shankframe(i,:));
end

moment_all(:,1) = moment_computed(:,1) + MxiPecs(:,1);
moment_all(:,2) = moment_computed(:,2) + MyiPecs(:,1);
moment_all(:,3) = moment_computed(:,3) + MziPecs(:,1);


clear moment_withXsensTime
moment_withXsensTime(:,1) = newTime;
moment_withXsensTime(:,2:4) = moment_all;

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

%% SECTION 9: FIND MEAN OF MOMENT AND SAG FORCE FOR EACH AMB TASK
%These next sections will be broken up into each amb task

% Here moment_all is set to moment arond the X since this is what we are
% using for DMAMA
moment_all = moment_all(:,1);

%% Wrong direction of moment?
moment_all = -1*(moment_all);
%% SECTION 9A: UP RAMP ANALYSIS
count = 0;
sumForce = 0;
sumMoment = 0;
for i = 1:length(moment_all)
    if i >= urStart1 && i < urEnd1 + 1 || ...
            i >= urStart2 && i < urEnd2 + 1
        count = count + 1;
        urForce(count,1) = sagForce(i,1);
        urMoment(count,1) = moment_all(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_all(i,1);
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
for i = 1:length(moment_all)
    if i >= lgStart1 && i < lgEnd1 + 1 || ...
            i >= lgStart2 && i < lgEnd2 + 1 || ...
            i >= lgStart3 && i < lgEnd3 + 1
        count = count + 1;
        lgForce(count,1) = sagForce(i,1);
        lgMoment(count,1) = moment_all(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_all(i,1);
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
for i = 1:length(moment_all)
    if i >= drStart1 && i < drEnd1 + 1 || ...
            i >= drStart2 && i < drEnd2 + 1
        count = count + 1;
        drForce(count,1) = sagForce(i,1);
        drMoment(count,1) = moment_all(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_all(i,1);
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
for i = 1:length(moment_all)
    if i >= usStart1 && i < usEnd1 + 1 || ...
            i >= usStart2 && i < usEnd2 + 1
        count = count + 1;
        usForce(count,1) = sagForce(i,1);
        usMoment(count,1) = moment_all(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_all(i,1);
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
for i = 1:length(moment_all)
    if i >= dsStart1 && i < dsEnd1 + 1 || ...
            i >= dsStart2 && i < dsEnd2 + 1
        count = count + 1;
        dsForce(count,1) = sagForce(i,1);
        dsMoment(count,1) = moment_all(i,1);
        sumForce = sumForce + sagForce(i,1);
        sumMoment = sumMoment + moment_all(i,1);
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


titleV=(['DMAMA: Subject ', num2str(subject), ' on Setting ', num2str(setting)]);

figure
bar(X,Y)
hold on
%errorbar(Y, error, '.')
ylim([0 50])
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
plot(lgForce)
xlim([0 length(lgForce)])
title('Level Ground')
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,2)
plot(urForce)
xlim([0 length(urForce)])
title('Up Ramp')
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,3)
plot(drForce)
title('Down Ramp')
xlim([0 length(drForce)])
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,4)
plot(usForce)
title('Up Stairs')
xlim([0 length(usForce)])
ylabel('Force (N)')
set(gca,'FontSize',15)

subplot(5,1,5)
plot(dsForce)
title('Down Stairs')
xlim([0 length(dsForce)])
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
plot(lgMoment, 'r')
xlim([0 length(lgForce)])
title('Level Ground')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,2)
plot(urMoment, 'r')
xlim([0 length(urForce)])
title('Up Ramp')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,3)
plot(drMoment, 'r')
xlim([0 length(drForce)])
title('Down Ramp')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,4)
plot(usMoment, 'r')
xlim([0 length(usForce)])
title('Up Stairs')
ylabel('Moment (Nm)')
set(gca,'FontSize',15)

subplot(5,1,5)
plot(dsMoment, 'r')
xlim([0 length(dsForce)])
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
xlim([0 length(lgForce)])
yyaxis left
plot(lgForce)
ylabel('Force (N)')
yyaxis right
plot(lgMoment)
ylabel('Moment (Nm)')
title('Level Ground')
set(gca,'FontSize',15)

subplot(5,1,2)
xlim([0 length(urForce)])
yyaxis left
plot(urForce)
ylabel('Force (N)')
yyaxis right
plot(urMoment)
ylabel('Moment (Nm)')
title('Up Ramp')
set(gca,'FontSize',15)

subplot(5,1,3)
xlim([0 length(drForce)])
yyaxis left
plot(drForce)
ylabel('Force (N)')
yyaxis right
plot(drMoment)
ylabel('Moment (Nm)')
title('Down Ramp')
set(gca,'FontSize',15)

subplot(5,1,4)
xlim([0 length(usForce)])
yyaxis left
plot(usForce)
ylabel('Force (N)')
yyaxis right
plot(usMoment)
ylabel('Moment (Nm)')
title('Up Stairs')
set(gca,'FontSize',15)

subplot(5,1,5)
xlim([0 length(dsForce)])
yyaxis left
plot(dsForce)
ylabel('Force (N)')
yyaxis right
plot(dsMoment)
ylabel('Moment (Nm)')
title('Down Stairs')

sgtitle(titleV)
set(gca,'FontSize',15)

shortTitle = ([pwd,'\Figures\allSub',num2str(subject), 'Set', num2str(setting),'.pdf']);
print(allFigure,shortTitle, '-dpdf','-fillpage')


