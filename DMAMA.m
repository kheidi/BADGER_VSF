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


% iPecs Filename, forces & moments in X, Y, Z
file_iPecs = 'IP11'; 
% Add a row to this matrix for each subject. These are 'cuts' used to zero
%the iPecs data. If unsure of what cuts to use, run code until Figure 1 is
%generated.
%Row is subject, then y & z for each setting
iPecsCuts = [-70, 335,0,0,0,0;
              -5,-25,-15,-25,-15,-25;
              0,0,0,0,0,0;
              -3,-50,-15,-50,-15,-25];
%Similarily input iPecs Thresholds
iPecsThresholds = [10, 20,0,0,0,0;
              10,20,10,20,10,20;
              10,20,0,20,0,20;
              0,20,0,20,0,20];   

          
% timeTracking, file that holds time stamps for the different ambulation
%modes (Up ramp, level ground). These time stamps correspond to the XSENS
%time. Follow "timetracking.xlsx" formatting for correct column usage.
file_ambModeTiming = 'timeTracking';


% XSENS foot position data, used to align heel contact (and toe off) point
%using position in the z-axis of the proximal foot (ankle). REALLY ONLY
%USED FOR AMBULATION TASKS IDENTIFICATION. See 'proxPosRFT.txt' for format.
file_heelZPos = 'proxPosRFT.txt';
file_toeZPos = 'distPosRFT.txt';

%% SECTION 1: IMPORT DATA

iPecsData = xlsread(file_iPecs);
timeTrack = xlsread(file_ambModeTiming);
tempFile = importdata(file_heelZPos);
heelZPos = tempFile.data;
tempFile = importdata(file_toeZPos);
toeZPos = tempFile.data;

%% SECTION 2: INITIAL GRAPHS

% Figure 1: Raw iPecs Data
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

% Apply cuts and thresholds
% This actually cuts the data off and creates a vector          
ipFy = iPecsData(:,3) - iPecsCuts(subject, setting*2-1);
ipFz = iPecsData(:,4) - iPecsCuts(subject, setting*2);
sagForce = (ipFy.^2 + ipFz.^2).^(1/2);
ipWindows = 1:length(ipFy);

% This creates vectors that can be graphed to show the thresholds         
thresholdFy(1:length(ipWindows),1) = iPecsThresholds(subject, setting*2-1);
thresholdFz(1:length(ipWindows),1) = iPecsThresholds(subject, setting*2);

% Figure 2: Zeroed iPecs Data
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









