%% OVERALL CODE DESCRIPTION

% This code is intended to calculate the EMAMA values for trials using the
% XSens and iPecs data collection systems.

%% ** REPLACE THIS DATA WITH TRIAL INFO **

% iPecs Filename, forces & moments in X, Y, Z
file_iPecs = 'IP11';

% timeTracking, file that holds time stamps for the different ambulation
%modes (Up ramp, level ground). These time stamps correspond to the XSENS
%time. Follow "timetracking.xlsx" formatting for correct column usage.
file_ambModeTiming = 'timeTracking';

% XSENS foot position data, used to align heel contact (and toe off) point
%using position in the z-axis of the proximal foot (ankle). See
%'proxPosRFT.txt' for format.
file_heelZPos = 'proxPosRFT.txt';
file_toeZPos = 'distPosRFT.txt';

%% SECTION 1: IMPORT DATA

iPecsData = xlsread(file_iPecs);

timeTrack = xlsread(file_ambModeTiming);

tempFile = importdata(file_heelZPos);
heelZPos = tempFile.data;
tempFile = importdata(file_toeZPos);
toeZPos = tempFile.data;
