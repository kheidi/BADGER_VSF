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

%Using vectorCalc3D file find the average A matrix and r vector for each
%subject

if subject == 4
    A = [0.424535672333893, -0.0898907646877880, -0.889254114645269;
        0.869743333202949, 0.116116555072736, 0.431152383609996;
        -0.06663080740520790, 0.958881344179918, -0.139396886556128];
    r = [0.00444078664916257,-0.0275620380043983,0.203752917697032]
end

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
              -3,-50, 0, -15,-50,0, -15,-25,0];
               % 0,0,-15,-50,-15,-25];
%Similarily input iPecs Thresholds
iPecsThresholds = [10, 20,0,0,0,0;
              10,20,10,20,10,20;
              10,20,0,20,0,20;
              0,20,0,0,20,0,0,20,0];

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



