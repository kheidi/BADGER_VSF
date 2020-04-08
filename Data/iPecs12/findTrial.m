%% TO-DO
% Keep playing with ipecs start number, implement ending, see if there's
% anyway that ipcs and xsens have the same number of steps and it be
% logical. Get ipstart and end for dmama simple. also get zeroing and
% threshold number.


%Search for correct iPecs

%% Import XSENS
subject = 12;
setting = 3;

fileName=(['toe', num2str(subject), num2str(setting),'.txt']);
file_toeZPos = fileName;
tempFile = importdata(file_toeZPos);
toePos = tempFile;

i = toePos(:,1);
toePosZ = toePos(:,4);

frameRate = 60;
xsensduration = (length(i)/frameRate)/60


%% Import Plot iPecs
filename = 'iPecs_Data_002.txt';    
[ forces,moments,time ] = ipecsSD2grfs( filename );

% Plot XSENS and iPecs
figure
subplot(2,1,1)
plot(i/frameRate,toePosZ)
hold on
ntime = time(:,1);
ntime = ntime-ntime(1);
nforces = forces(:,:);
xlim([0 200])

subplot(2,1,2)
plot(ntime,nforces(:,3))
iPecsDuration = ntime(end)/60
xlim([0 200])

hold off


%% Count steps, find TO
cut = 656;
thresholdFz = 40;
ipFz = nforces(:,3)-cut;
ipTime = ntime;
ipStart = 1;
ipEnd = length(ipFz);
[ipHCValues, ipTOValues] = iPecsHCTO(ipFz(ipStart:ipEnd,1), thresholdFz, ipTime);
numOfTOsiP = length(ipHCValues)

checkRange = 20;
xsensStart = 1;
xsensEnd = 8820; %8820
xsensTOValues = xsensTO(toePosZ(xsensStart:xsensEnd,1), checkRange);
numTOsXSENS = length(xsensTOValues);
numTOsXSENS =numTOsXSENS - 21 + 12*2 + 13*2 + 5
hold on

%% Data Alignment

% ipLength = ipEnd - ipStart;
% xsensLength = xsensEnd - xsensStart;
% 
% ipMultiplier = xsensLength/ipLength;
% newTime = (xsensStart:ipMultiplier:xsensEnd);
% 
% figure
% hold on
% plot(newTime, ipFz(ipStart:ipEnd), 'k-') % plots a continuous line of the adjusted iPecs readings
% plot(toePos(xsensStart:xsensEnd,1), toePos(xsensStart:xsensEnd,4)*430, 'r-') % matching xsens data lines
% plot(xsensTOValues, 0, 'ro', 'LineWidth', 2) % plots xsens toe-off data
% plot(ipTOValues*ipMultiplier, 0, 'bo', 'LineWidth',2)
% xlabel('Windows')


%% Graph Amb Modes
% file_ambModeTiming = 'timeTracking';
% cd ..
% timeTrack = xlsread(file_ambModeTiming);
% cd iPecs12
% 
% if subject == 1
%     if setting == 1
%         tTY = 2;
%     elseif setting == 2
%         tTY = 1;
%     elseif setting == 3
%         tTY = 3;
%     end
% end
% 
% if subject == 2
%     if setting == 1
%         tTY = 6;
%     elseif setting == 2
%         tTY = 5;
%     elseif setting == 3
%         tTY = 4;
%     end
% end
% 
% if subject == 3
%     if setting == 1
%         tTY = 7;
%     elseif setting == 2
%         tTY = 9;
%     elseif setting == 3
%         tTY = 8;
%     end
% end
% 
% if subject == 4
%     if setting == 1
%         tTY = 10;
%     elseif setting == 2
%         tTY = 12;
%     elseif setting == 3
%         tTY = 11;
%     end
% end
% 
% if subject == 12
%     if setting == 1
%         tTY = 28;
%     elseif setting == 2
%         tTY = 29;
%     elseif setting == 3
%         tTY = 30;
%     end
% end
% 
% urStart1X = timeTrack(tTY,7); % ur: up ramp
% urEnd1X = timeTrack(tTY,8);
% lgStart1X = timeTrack(tTY,9); % lg: level ground
% lgEnd1X = timeTrack(tTY,10);
% drStart1X = timeTrack(tTY,11); % dr: down ramp
% drEnd1X = timeTrack(tTY,12);
% urStart2X = timeTrack(tTY,13);
% urEnd2X = timeTrack(tTY,14);
% lgStart2X = timeTrack(tTY,15);
% lgEnd2X = timeTrack(tTY,16);
% usStart1X = timeTrack(tTY,17); % us: up stairs
% usEnd1X = timeTrack(tTY,18);
% usStart2X = timeTrack(tTY,19); % ds: down stairs
% usEnd2X = timeTrack(tTY,20);
% dsStart1X = timeTrack(tTY,21);
% dsEnd1X = timeTrack(tTY,22);
% dsStart2X = timeTrack(tTY,23);
% dsEnd2X = timeTrack(tTY,24);
% lgStart3X = timeTrack(tTY,25);
% lgEnd3X = timeTrack(tTY,26);
% drStart2X = timeTrack(tTY,27);
% drEnd2X = timeTrack(tTY,28);
% 
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
% xline(drEnd2X, ':r', 'DR2 End','HandleVisibility','off');
% title('XSENS Toe and Heel Position with Ambulation Modes')

%% Important Numbers for DMAMA3DSimple

disp('Here are the numbers you need to use in the DMAMA simple code')
fprintf('Subject: %d, Setting: %d\n' , subject, setting)
fprintf('iPStart: %d \n', ipStart)
