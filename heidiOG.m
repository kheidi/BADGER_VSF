
timeTrack = xlsread("timeTracking.xlsx");
filename = 'IP11';
iPecsData = xlsread(filename);

%% Graph initial data and thresholds
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