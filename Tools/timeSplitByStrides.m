clear
subject = 11;
setting =1;
save = 0;

m = 'wTime_';
if subject == 2
    mass = 83.91;
elseif subject ==4
    mass = 77.37;
elseif subject == 11
    mass = 74.52;
elseif subject == 12
    mass = 105.39;
end

disp('Level Ground:')
filename = ['KneeMo',num2str(subject),'-', num2str(setting),'_levelGround','.csv'];
filename = [m,num2str(subject),'-', num2str(setting),'_levelGround','.csv'];
M = csvread(filename,1,0);
SagForce = M(:,1);
AnkleMoment = M(:,2);
KneeMoment = M(:,3);
time= M(:,4);
lg = divideByStrides(AnkleMoment,KneeMoment,SagForce,time,filename,mass,save);
title('LG')

clear SagForce AnkleMoment KneeMoment time

disp('Down Ramp:')
filename = ['KneeMo',num2str(subject),'-', num2str(setting),'_downRamp','.csv'];
filename = [m,num2str(subject),'-', num2str(setting),'_downRamp','.csv'];
M = csvread(filename,1,0);
SagForce = M(:,1);
AnkleMoment = M(:,2);
KneeMoment = M(:,3);
time= M(:,4);
dr = divideByStrides(AnkleMoment,KneeMoment,SagForce,time,filename,mass,save);
title('DR')
clear SagForce AnkleMoment KneeMoment time

disp('Up Ramp:')
filename = ['KneeMo',num2str(subject),'-', num2str(setting),'_upRamp','.csv'];
filename = [m,num2str(subject),'-', num2str(setting),'_upRamp','.csv'];
M = csvread(filename,1,0);
SagForce = M(:,1);
AnkleMoment = M(:,2);
KneeMoment = M(:,3);
time= M(:,4);
ur = divideByStrides(AnkleMoment,KneeMoment,SagForce,time,filename,mass,save);
title('UR')
clear SagForce AnkleMoment KneeMoment time

disp('Down Stairs:')
filename = ['KneeMo',num2str(subject),'-', num2str(setting),'_downStairs','.csv'];
filename = [m,num2str(subject),'-', num2str(setting),'_downStairs','.csv'];
M = csvread(filename,1,0);
SagForce = M(:,1);
AnkleMoment = M(:,2);
KneeMoment = M(:,3);
time= M(:,4);
ds = divideByStrides(AnkleMoment,KneeMoment,SagForce,time,filename,mass,save);
title('DS')
clear SagForce AnkleMoment KneeMoment time

disp('Up Stairs:')
filename = ['KneeMo',num2str(subject),'-', num2str(setting),'_upStairs','.csv'];
filename = [m,num2str(subject),'-', num2str(setting),'_upStairs','.csv'];
M = csvread(filename,1,0);
SagForce = M(:,1);
AnkleMoment = M(:,2);
KneeMoment = M(:,3);
time= M(:,4);
us = divideByStrides(AnkleMoment,KneeMoment,SagForce,time,filename,mass,save);
title('US')
clear SagForce AnkleMoment KneeMoment time

all = [ur.x; lg.x; dr.x; us.x; ds.x];

filename = [num2str(subject),'-', num2str(setting),'.csv'];
T = table(all);
newName = ['Max',filename];
writetable(T,newName)
clear T



function [results] = divideByStrides(AnkleMoment,KneeMoment,SagForce,time,filename,mass,save)

collectionFrequency = 150;
% [transformedData,frequency,t] = dataProcessing.fastFourier(iPecsData(:,3), collectionFrequency);
cutoffFrequency = 20;
AnkleMoment = dataProcessing.apply4OButter(AnkleMoment, collectionFrequency, cutoffFrequency);
KneeMoment = dataProcessing.apply4OButter(KneeMoment, collectionFrequency, cutoffFrequency);
SagForce = dataProcessing.apply4OButter(SagForce, collectionFrequency, cutoffFrequency);

threshold = 100;
[HC] = iPecsHCTO(SagForce, threshold);
scount = 1;
% AnkleMoment = AnkleMoment/mass;
% KneeMoment = KneeMoment/mass;
% SagForce = SagForce/mass;


for i = 1:length(HC)
    if i == length(HC)
        AnkleMo_S{scount,1} = AnkleMoment(HC(i):length(AnkleMoment));
        KneeMo_S{scount,1} = KneeMoment(HC(i):length(AnkleMoment));
        Sag_S{scount,1} = SagForce(HC(i):length(AnkleMoment));
        Time_S{scount,1} = time(HC(i):length(AnkleMoment));
        [i_AnkleMo{scount,1}, tt] = dataProcessing.interpolateToLength(AnkleMo_S{scount,1}, 101);
        [i_KneeMo{scount,1}, tt] = dataProcessing.interpolateToLength(KneeMo_S{scount,1}, 101);
        [i_Sag{scount,1}, tt] = dataProcessing.interpolateToLength(Sag_S{scount,1}, 101);
    else
    AnkleMo_S{scount,1} = AnkleMoment(HC(i):HC(i+1));
    maxAnk(scount,1) = max(AnkleMoment(HC(i):HC(i+1)));
    KneeMo_S{scount,1} = KneeMoment(HC(i):HC(i+1));
    maxKnee(scount,1) = min(KneeMoment(HC(i):HC(i+1)));
    Sag_S{scount,1} = SagForce(HC(i):HC(i+1));
    Time_S{scount,1} = time(HC(i):HC(i+1));
    [i_AnkleMo{scount,1}, tt] = dataProcessing.interpolateToLength(AnkleMo_S{scount,1}, 101);
    [i_KneeMo{scount,1}, tt] = dataProcessing.interpolateToLength(KneeMo_S{scount,1}, 101);
    [i_Sag{scount,1}, tt] = dataProcessing.interpolateToLength(Sag_S{scount,1}, 101);
    DMAMA(scount,1) = trapz(Time_S{scount,1}(:,1),AnkleMo_S{scount,1}(:,1))/trapz(Time_S{scount,1}(:,1),Sag_S{scount,1}(:,1));
    DMAMA(scount,1) =(DMAMA(scount,1)/0.24)*100;
    oldDMAMA(scount,1) = mean(AnkleMo_S{scount,1}(:,1))/mean(Sag_S{scount,1}(:,1));
    oldDMAMA(scount,1) =(oldDMAMA(scount,1)/0.24)*100;
    meanSag(scount,1) = mean(Sag_S{scount,1});
    meanAnkMo(scount,1) = mean(AnkleMo_S{scount,1});
    scount = scount + 1;
    
    end
end
% Convert cell to a table and use first row as variable names
figure
plot(DMAMA, '*-' )
hold on
plot(oldDMAMA, '*-r')
legend('trapZ','Mean/Mean')


meanMaxAnk = mean(maxAnk);
meanMaxKnee = mean(maxKnee);
stdAnk = std(maxAnk);
stdKnee = std(maxKnee);
disp('DMAMA:')
mean(DMAMA)
x = [meanMaxAnk stdAnk meanMaxKnee stdKnee];
results.x = x;
results.oldDMAMA = oldDMAMA;
results.newDMAMA = DMAMA;
results.DMAMAdiff = [oldDMAMA-DMAMA];

% if save == 1
%     
%    
%     
%     T = cell2table(i_AnkleMo);
%     newName = ['Stride_Ankle_',filename];
%     writetable(T,newName)
%     clear T
% 
%     T = cell2table(i_KneeMo);
%     newName = ['Stride_Knee_',filename];
%     writetable(T,newName)
%     clear T
% 
%     T = cell2table(i_Sag);
%     newName = ['Stride_Sag_',filename];
%     writetable(T,newName)
%     clear T
% end


end