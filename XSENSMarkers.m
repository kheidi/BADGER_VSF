% XSENS Marker Data

% Step to take:
% - Plot shankAngle to check if the ankle corresponds to what I thinkit
% does, maybe plot with toe position

% To find innateOffset:
% 1. Shank angle through visual3D (you can use the V3DPipeline_shankAngle)
% 2. Watch V3D trial and find a frame (note the frame number)
%    where the prosthetic leg is in swing phase
% 3. Click on the markers RSK2, RSK3, and RSK4 (if the iPecs is on the 
%    left leg write the value in the corresponding variable)and write the
%    coordinates below.
% 4. Go to the same frame in the shank angle file you created.
% 5. Find the difference between them.

%% From visual3D
load visual3d_latest.mat
shankAngle_all = shkang{1,1}(:,1);
RSK2_all = RSK2{1,1}(:,1:3);
RSK3_all = RSK3{1,1}(:,1:3);
RSK4_all = RSK4{1,1}(:,1:3);

%% Loop

frames = [458 570 580 560 627 640 750 760 740 1187 1243 822 1400];
diff = zeros(length(frames),1);
diffR = zeros(length(frames),1);
for i = 1: length(frames)
    frame_curr = frames(i);
    sAngle = shankAngle_all(frame_curr);
    iP2 = RSK2_all(frame_curr, :);
    iP3 = RSK3_all(frame_curr, :);
    iP4 = RSK4_all(frame_curr, :);
    iP = [iP2;iP3;iP4];
    iP_normal = planeNormal(iP3,iP4,iP2);
    iP_normaldiv = iP_normal/iP_normal(2);
    iP_angD = atand(iP_normal(2)/iP_normal(3));
    iP_angR = atan(iP_normal(2)/iP_normal(3));
    diff(i) = iP_angD + sAngle;
    diffR(i) = diff(i) * (pi/180);
end

diffR
diffAvg = mean(diffR)
diffS = std(diffR)
    

%% vsfxa-004_ConditionB_Setting3_Take2.c3d
%frame 276
% shankAngle = 17.0191;
% RSK3 = [-0.155091 0.159179 0.321497];
% RSK4 = [-0.153173 0.102854 0.300557];
% RSK2 = [-0.237762 0.126015 0.305196];
% iP = [RSK3;RSK4;RSK2];
% iP_normal = planeNormal(RSK3,RSK4,RSK2)
% iP_normaldiv = iP_normal/iP_normal(2)
% iP_angD = atand(iP_normal(2)/iP_normal(3))
% iP_angR = atan(iP_normal(2)/iP_normal(3))
% diff = iP_angD + shankAngle
% diffR = diff * (pi/180)

%% vsfxa-004_ConditionA_Setting3_Take2.c3d
%frame 171
% shankAngle = [-17.6959552764893];
% RSK3 = [-0.421580 0.219827 0.311092];
% RSK4 = [-0.510214 0.223965 0.321821];
% RSK2 = [-0.435778 0.271934 0.330057];
% iP = [RSK3;RSK4;RSK2];
% iP_normal = planeNormal(RSK3,RSK4,RSK2);
% iP_normaldiv = iP_normal/iP_normal(3);
% iP_angD = atand(iP_normal(2)/iP_normal(3));
% iP_angR = atan(iP_normal(2)/iP_normal(3));
% diff = iP_angD - shankAngle
% diffR = diff * (pi/180)

%% Graph math
% Z = iP_normaldiv(2)/tand(sAngle);
% figure
% plot(iP_normaldiv(2), iP_normaldiv(3), '*' );
% hold on
% plot(iP_normaldiv(2),Z, 'o');
% hold on
% maxVal = max((abs(iP_normaldiv(2))),abs(iP_normaldiv(3)));
% xlim([-(maxVal+10) maxVal+10]);
% ylim([-(maxVal+10) maxVal+10]);
% xline(0);
% yline(0);
