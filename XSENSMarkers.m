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

%Randomly generated frames from 410 - 870
frames = randi([410 870], 1, 200);
diff = zeros(length(frames),1);
diffR = zeros(length(frames),1);
iP_normaldiv = zeros(length(frames),3);
store= zeros(length(frames),3);

for i = 1: length(frames)
    frame_curr = frames(i);
    sAngle = shankAngle_all(frame_curr);
    iP2 = RSK2_all(frame_curr, :);
    iP3 = RSK3_all(frame_curr, :);
    iP4 = RSK4_all(frame_curr, :);
    iP = [iP2;iP3;iP4];
    iP_normal = planeNormal(iP3,iP4,iP2);
    store(i,:) = iP_normal;
    %Normalizes
    iP_normaldiv(i,:) = iP_normal/iP_normal(2);
    iP_angD = atand(iP_normal(2)/iP_normal(3));
    iP_angR = atan(iP_normal(2)/iP_normal(3));
    diff(i) = iP_angD + sAngle;
    diffR(i) = diff(i) * (pi/180);
end

diffR;
diffAvg = mean(diffR)
diffS = std(diffR)
figure
plot(frames, diffR, '*' )
lsline
figure
bar(abs(diffAvg))
hold on
errorbar(abs(diffAvg), abs(diffS))

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
% legend('iPecs Normal', 'Shank Angle')

%% Translation
rInstance = frames(1)
rKnee = RKNEE{1,1}(:,1:3);
vsf_ankle = VSF_RANKLE{1,1}(:,1:3);


%Midpoint between RSK3 and RSK4
dx = (RSK3_all(rInstance, 1)+(RSK4_all(rInstance, 1)))/2;
dy = (RSK3_all(rInstance, 2)+(RSK4_all(rInstance, 2)))/2;
dz = (RSK3_all(rInstance, 3)+(RSK4_all(rInstance, 3)))/2;
mp1 = [dx, dy, dz]
%Mid point between mp1 and rsk2
dx = (mp1(1)+(RSK2_all(rInstance, 1)))/2;
dy = (mp1(2)+(RSK2_all(rInstance, 2)))/2;
dz = (mp1(3)+(RSK2_all(rInstance, 3)))/2;
mp2 = [dx, dy, dz]
iPMid = mp2;

rKnee = rKnee(rInstance,:);
vsf_ankle = vsf_ankle(rInstance,:);

iP2K_m =(rKnee(2)-iPMid(2))/(rKnee(3)-iPMid(3))
K2A_m = (rKnee(2)-vsf_ankle(2))/(rKnee(3)-vsf_ankle(3))
%angle between shanklength and knee to point
theta = atand(abs((K2A_m-iP2K_m)/(1+K2A_m*iP2K_m)))


%Perpendicular distance from shank
%distance from iPecs center to knee
iP2K = sqrt((rKnee(2)-iPMid(2))^2+(rKnee(3)-iPMid(3))^2)
K2A = sqrt((rKnee(2)-vsf_ankle(2))^2+(rKnee(3)-vsf_ankle(3))^2)
%angle between shanklength and knee to point
%distance from midpoint to shank midline
dist = iP2K*sind(theta)

hold off
figure
plot3(RSK2_all(rInstance,1), RSK2_all(rInstance,2), RSK2_all(rInstance,3), 'o')
hold on
plot3(RSK3_all(rInstance,1), RSK3_all(rInstance,2), RSK3_all(rInstance,3), 'o')
plot3(RSK4_all(rInstance,1), RSK4_all(rInstance,2), RSK4_all(rInstance,3), 'o')
plot3([rKnee(1,1) vsf_ankle(1,1)],[rKnee(1,2) vsf_ankle(1,2)], [rKnee(1,3) vsf_ankle(1,3)])
plot3(iPMid(1,1), iPMid(1,2), iPMid(1,3), '*')
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off



