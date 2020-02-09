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
clear
load visual3d_latest.mat
shankAngle_all = shkang{1,1}(:,1);
RSK2_all = RSK2{1,1}(:,1:3);
RSK3_all = RSK3{1,1}(:,1:3);
RSK4_all = RSK4{1,1}(:,1:3);

%% Loop

%Randomly generated frames from 410 - 870
frames = randi([410 870], 1, 300);
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
rKnee_all = RKNEE{1,1}(:,1:3);
vsf_ankle_all = VSF_RANKLE{1,1}(:,1:3);
iPMid = zeros(length(frames),3);
dist = zeros(length(frames),1);
allShankLength = zeros(length(frames),1);        

for i = 1: length(frames)
    frame_curr = frames(i);
    rKnee = rKnee_all(frame_curr,:);
    vsf_ankle = vsf_ankle_all(frame_curr,:);
    
    %FIND CENTER OF IPECS
    %Midpoint between RSK3 and RSK4
    dx = (RSK3_all(frame_curr, 1)+(RSK4_all(frame_curr, 1)))/2;
    dy = (RSK3_all(frame_curr, 2)+(RSK4_all(frame_curr, 2)))/2;
    dz = (RSK3_all(frame_curr, 3)+(RSK4_all(frame_curr, 3)))/2;
    mp1 = [dx, dy, dz];
    %Mid point between mp1 and rsk2
    dx = (mp1(1)+(RSK2_all(frame_curr, 1)))/2;
    dy = (mp1(2)+(RSK2_all(frame_curr, 2)))/2;
    dz = (mp1(3)+(RSK2_all(frame_curr, 3)))/2;
    mp2 = [dx, dy, dz];
    iPMid(i,:) = mp2;
   
    %FIND PERPENDICULAR DISTANCE OF IPECS CENTER FROM SHANK 
    %Finds the slope of segment iPecs to Knee and knee to ankle segment
    iP2K_m =(rKnee(2)-iPMid(i,2))/(rKnee(3)-iPMid(i,3));
    K2A_m = (rKnee(2)-vsf_ankle(2))/(rKnee(3)-vsf_ankle(3));
    %angle between shanklength and knee to point
    theta = atand(abs((K2A_m-iP2K_m)/(1+K2A_m*iP2K_m)));
    %distance from iPecs center to knee and knee to ankle
    iP2K = sqrt((rKnee(2)-iPMid(i,2))^2+(rKnee(3)-iPMid(i,3))^2);
    K2A = sqrt((rKnee(2)-vsf_ankle(2))^2+(rKnee(3)-vsf_ankle(3))^2);
    %distance from iPecs midpoint to shank midline
    dist(i) = iP2K*sind(theta);
    
    %FIND LENGTH FROM KNEE TO ANKLE
    allShankLength(i) = sqrt((rKnee(1)-vsf_ankle(1))^2 + (rKnee(2)-vsf_ankle(2))^2 +(rKnee(3)-vsf_ankle(3))^2);
    alliP2a_Length(i) = sqrt((iPMid(i,1)-rKnee(1))^2 + (iPMid(i,2)-rKnee(2))^2 + (iPMid(i,3)-rKnee(3))^2);
    alliP2a_Length(i) =  allShankLength(i) - alliP2a_Length(i)*cosd(theta);
end

rInstance = frames(end);
iPMidInst = length(frames);
hold off
figure
plot3(RSK2_all(rInstance,1), RSK2_all(rInstance,2), RSK2_all(rInstance,3), 'o')
hold on
plot3(RSK3_all(rInstance,1), RSK3_all(rInstance,2), RSK3_all(rInstance,3), 'o')
plot3(RSK4_all(rInstance,1), RSK4_all(rInstance,2), RSK4_all(rInstance,3), 'o')
plot3([rKnee(1,1) vsf_ankle(1,1)],[rKnee(1,2) vsf_ankle(1,2)], [rKnee(1,3) vsf_ankle(1,3)])
plot3(iPMid(iPMidInst,1), iPMid(iPMidInst,2), iPMid(iPMidInst,3), '*')
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
hold off

dist_mm = dist.*1000;
distAvg = mean(dist)
distS = std(dist) 
shanklength = mean(allShankLength)
shanklenthS = std(allShankLength)
iP2a_Length = mean(alliP2a_Length)
iP2a_LengthS = std(alliP2a_Length) 

figure
plot(frames, dist, '*' )
lsline
figure
bar(abs(distAvg))
hold on
errorbar(abs(distAvg), abs(distS))


