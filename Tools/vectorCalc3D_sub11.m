clear

load visual3d_11.mat
shankAngle_all = shkang{1,1}(:,1:3);
SK2_all = SK2{1,1}(:,1:3); %RS Marker 1; inline with #3, lateral marker
SK3_all = SK3{1,1}(:,1:3); %RS Marker 2; posterior side of ipecs
SK4_all = SK4{1,1}(:,1:3); %RS Marker 3; medial marker
Knee_all = KNEE{1,1}(:,1:3);
vsf_ankle_all = VSF_ANKLE{1,1}(:,1:3);

% shankAngle_all = shkang{1,1}(:,1:3);
% SK2_all = LSK2{1,1}(:,1:3);
% SK3_all = LSK3{1,1}(:,1:3);
% SK4_all = LSK4{1,1}(:,1:3);
% Knee_all = LKNEE{1,1}(:,1:3);
% vsf_ankle_all = VSF_LANKLE{1,1}(:,1:3);

% -- Change this frame section for each subject! --
frames = randi([3580 3781], 1, 200); %[3580 3781] for subject 11, [8532 9032] for subject 12
r = zeros(length(frames),3);
shankL = zeros(10,1);

for i= 1: length(frames)
    fN = frames(i); %frame number
    %fN = 2256;
    shankframe_origen = [vsf_ankle_all(fN,:)];
    knee = Knee_all(fN,:);

    LiPMark = SK4_all(fN,:) - [0,0,0.04872]; % the subraction lowers the markers since they were on a plate above the iPecs
    BiPMark = SK3_all(fN,:) - [0,0,0.04872];
    RiPMark = SK2_all(fN,:) - [0,0,0.04872];
    
    % Find iPecs center
    % Midpoint b/w RSK2 and RSK4
   
    iPecsMid = ((RiPMark + LiPMark)/2); % in LAB frame 
    
    
    v_iMid2iR = RiPMark - iPecsMid;
    v_iMid2iR = v_iMid2iR/norm(v_iMid2iR);
    v_iL2iB = BiPMark - LiPMark;
    v_iL2iB = v_iL2iB/norm(v_iL2iB);
    
    % Find iPecs frame
    u_iMid2iB = (BiPMark-iPecsMid);
    u_iMid2iB = u_iMid2iB / norm(u_iMid2iB);
    u_iUp = cross(v_iMid2iR, u_iMid2iB);
    u_iUp = u_iUp / norm(u_iUp);
    u_iMid2iF = -1*u_iMid2iB;
    v_iMid2iL = -1*v_iMid2iR;
    
    % These should all give ~0
    ucheck1 = dot(v_iL2iB, u_iUp);
    ucheck2 = dot(u_iUp, u_iMid2iF);
    ucheck3 = dot(v_iL2iB, u_iMid2iF);
    
    
    iX = v_iMid2iR; %X
    iY = u_iMid2iF; %Y
    iZ = u_iUp; %Z
    
    % Vector from shank frame origen to middle of iPecs in world frame
    r(i,:) = iPecsMid - shankframe_origen;
    ank2knee(i,:) = knee - shankframe_origen;

    % Puts X Y Z vectors in the correct matrix form, the following is the
    % rotation matrix that puts the iPecs into the worldframe
    wRip(i,:) = {[iX.',iY.',iZ.']};
    
    % If all is going smoothly the following calculation should result in
    % an identity matrix
    temp = [iX.',iY.',iZ.'];
    identity = temp.'*temp;
    
    % Finding the shank in the world frame
    % Converts shank angle to radians
    shankangle = deg2rad(shankAngle_all(fN,:));
    % Matlab function converts the set of XYZ angles into a rotation matrix
    euler2rotm = eul2rotm(shankangle, 'XYZ'); 
    % If all is going smoothly the following calculation should result in
    % an identity matrix
    identity = euler2rotm.'*euler2rotm;
    
    % This is the rotation matrix that puts the shank into the world frame
    wRs(i,:) = {euler2rotm};
    
    % Performing this multiplication creates the rotation matrix that sets
    %the iPecs frame into the shank frame
    Acurr = wRs{i,:}.' * wRip{i,:};
    A(i,:) = {Acurr};
    rodriguesV(i,:) = rotationMatrixToVector(Acurr);
    
    r(i,:) = transpose(wRs{i,:}.' * [r(i,1); r(i,2); r(i,3)]);
    identity = Acurr.'*Acurr;
    
    %find rknee in shank frame
    sknee(i,:) = transpose(wRs{i,:}.' * [ank2knee(i,1); ank2knee(i,2); ank2knee(i,3)]);
    r2knee(i,:) = r(i,:)-sknee(i,:);
    
    shankL(i) = norm(sknee(i,:));
end

% The following section finds the standard deviation of the A matrix and the
%r vector

%stdDev = cell(length(frames),1);
%rows are column wise
AinCols = zeros(length(frames), 9);
wRsinCols = zeros(length(frames), 9);
counter = 1;
for i= 1: length(frames)
    for j = 1:9
        curr = A{i}(j);
        AinCols(counter, j) = curr;
        curr = wRs{i}(j);
        wRsinCols(counter,j) = curr;
    end
    counter = counter + 1;
end

rAverage = mean(r);
rstdDev = std(r);
AAverage = mean(AinCols);
AAverage = reshape(AAverage, [3,3])
meanRodri = mean(rodriguesV);
rodR = rotationVectorToMatrix(meanRodri)
AstdDev = std(AinCols);
AstdDev = reshape(AstdDev, [3,3]);

wRsAverage = mean(wRsinCols);
wRsAverage = reshape(wRsAverage, [3,3]);
rkneeAverage = mean(r2knee);
shankLength = mean(shankL)
figure 
plot3(iPecsMid(1), iPecsMid(2), iPecsMid(3), '*')
hold on
plot3([iPecsMid(1) AAverage(1,1)], [iPecsMid(2) AAverage(2,1)], [iPecsMid(3) AAverage(3,1)])
plot3([iPecsMid(1) AAverage(1,2)], [iPecsMid(2) AAverage(2,2)], [iPecsMid(3) AAverage(3,2)])
plot3([iPecsMid(1) AAverage(1,3)], [iPecsMid(2) AAverage(2,3)], [iPecsMid(3) AAverage(3,3)])
plot3([0 wRsAverage(1,1)], [0 wRsAverage(2,1)], [0 wRsAverage(3,1)], '-r')
plot3([0 wRsAverage(1,2)], [0 wRsAverage(2,2)], [0 wRsAverage(3,2)], '-g')
plot3([0 wRsAverage(1,3)], [0 wRsAverage(2,3)], [0 wRsAverage(3,3)], '-b')
plot3([shankframe_origen(1) iPecsMid(1)], [shankframe_origen(2) iPecsMid(2)], [shankframe_origen(3) iPecsMid(3)],...
    'LineWidth',3)
plot3([shankframe_origen(1) knee(1)], [shankframe_origen(2) knee(2)], [shankframe_origen(3) knee(3)],...
    'LineWidth',3)
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
legend('iPecs Origen',"F, x' ", "G, y'", "H, z'",  'x shank','y shank','z shank','r', 'knee');
grid on
pbaspect([1 1 1])
xlabel('X')
ylabel('Y')
zlabel('Z')
set(gca,'FontSize',18)
hold off

% figure
% plot3(rsk2(1), rsk2(2), rsk2(3), 'o')
% hold on
% plot3(rsk3(1), rsk3(2), rsk3(3), 'o')
% plot3(rsk4(1), rsk4(2), rsk4(3), 'o')
% plot3(u_firstN(1), u_firstN(2), u_firstN(3), '*')
% plot3(u_secondN(1), u_secondN(2), u_secondN(3), '*')
% plot3([rsk4(1) u_firstN(1)], [rsk4(2) u_firstN(2)], [rsk4(3) u_firstN(3)])
% plot3([rsk4(1) u_secondN(1)], [rsk4(2) u_secondN(2)], [rsk4(3) u_secondN(3)])
% plot3([rsk4(1) rsk3(1)], [rsk4(2) rsk3(2)], [rsk4(3) rsk3(3)])
% legend('RSK2', 'RSK3', 'RSK4', '1', '2','firstN', 'secondN');
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

