clear
load visual3d_04.mat
shankAngle_all = shkang{1,1}(:,1:3);
SK2_all = SK2{1,1}(:,1:3);
SK3_all = SK3{1,1}(:,1:3);
SK4_all = SK4{1,1}(:,1:3);
Knee_all = KNEE{1,1}(:,1:3);
vsf_ankle_all = VSF_ANKLE{1,1}(:,1:3);

% -- Change this frame section for each subject! --
frames = [410:410+299]; %[410 870] for subject 4, [665:665+299] for sub 2
% frames = [665:665+299];
r = zeros(length(frames),3);

for i= 1: length(frames)
    fN = frames(i); %frame number
    %fN = 2256;
    shankframe_origen = vsf_ankle_all(fN,:);
    knee = Knee_all(fN,:);

    BackLeftiP = SK4_all(fN,:); %left as viewed when facing the person, regardless of lef it's on
    FrontLeftiP = SK3_all(fN,:);
    RightiP = SK2_all(fN,:);
    
    % Find iPecs center
    % Midpoint b/w RSK3 and RSK4
    mp1 = (FrontLeftiP + BackLeftiP)/2;
    iPecsMid = (mp1 + RightiP)/2;
    
    
    BackLeft2Right = RightiP - BackLeftiP;
    BackLeft2Right = BackLeft2Right/norm(BackLeft2Right);
    BackLeft2FrontLeft = FrontLeftiP - BackLeftiP;
    BackLeft2FrontLeft = BackLeft2FrontLeft/norm(BackLeft2FrontLeft);
    % Find iPecs frame
    u_iPUp = cross(BackLeft2FrontLeft, BackLeft2Right);
    u_iPUp = u_iPUp / norm(u_iPUp);
    u_Right2Left = cross(BackLeft2FrontLeft, u_iPUp);
    u_Right2Left = u_Right2Left / norm(u_Right2Left);
    u_Left2Right = -1*u_Right2Left;
    
    % These should all give ~0
    ucheck1 = dot(BackLeft2FrontLeft, u_iPUp);
    ucheck2 = dot(u_iPUp, u_Right2Left);
    ucheck3 = dot(BackLeft2FrontLeft, u_Right2Left);
    
    F = u_Right2Left; %X
    G = -BackLeft2FrontLeft; %Y
    H = -u_iPUp; %Z
    
%     F = -u_Right2Left; %X
%     G = -BackLeft2FrontLeft; %Y
%     H = u_iPUp; %Z
%     
    
    % Vector from world frame origen to middle of iPecs
    r(i,:) = iPecsMid - shankframe_origen;
    rknee(i,:) = knee - shankframe_origen;

    % Puts X Y Z vectors in the correct matrix form, the following is the
    % rotation matrix that puts the iPecs into the worldframe
    wRip(i,:) = {[F.',G.',H.']};
    
    % If all is going smoothly the following calculation should result in
    % an identity matrix
    temp = [F.',G.',H.'];
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
    sknee(i,:) = transpose(wRs{i,:}.' * [rknee(i,1); rknee(i,2); rknee(i,3)]);
    r2knee(i,:) = r(i,:)-sknee(i,:);
    
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

rAverage = mean(r)
rstdDev = std(r)
AAverage = mean(AinCols);
AAverage = reshape(AAverage, [3,3])
meanRodri = mean(rodriguesV);
rodR = rotationVectorToMatrix(meanRodri)
AstdDev = std(AinCols);
AstdDev = reshape(AstdDev, [3,3])

wRsAverage = mean(wRsinCols);
wRsAverage = reshape(wRsAverage, [3,3]);
rkneeAverage = mean(r2knee)
figure 
plot3(BackLeftiP(1), BackLeftiP(2), BackLeftiP(3), '*')
hold on
plot3([BackLeftiP(1) AAverage(1,1)], [BackLeftiP(2) AAverage(2,1)], [BackLeftiP(3) AAverage(3,1)])
plot3([BackLeftiP(1) AAverage(1,2)], [BackLeftiP(2) AAverage(2,2)], [BackLeftiP(3) AAverage(3,2)])
plot3([BackLeftiP(1) AAverage(1,3)], [BackLeftiP(2) AAverage(2,3)], [BackLeftiP(3) AAverage(3,3)])
plot3([0 wRsAverage(1,1)], [0 wRsAverage(2,1)], [0 wRsAverage(3,1)], '-r')
plot3([0 wRsAverage(1,2)], [0 wRsAverage(2,2)], [0 wRsAverage(3,2)], '-g')
plot3([0 wRsAverage(1,3)], [0 wRsAverage(2,3)], [0 wRsAverage(3,3)], '-b')
plot3([shankframe_origen(1) BackLeftiP(1)], [shankframe_origen(2) BackLeftiP(2)], [shankframe_origen(3) BackLeftiP(3)],...
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

