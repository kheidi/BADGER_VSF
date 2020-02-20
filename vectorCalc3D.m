
load visual3d_latest.mat
shankAngle_all = shkang{1,1}(:,1:3);
RSK2_all = RSK2{1,1}(:,1:3);
RSK3_all = RSK3{1,1}(:,1:3);
RSK4_all = RSK4{1,1}(:,1:3);
rKnee_all = RKNEE{1,1}(:,1:3);
vsf_ankle_all = VSF_RANKLE{1,1}(:,1:3);

frames = randi([410 870], 1, 3);
wRip = cell(length(frames),1);
r = zeros(length(frames),3);

for i= 1: length(frames)
    %fN = frames(i); %frame number
    fN = 2256;
    shankframe_origen = [vsf_ankle_all(fN,:)];
    knee = rKnee_all(fN,:);

    rsk4 = RSK4_all(fN,:);
    rsk3 = RSK3_all(fN,:);
    rsk2 = RSK2_all(fN,:);
    
    % Find iPecs center
    % Midpoint b/w RSK3 and RSK4
    mp1 = (rsk3 + rsk4)/2;
    iPecsMid = (mp1 + rsk2)/2;

    v_4to2 = rsk2 - rsk4;
    v_4to2 = v_4to2/norm(v_4to2);
    v_4to3 = rsk3 - rsk4;
    v_4to3 = v_4to3/norm(v_4to3);

    u_firstN = cross(v_4to3, v_4to2);
    u_firstN = u_firstN / norm(u_firstN);

    u_secondN = cross(v_4to3, u_firstN);
    u_secondN = u_secondN / norm(u_secondN);

    ucheck1 = dot(v_4to3, u_firstN);
    ucheck2 = dot(u_firstN, u_secondN);
    ucheck3 = dot(v_4to3, u_secondN);

    F = u_secondN;
    G = v_4to3;
    H = u_firstN;
    r(i,:) = iPecsMid - shankframe_origen;

    wRip(i,:) = {[F.',G.',H.']};
    temp = [F.',G.',H.'];
    identity = temp.'*temp;
    
    shankangle = deg2rad(shankAngle_all(fN,:))
    euler2rotm = eul2rotm(shankangle, 'XYZ')
    identity = euler2rotm.'*euler2rotm;
    wRs(i,:) = {euler2rotm};
    
    Acurr = wRs{i,:}.' * wRip{i,:};
    A(i,:) = {Acurr};
    identity = Acurr.'*Acurr;
    
end

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
AstdDev = std(AinCols);
AstdDev = reshape(AstdDev, [3,3])

wRsAverage = mean(wRsinCols);
wRsAverage = reshape(wRsAverage, [3,3])

figure 
plot3(rsk4(1), rsk4(2), rsk4(3), '*')
hold on
plot3([rsk4(1) AAverage(1,1)], [rsk4(2) AAverage(2,1)], [rsk4(3) AAverage(3,1)])
plot3([rsk4(1) AAverage(1,2)], [rsk4(2) AAverage(2,2)], [rsk4(3) AAverage(3,2)])
plot3([rsk4(1) AAverage(1,3)], [rsk4(2) AAverage(2,3)], [rsk4(3) AAverage(3,3)])
plot3([0 wRsAverage(1,1)], [0 wRsAverage(2,1)], [0 wRsAverage(3,1)], '-r')
plot3([0 wRsAverage(1,2)], [0 wRsAverage(2,2)], [0 wRsAverage(3,2)], '-g')
plot3([0 wRsAverage(1,3)], [0 wRsAverage(2,3)], [0 wRsAverage(3,3)], '-b')
plot3([shankframe_origen(1) rsk4(1)], [shankframe_origen(2) rsk4(2)], [shankframe_origen(3) rsk4(3)],...
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

