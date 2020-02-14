
load visual3d_latest.mat
shankAngle_all = shkang{1,1}(:,1);
RSK2_all = RSK2{1,1}(:,1:3);
RSK3_all = RSK3{1,1}(:,1:3);
RSK4_all = RSK4{1,1}(:,1:3);
rKnee_all = RKNEE{1,1}(:,1:3);
vsf_ankle_all = VSF_RANKLE{1,1}(:,1:3);

frames = randi([410 870], 1, 300);
A = cell(length(frames),1);
r = zeros(length(frames),3);

for i= 1: length(frames)
    fN = frames(i); %frame number
    shankframe_origen = [vsf_ankle_all(fN,:)];
    knee = rKnee_all(fN,:);

    rsk4 = RSK4_all(fN,:)
    rsk3 = RSK3_all(fN,:)
    rsk2 = RSK2_all(fN,:)

    v_4to2 = rsk2 - rsk4;
    v_4to2 = v_4to2/norm(v_4to2);
    v_4to3 = rsk3 - rsk4;
    v_4to3 = v_4to3/norm(v_4to3);

    u_firstN = cross(v_4to3, v_4to2);
    u_firstN = u_firstN / norm(u_firstN);

    u_secondN = cross(u_firstN, v_4to3);
    u_secondN = u_secondN / norm(u_secondN);

    ucheck1 = dot(v_4to3, u_firstN);
    ucheck2 = dot(u_firstN, u_secondN);
    ucheck3 = dot(v_4to3, u_secondN);

    F = v_4to3;
    G = u_firstN;
    H = u_secondN;
    r(i,:) = rsk4 - shankframe_origen;

    A(i,:) = {[F.',G.',H.']};
    %identity = A.'*A;
end

%stdDev = cell(length(frames),1);
%rows are column wise
AinCols = zeros(length(frames), 9);
counter = 1;
for i= 1: length(frames)
    for j = 1:9
        curr = A{i}(j);
        AinCols(counter, j) = curr;
    end
    counter = counter + 1;
end

rAverage = mean(r)
rstdDev = std(r)
AAverage = mean(AinCols);
AAverage = reshape(AAverage, [3,3])
AstdDev = std(AinCols);
AstdDev = reshape(AstdDev, [3,3])

figure 
plot3(rsk4(1), rsk4(2), rsk4(3), '*')
hold on
plot3([rsk4(1) F(1)], [rsk4(2) F(2)], [rsk4(3) F(3)])
plot3([rsk4(1) G(1)], [rsk4(2) G(2)], [rsk4(3) G(3)])
plot3([rsk4(1) H(1)], [rsk4(2) H(2)], [rsk4(3) H(3)])
plot3([shankframe_origen(1) rsk4(1)], [shankframe_origen(2) rsk4(2)], [shankframe_origen(3) rsk4(3)],...
    'LineWidth',3)
plot3([shankframe_origen(1) knee(1)], [shankframe_origen(2) knee(2)], [shankframe_origen(3) knee(3)],...
    'LineWidth',3)
xlim([-1 1]);
ylim([-1 1]);
zlim([-1 1]);
legend('iPecs Origen','F', 'G', 'H', 'r', 'knee');
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
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

