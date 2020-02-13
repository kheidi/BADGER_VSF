
load visual3d_latest.mat
shankAngle_all = shkang{1,1}(:,1);
RSK2_all = RSK2{1,1}(:,1:3);
RSK3_all = RSK3{1,1}(:,1:3);
RSK4_all = RSK4{1,1}(:,1:3);
rKnee_all = RKNEE{1,1}(:,1:3);
vsf_ankle_all = VSF_RANKLE{1,1}(:,1:3);

fN = 640; %frame number
firstN = planeNormal(RSK4_all(fN,:), RSK3_all(fN,:), RSK2_all(fN,:))
secondN = planeNormal(RSK4_all(fN,:), RSK3_all(fN,:), firstN(1,:))

check1 = dot(RSK3_all(fN,:)-RSK4_all(fN,:), firstN)
check2 = dot(RSK3_all(fN,:)-RSK4_all(fN,:), secondN)
check3 = dot(firstN, secondN)

%Unit vectors
u_rsk4 = RSK4_all(fN,:) / norm(RSK4_all(fN,:))
u_rsk3 = RSK3_all(fN,:) / norm(RSK3_all(fN,:))
u_rsk2 = RSK2_all(fN,:) / norm(RSK2_all(fN,:))

v_4to2 = u_rsk2 - u_rsk4
v_4to3 = u_rsk3 - u_rsk4

u_firstN = cross(v_4to3, v_4to2)
u_firstN = u_firstN / norm(u_firstN)

u_secondN = cross(u_firstN, v_4to2)
u_secondN = u_secondN / norm(u_secondN)

ucheck1 = dot(v_4to3, u_firstN)
ucheck2 = dot(u_firstN, u_secondN)
ucheck3 = dot(v_4to3, u_secondN)

figure
plot3(u_rsk2(1), u_rsk2(2), u_rsk2(3), 'o')
hold on
plot3(u_rsk3(1), u_rsk3(2), u_rsk3(3), 'o')
plot3(u_rsk4(1), u_rsk4(2), u_rsk4(3), 'o')
plot3(u_firstN(1), u_firstN(2), u_firstN(3), '*')
plot3(u_secondN(1), u_secondN(2), u_secondN(3), '*')
plot3([u_rsk4(1) u_firstN(1)], [u_rsk4(2) u_firstN(2)], [u_rsk4(3) u_firstN(3)])
plot3([u_rsk4(1) u_secondN(1)], [u_rsk4(2) u_secondN(2)], [u_rsk4(3) u_secondN(3)])
plot3([u_rsk4(1) u_rsk3(1)], [u_rsk4(2) u_rsk3(2)], [u_rsk4(3) u_rsk3(3)])
legend('RSK2', 'RSK3', 'RSK4', '1', '2');
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')

% figure
% plot3(RSK2_all(fN,1), RSK2_all(fN,2), RSK2_all(fN,3), 'o')
% hold on
% plot3(RSK3_all(fN,1), RSK3_all(fN,2), RSK3_all(fN,3), 'o')
% plot3(RSK4_all(fN,1), RSK4_all(fN,2), RSK4_all(fN,3), 'o')
% plot3(firstN(1), firstN(2), firstN(3), '*')
% plot3(secondN(1), secondN(2), secondN(3), '*')
% plot3([RSK4_all(fN,1) firstN(1)], [RSK4_all(fN,2) firstN(2)], [RSK4_all(fN,3) firstN(3)])
% plot3([RSK4_all(fN,1) secondN(1)], [RSK4_all(fN,2) secondN(2)], [RSK4_all(fN,3) secondN(3)])
% plot3([RSK4_all(fN,1) RSK3_all(fN,1)], [RSK4_all(fN,2) RSK3_all(fN,2)], [RSK4_all(fN,3) RSK3_all(fN,3)])
% legend('RSK2', 'RSK3', 'RSK4', '1', '2');
% grid on
% xlabel('X')
% ylabel('Y')
% zlabel('Z')