
load visual3d_latest.mat
shankAngle_all = shkang{1,1}(:,1);
RSK2_all = RSK2{1,1}(:,1:3);
RSK3_all = RSK3{1,1}(:,1:3);
RSK4_all = RSK4{1,1}(:,1:3);
rKnee_all = RKNEE{1,1}(:,1:3);
vsf_ankle_all = VSF_RANKLE{1,1}(:,1:3);

fN = 640; %frame number
shankframe_origen = [vsf_ankle_all(fN,:)]

%Unit vectors
u_rsk4 = RSK4_all(fN,:) / norm(RSK4_all(fN,:))
u_rsk3 = RSK3_all(fN,:) / norm(RSK3_all(fN,:))
u_rsk2 = RSK2_all(fN,:) / norm(RSK2_all(fN,:))

v_4to2 = u_rsk2 - u_rsk4
v_4to2 = v_4to2/norm(v_4to2)
v_4to3 = u_rsk3 - u_rsk4
v_4to3 = v_4to3/norm(v_4to3)

u_firstN = cross(v_4to3, v_4to2)
u_firstN = u_firstN / norm(u_firstN)

u_secondN = cross(u_firstN, v_4to3)
u_secondN = u_secondN / norm(u_secondN)

ucheck1 = dot(v_4to3, u_firstN)
ucheck2 = dot(u_firstN, u_secondN)
ucheck3 = dot(v_4to3, u_secondN)

F = v_4to3;
G = u_firstN;
H = u_secondN;
r = u_rsk4 - shankframe_origen;

A = [F.',G.',H.'];
identity = A.'*A

figure 
plot3(u_rsk4(1), u_rsk4(2), u_rsk4(3), '*')
hold on
plot3([u_rsk4(1) F(1)], [u_rsk4(2) F(2)], [u_rsk4(3) F(3)])
plot3([u_rsk4(1) G(1)], [u_rsk4(2) G(2)], [u_rsk4(3) G(3)])
plot3([u_rsk4(1) H(1)], [u_rsk4(2) H(2)], [u_rsk4(3) H(3)])
plot3([shankframe_origen(1) u_rsk4(1)], [shankframe_origen(2) u_rsk4(2)], [shankframe_origen(3) u_rsk4(3)])
legend('iPecs Origen','F', 'G', 'H', 'r');
grid on
hold off

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
legend('RSK2', 'RSK3', 'RSK4', '1', '2','firstN', 'secondN');
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
