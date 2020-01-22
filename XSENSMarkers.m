% XSENS Marker Data

% From static trial
% "ANKLE" 
RHLA = [-.071931 -0.471 0.051968];
RHCE = [-.119259 -0.519362 0.059681];
RHME = [-0.168303 -0.472870 0.0486961];
a = [RHLA;RHCE;RHME];
a_normal = planeNormal(RHLA,RHCE,RHME);

RSK3 = [-0.102513 -0.400116 0.288753];
RSK4 = [-0.099548 -0.459318 0.295123];
RSK2 = [-0.184406 -0.434458 0.283818];
iP = [RSK3;RSK4;RSK2];
iP_normal = planeNormal(RSK3,RSK4,RSK2);

floor_normal = [0 0 -1];

s_ThetaInRad = atan2(norm(cross(a_normal,iP_normal)),dot(a_normal,iP_normal));
s_ThetaInDegrees = atan2d(norm(cross(a_normal,iP_normal)),dot(a_normal,iP_normal));

sF_ThetaInRad = atan2(norm(cross(floor_normal,iP_normal)),dot(floor_normal,iP_normal));
sF_ThetaInDegrees = atan2d(norm(cross(floor_normal,iP_normal)),dot(floor_normal,iP_normal));

figure
quiver3(0,0,0,iP_normal(1), iP_normal(2), iP_normal(3),'-r');
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal

ang = atan2(iP_normal(2), iP_normal(3))

figure
plot3(a(:,1),a(:,2),a(:,3),'o',iP(:,1),iP(:,2),iP(:,3),'o')
xlabel('X')
ylabel('Y')
zlabel('Z')
xlim([-.8 0]);
ylim([-.8 0]);
zlim([0 .8]);
grid on

% ------------------------------------------------------------------------

%Finding Angle offset based on what Jenny explained
%vsfxa-004_ConditionB_Setting3_Take2.c3d
%frame 276
RHLA = [-0.138339 0.197665 0.075602];
RHCE = [-0.184464 0.148693 0.064276];
RHME = [-0.233509 0.192931 0.074646];
a = [RHLA;RHCE;RHME];
a_normal=planeNormal(RHLA,RHCE,RHME)

RSK3 = [-0.155091 0.159179 0.321497];
RSK4 = [-0.153173 0.102854 0.300557];
RSK2 = [-0.237762 0.126015 0.305196];
iP = [RSK3;RSK4;RSK2];
iP_normal = planeNormal(RSK3,RSK4,RSK2)

floor_normal = [0 0 -1]

ThetaInRad_a2iP = atan2(norm(cross(a_normal,iP_normal)),dot(a_normal,iP_normal))
ThetaInDegrees_a2iP = atan2d(norm(cross(a_normal,iP_normal)),dot(a_normal,iP_normal))



ThetaInRad_f2a = atan2(norm(cross(floor_normal,a_normal)),dot(floor_normal,a_normal))
ThetaInDegrees_f2a = atan2d(norm(cross(floor_normal,a_normal)),dot(floor_normal,a_normal))

figure
plot3(a(:,1),a(:,2),a(:,3),'o',iP(:,1),iP(:,2),iP(:,3),'o')
xlabel('X')
ylabel('Y')
zlabel('Z')
axis equal
% xlim([-.8 0]);
% ylim([-.8 0]);
% zlim([0 .8]);
grid on
hold off

% 
% hold on
% q(1) = quiver3(0,0,0,floor(1), floor(2), floor(3),'-r');
% q(2) = quiver3(0,0,0,iP_normal(1), iP_normal(2), iP_normal(3), '-b');
% axis equal
% hold off
