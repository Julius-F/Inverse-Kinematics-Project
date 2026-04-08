% Inverse Kinematics
% Project 3
% Julius Faller
% EE 473

% Define needed arm values

l1 = 103.91;
l2 = 157.7;
l3 = 150;
l6 = 131;
alpha_1 = 90;
alpha_2 = 0;
alpha_3 = 0;

% Input Variables

ox = input('Input end effector x component: ');
oy = input('Input end effector y component: ');
oz = input('Input end effector z component: ');

axangx = input('Input axes angles x value: ');
axangy = input('Input axes angles y value: ');
axangz = input('Input axes angles z value: ');
axangtheta = input('Input axes angles angle value in radians: ');

axang = [axangx,axangy,axangz,axangtheta];
R = axang2rotm(axang);

x0 = ox - l6*R(1,3);
y0 = oy - l6*R(2,3);
z0 = oz - l6*R(3,3);

% Calculate thetas 1, 2, and 3

s = z0 - l1;
r = sqrt(y0^2 + x0^2);
a2 = l2;
a3 = l3;
D = (s^2 + r^2 - (a2^2 + a3^2))/(2 * a2 * a3);

theta1 = atan2(y0,x0);
theta3 = atan2(sqrt(1-(D^2)),D);
theta2 = atan2(s,r) - atan2(a3*sin(theta3),(a2 + a3*cos(theta3)));

% Find Rotation matrix R_3_5
R03 =...
[cos(theta1)*cos(theta2)*cos(theta3) - cos(theta1)*sin(theta2)*sin(theta3), - cos(theta1)*cos(theta2)*sin(theta3) - cos(theta1)*cos(theta3)*sin(theta2), (4026203041061939*sin(theta1))/4503599627370496;
cos(theta2)*cos(theta3)*sin(theta1) - sin(theta1)*sin(theta2)*sin(theta3), - cos(theta2)*sin(theta1)*sin(theta3) - cos(theta3)*sin(theta1)*sin(theta2),                                     -cos(theta1);
                        cos(theta2)*sin(theta3) + cos(theta3)*sin(theta2),                           cos(theta2)*cos(theta3) - sin(theta2)*sin(theta3),                                               0];

R35 = R03.'*R;

% Calculate theta 4 and 5

theta4 = atan2(R35(1,3),-R35(2,3));
theta5 = atan2(-R35(2,2),R35(2,1));


% Display end result

% Radians

thetas = [theta1,theta2,theta3,theta4,theta5];
disp('Thetas in radians: ')
disp(thetas);

% Degrees

for i = 1:5
    thetas(i) = rad2deg(thetas(i));
end

disp('Thetas in Degrees: ')
disp(thetas);
