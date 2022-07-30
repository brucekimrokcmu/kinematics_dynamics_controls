%% KDC Assignment2
theta = importdata("JointData.txt"); 
% Homogeneous transformation from world to base
pt_world = [0; 0; 0];
Rwb = [cos(pi/2), -sin(pi/2), 0;...
       sin(pi/2), cos(pi/2), 0;...
       0, 0, 1];
Gwb = [Rwb, pt_world;...
    0, 0, 0, 1];
% Homogeneous transformation from base to spatial 
I = eye(3);
pt_base = [750; 500; 1000];
Gbs = [I, pt_base;...
    0, 0, 0, 1];
% define reference configuration
pt_end = [0; 0; 1030]; % 910+120
Gse_0 = [I, pt_end;...
    0, 0, 0, 1];
pt_end_bframe = [0; 0; 1030; 1];
% q, w, wh, v
q = [0, 0, 0, 45,   0,   0,   0; ...
     0, 0, 0,  0,   0,   0,   0; ...
     0, 0, 0, 550, 850, 850, 850]; 
w = [0, 0, 0, 0, 0, 0, 0; ...
     0, 1, 0, 1, 0, 1, 0; ...
     1, 0, 1, 0, 1, 0, 1];
wh(:, :, 1) = zeros(3);
v(:, 1) = zeros(3, 1);
for i = 1:7
    wh(:, :, i) = what_f(w(:,i));
    v(:, i) = cross(-w(:,i), q(:,i));
end

% forward kinematics
[trajectory,Gst_theta] = fk_f(theta, pt_end_bframe, Gwb, Gbs);
plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:));
writematrix(transpose(trajectory(1:3,:)), 'trajectory.txt');
%% KDC Assignment 3-1
theta_s = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7].';
jacobian = jacobian_f(theta_s, w, wh, v);

%% KDC Assignment 3-2
% xs: position plus a quaternion - x, y, z, qx, qy, qz, q0 
xs = [0.44543, 1.12320, 2.22653, -0.29883, 0.44566, 0.84122, 0.06664].';
gs = [-0.81252, -0.15424, -0.56216, 0.44543;...
      -0.37847, -0.59390,  0.70996, 1.12320;...
      -0.44337,  0.78962,  0.42418, 2.22653;...
       0.00000,  0.00000,  0.00000, 1.00000];
xd = [0.46320, 1.16402, 2.22058, -0.29301, 0.41901, 0.84979, 0.12817].';

kp = 1; ko = 1;
vs = vs_f(xs,xd,kp,ko); 
%% KDC Assignment 3-3 (1)
xs1 = [0.44543, 1.12320, 2.22653, -0.29883, 0.44566, 0.84122, 0.06664].';
xd1 = [0.46320, 1.16402, 2.22058, -0.29301, 0.41901, 0.84979, 0.12817].';
kp1 = 1; ko1 = 1;

theta_a= [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7].';
step = 0.01; n = 2000;
for i = 1:n
    vs1 = vs_f(xs1, xd1, kp1, ko1);
    jacobian = jacobian_f(theta_a, w, wh, v);
    j_pseudo = jacobian.'*inv(jacobian*jacobian.');
    %theta dot
    theta_dot = j_pseudo*vs1;
    %integral
    theta_a = theta_dot*step + theta_a;
    [trajectory, G] = fk_hw3(theta_a, pt_end_bframe, Gwb, Gbs);
    quat = tform2quat(G);
    xs1 = [G(1,4), G(2,4), G(3,4), ...
            quat(2), quat(3), quat(4), quat(1)].';

end

writematrix(transpose(xs1(1:3,:)), 'trajectory_3-3(1).txt');

%% KDC Assignment 3-3 (2)
step = 0.01; n = 5000;
xs2 = [0.44543, 1.12320, 2.22653, -0.29883, 0.44566, 0.84122, 0.06664].';
xd2 = [0.49796, 0.98500, 2.34041, -0.11698, 0.07755, 0.82524, 0.54706].';
kp1 = 1; ko1 = 1;

theta_s2 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7].';
for i = 1:n
    vs1 = vs_f(xs2, xd2, kp1, ko1);
    jacobian = jacobian_f(theta_s2, w, wh, v);
    j_pseudo = jacobian.'*inv(jacobian*jacobian.');
    theta_dot = j_pseudo*vs1;
    theta_s2 = theta_dot*step + theta_s2;
    [trajectory, G] = fk_hw3(theta_s2, pt_end_bframe, Gwb, Gbs);
    quat = tform2quat(G);
    xs2 = [G(1,4), G(2,4), G(3,4), ...
            quat(2), quat(3), quat(4), quat(1)].';

end

writematrix(transpose(xs2(1:3,:)), 'trajectory_3-3(2).txt');


