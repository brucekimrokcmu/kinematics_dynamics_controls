%% Step 1: Define zero configuration
syms l0 l1 l2 real %link
syms r0 r1 r2 real % link to COM
syms theta1 theta2 theta3 real %joint angles
syms m1 m2 m3 real %link mass
syms Ix1 Iy1 Iz1 Ix2 Iy2 Iz2 Ix3 Iy3 Iz3 %link moment of inertia
syms gravity % gravity

% basic configuration of q, w, L, theta
q = [0, 0, 0;...
     0, 0, l1;...
     l0, l0, l0]; q = sym(q);
w = [0, -1, -1;...
     0, 0, 0;...   
     1, 0, 0]; w = sym(w);
L = [0, 0, 0;...
     0, r1, l1+r2;...
     r0, l0, l0]; L = sym(L);
theta_list = sym([theta1, theta2, theta3]);
% get w^, v, twist, twist^, g_sl0 for each link, and expm(twist^,theta) = et 
wh(:, :, 1) = sym(zeros(3));
v(:, 1) = sym(zeros(3,1));
twist(:, 1) = sym(zeros(6,1));
twist_hat(:,:,1) = sym(zeros(4));
g_sl0(:,:,1) = sym(zeros(4));

et(:,:,1) = sym(zeros(4)); ewh(:,:,1) = sym(zeros(3)); 
for i = 1:3
    wh(:, :, i) = sym(what_f(w(:,i)));
    v(:, i) = -cross(w(:,i), q(:,i));
    twist(:,i) = [v(:,i);w(:,i)];
    twist_hat(:,:,i) = [wh(:, :, i), v(:,i); 0, 0, 0, 0];
    g_sl0(:,:,i) = [eye(3), L(:, i);...
                    0, 0, 0, 1];
    et(:,:,i) = sym(et_f(wh(:,:,i), w(:,i), v(:,i), theta_list(i)));

end
% Then with the homogeneous transformation, get Adj
% twist' = Adj * twist 
et1 = expm(twist_hat(:,:,1)*theta1);
ad1 = [et1(1:3,1:3) , what_f(et1(1:3,4))*et1(1:3,1:3);
       zeros(3,3)   , et1(1:3,1:3)];

et2 = et(:,:,1)*expm(twist_hat(:,:,2)*theta2);
ad2 = [et2(1:3,1:3) , what_f(et2(1:3,4))*et2(1:3,1:3);
       zeros(3,3)   , et2(1:3,1:3)];

twist_prime_1 = twist(:,1);
twist_prime_2 = ad1 * twist(:,2);
twist_prime_3 = ad1 * ad2 * twist(:,3);

% From here, we calculate Adj_inv, which will be multiplied by twist' (or
% twist_prime). This outputs twist+ (or twist_cross)
Gst_theta_1 = et(:,:,1) * g_sl0(:,:,1);
Gst_theta_2 = et(:,:,1) * et(:,:,2) * g_sl0(:,:,2);
Gst_theta_3 = et(:,:,1) * et(:,:,2) * et(:,:,3)* g_sl0(:,:,3);

R1 = Gst_theta_1(1:3,1:3);
R2 = Gst_theta_2(1:3,1:3);
R3 = Gst_theta_3(1:3,1:3);

p_hat_1 = what_f(Gst_theta_1(1:3,4));
p_hat_2 = what_f(Gst_theta_2(1:3,4));
p_hat_3 = what_f(Gst_theta_3(1:3,4));

Adg_inv_1 = [transpose(R1), -transpose(R1)*p_hat_1; ...
           zeros(3), transpose(R1)];
twist_cross1 = Adg_inv_1*twist_prime_1; 

Adg_inv_2 = [transpose(R2), -transpose(R2)*p_hat_2; ...
           zeros(3), transpose(R2)];
twist_cross2 = Adg_inv_2*[twist_prime_1,twist_prime_2]; 

Adg_inv_3 = [transpose(R3), -transpose(R3)*p_hat_3; ...
           zeros(3), transpose(R3)];
twist_cross3 = Adg_inv_3*[twist_prime_1, twist_prime_2, twist_prime_3]; 
%Finally with twist_cross, we can calculate the Jacobian
J1 = simplify([twist_cross1,zeros(6,1),zeros(6,1)]);
J2 = simplify([twist_cross2, zeros(6,1)]);
J3 = simplify([twist_cross3]);

%% Step 2: Compute Manipulator mass matrix
%From the generalized inertia matrix, we can calculate the mass inertial by
% multiplying by Jacobian transpose and then Jacobian
Mu1 = [[m1 0 0; 0 m1 0; 0 0 m1], zeros(3); ...
        zeros(3), [Ix1 0 0; 0 Iy1 0; 0 0 Iz1]];
Mu2 = [[m2 0 0; 0 m2 0; 0 0 m2], zeros(3); ...
        zeros(3), [Ix2 0 0; 0 Iy2 0; 0 0 Iz2]];
Mu3 = [[m3 0 0; 0 m3 0; 0 0 m3], zeros(3); ...
        zeros(3), [Ix3 0 0; 0 Iy3 0; 0 0 Iz3]];
M = J1.'*Mu1*J1 + J2.'*Mu2*J2 + J3.'*Mu3*J3;
M11 = M(1,1); 
% The answer is:
% M(1,1) = Iz1 + m3*(r2*cos(theta2 + theta3) + l1*cos(theta2))^2 + Iz3*cos(theta2 + theta3)^2 + Iy3*sin(theta2 + theta3)^2 + Iz2*cos(theta2)^2 + Iy2*sin(theta2)^2 + m2*r1^2*cos(theta2)^2

%% Step 3: Compute Coriolis matrix
C = sym(zeros(6));
for i = 1:3
    for j = 1:3
        C(i,j) = simplify(0.5*((diff(M(i,j),theta_list(1)) + diff(M(i,1),theta_list(j))- diff(M(1,j),theta_list(i))) * diff(theta_list(1)) +...
            (diff(M(i,j),theta_list(2)) + diff(M(i,2),theta_list(j))- diff(M(2,j),theta_list(i))) * diff(theta_list(2)) +...
            (diff(M(i,j),theta_list(3)) + diff(M(i,3),theta_list(j))- diff(M(3,j),theta_list(i))) * diff(theta_list(3))));    
    end
end
% The answer is:
% C(2,1) = (Iz3*sin(2*theta2 + 2*theta3))/2 - (Iy3*sin(2*theta2 + 2*theta3))/2 - (Ix3*sin(theta1))/2 + m3*((sin(2*theta2)*l1^2)/2 + sin(2*theta2 + theta3)*l1*r2 + (sin(2*theta2 + 2*theta3)*r2^2)/2) - (Iy2*sin(2*theta2))/2 + (Iz2*sin(2*theta2))/2 - (Iy3*cos(theta2 + theta3)^2*sin(theta1))/2 + (Iz3*cos(theta2 + theta3)^2*sin(theta1))/2 + (Iy3*sin(theta2 + theta3)^2*sin(theta1))/2 - (Iz3*sin(theta2 + theta3)^2*sin(theta1))/2 + (m2*r1^2*sin(2*theta2))/2 - (m3*sin(theta1)*(r2 + l1*cos(theta3))*(2*r2 + l1*cos(theta3) + l1*cos(2*theta2 + theta3)))/4 + (m3*r2*cos(theta2 + theta3)*sin(theta1)*(r2*cos(theta2 + theta3) + l1*cos(theta2)))/2 - (m3*r2*sin(theta2 + theta3)*sin(theta1)*(r2*sin(theta2 + theta3) + l1*sin(theta2)))/2 - (l1^2*m3*sin(theta1)*sin(theta3)*(sin(2*theta2 + theta3) + sin(theta3)))/4
 

%% Step 4: Compute N = dv/dtheta

h1 = simplify(Gst_theta_1(3,4));
h2 = simplify(Gst_theta_2(3,4));
h3 = simplify(Gst_theta_3(3,4));
V = m1*gravity*h1 + m2*gravity*h2 + m3*gravity*h3;
N = simplify([diff(V, theta1); diff(V, theta2); diff(V, theta3)]);
% The answer is:
% N(3) = -gravity*m3*r2*cos(theta2 + theta3)