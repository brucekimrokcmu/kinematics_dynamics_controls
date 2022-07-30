clear;
close all;

%% Question 1-b 

K = 1; C = 0.1; J1 = 10/9; J2 = 10;
w0 = sqrt(K*(J1+J2)/(J1*J2));

A = [0, 0, w0, 0;...
    0, 0, 0, w0;...
    -K/(J1*w0), K/(J1*w0), -C/J1, C/J1;...
    K/(J2*w0), -K/(J2*w0), C/J2, -C/J2];
eig(A);
%% Question 1-c

tau_d = (1-0)*rand(1,1) + 0;
B = [0,0,1/(J1*w0),0]';
poles = [-2,-1,-1+1i, -1-1i];
Kc = place(A, B, poles)
%xdot = (A-BKc)*X + disturbance

%% Question 1-d
psi1 = 0; psi2 = 0; psi1_dot = 0; psi2_dot = 0;
dt = 0.1;
times  = 1:dt:100;
psi2_ref = 0*ones(1,length(times));
psi2_ref(length(times)/2:end) = 1; 

% x_dot(:,1) = zeros(4,1);
x(:,1) = [psi1, psi2-psi2_ref(1), psi1_dot/w0, psi2_dot/w0];

temp_v = [];
temp_vv = [];
M = (A-B*Kc);
C = [1, 0, 0, 0; 0, 1, 0, 0];
K1 = -(1/(C*inv(M)*B));
K1 = [K1(2), K1(1)];
for i = 1:length(times)   
    x_dot(:,i) = M*[psi1, psi2, psi1_dot/w0, psi2_dot/w0]' + B*K1*[0,psi2_ref(i)]';
%     + [0;0;0;(tau_d/(J2*w0))];

    temp     = x_dot(:,i);
    psi1     = psi1 + temp(1)*dt + 0.5*w0*temp(3)*dt^2;
    psi2     = psi2 + temp(2)*dt + 0.5*w0*temp(4)*dt^2;
    temp_v   = [temp_v, psi2];
    temp_vv = [temp_vv, psi1];

    psi1_dot = psi1_dot + w0*temp(3)*dt;
    psi2_dot = psi2_dot + w0*temp(4)*dt;

end
figure;

plot(times,temp_v); 
hold on
plot(times,psi2_ref)
plot(times, temp_vv)
legend('ψ2','Step Response','ψ1')
title('ψ2 resposes to reference step change')

%% Question 1-d
psi1 = 0; psi2 = 0; psi1_dot = 0; psi2_dot = 0;
dt = 0.1;
times  = 1:dt:100;
taud_ref = 0*ones(1,length(times));
taud_ref(length(times)/2:end) = 1; 

% x_dot(:,1) = zeros(4,1);
x(:,1) = [psi1, psi2-psi2_ref(1), psi1_dot/w0, psi2_dot/w0];

temp_v = [];
temp_vv = [];

M = (A-B*Kc);
C = [1, 0, 0, 0; 0, 1, 0, 0];
K1 = -(1/(C*inv(M)*B));
K1 = [K1(2), K1(1)];
for i = 1:length(times)   
    x_dot(:,i) = M*[psi1, psi2, psi1_dot/w0, psi2_dot/w0]' + [0;0;0;(taud_ref(i)/(J2*w0))];

    temp     = x_dot(:,i);
    psi1     = psi1 + temp(1)*dt + 0.5*w0*temp(3)*dt^2;
    psi2     = psi2 + temp(2)*dt + 0.5*w0*temp(4)*dt^2;
    temp_v   = [temp_v, psi2];
    temp_vv = [temp_vv, psi1];
    

    psi1_dot = psi1_dot + w0*temp(3)*dt;
    psi2_dot = psi2_dot + w0*temp(4)*dt;

end
figure;    
plot(times,temp_v); 
hold on
plot(times,temp_vv);
plot(times, taud_ref)
legend('ψ2','ψ1','Step Response')
title('a step change in the distrubance torque 𝜏d')


%% Question 2-c
data = load('ydata.txt');
dt = 1; M = 0.01; numit = 20;
% define state space eqtn
A = [eye(3), eye(3)*dt;
    zeros(3), eye(3)];
B = [(dt^2/(2*M))*eye(3);
    (dt/M)*eye(3)];
u = [0.01,0.01,0.01]';
Rv = diag([10^-5,10^-5,10^-5]);
Rw = diag([50,50,50]);
% initialize state x, cov P, y, z, H, S, K
x = zeros(6,numit); x_hat = zeros(6,numit);
P = zeros(6,6*numit); P_hat = zeros(6,6*numit);
P(:,1:6) = diag([50,50,50,10,10,10]);
P_hat(:,1:6) = diag([50,50,50,10,10,10]);
y = zeros(3,numit);
z = data';
H = [eye(3), zeros(3)];
S = zeros(3,3*numit);
K = zeros(6,3*numit);

for i = 1:numit-1
    %prediction
    x(:,i+1) = A*x_hat(:,i) + B*u;
    P(:, 6*i+1:6*i+6 ) = A*P_hat(:, 6*(i-1)+1:6*(i-1)+6)*A' + B*Rv*B';
    
    %update
    y(:,i+1) = z(:,i+1) - H*x(:,i+1);
    S(:,3*i+1:3*i+3 ) = H*P(:, 6*i+1:6*i+6 )*H' + Rw;
    K(:,3*i+1:3*i+3 ) = P(:, 6*i+1:6*i+6 ) * H' *  pinv(S(:,3*i+1:3*i+3 ));
    x_hat(:,i+1) = x(:,i+1) + K(:,3*i+1:3*i+3 )*y(:,i+1);
    x_hat_one = x_hat(:,i+1);
    P_hat(:, 6*i+1:6*i+6 ) = (eye(6)- K(:,3*i+1:3*i+3 )*H)*P(:, 6*i+1:6*i+6 ) ;

end

figure;
hold on
title("KF");
xlabel("x(m)");
ylabel("y(m)");
zlabel("z(m)");
plot3(x_hat(1,:),x_hat(2,:),x_hat(3,:));
plot3(data(:,1),data(:,2),data(:,3));
legend('Prediction', 'Noise')
title('Kalman Filter to improve prediction model')


x_hat(:,20)
P(:, 115:120)
