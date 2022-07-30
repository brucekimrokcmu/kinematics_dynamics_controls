%main-assignment4
poses = importdata("poses.txt"); 
kp = 1; ko = 1;

A = zeros(3*60, 9);
B = zeros(3*60, 1);
omega_b = zeros(3*60, 6);

for i = 2:length(poses)-1
    xs = poses(i,2:end);
    xd = poses(i+1,2:end);
    xdm = poses(i-1, 2:end);
    
    vs_tmp = vs_f(xdm,xs,kp,ko);
    vs = vs_tmp(1:3);
    ws = vs_tmp(4:end);
    what_s = what_f(ws);
    quat = [xs(end), xs(4), xs(5), xs(6)];
    rotm = quat2rotm(quat);
%     qa = transpose(xs(1:3));
    A_tmp = [eye(3) i*eye(3) rotm];
    B_tmp = xs(1:3)';
%     A_tmp = [rotm', rotm'*what_s*rotm];
%     B_tmp = [rotm'*vs + rotm'*what_s*qa]; 
    A(3*(i-1)+1:3*(i-1)+3, :) = A_tmp;
    B(3*(i-1)+1:3*(i-1)+3, :) = B_tmp;

    vs_tmp2 = vs_f(xs,xd,kp,ko);
    vs2 = vs_tmp2(1:3);
    ws2 = vs_tmp2(4:end);
    what_s2 = what_f(ws2);
    quat = [xd(end), xd(4), xd(5), xd(6)];
    rotmm = quat2rotm(quat);
    wb = rotmm'*ws2;

    temp1 = [xdm(end), xdm(4:6)];
    rotm1 = quat2rotm(temp1);

    temp2 = [xs(end), xs(4:6)];
    rotm2 = quat2rotm(temp2);
    
    temp3 = [xd(end), xd(4:6)];
    rotm3 = quat2rotm(temp3);

    what_b1 = rotm1'*(rotm2 - rotm1);
    what_b2 = rotm2'*(rotm3 - rotm2);

    wb1 = hat2none_f(what_b1);
    wb2 = hat2none_f(what_b2);

    diff_wb_til = wtil_f(wb2-wb1);

    w_temp = what_f(wb2)*wtil_f(wb2) + diff_wb_til;
    omega_b(3*(i-1)+1:3*(i-1)+3, :) = w_temp;

end

sol = inv(A'*A)*A'*B;
p0 = sol(1:3);
com_vel = sol(4:6);
qb_final = sol(7:end);

[U, S, V] = svd(omega_b);
Ib_tilda = V(:, end);
Ib = [Ib_tilda(1,1), Ib_tilda(2,1), Ib_tilda(3,1);...
      Ib_tilda(2,1), Ib_tilda(4,1), Ib_tilda(5,1);...
      Ib_tilda(3,1), Ib_tilda(5,1), Ib_tilda(6,1)];

distanceFromCom = norm(qb_final);
distance = qb_final - p0 + com_vel*60

angMomentum = rotmm*Ib*wb;


p120 = p0 + com_vel*120;