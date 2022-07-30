function vs = vs_f(xs, xd, kp, ko)

    ep = transpose(xd(1:3) - xs(1:3));
    xsq = [xs(7), xs(4), xs(5), xs(6)];
    xsq_inv = quatinv(xsq);
    xdq = [xd(7), xd(4), xd(5), xd(6)];
    eo = quatmultiply(xdq,xsq_inv);
    %eo = eo(2:4).';

    eo = sign(eo(1))*eo(2:4)';
    
    v = cross(eo,xs(1:3));
    ep = ep - v';
    vs = [kp*ep; ko*eo];

end

