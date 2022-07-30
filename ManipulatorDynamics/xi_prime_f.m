function xi_prime = xi_prime_f(xih, theta_s)
    g(:,:,1) = eye(4);    
    g_inv(:,:,1) = eye(4);

    for i = 2:size(theta_s, 1)
        g(:,:,i) = g(:,:,i-1) * expm(xih(:,:,i-1) * theta_s(i-1));
    end

    for i = 2:size(theta_s, 1)
        g_inv(:,:,i) = expm(-xih(:,:,i-1) * theta_s(i-1)) * g_inv(:,:,i-1);

    end

    xih_prime(:,:,1) = zeros(4);
    xi_prime(:,1) = zeros(6,1);

    for i = 2:size(theta_s,1)       
        xih_prime(:,:,i) = g(:,:,i) * xih(:,:,i) * g_inv(:,:,i);
        xi_prime(:,i) = [xih_prime(1:3,4,i); xih_prime(3,2,i); ...
                        xih_prime(1,3,i); xih_prime(2,1,i)];
    end

    

end