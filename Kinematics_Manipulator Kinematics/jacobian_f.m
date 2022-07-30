function jacobian = jacobian_f(theta_s, w, wh, v)

    % find xi, xi_hat
    xi(:, 1) = [v(:,1);
                w(:,1)];
    xih(:,:, 1) = [wh(:,:,1), v(:,1);
                         0, 0, 0, 0];  
    for i = 2:size(theta_s, 1)
        xi(:, i) = [v(:,i);
                    w(:,i)];
        xih(:,:, i) = [wh(:, :, i), v(:,i);
                               0, 0, 0, 0];
    end
    
    % Calculate xi_prime 
    xi_prime = xi_prime_f(xih, theta_s);
    % Calculate jacobian
    jacobian = [xi(:,1), xi_prime(:,2:end)]; 
 
end
