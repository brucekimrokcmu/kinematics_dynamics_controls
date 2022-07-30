function [et,ewh] = et_f(wh, w, v, theta)
    
    ewh = eye(3) + wh*sin(theta) + (wh^2)*(1 - cos(theta));

    et = [ewh, (eye(3) - ewh)*(cross(w, v)) + w*transpose(w)*v*theta;...
          0, 0, 0, 1];
end