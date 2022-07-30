function norm_vec = norm_f(trajectory)

    r = randi([1 7835],1,3);
    pt1 = trajectory(:, r(1));
    pt2 = trajectory(:, r(2));
    pt3 = trajectory(:, r(3));
    
    a = pt2(1:3) - pt1(1:3);
    b = pt3(1:3) - pt2(1:3);
    
    norm_vec = cross(a, b)/norm(cross(a,b));

end