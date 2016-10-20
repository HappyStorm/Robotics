function t3 = compute_theta_3(theta_1, x, y, z)
    xx = x + 35*sin(theta_1) - 50*cos(theta_1);
    yy = y - 35*cos(theta_1) - 50*sin(theta_1);
    zz = z - 107.5;
    t3 = acos((xx^2 + yy^2 + zz^2 - 350^2 - 300^2) / (2*350*300));
end
