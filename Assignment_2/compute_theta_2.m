function t2 = compute_theta_2(theta_1, theta_3, x, y)
    xx = x + 35*sin(theta_1) - 50*cos(theta_1);
    yy = y - 35*cos(theta_1) - 50*sin(theta_1);
    cp = 350*cos(theta_3) + 300;
    sp = 350*sin(theta_3);
    cos_phi = (cp / sqrt(cp^2 + sp^2));
    sin_phi = (sp / sqrt(cp^2 + sp^2));
    atan2_phi = atan2(sin_phi, cos_phi);
    t2 = acos(sqrt((xx^2 + yy^2) / (cp^2 + sp^2))) - atan2_phi;
end
