function [pos_t5, neg_t5] = compute_theta_5(phi, psi, theta, x, y, d3)
    pos_t5 = atan2(-sin(phi)*cos(psi), -cos(phi)*cos(theta)) + atan2(y, x) - atan2(d3, sqrt(x^2+y^2-d3^2));
    neg_t5 = atan2(-sin(phi)*cos(psi), -cos(phi)*cos(theta)) + atan2(y, x) - atan2(d3, -sqrt(x^2+y^2-d3^2));
end
