function [pos_t1, neg_t1] = compute_theta_1(x, y, d3)
    pos_t1 = atan2(y, x) - atan2(d3, sqrt(x^2+y^2-d3^2));
    neg_t1 = atan2(y, x) - atan2(d3, -sqrt(x^2+y^2-d3^2));
end
