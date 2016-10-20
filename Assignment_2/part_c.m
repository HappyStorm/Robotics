%% Date 2016/10/21
% syms cos(1) sin(1) cos(2) sin(2) cos(3) sin(3) cos(4) sin(4) cos(5) sin(5)
% syms c1 s1 c2 s2 c3 s3 c4 s4 c5 s5 c23 s23

T01 = [c1, -s1,  0,     0;
       s1,  c1,  0,     0;
        0,   0,  1, 358.5;
        0,   0,  0,     1];

T12 = [c2, -s2,  0, 50;
        0,   0,  1,  0;
      -s2, -c2,  0,  0;
        0,   0,  0,  1];

T23 = [c3, -s3,  0,   300;
       s3,  c3,  0,     0;
        0,   0,  1,     0;
        0,   0,  0,     1];

T34 = [c4, -s4,  0,   350;
       s4,  c4,  0,     0;
        0,   0,  1,    35;
        0,   0,  0,     1];

T45 = [c5, -s5,  0,    0;
        0,   0, -1, -251;
       s5,  c5,  0,    0;
        0,   0,  0,    1];

T14 = [-c5, s5, 0,   350*c23+300*c2+50;
        s5, c5, 0,                  35;
        0,  0, -1, -251-350*s23-300*s2;
        0,  0,  0,               1];

T05 = T01 * T12 * T23 * T34 * T45;
T15 = T12 * T23 * T34 * T45;
invT01 = inv(T01);
invT12 = inv(T12);
invT23 = inv(T23);
invT34 = inv(T34);
invT45 = inv(T45);

Px = -251*c1 + 350*c1*c23 + 300*c1*c2 - 35*s1 + 50*c1;
Py = 350*s1*c23 + 300*s1*c2 + 35*c1 + 50*s1;
Pz = -350*s23 - 300*s2 + 107.5;

%% (a)
d3 = 35;
x = 400;
y = 100;
z = 0;
phi = pi/4;
theta = 0;
psi = pi;
[pos_t1, ~] = compute_theta_1(x, y, d3);
[pos_t5, ~] = compute_theta_5(phi, psi, theta, x, y, d3);
t3 = compute_theta_3(pos_t1, x, y, z);
t2 = compute_theta_2(pos_t1, t3, x, y);
t4 = compute_theta_4(t2, t3);
fprintf('Question (a)\n');
fprintf(['t1(radian): %.6f\t t1(degree): %.6f\n'...
        't2(radian): %.6f\t t2(degree): %.6f\n'...
        't3(radian): %.6f\t t3(degree): %.6f\n'...
        't4(radian): %.6f\t t4(degree): %.6f\n'...
        't5(radian): %.6f\t t5(degree): %.6f\n\n\n']...
        ,pos_t1, pos_t1*180/pi, t2, t2*180/pi,...
        t3, t3*180/pi, t4, t4*180/pi,...
        pos_t5, pos_t5*180/pi);

%% (b)
d3 = 35;
x = 400;
y = 120;
z = 100;
phi = pi/4;
theta = 0;
psi = pi;
[pos_t1, ~] = compute_theta_1(x, y, d3);
[pos_t5, ~] = compute_theta_5(phi, psi, theta, x, y, d3);
t3 = compute_theta_3(pos_t1, x, y, z);
t2 = compute_theta_2(pos_t1, t3, x, y);
t4 = compute_theta_4(t2, t3);
fprintf('Question (b)\n');
fprintf(['t1(radian): %.6f\t t1(degree): %.6f\n'...
        't2(radian): %.6f\t t2(degree): %.6f\n'...
        't3(radian): %.6f\t t3(degree): %.6f\n'...
        't4(radian): %.6f\t t4(degree): %.6f\n'...
        't5(radian): %.6f\t t5(degree): %.6f\n\n\n']...
        ,pos_t1, pos_t1*180/pi, t2, t2*180/pi,...
        t3, t3*180/pi, t4, t4*180/pi,...
        pos_t5, pos_t5*180/pi);

%% (c)
d3 = 35;
x = 400;
y = -100;
z = 120;
phi = -pi/4;
theta = 0;
psi = pi;
[pos_t1, neg_t1] = compute_theta_1(x, y, d3);
[pos_t5, neg_t5] = compute_theta_5(phi, psi, theta, x, y, d3);
t3 = compute_theta_3(pos_t1, x, y, z);
t2 = compute_theta_2(pos_t1, t3, x, y);
t4 = compute_theta_4(t2, t3);
fprintf('Question (c)\n');
fprintf(['t1(radian): %.6f\t t1(degree): %.6f\n'...
        't2(radian): %.6f\t t2(degree): %.6f\n'...
        't3(radian): %.6f\t t3(degree): %.6f\n'...
        't4(radian): %.6f\t t4(degree): %.6f\n'...
        't5(radian): %.6f\t t5(degree): %.6f\n\n\n']...
        ,pos_t1, pos_t1*180/pi, t2, t2*180/pi,...
        t3, t3*180/pi, t4, t4*180/pi,...
        pos_t5, pos_t5*180/pi);

