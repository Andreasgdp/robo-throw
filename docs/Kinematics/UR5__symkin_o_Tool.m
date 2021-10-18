%kinematics UR5
%q - joints
syms q1 q2 q3 q4 q5 q6 
syms cq1 sq1 cq2 sq2 cq3 sq3 cq4 sq4 cq5 sq5 cq6 sq6 
syms A1 A2 A3 A4 A5 A6
syms R1 R2 R3 R4 R5 R6
syms z1 z2 z3 z4 z5 z6

cq1 = cos(q1);
sq1 = sin(q1);
cq2 = cos(q2);
sq2 = sin(q2);
cq3 = cos(q3);
sq3 = sin(q3);
cq4 = cos(q4);
sq4 = sin(q4);
cq5 = cos(q5);
sq5 = sin(q5);
cq6 = cos(q6);
sq6 = sin(q6);

%ROTATION
A1 = [  cq1   0  sq1 0
        sq1   0 -cq1 0
        0     1   0    0.08916
        0     0   0    1];
A2 = [  cq2 -sq2 0 -0.425*cq2
        sq2  cq2 0 -0.425*sq2
        0     0      1  0
        0     0      0  1];
A3 = [  cq3 -sq3 0 -0.392*cq3
        sq3  cq3 0 -0.392*sq3
        0     0      1  0
        0     0      0  1];
A4 = [  cq4   0  sq4   0
        sq4   0 -cq4   0
        0     1   0    0.1092
        0     0   0    1];
A5 = [  cq5   0 -sq5   0
        sq5   0  cq5   0
        0    -1   0    0.0947
        0     0   0    1];
A6 = [  cq6 -sq6   0  0
        sq6  cq6   0  0
        0     0    1  0.0823
        0     0    0  1];

%{
A1 = [  1   1  1 1
        1   1 1 1
        1     1   1    1
        1     1   1    1];
A2 = [  2 2 2 2
        2  2 2 2
        2     2      2  2
        2     2      2  2];
A3 = [  3 3 3 3
        3  3 3 3
        3     3      3  3
        3     3      3  3];
A4 = [  4   4  4   4
        4   4 4   4
        4     4   4    4
        4     4   4    4];
A5 = [  5   5 5   5
        5   5  5   5
        5    5   5    5
        5     5   5    5];
A6 = [  6 6   6  6
        6  6   6  6
        6     6    6  6
        6     6    6  6];
%}

%{
A1 = [  1   1  1 1
        1   1 1 1
        1     1   1    1
        1     1   1    1];
A2 = [  1   1  1 1
        1   1 1 1
        1     1   1    1
        1     1   1    1];
A3 = [  1   1  1 1
        1   1 1 1
        1     1   1    1
        1     1   1    1];
A4 = [  1   1  1 1
        1   1 1 1
        1     1   1    1
        1     1   1    1];
A5 = [  1   1  1 1
        1   1 1 1
        1     1   1    1
        1     1   1    1];
A6 = [  1   1  1 1
        1   1 1 1
        1     1   1    1
        1     1   1    1];
%}
    
%FORWARD KINEMATICS    
T = A1*A2*A3*A4*A5*A6;

T1=A1;
T2=A1*A2;
T3=A1*A2*A3;
T4=A1*A2*A3*A4;
T5=A1*A2*A3*A4*A5;
T6=T;

p = T(1:3,4);

p0 = [0;0;0];
p1 = T1(1:3,4);
p2 = T2(1:3,4);
p3 = T3(1:3,4);
p4 = T4(1:3,4);
p5 = T5(1:3,4);
p6 = T6(1:3,4);

R = T(1:3,1:3);
R1 = A1(1:3,1:3); 
R2 = A2(1:3,1:3); 
R3 = A3(1:3,1:3); 
R4 = A4(1:3,1:3); 
R5 = A5(1:3,1:3); 
R6 = A6(1:3,1:3); 

%GEOMETRICAL JACOBIAN
%position part
J(1:3,1) = diff(p,q1);
J(1:3,2) = diff(p,q2);
J(1:3,3) = diff(p,q3);
J(1:3,4) = diff(p,q4);
J(1:3,5) = diff(p,q5);
J(1:3,6) = diff(p,q6);

%rotation part
k = [0 0 1]';
J(4:6,1) = k;
J(4:6,2) = R1*k;
J(4:6,3) = R1*R2*k;
J(4:6,4) = R1*R2*R3*k;
J(4:6,5) = R1*R2*R3*R4*k;
J(4:6,6) = R1*R2*R3*R4*R5*k;

%disp(J);

J_I = inv(J);
%disp(J_I);
   
%res = simplify(J * inv(J));
%disp(res);

symbols.q1 = q1;
symbols.q2 = q2;
symbols.q3 = q3;
symbols.q4 = q4;
symbols.q5 = q5;
symbols.q6 = q6;

params.q1 = 1;
params.q2 = 1;
params.q3 = 1;
params.q4 = 1;
params.q5 = 1;
params.q6 = 1;

res = inverseJacobian(J_I, symbols, params);
disp(eval(simplify(res)));


function out = inverseJacobian(J_I, symbols, params)
    M1 = subs(J_I, symbols.q1, params.q1);
    M2 = subs(M1, symbols.q2, params.q2);
    M3 = subs(M2, symbols.q3, params.q3);
    M4 = subs(M3, symbols.q4, params.q4);
    M5 = subs(M4, symbols.q5, params.q5);
    M6 = subs(M5, symbols.q6, params.q6);
    out = M6;
end
    