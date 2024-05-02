function TA=Geometric_J_to_Analytical_J(phi,nu,psi)
T=[0 -sin(phi) cos(phi)*sin(nu);
    0  cos(phi) sin(phi)*sin(nu);
    1  0        cos(nu)        ];
TA=[eye(3,3) zeros(3,3);
    zeros(3,3) T];