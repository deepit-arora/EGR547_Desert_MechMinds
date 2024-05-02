function [JAd,JAdtrans] = JAdesired(Td,Te,t,jac0)
J=jac0;
Rd=Td(1:3,1:3);
Re=Te(1:3,1:3);
Rde=Rd'*Re;
phi=atan2(Rde(1,3),Rde(2,3));
nu=atan2(sqrt((Rde(1,3))^2+(Rde(1,3)))^2,Rde(3,3));
psi=atan2(Rde(3,2),-Rde(3,1));
if nu==0
    TA=eye(6,6);
else
TA=Geometric_J_to_Analytical_J(phi,nu,psi);
end
JAd=inv(TA)*[Rd' zeros(3,3);zeros(3,3) Rd']*J;
JAdtrans=JAd';