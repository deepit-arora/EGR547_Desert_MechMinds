function xtilda = error_compliance_control(Td,Te)
od=Td(1:3,4);
oe=Te(1:3,4);
Rd=Td(1:3,1:3);
Re=Te(1:3,1:3);
Rde=Rd'*Re;
phi=atan2(Rde(1,3),Rde(2,3));
nu=atan2(sqrt((Rde(1,3))^2+(Rde(1,3)))^2,Rde(3,3));
psi=atan2(Rde(3,2),-Rde(3,1));
odde=Rd'*(oe-od);
phide=[phi;nu;psi];
xtilda = -[odde;phide];