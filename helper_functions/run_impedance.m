function [qvec, qdotvec, xevec, hevec, tvec] = run_impedance(initial_end_effector_position, ...
    initial_euler_angles, desired_time, my_robot, desired_kp, desired_ke, ...
    desired_kd, desired_kde, desired_md, desired_mde, ...
    desired_pos_coeff)
    global jac0 tau;
    syms x
    % initial euler angles zyz from user
    phi0=initial_euler_angles(1);
    theta0=initial_euler_angles(2);
    psi0=initial_euler_angles(3);

    % initial end effector position from user
    xe0=initial_end_effector_position;
    xe=xe0;
    xevec=xe;

    Te0 = eul2tr(phi0, theta0, psi0);
    Te0=Te0*[eye(3,3) xe0';0 0 0 1];
    Te=Te0;

    q0= my_robot.ikunc(Te0);
    q=q0;
    qvec=q;
    qdot0=zeros(1,length(q));
    qdot=qdot0;
    qdotvec=qdot;

    [tau,jac0] = my_robot.gravjac(q);
    t=0;
    tvec=t;
    hevec=zeros(1,3);
    dt=0.5;

    % user inputs for K
    KP=diag([desired_kp(1) desired_kp(2) desired_kp(3)]);
    Ke=diag([desired_ke(1) desired_ke(2) desired_ke(3)]);
    KD=diag([desired_kd(1) desired_kd(2) desired_kd(3)]);
    KDe=diag([desired_kde(1) desired_kde(2) desired_kde(3)]);
    Md=diag([desired_md(1), desired_md(2), desired_md(3)]);
    Mdinv=inv(Md);   
    Md_e = diag([desired_mde(1), desired_mde(2), desired_mde(3)]);

    % Calculate position symbolicly
    xd_sym = poly2sym(desired_pos_coeff, x);
    xd = zeros(1, 3);
    xd(1) = double(subs(xd_sym, t)); % Substitute t and convert to double
    
    % Calculate velocity
    xddot_sym = diff(xd_sym, x);
    xddot = zeros(1, 3);
    xddot(1) = double(subs(xddot_sym, t)); % Substitute t and convert to double
    
    % Calculate acceleration
    xddotdot_sym = diff(xddot_sym, x);
    xddotdot = zeros(1, 3);
    xddotdot(1) = double(subs(xddotdot_sym, t)); % Substitute t and convert to double

    while t<desired_time
        % Calculate position symbolicly
        xd(1) = double(subs(xd_sym, t)); % Substitute t and convert to double
        
        % Calculate velocity
        xddot(1) = double(subs(xddot_sym, t)); % Substitute t and convert to double
        
        % Calculate acceleration
        xddotdot(1) = double(subs(xddotdot_sym, t)); % Substitute t and convert to double

        eul = tr2eul(Te);
        j0 = my_robot.jacob0(q(end,1:3),'eul',eul);
        JA=j0(1:3,1:end);
        jac0d = my_robot.jacob_dot(q(end,1:3), qdot(end,1:3));
        JAdotqdot=jac0d(1:3,1:end);
        TA = eul2jac(eul);
        xtilda = xd-xe;
        xedot=JA*qdot';
        xtildadot=xddot-xedot';
        % JApi=transpose(JA)/(JA*transpose(JA));
        y=pinv(JA)*Mdinv*(Md*xddotdot'+KD*xtildadot'+KP*xtilda'-Md*JAdotqdot-TA'*(Ke*(xe-xe0)'+KDe*xedot));
        u = my_robot.rne(q(end,1:3), qdot(end,1:3), y(1:3,end)');
        torqfun= @(my_robot,dt,q,qdot) u;
        [ti,q,qdot] = my_robot.fdyn(dt,torqfun,q(end,1:3),qdot(end,1:3));
        Te = my_robot.fkine(q(end,1:3));
        xe=(Te(1:3,4))';
        he=-Ke*(xe-xe0)'-KDe*xedot;
        hevec=[hevec;he(1:3,end)'];
        xevec=[xevec;(Te(1:3,4))'];
        tvec=[tvec;ti(end,1)+t];
        qvec=[qvec;q];
        qdotvec=[qdotvec;qdot];
        t=t+dt;
    end
end