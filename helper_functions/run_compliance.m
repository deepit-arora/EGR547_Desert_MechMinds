%% Compliance control function
function [qvec, qdotvec, xevec, hevec, tvec] = run_compliance(desired_position, ...
    initial_end_effector_position, initial_euler_angles, desired_time, my_robot, desired_kp, ...
    desired_kd, desired_k)
    % This function computes the q, q_dot, end effector position, and forces using
    % compliance control for specified robot.
    num_joint_variables = my_robot.n;
    global jac0 tau;
    
% initial euler angles zyz from user
    phi0=initial_euler_angles(1);
    theta0=initial_euler_angles(2);
    psi0=initial_euler_angles(3);
% initial end effector position from user
    xe0=initial_end_effector_position;
    xe=xe0;
    xevec=xe;
% desired end effector positon from user
    xd=desired_position;
% transformation matrix for euler angles and end effector position
    Te0 = eul2tr(phi0, theta0, psi0);
    Te0=Te0*[eye(3,3) xe0';0 0 0 1];
    Te=Te0;
% initial joint parameters
    q0= R.ikunc(Te0);
    q=q0;
    qvec=q;
% initial velocities are zero
    qdot0=zeros(1,length(q));
    qdot=qdot0;
    qdotvec=qdot;

    [tau,jac0] = my_robot.gravjac(q);
    t=0;
    tvec=t;
    hevec=zeros(1,3);
    dt=0.1;  % time step of .1

% user inputs for K
    K=diag([desired_k(1) desired_k(2) desired_k(3)]);
    KP=diag([desired_kp(1) desired_kp(2) desired_kp(3)]);
    KD=diag([desired_kd(1) desired_kd(2) desired_kd(3)]);

    while t<desired_time
        [tau,jac0] = my_robot.gravjac(q(end,1:num_joint_variables));
        eul = tr2eul(Te);
        j0 = my_robot.jacob0(q(end,1:num_joint_variables), 'eul',eul);
        JA=j0(1:3,1:end);
        xtilda = xd-xe;
        TA = eul2jac(eul);
        u=tau'+JA'*(KP*xtilda'-KD*JA*qdot(end,1:num_joint_variables)'-TA'*K*(xe-xe0)');
        torqfun= @(my_robot,dt,q,qdot) u';
        [ti,q,qdot] = my_robot.fdyn(dt,torqfun,q(end,1:num_joint_variables),qdot(end,1:3));
        Te = my_robot.fkine(q(end,1:num_joint_variables));
        xe=(Te(1:3,4))';
        he=-K*(xe-xe0)';
        hevec=[hevec;he'];
        xevec=[xevec;(Te(1:3,4))'];
        tvec=[tvec;ti(end,1)+t];
        qvec=[qvec;q];
        qdotvec=[qdotvec;qdot];
        t=t+dt;
    end
end
