%% Compliance control function
function [qvec, qdotvec, xevec, he, tvec] = run_compliance(desired_position, ...
    initial_joint_variables, desired_time, my_robot, desired_kp, desired_kd)
    % This function computes the q, q_dot, and end effector position using
    % compliance control for specified robot.
    num_joint_variables = length(initial_joint_variables);
    global jac0 tau;
    
    % formatting required conditions and desired conditions
    Te0=my_robot.fkine(initial_joint_variables);
    Td=[cos(pi/4) -sin(pi/4) 0 desired_position(1);sin(pi/4) cos(pi/4) 0 desired_position(2);0 0 1 desired_position(3);0 0 0 1];
    Te=Te0;
    xevec=(Te(1:3,4))';
    q0= my_robot.ikunc(Te);
    q=q0;
    qvec=q;
    qdot0=zeros(1,length(q));
    qdot=qdot0;
    qdotvec=qdot;
    [tau,jac0] = my_robot.gravjac(q);
    hevec=zeros(1,6);
    t=0;
    tvec=t;
    dt=0.1;

    % getting user gains input
    KP=desired_kp*eye(6,6);
    KD=desired_kd*eye(6,6);
    
    while t<desired_time
        [tau,jac0] = my_robot.gravjac(q(end,1:num_joint_variables));
        [JAd,JAdtrans] = JAdesired(Td,Te,t,jac0);
        xtilda = error_compliance_control(Td,Te);
        u=tau'+JAdtrans*(KP*xtilda-KD*JAd*qdot(end,1:num_joint_variables)');
        torqfun= @(my_robot,dt,q,qdot) u';
        [ti,q,qdot] = my_robot.fdyn(dt,torqfun,q(end,1:num_joint_variables),qdot(end,1:num_joint_variables));
        Te = my_robot.fkine(q(end,1:num_joint_variables));
        he=pinv(jac0')*JAdtrans*KP*xtilda;
        xevec=[xevec;(Te(1:3,4))'];
        tvec=[tvec;ti(end,1)+t];
        qvec=[qvec;q];
        qdotvec=[qdotvec;qdot];
        hevec=[hevec;he'];
        t=t+dt;
    end
end
