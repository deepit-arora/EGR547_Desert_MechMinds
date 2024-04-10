%% Function - Jacobian from DH parameters

function J_a = analytical_jacobian(thetas, alphas, a, d, Q)
" This function accepts thetas in a list, alphas in a list," + ...
    "a values in a list, and d values in a list, as the dh parameters" + ...
    "of a robot. This function also requires Q, which are the joint" + ...
    "parameters of a 1xn matrix.";

    number_joints = length(thetas);
    if (length(alphas) ~= number_joints) || (length(a) ~= number_joints) || (length(d) ~= number_joints)
        error('The input vectors must have the same length as the number of joints.');
        return
    end
    
    dh = [thetas' d' a' alphas'];

    for i = 1:number_joints
        L(i) = Link('d', dh(1,2), 'a', dh(1,3), 'alpha', dh(1,4));
    end

    Robot = SerialLink(L);
    
    forward_kinematics = Robot.fkine(Q); % answer to the forward kinematics
    
    T = @(theta,alpha,a,d) [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1];

    q_list = cell(1, number_joints);
    alpha_list = cell(1, number_joints);
    d_sym = cell(1, number_joints);
    a_sym = cell(1, number_joints);
    for i = 1:number_joints
        q_list{i} = sym(sprintf('q%d', i));
        alpha_list{i} = sym(sprintf('alpha%d', i));
        d_sym{i} = sym(sprintf('d%d', i));
        a_sym{i} = sym(sprintf('a%d', i));
    end

    % Getting symbolic expression for forward kinematics for
    % analytical jacobian computation
    T0H = eye(4);
    for i=1:number_joints
        T0H = T0H * T(q_list{i},alpha_list{i},d_sym{i},a_sym{i});
    end
    
    T0H = simplify(T0H);

    % assigning J_a as a 3xn matrix - it is NOT 6xn because we do not
    % care about the orientation
    J_a = sym('J_a', [3, number_joints]);

    % looping thru the number of joints and taking the partial derivative
    % wrt to each 'q' for each column.
    for i=1:number_joints
        J_a(:, i) = diff(T0H(1:3, i), q_list{i});
    end
    
    J_a = simplify(J_a)
end


