%% Generates the forward kinematics T0H cell array with T0H{i} matricies.

function [T0H, T0H_sym] = f_kinematics(thetas, alphas, a, d)
    % This funciton calculates the T0H array (1xn array with n T0H 4x4
    % matricies), from a list of thetas, alphas, a's, and d's. This
    % function returns the calculated T0H and the symbolic T0H, as a
    % tuple.
    
    number_joints = length(thetas);
    if (length(alphas) ~= number_joints) || (length(a) ~= number_joints) || (length(d) ~= number_joints)
        error('The input vectors must have the same length as the number of joints.');
    end
    
    dh = [thetas' d' a' alphas'];
    
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

    % Getting symbolic expression for geometric jacobian
    for i=1:number_joints
        T0H_sym{i} = eye(4);
    end

    % Getting T0H array
    for i=1:number_joints
        if i == 1
            T0H_sym{i} = T(q_list{i},alpha_list{i},a_sym{i}, d_sym{i});
        else
            T0H_sym{i} = T0H_sym{i-1} * T(q_list{i},alpha_list{i},a_sym{i}, d_sym{i});
        end
    end

    % Simplifying T0H array
    for i=1:number_joints
        T0H_sym{i} = simplify(T0H_sym{i});
    end

    thetas_sym = arrayfun(@sym, thetas, 'UniformOutput', false);
    alphas_sym = arrayfun(@sym, alphas, 'UniformOutput', false);
    a_sym_values = arrayfun(@sym, a, 'UniformOutput', false);
    d_sym_values = arrayfun(@sym, d, 'UniformOutput', false);

    subs_map = [q_list', thetas_sym'; alpha_list', alphas_sym'; a_sym', a_sym_values'; d_sym', d_sym_values'];
    T0H = cellfun(@(x) subs(x, subs_map(:, 1), subs_map(:, 2)), T0H_sym, 'UniformOutput', false);
    
    % Return numeric T0H matrices
    T0H = cellfun(@double, T0H, 'UniformOutput', false);

end
