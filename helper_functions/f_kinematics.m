%% Generates the forward kinematics T0H cell array with T0H{i} matricies.
function [T0H, T0H_sym, joint_variables] = f_kinematics(thetas, alphas, a, d)
    % This funciton calculates the T0H array (1xn array with n T0H 4x4
    % matricies), from a list of thetas, alphas, a's, and d's. This
    % function returns the calculated T0H and the symbolic T0H, as a
    % tuple.
    
    % USAGE: 
    %        input thetas as radians or variables as a cell matrix: FOR JOINT VARIABLES, 
    %           INPUT A '_' NEXT TO THE VARIABLE: i.e thetas = {'_theta1' 0 0}, or  
    %        input alphas as radians as cell matrix, i.e alphas = {0 pi/2 pi/4}
    %        input a as numbers as cell matrix, i.e a = {a1 0 0}
    %        input d as numbers or variables as cell matrix: FOR JOINT VARIABLES, 
    %           INPUT A '_' NEXT TO THE VARIABLE: i.e d = {0 'd2' '_d3'}
    %     to return the reduced T0H matrix, the symbolic T0H matrix, and joint variables do:
    %     [T0H, T0H_sym, joint_variables] = f_kinematics(thetas, alphas, a, d)

    number_joints = length(thetas);
    if (length(alphas) ~= number_joints) || (length(a) ~= number_joints) || (length(d) ~= number_joints)
        error('The input vectors must have the same length as the number of joints.');
    end
    
    
    T = @(theta,alpha,a,d) [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1];
    
    % Creating mixed symbolic array for user inputs
    q_list = createMixedArray(thetas);
    d_sym = createMixedArray(d);
    for i=1:number_joints
        if a{i} ~= 0
            a_sym{i} = sym(sprintf('a%d', i));
        else
            a_sym{i} = 0;
        end
        if alphas{i} ~= 0
            alpha_list{i} = sym(sprintf('alpha%d', i));
        else
            alpha_list{i} = 0;
        end
    end

    % getting joint variables
    joint_variables = {};
    for i=1:number_joints
        if isa(q_list{i}, 'sym') && contains(char(q_list{i}), '_')
            joint_variables(i) = q_list(i);
        elseif isa(d_sym{i}, 'sym') && contains(char(d_sym{i}), '_')
            joint_variables(i) = d_sym(i);
        end
    end

    % stripping the '_' from jointvariables
    for i = 1:length(joint_variables)
        var_str = string(joint_variables{i});
        var_str = strrep(var_str, '_', '');
        joint_variables{i} = sym(var_str);
    end
    
    % stripping the '_' from the input lists
    for i = 1:number_joints
        var_str_theta = string(thetas{i});
        var_str_theta = strrep(var_str_theta, '_', '');
        q_list{i} = sym(var_str_theta);
        var_str_d = string(d{i});
        var_str_d = strrep(var_str_d, '_', '');
        d_sym{i} = sym(var_str_d);
    end

    % initializing T0H
    for i=1:number_joints
        T0H_sym{i} = zeros(4);
    end

    % Creating Symbolic T0H array
    for i=1:number_joints
        if i == 1
            T0H_sym{i} = T(q_list{i},alpha_list{i},a_sym{i}, d_sym{i});
        else
            T0H_sym{i} = T0H_sym{i-1} * T(q_list{i},alpha_list{i},a_sym{i}, d_sym{i});
        end
        T0H_sym{i} = simplify(T0H_sym{i});
    end

    % Creating Numerical T0H array
    for i=1:number_joints
        T0H{i} = zeros(4);
    end

    for i = 1:number_joints
        T0H{i} = subs(T0H_sym{i}, [q_list, alpha_list, a_sym, d_sym], [q_list, alphas, a, d_sym]);
        T0H{i} = simplify(T0H{i});
    end
    joint_variables = joint_variables';
end

function mixedArray = createMixedArray(inputArray)
    % Initialize an empty cell array
    mixedArray = cell(size(inputArray));
    for i = 1:length(inputArray)
        if ischar(inputArray{i}) % Check if the input is a character array (string)
            mixedArray{i} = sym(inputArray{i}, 'real'); % Create a symbolic variable
        else
            mixedArray{i} = inputArray{i}; % Use the numeric value directly
        end
    end
end
