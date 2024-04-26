%% Derive n(q, qdot) Matrix based on DH inputs
function n = n_matrix(thetas, alphas, a, d, jointTypes, g0, friction_coeff)
    % This function accepts DH parameters, gravity matrix, and friction
    % coefficient, and returns the n_q_qdot matrix as a symbolic matrix.

    % USAGE:
    %        input thetas as radians or variables as a cell matrix: FOR JOINT VARIABLES, 
    %           INPUT A '_' NEXT TO THE VARIABLE: i.e thetas = {'_theta1' 0 0}, or  
    %        input alphas as radians as cell matrix, i.e alphas = {0 pi/2 pi/4}
    %        input a as numbers as cell matrix, i.e a = {a1 0 0}
    %        input d as numbers or variables as cell matrix: FOR JOINT VARIABLES, 
    %           INPUT A '_' NEXT TO THE VARIABLE: i.e d = {0 'd2' '_d3'}
    %        input jointTypes as a list of joint types, i.e jointTypes = ['R', 'P', 'P']
    %        input g0 as the initial gravity matrix, i.e g0 = [0 0 -9.81]
    %        input friction_coefficient as a number, i.e friction_coeff = .3
    %
    % TO RUN THE CODE, DO:
    % n = n_matrix(thetas, alphas, a, d, jointTypes, g0, friction_coeff)


    % getting joint variables
    [~, ~, joint_variables] = f_kinematics(thetas, alphas, a, d);
    
    % getting q dot
    for i = 1:length(joint_variables)
        joint_variables_dot{i} = sym([char(joint_variables{i}), '_dot']);
    end
    joint_variables_dot = joint_variables_dot';

    % getting c matrix
    c = c_matrix(thetas, alphas, a, d, jointTypes);

    % getting gravity terms
    gravity_terms = gravity_matrix(thetas, alphas, a, d, jointTypes, g0);

    % n(q, qdot) = C*q_dot*F*q_dot (Friction coef) + g (gravity terms)
    n = sym(c)*sym(joint_variables_dot) + sym(friction_coeff)*sym(joint_variables_dot) + sym(gravity_terms);
    n = simplify(n);
end
