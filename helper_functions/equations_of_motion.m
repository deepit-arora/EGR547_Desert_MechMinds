%% Combine Equations to get Equations of motion
function equations = equations_of_motion(thetas, alphas, a, d, jointTypes, g0, friction_coeff)
    % This function combines all previous functions to use the 
    % Joint Space Dynamic Model Equation, and then outputs the equations of
    % motion. 

    % USAGE:
    %        input thetas as radians or variables as cell matrix, i.e thetas = {'theta1' 0 0}
    %        input alphas as radians as cell matrix, i.e alphas = {0 -pi/2 0}
    %        input a as numbers as cell matrix, i.e a = {0 0 0}
    %        input d as numbers or variables as cell matrix, i.e d = {0 'd2' 'd3'}
    %        input jointTypes as letters, i.e jointTypes = ['R', 'P', 'P']
    %        input g0 as an initial gravity matrix, i.e g = [0 -9.81 0]
    %        input friction coefficient as a number, i.e friction = .3
    %    To return equations of motion, do:
    %    equations = equations_of_motion(thetas, alphas, a, d, jointTypes, g0, friction)
    %    equations will the the nx1 matrix representing the equations of
    %    motion


    % Combining all equation functions
    [~, ~, joint_variables] = f_kinematics(thetas, alphas, a, d);
    [B_sum, ~, ~, ~, ~] = inertia_matrix(thetas, alphas, a, d, jointTypes);
    gravity_terms = gravity_matrix(thetas, alphas, a, d, jointTypes, g0);
    c = c_matrix(thetas, alphas, a, d, jointTypes);


    % Using Joint Space Dynamic Model Equation
    
    % getting q dot
    for i = 1:length(joint_variables)
        joint_variables_dot{i} = sym([char(joint_variables{i}), '_dot']);
    end
    joint_variables_dot = joint_variables_dot';

    % getting q_ddot
    for i = 1:length(joint_variables)
        joint_variables_ddot{i} = sym([char(joint_variables{i}), '_ddot']);
    end
    joint_variables_ddot = joint_variables_ddot';
    
    % Summing terms into equations of motion
    % B(q)q_ddot + C(q, q_dot)*q_dot + Fv*q_dot + Fssgn(q_dot) + g(q) = tau
    equations = simplify(B_sum*sym(joint_variables_ddot) + c*sym(joint_variables_dot) + ...
        friction_coeff*sym(joint_variables_dot) + gravity_terms);

    % simplifying equations
    for i=1:length(equations)
        equations(i) = simplify(equations(i));
    end
end
