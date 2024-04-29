%% Combine Equations to get Equations of motion
function equations = equations_of_motion(thetas, alphas, a, d, jointTypes, g0, ...
    Il_list_numeric, Im_list_numeric, ml_list_numeric, mm_list_numeric, kr_list_numeric, ...
    friction_coeff_numeric)
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
    %        input ALL _numeric LISTS AS LISTS, i.e mm_list_numeric = [1 1 1]
    %    To return equations of motion, do:
    %    equations = equations_of_motion(thetas, alphas, a, d, jointTypes, g0, ...
    %    Il_list_numeric, Im_list_numeric, ml_list_numeric, mm_list_numeric, kr_list_numeric, ...
    %    friction_coeff_numeric)
    %    equations will the the nx1 matrix representing the equations of
    %    motion
    
    number_joints = length(thetas);

    % Combining all equation functions
    [~, ~, joint_variables, ~] = f_kinematics(thetas, alphas, a, d);
    [B_sum, ~, ~, ~, ~, Il_list, Im_list, ml_list, mm_list, kr_list] = inertia_matrix(thetas, alphas, a, d, jointTypes);
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
        friction_coeff_numeric*sym(joint_variables_dot) + gravity_terms);
    
    % simplifying equations
    for i=1:length(equations)
        equations(i) = simplify(equations(i));
    end

    % SUBSTITUTING IN I's and M's FOR EVERY JOINT/MOTOR
    for i=1:number_joints
        equations(i) = simplify(subs(equations(i), sym(Il_list), Il_list_numeric));
        equations(i) = simplify(subs(equations(i), sym(Im_list), Im_list_numeric));
        equations(i) = simplify(subs(equations(i), sym(ml_list), ml_list_numeric));
        equations(i) = simplify(subs(equations(i), sym(mm_list), mm_list_numeric));
        equations(i) = simplify(subs(equations(i), sym(kr_list), kr_list_numeric));
    end
end
