%% Derive Intertia Matrix based on DH inputs
function gravity_terms = gravity_matrix(thetas, alphas, a, d, jointTypes, g0)
    % This function accepts DH parameters, jointTypes, and a g0 matrix
    % representing the initial gravity terms, and returns the g(q) matrix.

    % USAGE: 
    %        input thetas as radians or variables as cell matrix, i.e thetas = {'theta1' 0 0}
    %        input alphas as radians as cell matrix, i.e alphas = {0 pi/2 pi/4}
    %        input a as numbers as cell matrix, i.e a = {a1 0 0}
    %        input d as numbers or variables as cell matrix, i.e d = {0 d2 d3}
    %        input jointTypes as letters, i.e jointTypes = ['R', 'P', 'P']
    %        input g0 as an initial gravity matrix, i.e g = [0 -9.81 0]
    %     to return the reduced gravity_terms matrix, do:
    %     gravity_terms = gravity_matrix(thetas, alphas, a, d, jointTypes, g0)

    % getting number of gravity terms
    number_joints = length(thetas);
    
    % getting symbolic gravity list
    for i = 1:length(g0)
        if g0(i) ~= 0
            g_list{i} = sym(sprintf('g%d', i));  % Create a symbolic variable g_i
            if g0(i) < 0
                % When g0(i) is negative, substitute 'g_i' with '-g_i'
                g_list{i} = subs(g_list{i}, g_list{i}, -g_list{i});
            end
        else
            g_list{i} = 0;
        end
    end

    % getting m list
    for i = 1:number_joints
        ml_list{i} = sym(sprintf('ml_%d', i));
        mm_list{i} = sym(sprintf('mm_%d', i));
    end

    g_list = g_list';
    g0 = g0';

    % Getting initial intertia matrix
    [B, Jp_l, Jo_l, Jp_m, Jo_m] = inertia_matrix(thetas, alphas, a, d, jointTypes);
    
    for i=1:number_joints
        g{i} = sym(zeros(3, 1));
    end

    % Calculating gravitational terms
    for i=1:number_joints
        for j=1:number_joints
            if j == 1
                g{j} = ((ml_list{j}.*g_list'*Jp_l{j}(:, i)) + (mm_list{j}.*g_list'*Jp_m{j}(:, i)));
            else
                g_sum = ((ml_list{j}.*g_list'*Jp_l{j}(:, i)) + (mm_list{j}.*g_list'*Jp_m{j}(:, i)));
                g{j} = g{j-1} + g_sum;
            end
        end
        gravity_terms{i} = simplify(-g{j});
    end
    gravity_terms = gravity_terms';
end
