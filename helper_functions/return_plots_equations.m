%% Plotting the equations of motion based on the joint variables
function plotinfo = return_plots_equations(equations)
    vars = symvar(equations);
    t = sym('t', 'real'); % Define a symbolic variable for time
    for i = 1:length(vars)
        if ~isequal(vars(i), t) % Check if the variable is not the time variable
            assignin('caller', char(vars(i)), sin(t + i)); % Example initialization
        end
    end
    
    % Getting Equations of motion and assigning them
    plotinfo = sym(zeros(length(equations), 2));
    if numel(equations) > 0
        for i=1:length(equations)
            plotinfo(i, 1) = (equations(i));
            plotinfo(i, 2) = vars;
        end
    end
    % FOR PLOTTING:
        % fplot(subs(equations_of_motion(1), vars, eval(vars)), [0, 10]);
        % xlabel('Time (s)');
        % ylabel('Values of Equation of Motion');
        % title('Plot of the Equation of Motion');
end
