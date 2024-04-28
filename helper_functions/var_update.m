function var_update(exportedData,controlsexportedData)

        % this function is designed to extract values from thje gui and
        % format them
        %this funtion also runs the helper functions to calculate the
        %equations

        % for dynamics enter as var.. for controls as numbers 
variableNames = {'thetas', 'alphas', 'as', 'ds', 'joint_types', 'link_masses', 'motor_mass', 'I_motors', 'I_links', 'trans_ratios', 'friction_coeffs'};
for k = 1:length(variableNames)
    currentData = exportedData{:, k};
    if isnumeric(currentData)
        % Remove NaNs and ensure it is a column vector if it is numeric
        currentData = currentData(~isnan(currentData));
        if size(currentData, 1) == 1 && size(currentData, 2) > 1
            currentData = currentData'; % Transpose if single row
        end
    else
        % For non-numeric data, just ensure it is in column format
        currentData = currentData(~cellfun(@isempty, currentData));
        if size(currentData, 1) == 1 && size(currentData, 2) > 1
            currentData = currentData';
        end
    end
    % Handle cases where the entire column is empty or non-applicable data
    if isempty(currentData)
        currentData = NaN; 
    end
    assignin('base', variableNames{k}, currentData');
end

% CONTROL TABLE
contVariableNames = {'intial_xyz', 'intial_phi_theta_rho', 'desired_xyz', 'des_phi_theta_rho', 'num_theta', 'num_aplhpa', 'num_a', 'num_d', 'gravity_mat', 'des_time','maxEF_accel'};
for k = 1:length(contVariableNames)
    controlscurrentData = controlsexportedData{:, k};
    if isnumeric(controlscurrentData)
        % Remove NaNs and ensure it is a column vector if it is numeric
        controlscurrentData = controlscurrentData(~isnan(controlscurrentData));
        if size(controlscurrentData, 1) == 1 && size(controlscurrentData, 2) > 1
            controlscurrentData = controlscurrentData'; % Transpose if single row
        end
    else
        % For non-numeric data, just ensure it is in column format
        controlscurrentData = controlscurrentData(~cellfun(@isempty, controlscurrentData));
        if size(controlscurrentData, 1) == 1 && size(controlscurrentData, 2) > 1
            controlscurrentData = controlscurrentData';
        end
    end
    % Handle cases where the entire column is empty or non-applicable data
    if isempty(controlscurrentData)
        controlscurrentData = NaN; 
    end
    assignin('base', contVariableNames{k}, controlscurrentData');
end

thetas = evalin('base', 'thetas');
alphas = evalin('base', 'alphas');
as = evalin('base', 'as');
ds = evalin('base', 'ds');
link_masses =evalin('base', 'link_masses');
motor_mass = evalin('base', 'motor_mass');
I_motors = evalin('base', 'I_motors');
I_links = evalin('base', 'I_links');

joint_types = evalin('base', 'joint_types');

if exist('joint_types', 'var') == 1  
    max_length = max(cellfun(@length, joint_types));
    joint_types_list = char(cellfun(@(x) [x repmat(' ', 1, max_length - length(x))], joint_types, 'UniformOutput', false));
    joint_types_list = joint_types_list';
    assignin('base', 'joint_types_list', joint_types_list);
end



% eqns
%  equations_of_motion = equations_of_motion(thetas, alphas, as, ds, jointTypes, g0)
%
% ja = analytical_Ja(thetas, alphas, as, ds);
%
% anal_jac = analytical_jacobian(thetas, alphas, a, d, Q);
%


% assignin('base', "T0H_sym", T0H_sym )



end


