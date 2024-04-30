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
contVariableNames = {'initial_xyz', 'initial_phi_theta_rho', 'desired_xyz', 'des_phi_theta_rho', 'num_theta', 'num_alpha', 'num_a', 'num_d', 'gravity_mat', 'des_time', 'maxEF_accel'};
for k = 1:length(contVariableNames)
    controlscurrentData = str2num(cell2mat(controlsexportedData{1, k}));
  controlscurrentData = controlscurrentData';
    % Handle cases where the entire column is empty or non-applicable data
    if isempty(controlscurrentData)
        controlscurrentData = NaN; 
    end
%     controlscurrentData = str2double(controlscurrentData);
    assignin('base', contVariableNames{k}, controlscurrentData');
end

joint_types = evalin('base', 'joint_types');


if exist('joint_types', 'var') == 1  
    max_length = max(cellfun(@length, joint_types));
    joint_types_list = char(cellfun(@(x) [x repmat(' ', 1, max_length - length(x))], joint_types, 'UniformOutput', false));
    joint_types_list = joint_types_list';
    assignin('base', 'joint_types_list', joint_types_list);
end


% FORMAT FIX
fixvars = {'link_masses', 'motor_mass', 'I_motors', 'I_links', 'trans_ratios', 'friction_coeffs'};
for k = 1:length(fixvars)
    value = evalin('base', fixvars{k});
    converted_value = str2double(value);
    assignin('base', fixvars{k}, converted_value);
end


% link_masses = evalin('base', 'link_masses');
% motor_mass = evalin('base', 'motor_mass');
% I_motors = evalin('base', 'I_motors');
% trans_ratios = evalin('base', 'trans_ratios');
% friction_coeffs = evalin('base', 'friction_coeffs');
% I_links = evalin('base', 'I_links');
% 
% 
% 
% link_masses = str2double(link_masses);
% motor_mass = str2double(motor_mass);
% I_motors = str2double(I_motors);
% I_links = str2double(I_links);
% trans_ratios = str2double(trans_ratios);
% friction_coeffs = str2double(friction_coeffs);
% 
% assignin('base', 'link_masses', link_masses);
% assignin('base', 'motor_mass', motor_mass);
% assignin('base', 'I_motors', I_motors);
% assignin('base', 'I_links', I_links);
% assignin('base', 'trans_ratios', trans_ratios);
% assignin('base', 'friction_coeffs', friction_coeffs);

end


