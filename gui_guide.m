function varargout = gui_guide(varargin)
% GUI_GUIDE MATLAB code for gui_guide.fig
%      GUI_GUIDE, by itself, creates a new GUI_GUIDE or raises the existing
%      singleton*.
%
%      H = GUI_GUIDE returns the handle to a new GUI_GUIDE or the handle to
%      the existing singleton*.
%
%      GUI_GUIDE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_GUIDE.M with the given input arguments.
%
%      GUI_GUIDE('Property','Value',...) creates a new GUI_GUIDE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_guide_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_guide_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_guide

% Last Modified by GUIDE v2.5 29-Apr-2024 17:01:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @gui_guide_OpeningFcn, ...
    'gui_OutputFcn',  @gui_guide_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui_guide is made visible.
function gui_guide_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_guide (see VARARGIN)

% Choose default command line output for gui_guide
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_guide wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_guide_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over text2.
function text2_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function dataTable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dataTable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in exportButton.
% Callback function for the button to export table data
function exportButton_Callback(hObject, eventdata, handles)


% Fetch the data from the table
dataTable = findobj('Tag', 'dataTable'); % find table by Tag
tableData = get(dataTable, 'Data'); % get data from table
exportedData = array2table(tableData, 'VariableNames',{'thetas', 'alphas', 'as', 'ds', 'joint_types', 'link_masses', 'motor_mass', 'I_motors', 'I_links', 'trans_ratios','friction_coeffs'});
assignin('base', 'exportedData', tableData);

% Fetch the data from the controls table
controlsDataTable = findobj('Tag', 'controlsData'); % find table by Tag
controlsTableData = get(controlsDataTable, 'Data'); % get data from table
controlsexportedData = array2table(controlsTableData, 'VariableNames',{'intial_xyz', 'intial_phi_theta_rho', 'desired_xyz', 'des_phi_theta_rho', 'num_theta', 'num_aplhpa', 'num_a', 'num_d', 'gravity_mat', 'des_time','maxEF_accel'});
assignin('base', 'controlsexportedData', controlsexportedData);

% Optionally, display a message to the user
disp('Data has been exported to the base workspace under the variable name "exportedData".');

% VARS 
var_update(exportedData,controlsexportedData);

% get data from workspace
thetas = evalin('base', 'thetas');
alphas = evalin('base', 'alphas');
as = evalin('base', 'as');
ds = evalin('base', 'ds');
link_masses = evalin('base', 'link_masses');
motor_mass = evalin('base', 'motor_mass');
I_motors = evalin('base', 'I_motors');
trans_ratios = evalin('base', 'trans_ratios');
friction_coeffs = evalin('base', 'friction_coeffs');
I_links = evalin('base', 'I_links');
gravity_mat = evalin('base', 'gravity_mat');
joint_types_list = evalin('base', 'joint_types_list');

% equations 
equations = equations_of_motion(thetas, alphas, as, ds, joint_types_list, gravity_mat, ...
I_links, I_motors, link_masses, motor_mass, trans_ratios, friction_coeffs);
[plotinfo varsplotinfo] = return_plots_equations(equations);


%random eqn
syms x
a = rand; b = rand; c = rand;
symbExpr = a*x^2+ c;  % Example quadratic equation
latexStr = latex(symbExpr);


% eqn1
latexStr1 = latex(equations);

axes(handles.eqn1); % Ensure 'axesEquation' is your axes' tag
cla; % Clear axes
text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr1 '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 9);

% 
% % eqn2
% axes(handles.eqn2); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn2, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn3
% axes(handles.eqn3); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn3, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn4
% axes(handles.eqn4); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn4, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn5
% axes(handles.eqn5); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn5, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn6
% axes(handles.eqn6); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn6, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);

% % eqn7
% axes(handles.eqn7); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn7, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn8
% axes(handles.eqn8); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn8, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn9
% axes(handles.eqn9); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn9, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn10
% axes(handles.eqn10); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn10, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn11
% axes(handles.eqn11); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn11, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);
% 
% % eqn12
% axes(handles.eqn12); % Ensure 'axesEquation' is your axes' tag
% cla; % Clear axes
% set(handles.eqn12, 'Visible','off')
% text('Units', 'normalized', 'Position', [0.5 0.5], 'String', ['$' latexStr '$'], 'Interpreter', 'latex', 'HorizontalAlignment', 'center', 'FontSize', 12);

% PLOT 1


axesHandle1 = handles.axes1;
axes(axesHandle1);
    cla(axesHandle1, 'reset'); 

hold on;  % Keep the plot from refreshing each loop iteration
legendLabels1 = cell(1, length(equations));
for ii = 1:length(equations)
    fplot(axesHandle1, subs(equations(ii), varsplotinfo, eval(varsplotinfo)), [0, 10]);
    
    legendLabels1{ii} = sprintf('Equation of Motion %d', ii);

end

% Label the axes and give a title to the graph
xlabel(axesHandle1,'Time (s)');
ylabel(axesHandle1, 'Values of Equation of Motion');
title(axesHandle1, 'Plot of the Equation of Motion');
legend(axesHandle1, legendLabels1, 'Location', 'best');
hold off;  % Release the hold on the current plot

% 
% 
% %plot 2
% axesHandle2 = handles.axes2;% Access the axes handle
% axes(axesHandle2);% Make the GUI's axes current
% plot(axesHandle2, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle2,"x axis")
% ylabel(axesHandle2,"y axis")
% title(axesHandle2," plot title")
% 
% %plot 3
% axesHandle3 = handles.axes3;% Access the axes handle
% axes(axesHandle3);% Make the GUI's axes current
% plot(axesHandle3, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle3,"x axis")
% ylabel(axesHandle3,"y axis")
% title(axesHandle3," plot title")
% 
% %plot 4
% axesHandle4 = handles.axes4;% Access the axes handle
% axes(axesHandle4);% Make the GUI's axes current
% plot(axesHandle4, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle4,"x axis")
% ylabel(axesHandle4,"y axis")
% title(axesHandle4," plot title")
% 
% %plot 5
% axesHandle5 = handles.axes5;% Access the axes handle
% axes(axesHandle5);% Make the GUI's axes current
% plot(axesHandle5, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle5,"x axis")
% ylabel(axesHandle5,"y axis")
% title(axesHandle5," plot title")
% 
% %plot 6
% axesHandle6 = handles.axes6;% Access the axes handle
% axes(axesHandle6);% Make the GUI's axes current
% plot(axesHandle6, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle6,"x axis")
% ylabel(axesHandle6,"y axis")
% title(axesHandle6," plot title")

%plot 7
% axesHandle7 = handles.axes7;% Access the axes handle
% axes(axesHandle7);% Make the GUI's axes current
% plot(axesHandle7, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle7,"x axis")
% ylabel(axesHandle7,"y axis")
% title(axesHandle7," plot title")
% 
% %plot 8
% axesHandle8 = handles.axes8;% Access the axes handle
% axes(axesHandle8);% Make the GUI's axes current
% plot(axesHandle8, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle8,"x axis")
% ylabel(axesHandle8,"y axis")
% title(axesHandle8," plot title")

   


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over compliance_control.


function compliance_control_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to compliance_control (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over impedence_control.
function impedence_control_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to impedence_control (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when selected object is changed in controlSelection.
function controlSelection_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in controlSelection 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

controlSelectedObj = get(handles.controlSelection, 'SelectedObj');
disp(controlSelectedObj);

if controlSelectedObj == handles.compliance_control
    % plot 7
    axesHandle7 = handles.axes7;
    axes(axesHandle7);
    cla(axesHandle7, 'reset'); 
    plot(axesHandle7, rand(10,1), rand(10,1), 'b', rand(10,1), rand(10,1), 'r--'); % Blue solid and red dashed lines
    xlabel(axesHandle7, 'x axis');
    ylabel(axesHandle7, 'y axis');
    title(axesHandle7, 'Compliance Plot 1');
    legend(axesHandle7,'desired', 'actual')
    
    % plot 8
    axesHandle8 = handles.axes8;
    axes(axesHandle8);
    cla(axesHandle8, 'reset'); 
    plot(axesHandle8, rand(10,1), rand(10,1));
    xlabel(axesHandle8, 'x axis');
    ylabel(axesHandle8, 'y axis');
    title(axesHandle8, 'Compliance Plot 2');


elseif controlSelectedObj == handles.impedence_control
    % plot 7
    axesHandle7 = handles.axes7;
    axes(axesHandle7);
    cla(axesHandle7, 'reset'); 
    plot(axesHandle7, rand(10,1), rand(10,1), 'b', rand(10,1), rand(10,1), 'r--'); % Blue solid and red dashed lines
    xlabel(axesHandle7, 'x axis');
    ylabel(axesHandle7, 'y axis');
    title(axesHandle7, 'Impedance Plot 1');
    legend(axesHandle7,'desired', 'actual')
    
    % plot 8
    axesHandle8 = handles.axes8;
    axes(axesHandle8);
    cla(axesHandle8, 'reset'); 
    plot(axesHandle8, rand(10,1), rand(10,1));
    xlabel(axesHandle8, 'x axis');
    ylabel(axesHandle8, 'y axis');
    title(axesHandle8, 'Impedance Plot 2');
   
end


% --- Executes when entered data in editable cell(s) in controlsData.
function controlsData_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to controlsData (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
