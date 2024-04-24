% Main script

% Open the GUI and store the handle to the GUI
hGui = gui_guide;
handles = guidata(hGui);



% HELPER FUNCTION INTEGRATION

% [T0H, T0H_sym] = f_kinematics(thetas, alphas, as, ds)



% plot1
% axesHandle1 = handles.axes1;% Access the axes handle
% axes(axesHandle1);% Make the GUI's axes current
% plot(axesHandle1, rand(10,1), rand(10,1)); % Plot something
% xlabel(axesHandle1,"x axis")
% ylabel(axesHandle1,"y axis")
% title(axesHandle1," plot title")
% 
% 
% % set(handles.myStaticText, 'String', 'New Text Here');


% Update the GUI data
guidata(hGui, handles);
