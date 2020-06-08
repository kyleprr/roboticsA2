function varargout = Robotics_A2_GUI(varargin)
% ROBOTICS_A2_GUI MATLAB code for Robotics_A2_GUI.fig
%      ROBOTICS_A2_GUI, by itself, creates a new ROBOTICS_A2_GUI or raises the existing
%      singleton*.
%
%      H = ROBOTICS_A2_GUI returns the handle to a new ROBOTICS_A2_GUI or the handle to
%      the existing singleton*.
%
%      ROBOTICS_A2_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOTICS_A2_GUI.M with the given input arguments.
%
%      ROBOTICS_A2_GUI('Property','Value',...) creates a new ROBOTICS_A2_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Robotics_A2_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Robotics_A2_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Robotics_A2_GUI

% Last Modified by GUIDE v2.5 08-Jun-2020 15:43:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Robotics_A2_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Robotics_A2_GUI_OutputFcn, ...
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

% --- Executes just before Robotics_A2_GUI is made visible.
function Robotics_A2_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Robotics_A2_GUI (see VARARGIN)

% Choose default command line output for Robotics_A2_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
%default joint angles on load up:
handles.user.jointAngle = deg2rad([0 -85 45 40 0]);

if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
    %disp_Callback(@disp_Callback, eventdata, handles);
end

% --- Outputs from this function are returned to the command line.
function varargout = Robotics_A2_GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in disp.
function disp_Callback(hObject, eventdata, handles)
% hObject    handle to disp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
cla;

L1 = Link('d',0.138,'a',0,'alpha',-pi/2,'offset', 0, 'qlim',[-pi pi]);
L2 = Link('d',0,'a',0.135,'alpha',0,'offset', 0, 'qlim',deg2rad([-85 15]));
L3 = Link('d',0,'a',0.147,'alpha',0,'offset', 0, 'qlim',deg2rad([30 120])) ;  
L4 = Link('d', 0, 'a', 0.055, 'alpha', -pi/2, 'offset', 0, 'qlim', deg2rad([-20 60]));
L5 = Link('d', 0.055, 'a', 0, 'alpha', 0, 'offset', 0,  'qlim', [0 0]);

model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot');
for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri');         
    model.faces{linkIndex+1} = faceData;
    model.points{linkIndex+1} = vertexData;
end
% Display robot
workspace = [-0.4 0.4 -0.4 0.4 0 0.4];   
model.plot3d(zeros(1,model.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end  
model.delay = 0;
% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles,'UserData');
    try 
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                      , plyData{linkIndex+1}.vertex.green ...
                                                      , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end
data = guidata(hObject);
data.model = model;
guidata(hObject,data);

% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});

% --- Executes on slider movement.
function joint_1_Callback(hObject, eventdata, handles)
% hObject    handle to joint_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_change(hObject, handles);

% --- Executes during object creation, after setting all properties.
function joint_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint_2_Callback(hObject, eventdata, handles)
% hObject    handle to joint_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% joint_2_angle = handles.user.joint_2_val
% set(handles.joint2val,'String',joint_2_angle);
slider_change(hObject, handles);

% --- Executes during object creation, after setting all properties.
function joint_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint_3_Callback(hObject, eventdata, handles)
% hObject    handle to joint_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_change(hObject, handles);

% --- Executes during object creation, after setting all properties.
function joint_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint_4_Callback(hObject, eventdata, handles)
% hObject    handle to joint_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
slider_change(hObject, handles);

% --- Executes during object creation, after setting all properties.
function joint_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function joint_5_Callback(hObject, eventdata, handles)
% hObject    handle to joint_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% --- Executes during object creation, after setting all properties.
function joint_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function slider_change(hObject, handles)
handles.user.jointAngle(1) = deg2rad(get(handles.joint_1, 'Value'));
handles.user.jointAngle(2) = deg2rad(get(handles.joint_2, 'Value'));
handles.user.jointAngle(3) = deg2rad(get(handles.joint_3, 'Value'));
handles.user.jointAngle(4) = deg2rad(get(handles.joint_4, 'Value'));
handles.user.jointAngle(5) = deg2rad(get(handles.joint_5, 'Value'));

%joint_angle_string = sprintf('%d  %d  %d  %d  %d', handles.user.jointAngle)
joint_angle_string_box = sprintf('%d  %d  %d  %d  %d', round(rad2deg(handles.user.jointAngle)));
set(handles.cur_ang, 'String', joint_angle_string_box);
move_arm(hObject, handles);

% Custom function to move arm to user inputed joint limits
function move_arm(hObject, handles)
handles.model.animate(handles.user.jointAngle);

function joint1val_Callback(hObject, eventdata, handles)
% hObject    handle to joint1val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint1val as text
%        str2double(get(hObject,'String')) returns contents of joint1val as a double

% --- Executes during object creation, after setting all properties.
function joint1val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint1val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint2val_Callback(hObject, eventdata, handles)
% hObject    handle to joint2val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint2val as text
%        str2double(get(hObject,'String')) returns contents of joint2val as a double

% --- Executes during object creation, after setting all properties.
function joint2val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint2val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint3val_Callback(hObject, eventdata, handles)
% hObject    handle to joint3val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint3val as text
%        str2double(get(hObject,'String')) returns contents of joint3val as a double

% --- Executes during object creation, after setting all properties.
function joint3val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint3val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint4val_Callback(hObject, eventdata, handles)
% hObject    handle to joint4val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint4val as text
%        str2double(get(hObject,'String')) returns contents of joint4val as a double

% --- Executes during object creation, after setting all properties.
function joint4val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint4val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint5val_Callback(hObject, eventdata, handles)
% hObject    handle to joint5val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of joint5val as text
%        str2double(get(hObject,'String')) returns contents of joint5val as a double

% --- Executes during object creation, after setting all properties.
function joint5val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint5val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function gtposex_Callback(hObject, eventdata, handles)
% hObject    handle to gtposex (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gtposex as text
%        str2double(get(hObject,'String')) returns contents of gtposex as a double

% --- Executes during object creation, after setting all properties.
function gtposex_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gtposex (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function gtposey_Callback(hObject, eventdata, handles)
% hObject    handle to gtposey (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gtposey as text
%        str2double(get(hObject,'String')) returns contents of gtposey as a double

% --- Executes during object creation, after setting all properties.
function gtposey_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gtposey (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function gtposez_Callback(hObject, eventdata, handles)
% hObject    handle to gtposez (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gtposez as text
%        str2double(get(hObject,'String')) returns contents of gtposez as a double

% --- Executes during object creation, after setting all properties.
function gtposez_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gtposez (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in gotoxyzpose.
function gotoxyzpose_Callback(hObject, eventdata, handles)
% hObject    handle to gotoxyzpose (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1

% --- Executes on button press in jogxminus.
function jogxminus_Callback(hObject, eventdata, handles)
% hObject    handle to jogxminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in jogxplus.
function jogxplus_Callback(hObject, eventdata, handles)
% hObject    handle to jogxplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in jogyminus.
function jogyminus_Callback(hObject, eventdata, handles)
% hObject    handle to jogyminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in jogyplus.
function jogyplus_Callback(hObject, eventdata, handles)
% hObject    handle to jogyplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in jogzminus.
function jogzminus_Callback(hObject, eventdata, handles)
% hObject    handle to jogzminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in jogzplus.
function jogzplus_Callback(hObject, eventdata, handles)
% hObject    handle to jogzplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in e_stop.
function e_stop_Callback(hObject, eventdata, handles)
% hObject    handle to e_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pause('on')

% --- Executes on button press in homeCall.
function homeCall_Callback(hObject, eventdata, handles)
% hObject    handle to homeCall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% setting values for joints
handles.user.jointAngle(1) = deg2rad(0);
handles.user.jointAngle(2) = deg2rad(-85);
handles.user.jointAngle(3) = deg2rad(45);
handles.user.jointAngle(4) = deg2rad(40);
handles.user.jointAngle(5) = deg2rad(0);

% extracting the matrix to animate it
joint_angle_string_box = sprintf('%d  %d  %d  %d  %d', round(rad2deg(handles.user.jointAngle)));
set(handles.cur_ang, 'String', joint_angle_string_box);

% updating sliders
set(handles.joint_1, 'Value', rad2deg(handles.user.jointAngle(1)));
set(handles.joint_2, 'Value', rad2deg(handles.user.jointAngle(2)));
set(handles.joint_3, 'Value', rad2deg(handles.user.jointAngle(3)));
set(handles.joint_4, 'Value', rad2deg(handles.user.jointAngle(4)));
set(handles.joint_5, 'Value', rad2deg(handles.user.jointAngle(5)));

move_arm(hObject, handles);

% --- Executes on button press in posCall.
function posCall_Callback(hObject, eventdata, handles)
% hObject    handle to posCall (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes on button press in posSet.
function posSet_Callback(hObject, eventdata, handles)
% hObject    handle to posSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)         