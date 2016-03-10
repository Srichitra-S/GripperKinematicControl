function varargout = Gripper_Simulation(varargin)
% GRIPPER_SIMULATION MATLAB code for Gripper_Simulation.fig
%      GRIPPER_SIMULATION, by itself, creates a new GRIPPER_SIMULATION or raises the existing
%      singleton*.
%
%      H = GRIPPER_SIMULATION returns the handle to a new GRIPPER_SIMULATION or the handle to
%      the existing singleton*.
%
%      GRIPPER_SIMULATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GRIPPER_SIMULATION.M with the given input arguments.
%
%      GRIPPER_SIMULATION('Property','Value',...) creates a new GRIPPER_SIMULATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Gripper_Simulation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Gripper_Simulation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Gripper_Simulation

% Last Modified by GUIDE v2.5 10-Jan-2016 13:55:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Gripper_Simulation_OpeningFcn, ...
                   'gui_OutputFcn',  @Gripper_Simulation_OutputFcn, ...
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


% --- Executes just before Gripper_Simulation is made visible.
function Gripper_Simulation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Gripper_Simulation (see VARARGIN)

% Initialize
% domain = [configs(2) configs(3) configs(4) configs(5)];
% length(symvar(sym(get(handles.equation,'String'))))<=2        
% ezsurf(sym(get(handles.equation,'String')),domain)

% Startup
global anim r_t dr_t
startup_rvc

cview = [-37 30];   % initial viewpoint for animation
% cview = [0 0];
% cview = [0 90];

anim = Animation( handles.axes1 , cview );
r_t = sym(get(handles.r,'String'))
% dr_t = sym(get(handles.dr,'String'))

% Save current view configuration
[az,el] = view(handles.axes1);
handles.custom_data.plot.init_yaw   = az;
handles.custom_data.plot.init_pitch = el;

% Add listener to call the sliders callback while dragging
addlistener(handles.slider1, 'Value', 'PreSet', ...
            @(~,~) slider1_Callback(handles.slider1, 0, handles));
addlistener(handles.slider2, 'Value', 'PreSet', ...
            @(~,~) slider2_Callback(handles.slider2, 0, handles));
addlistener(handles.stop, 'Value', 'PreSet', ...
            @(~,~) stop_Callback(handles.stop, 0, handles));
        
% Choose default command line output for Gripper_Simulation
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Gripper_Simulation wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function [az,el,gui,varargout] = Gripper_Simulation_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
gui = hObject;
[az,el] = view(handles.axes1);
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% init_yaw = handles.custom_data.plot.init_yaw;
% Assuming slider is from -1 to 1
delta = get(hObject, 'Value') * 180;
[curr_yaw, ~] = view(handles.axes1);
view(handles.axes1, [curr_yaw delta]);


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% init_pitch = handles.custom_data.plot.init_pitch;
% Assuming slider is from -1 to 1
delta = get(hObject, 'Value') * 180;
[~, curr_pitch] = view(handles.axes1);
view(handles.axes1, [delta curr_pitch]);


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Run.
function Run_Callback(hObject, eventdata, handles)
% hObject    handle to Run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global anim r_t dr_t

% Definição do objeto
r = 30;             % raio da bola
z1 = 60;            % altura inicial da bola
% Posições dos pontos de contato (frame do objeto)
ro1 = [0 -r 0]';
ro2 = [ r*sin(pi/3) r*cos(pi/3) 0]';
ro3 = [-r*sin(pi/3) r*cos(pi/3) 0]';
% Posição e orientação inicial do objeto
po = [ 0 0 z1 ]';
Rpo = eye(3);
% Quaternion inicial
qo = Quaternion(Rpo);
eta = qo.s;
epsi = qo.v;
% Posições dos pontos de contato iniciais
p1d = ro1 + po;
p2d = ro2 + po;
p3d = ro3 + po;

% Erros iniciais
error = inf;

% Define initial conditions for simulation
thetaHand = deg2rad([ -30 ; 30 ; 30 ; 0 ; -30 ; 30 ; 30 ; -10 ; -30 ; 30 ; 30 ; -10 ]);

% Calcula os parâmetros de grasping.
GP = [ (Rpo*ro1)' ;...
       (Rpo*ro2)' ;...
       (Rpo*ro3)' ];

% Initialize time parameters
it = 1;
h = 0.05;
t = 0;
Total_time = get(handles.sim_time,'String');

% Control loop to set fingers in position
while norm(error) >= 0.001
% tic 

    % Cinemática direta dos dedos
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics_Free( thetaHand );
    
    % Compose states
    e1 = p1;
    e2 = p2;
    e3 = p3;
    e = [ e1 ; e2 ; e3 ]; % ---> state vector
    
    % Reference signals
    ed = [ p1d ; p2d ; p3d ];
    ed_dot = zeros(length(ed),1);
    
    % Error
    error = ed - e;
     
    % Gain definition
    sigma = 10;
    K = sigma*eye(length(e));

    % Cartesian control signals
    v = K*error + ed_dot;
    
    % Geometric jacobian
    o1 = thetaHand(1:4);
    o2 = thetaHand(5:8);
    o3 = thetaHand(9:12);
    
    [ J1, J2, J3 ] = Finger_Jacobians( o1 , o2 , o3 );
    
    % Linear parts
    JP1 = J1(1:3,:);
    JP2 = J2(1:3,:);
    JP3 = J3(1:3,:);
    
    % State jacobian for shared control
    Jstate = blkdiag(JP1,JP2,JP3);
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = pinv(Jstate)*v;
%     dom = pinv(Jhand(1:3,:))*[ eye(3) zeros(3,1) ]*v;
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    
    % Call animation
    [az,el] = view(handles.axes1);
    cview = [az el];
    anim.Animate_Obj(thetaHand,r,po,Rpo,cview);
    
end

% Posição relativa inicial do grasping
dp12_const = p1 - p2;
dp23_const = p2 - p3;

error = inf;

% Beginning of the control loop
while norm(error) >= 0.01
% while time <= Total_time
    tic
    
    % Ar matrix
    Ar = [   eye(3)   -eye(3)  zeros(3,3) ;...
           zeros(3,3)  eye(3)   -eye(3)   ];
    
    % Hand direct kinematics (angles in radians)
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics_Free ( thetaHand );
    
    % Relative vectors
    dp12 = p1 - p2;
    dp23 = p2 - p3;
    
    % Estima posição do objeto (mm)
    po = mean([ p1 - Rpo*ro1 , p2 - Rpo*ro2 , p3 - Rpo*ro3 ]')';
    % Calcula os parâmetros de grasping.
    GP = [ (Rpo*ro1)' ;...
           (Rpo*ro2)' ;...
           (Rpo*ro3)' ];
    
    % Definition of the object state
    eo = po;
    % Definition of the relative state
    er = [ dp12 ; dp23 ];
    % Complete state vector
    e = [ eo ; er ]; % ---> state vector
    
    % Reference signal
    eod = eval(r_t)';
    erd = [ dp12_const ; dp23_const ];        % relative reference
    ed = [ eod ; erd ];                       % desired state vector
    
    % Velocity feedforward terms
%     eod_dot = eval(dr_t)';
    eod_dot = [ 0 0 0 ]';
    erd_dot = zeros(6,1);
    ed_dot = [ eod_dot ; erd_dot ]; % derivative of desired state vector
    
    % State error
    error = ed - e;
    
    % Cartesian control signals
    sigma_obj = 5;
    sigma_rel = 10;
    K = blkdiag(sigma_obj*eye(length(eo)),sigma_rel*eye(length(er)));
    v = K*error + ed_dot;
    
    % Geometric jacobian
    o1 = thetaHand(1:4);
    o2 = thetaHand(5:8);
    o3 = thetaHand(9:12);
    
    [ J1, J2, J3 ] = Finger_Jacobians( o1 , o2 , o3 );
    
    % Linear parts
    JP1 = J1(1:3,:);
    JP2 = J2(1:3,:);
    JP3 = J3(1:3,:);
    
    % Object jacobian matrix
    Jhand = blkdiag(J1,J2,J3);
    JhandP = blkdiag(JP1,JP2,JP3);
    
    % Object jacobian matrix
    Jo = Object_Jacobian( Jhand , GP );
    JPo = Jo(1:3,:);
    JOo = Jo(4:6,:);
    
    % State jacobian matrix
    Jstate = [    JPo    ;...
               Ar*JhandP ];
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = Jstate\v;
    d_qo = qo.dot(JOo*dThetaHand);
    d_eta = d_qo.s;
    d_epsi = d_qo.v;
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    eta = eta + h*d_eta;
    epsi = epsi + h*d_epsi;
    qo = Quaternion([ eta , epsi ]);
    Rpo = qo.R;         % nova matriz de rotação
    
    % Call animation
    [az,el] = view(handles.axes1);
    cview = [az el];
    anim.Animate_Obj(thetaHand,r,po,Rpo,cview);
    
    % Register data
    time_vec(it) = t;
    error_vec(:,it) = error;
    ref_vec(:,it) = ed;
    state_vec(:,it) = e;
    joint_ctrl_vec(:,it) = thetaHand;
    v_ctrl_vec(:,it) = v;
    dstate_vec(:,it) = dThetaHand;
    
    % Time of simulation
    it = it + 1;
    h = toc;
    t = t + h;
%     clc
    
end

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function sim_time_Callback(hObject, eventdata, handles)
% hObject    handle to sim_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sim_time as text
%        str2double(get(hObject,'String')) returns contents of sim_time as a double


% --- Executes during object creation, after setting all properties.
function sim_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sim_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function r_Callback(hObject, eventdata, handles)
% hObject    handle to r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r as text
%        str2double(get(hObject,'String')) returns contents of r as a double


% --- Executes during object creation, after setting all properties.
function r_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function dr_Callback(hObject, eventdata, handles)
% hObject    handle to dr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dr as text
%        str2double(get(hObject,'String')) returns contents of dr as a double


% --- Executes during object creation, after setting all properties.
function dr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
