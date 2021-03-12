function varargout = gui1(varargin)
% GUI1 MATLAB code for gui1.fig
%      GUI1, by itself, creates a new GUI1 or raises the existing
%      singleton*.
%
%      H = GUI1 returns the handle to a new GUI1 or the handle to
%      the existing singleton*.
%
%      GUI1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI1.M with the given input arguments.
%
%      GUI1('Property','Value',...) creates a new GUI1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui1

% Last Modified by GUIDE v2.5 12-Mar-2021 15:57:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui1_OpeningFcn, ...
                   'gui_OutputFcn',  @gui1_OutputFcn, ...
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


% --- Executes just before gui1 is made visible.
function gui1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui1 (see VARARGIN)

    r1 = 1;
    r2 = 1;
    ang1 = 0;
    ang2 = 0;
    mass1 = 3;
    mass2 = 3;
    time = str2double(get(handles.tempo,'string'));
    
    x1 = r1 * sin(ang1);
    y1 = -r1 * cos(ang1);
    
    x2 = x1 + r2 * sin(ang2);
    y2 = y1 - r2 * cos(ang2);
    
    vetor1_x = [0 x1];
    vetor1_y = [0 y1];
    
    vetor2_x = [x1 x2];
    vetor2_y = [y1 y2];
    
    plot(handles.animacao,vetor1_x,vetor1_y,'-k','Linewidth',1)
    hold(handles.animacao,'on')
    plot(handles.animacao,vetor2_x,vetor2_y,'-k','Linewidth',1)
    hold(handles.animacao,'on')
    plot(handles.animacao,x1,y1,'r.','Markersize',mass1*10)
    plot(handles.animacao,x2,y2,'b.','Markersize',mass2*10)
    axis(handles.animacao,[-2 2 -3 0.5])
    title(handles.animacao,'Double Pendulum')
    
    
    title(handles.grafpos,'Position Variation of mass 1 and mass 2')
    ylim(handles.grafpos,[-6 6])
    xlim(handles.grafpos,[-4 4])
    grid(handles.grafpos,'on')
    
    ylabel(handles.grafpos,'y1 and y2')
    xlabel(handles.grafpos,'x1 and x2')
    
    ylabel(handles.grafang,'Angle 1')
    xlabel(handles.grafang,'Angle 2')
    title(handles.grafang,'Angle Variation of angle 1 and angle 2')
    axis(handles.grafang,[-2.5 2.5 -20 20])
    
    title(handles.grafec,'Kinetic Energy Variation')
    xlim(handles.grafec,[0 time])
    ylim(handles.grafec,[0 500])
    ylabel(handles.grafec,'Kinetic Energy (J)')
    xlabel(handles.grafec,'time (s)')
    
    
% Choose default command line output for gui1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function r1_Callback(hObject, eventdata, handles)
% hObject    handle to r1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r1 as text
%        str2double(get(hObject,'String')) returns contents of r1 as a double


% --- Executes during object creation, after setting all properties.
function r1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function m1_Callback(hObject, eventdata, handles)
% hObject    handle to m1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of m1 as text
%        str2double(get(hObject,'String')) returns contents of m1 as a double


% --- Executes during object creation, after setting all properties.
function m1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to m1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ang1_Callback(hObject, eventdata, handles)
% hObject    handle to ang1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ang1 as text
%        str2double(get(hObject,'String')) returns contents of ang1 as a double


% --- Executes during object creation, after setting all properties.
function ang1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ang1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function v1_Callback(hObject, eventdata, handles)
% hObject    handle to v1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v1 as text
%        str2double(get(hObject,'String')) returns contents of v1 as a double


% --- Executes during object creation, after setting all properties.
function v1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function m2_Callback(hObject, eventdata, handles)
% hObject    handle to m2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of m2 as text
%        str2double(get(hObject,'String')) returns contents of m2 as a double


% --- Executes during object creation, after setting all properties.
function m2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to m2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function r2_Callback(hObject, eventdata, handles)
% hObject    handle to r2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r2 as text
%        str2double(get(hObject,'String')) returns contents of r2 as a double


% --- Executes during object creation, after setting all properties.
function r2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function v2_Callback(hObject, eventdata, handles)
% hObject    handle to v2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of v2 as text
%        str2double(get(hObject,'String')) returns contents of v2 as a double


% --- Executes during object creation, after setting all properties.
function v2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to v2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ang2_Callback(hObject, eventdata, handles)
% hObject    handle to ang2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ang2 as text
%        str2double(get(hObject,'String')) returns contents of ang2 as a double


% --- Executes during object creation, after setting all properties.
function ang2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ang2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in START.
function Start_Callback(hObject, eventdata, handles)
% hObject    handle to START (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

r1 = str2double(get(handles.r1,'string'));
r2 = str2double(get(handles.r2,'string'));
ang1 = deg2rad(str2double(get(handles.ang1,'string')));
ang2 = deg2rad(str2double(get(handles.ang2,'string')));
mass1 = str2double(get(handles.m1,'string'));
mass2 = str2double(get(handles.m2,'string'));
vel1 = str2double(get(handles.v1,'string'));
vel2 = str2double(get(handles.v2,'string'));
g= str2double(get(handles.gravidade,'string'));
delta_t = 0.02;
time = str2double(get(handles.tempo,'string'));

if (get(handles.popupmenu1,'Value')==1)
    for t = 0:delta_t:time

        x1 = r1 * sin(ang1);
        y1 = -r1 * cos(ang1);

        x2 = x1 + r2 * sin(ang2);
        y2 = y1 - r2 * cos(ang2);

        vetor1_x = [0 x1];
        vetor1_y = [0 y1];

        vetor2_x = [x1 x2];
        vetor2_y = [y1 y2];


    % Double pendelum animation

        plot(handles.animacao,vetor1_x,vetor1_y,'-k','Linewidth',1)
        hold(handles.animacao,'on')
        plot(handles.animacao,vetor2_x,vetor2_y,'-k','Linewidth',1)
        plot(handles.animacao,x1,y1,'r.','Markersize',mass1*10)
        plot(handles.animacao,x2,y2,'b.','Markersize',mass2*10)
        axis(handles.animacao,[(-r1-r2) (r1+r2) (-r1-r2) (r1+r2)])
        hold(handles.animacao,'off')

    % Angle Variation Graphics

        plot(handles.grafang,ang1,ang2,'k.')
        ylabel(handles.grafang,'Angle 1')
        xlabel(handles.grafang,'Angle 2')
        title(handles.grafang,'Angle Variation of angle 1 and angle 2')
        hold(handles.grafang,'on')
        axis(handles.grafang,[-2.5 2.5 -20 20])

    % Position Variation Graphics

        plot(handles.grafpos,t,y1,'k.')
        hold(handles.grafpos,'on')
        plot(handles.grafpos,t,y2,'r.')
        hold(handles.grafpos,'on')
        ylabel(handles.grafpos,'y1 and y2')
        xlabel(handles.grafpos,'x1 and x2')
        title(handles.grafpos,'Position Variation of mass 1 and mass 2')
        ylim(handles.grafpos,[-6 6])
        xlim(handles.grafpos,[t-4 t+4])
        grid(handles.grafpos,'on')
        
        % Euler method

        [ang1,ang2,vel1,vel2] = Euler_method(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t);
        ec = Kinetic_Energy(mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
        plot(handles.grafec,t,ec,'b.')
        hold(handles.grafec,'on')
        xlim(handles.grafec,[0 time])
        ylim(handles.grafec,[0 500])
        title(handles.grafec,'Kenetic Energy Variation')
        ylabel(handles.grafec,'Kenetic Energy (J)')
        xlabel(handles.grafec,'time (s)')
        drawnow
    end
    
else
    
    r1 = str2double(get(handles.r1,'string'));
    r2 = str2double(get(handles.r2,'string'));
    ang1 = deg2rad(str2double(get(handles.ang1,'string')));
    ang2 = deg2rad(str2double(get(handles.ang2,'string')));
    mass1 = str2double(get(handles.m1,'string'));
    mass2 = str2double(get(handles.m2,'string'));
    vel1 = str2double(get(handles.v1,'string'));
    vel2 = str2double(get(handles.v2,'string'));
    g= str2double(get(handles.gravidade,'string'));
    delta_t = 0.02;
    time = str2double(get(handles.tempo,'string'));
    
        for t = 0:delta_t:time

        x1 = r1 * sin(ang1);
        y1 = -r1 * cos(ang1);

        x2 = x1 + r2 * sin(ang2);
        y2 = y1 - r2 * cos(ang2);

        vetor1_x = [0 x1];
        vetor1_y = [0 y1];

        vetor2_x = [x1 x2];
        vetor2_y = [y1 y2];


    % Double Pendelum animation
    
        title(handles.animacao,'Double Pendulum')
        plot(handles.animacao,vetor1_x,vetor1_y,'-k','Linewidth',1)
        hold(handles.animacao,'on')
        plot(handles.animacao,vetor2_x,vetor2_y,'-k','Linewidth',1)
        plot(handles.animacao,x1,y1,'r.','Markersize',mass1*10)
        plot(handles.animacao,x2,y2,'b.','Markersize',mass2*10)
        axis(handles.animacao,[(-r1-r2) (r1+r2) (-r1-r2) (r1+r2)])
        hold(handles.animacao,'off')

    % Angle Variation Graphics

        plot(handles.grafang,ang1,ang2,'k.')
        ylabel(handles.grafang,'Angle 1')
        xlabel(handles.grafang,'Angle 2')
        title(handles.grafang,'Angle Variation of angle 1 and angle 2')
        hold(handles.grafang,'on')
        axis(handles.grafang,[-2.5 2.5 -20 20])

    % Position Variation Graphics

        plot(handles.grafpos,t,y1,'k.')
        hold(handles.grafpos,'on')
        plot(handles.grafpos,t,y2,'r.')
        hold(handles.grafpos,'on')
        ylabel(handles.grafpos,'y1 and y2')
        xlabel(handles.grafpos,'x1 and x2')
        title(handles.grafpos,'Position variation of mass 1 and mass 2')
        ylim(handles.grafpos,[-6 6])
        xlim(handles.grafpos,[t-4 t+4])
        grid(handles.grafpos,'on')

        % Rungue-Kutta 4º degree

        [ang1,ang2,vel1,vel2] = Runge_Kutta_method(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t);
        ec = Kinetic_Energy(mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
        plot(handles.grafec,t,ec,'r.')
        hold(handles.grafec,'on')
        xlim(handles.grafec,[0 time])
        ylim(handles.grafec,[0 500])
        title(handles.grafec,'Kenetic Energy Variation')
        ylabel(handles.grafec,'Kenetic Energy (J)')
        xlabel(handles.grafec,'time (s)')
        drawnow
        
        end
end

function f = aceleration1(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2)
    d1 = -g*(2*mass1 + mass2) * sin(ang1) - mass2*g*sin(ang1 - 2*ang2) - 2*sin(ang1 - ang2)*mass2*((vel2^2)*r2 + (vel1^2)*r1*cos(ang1- ang2));
    d2 = r1*(2*mass1+mass2-mass2*cos(2*ang1-2*ang2));
f = (d1)/d2;


function f = aceleration2(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2)
    d1 = 2*sin(ang1 - ang2)*((vel1^2)*r1*(mass1 + mass2) + g*(mass1 + mass2)*cos(ang1) + (vel2^2)*r2*mass2*cos(ang1 - ang2));
    d2 = r2*(2*mass1+mass2-mass2*cos(2*ang1-2*ang2));
f = (d1)/d2;


function [angle1,angle2,velocity1,velocity2] = Runge_Kutta_method(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t)
    
    %valores iniciais
    
    v1=vel1;
    v2=vel2;
    a1=ang1;
    a2=ang2;
    ac1=aceleration1(g,mass1,mass2,a1,a2,v1,v2,r1,r2);
    ac2=aceleration2(g,mass1,mass2,a1,a2,v1,v2,r1,r2);
    
    %k1

    v1_1=v1+(delta_t*ac1)/2;
    v2_1=v2+(delta_t*ac2)/2;
    a1_1=a1+(delta_t*v1_1)/2;
    a2_1=a2+(delta_t*v2_1)/2;
    acl_1=aceleration1(g,mass1,mass2,a1_1,a2_1,v1_1,v2_1,r1,r2);
    ac2_1=aceleration2(g,mass1,mass2,a1_1,a2_1,v1_1,v2_1,r1,r2);
    
    %k2

    vl_2=v1_1+(delta_t*acl_1)/2;
    v2_2=v2_1+(delta_t*ac2_1)/2;
    a1_2=a1_1+(delta_t*vl_2)/2;
    a2_2=a2_1+(delta_t*v2_2)/2;
    acl_2=aceleration1(g,mass1,mass2,a1_2,a2_2,vl_2,v2_2,r1,r2);
    ac2_2=aceleration2(g,mass1,mass2,a1_2,a2_2,vl_2,v2_2,r1,r2);
    
    %k3

    vl_3=v1_1+(delta_t*acl_2);
    v2_3=v2_1+(delta_t*ac2_2);
    a1_3=a1_1+(delta_t*vl_3);
    a2_3=a2_1+(delta_t*v2_3);
    acl_3=aceleration1(g,mass1,mass2,a1_3,a2_3,vl_3,v2_3,r1,r2);
    ac2_3=aceleration2(g,mass1,mass2,a1_3,a2_3,vl_3,v2_3,r1,r2);
    
    %k4
    
    velocity1=v1+(delta_t*(ac1+2*acl_1+2*acl_2+acl_3))/6;
    angle1=a1+(delta_t*(v1+2*v1_1+2*vl_2+vl_3))/6;
    
    velocity2=v2+(delta_t*(ac2+2*ac2_1+2*ac2_2+ac2_3))/6;
    angle2=a2+(delta_t*(v2+2*v2_1+2*v2_2+v2_3))/6;

function [ang1,ang2,vel1,vel2] = Euler_method(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2,delta_t)

    ac1=aceleration1(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
    ac2=aceleration2(g,mass1,mass2,ang1,ang2,vel1,vel2,r1,r2);
    
    vel1 = vel1+ac1*delta_t;
    vel2 = vel2+ac2*delta_t;
    
    ang1 = ang1+vel1*delta_t;
    ang2 = ang2+vel2*delta_t;


function f= Kinetic_Energy(mass1,mass2,ang1,ang2,vel1,vel2,r1,r2)
    f=0.5*mass1*(r1^2)*(vel1^2)+0.5*mass2*((r1^2)*(vel1^2)+(r2^2)*(vel2^2)+2*r1*r2*vel1*vel2*cos(ang1-ang2));




function tempo_Callback(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of time as text
%        str2double(get(hObject,'String')) returns contents of time as a double


% --- Executes during object creation, after setting all properties.
function tempo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function gravidade_Callback(hObject, eventdata, handles)
% hObject    handle to gravity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of gravity as text
%        str2double(get(hObject,'String')) returns contents of gravity as a double


% --- Executes during object creation, after setting all properties.
function gravidade_CreateFcn(hObject, eventdata, handles)
% hObject    handle to gravity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
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
