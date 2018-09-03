function varargout = QAMModDem(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @QAMModDem_OpeningFcn, ...
                   'gui_OutputFcn',  @QAMModDem_OutputFcn, ...
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


% --- Executes just before QAMModDem is made visible.
function QAMModDem_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to QAMModDem (see VARARGIN)

% Choose default command line output for QAMModDem
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes QAMModDem wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = QAMModDem_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Demodul.
function Demodul_Callback(hObject, eventdata, handles)
% hObject    handle to Demodul (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% %XXXXXXXXXXXXXXXXXXXXXX  M-array QAM modulation XXXXXXXXXXXXXXXXXXXXXXXXXXX


% --- Executes on button press in Modulate.
function Modulate_Callback(hObject, eventdata, handles)
% hObject    handle to Modulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%>>>>>>>>>>>>>>>>>> Matlab code for M ary-QAM modulation >>>>>>>>>>>>>>>>>%

M=256;
nbit=4;                                        %number of information bits
data=round(rand(nbit,1))';           % information generation as binary form
disp(' binary data at transmitter: ');
disp(data);
fprintf('\n\n');

%XXXXXXXXXXXXXXXXXXX Random Spreading Code Generation XXXXXXXXXXXXXXXXXXXXXXXX
nbit=3;                                        %number of information bits
spreadsig = round(rand(nbit,1))';
spreadsig = [0 spreadsig];
disp('Random Spread Signal ');
disp(spreadsig);
fprintf('\n\n');

Result1 = xor(spreadsig,[0 0 0 data(1)]);
Result2 = xor(spreadsig,[0 0 data(2) 0]);
Result3 = xor(spreadsig,[0 data(3) 0 0]);
Result4 = xor(spreadsig,[data(4) 0 0 0]);
msg = [Result4 Result3 Result2 Result1];
%XX representation of transmitting binary data as digital signal XXX
x=data;
bp=.000001;                                                    % bit period
bit=[]; 
for n=1:1:length(x)
    if x(n)==1;
       se=ones(1,100);
    else x(n)=0;
        se=zeros(1,100);
    end
     bit=[bit se];

end
axes(handles.axes1)
t1=bp/100:bp/100:100*length(x)*(bp/100);
plot(t1,bit,'lineWidth',2.5);grid on;
axis([ 0 bp*length(x) -.5 1.5]);
ylabel('amplitude(volt)');
xlabel(' time(sec)');
title('Data binary stream information as digital signal');

%XX representation of transmitting binary data as digital signal XXX
x=msg;
bp=.000001;                  
bit=[]; 
for n=1:1:length(x)
    if x(n)==1;
       se=ones(1,100);
    else x(n)=0;
        se=zeros(1,100);
    end
     bit=[bit se];

end
t1=(bp/100:bp/100:100*length(x)*(bp/100));
axes(handles.axes2);
plot(t1,bit,'lineWidth',2.5);grid on;
axis([ 0 bp*length(x) -.5 1.5]);
ylabel('amplitude(volt)');
xlabel(' time(sec)');
title('Encrypted message as digital signal');

% %XXXXXXXXXXXXXX Mapping for M-array QAM modulation XXXXXXXXXXXXXXXXXXXXXXXX
msg = [bi2de(fliplr(Result4)) bi2de(fliplr(Result3)) bi2de(fliplr(Result2)) bi2de(fliplr(Result1))];
p=qammod(msg,M); %constalation design for M-array QAM acording to symbol


RR=real(p)
II=imag(p)
sp=bp*2;                                     %symbol period for M-array QAM
sr=1/sp;                                                      % symbol rate
f=sr*2;
t=sp/100:sp/100:sp;
ss=length(t);
m=[];
for(k=1:1:length(RR))
    yr=RR(k)*cos(2*pi*f*t);                     % inphase or real component
    yim=II(k)*sin(2*pi*f*t);            % Quadrature or imaginary component 
    y=yr+yim;
    m=[m y];
end
tt=sp/100:sp/100:sp*length(RR);
axes(handles.axes3)
plot(tt,m);
title('waveform for M-array QAM modulation');
xlabel('time(sec)');
ylabel('amplitude(volt)');

%%
% %XXXXXXXXXXXXXXXXXXXX M-array QAM demodulation XXXXXXXXXXXXXXXXXXXXXXXXXXXX
m1=[];
m2=[];
for n=ss:ss:length(m)
  t=sp/100:sp/100:sp;
  y1=cos(2*pi*f*t);                                     % inphase component
  y2=sin(2*pi*f*t);                                  % quadrature component
  mm1=y1.*m((n-(ss-1)):n);                                    
  mm2=y2.*m((n-(ss-1)):n);                                    
  z1=trapz(t,mm1);                                             % integration
  z2=trapz(t,mm2);                                             % integration
  zz1=round(2*z1/sp);
  zz2=round(2*z2/sp);
  m1=[m1 zz1];
  m2=[m2 zz2];
end
 
% %XXXXXXXXXXXXXXXXXXX de-mapping for M-array QAM modulation XXXXXXXXXXXXXXXX
clear i;
clear j;
for (k=1:1:length(m1))  
gt(k)=m1(k)+j*m2(k);
end

ax=qamdemod(gt,M);
% axes(handles.axes5)
% stem(ax,'linewidth',2);
% title(' re-obtain symbol after M-array QAM demodulation ');
% xlabel('n(discrete time)');
% ylabel(' magnitude');
% %XX representation of receiving binary information as digital signal XXXXXX
x=[fliplr(de2bi(ax(1),4)) fliplr(de2bi(ax(2),4)) fliplr(de2bi(ax(3),4)) fliplr(de2bi(ax(4),4))];
bp=.000001;                                                    % bit period
bit=[]; 
for n=1:1:length(x)
    if x(n)==1;
       se=ones(1,100);
    else x(n)=0;
        se=zeros(1,100);
    end
     bit=[bit se];

end
t1=(bp/100:bp/100:100*length(x)*(bp/100));
axes(handles.axes4)
plot(t1,bit,'lineWidth',2.5);grid on;
axis([ 0 bp*length(x) -.5 1.5]);
ylabel('amplitude(volt)');
xlabel(' time(sec)');
title('Encrypted message as digital signal');


Result4 = ax(1); Result3 = ax(2);Result2 = ax(3);Result1 = ax(4);
Data1 = not(xor(fliplr(de2bi(Result4,4)),(spreadsig))),Data1 = data(end);
Data2 = not(xor(fliplr(de2bi(Result3,4)),(spreadsig))),Data2 = data(end-1);
Data3 = not(xor(fliplr(de2bi(Result2,4)),(spreadsig))),Data3 = data(end-2);
Data4 = not(xor(fliplr(de2bi(Result1,4)),(spreadsig))),Data4 = data(end-3);
Data = [Data4 Data3 Data2 Data1];

x=Data;
bp=.000001;                                                    % bit period
bit=[]; 
for n=1:1:length(x)
    if x(n)==1;
       se=ones(1,100);
    else x(n)=0;
        se=zeros(1,100);
    end
     bit=[bit se];

end
t1=bp/100:bp/100:100*length(x)*(bp/100);
axes(handles.axes5)
plot(t1,bit,'lineWidth',2.5);grid on;
axis([ 0 bp*length(x) -.5 1.5]);
ylabel('amplitude(volt)');
xlabel(' time(sec)');
title('Data binary stream after Demodulation');
