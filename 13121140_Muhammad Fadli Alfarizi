clc
clear

%=================%
%Identity%
%=================%
% Name   : Muhammad Fadli Alfarizi
% NIM    : 13121140

%=================%
%Input%
%=================%
XYZ = 140; % NIM

%=================%
%System Parameters%
%=================%
strXYZ = string(XYZ);
strXYZ = char(strXYZ);
X = strXYZ(1);
Y = strXYZ(2);
Z = strXYZ(3);

mod1 = mod(XYZ,3);
if mod1 == 0
    H = 22.5*(10^-2); % in meter
elseif mod1 == 1
    H = 25*(10^-2); % in meter
else
    H = 25*(10^-2); % in meter
end

mod2 = mod(XYZ,2);
% angular velocity
if mod2 == 0
    w2 = 4; % in rad/s
else
    w2 = 2; % in rad/s
end

L2=(30+str2num(Z))*(10^-2) ; % in meter
L3=(136+str2num(X))*(10^-2); % in meter
L5=(57 + str2num(Y)/4)*(10^-2); % in meter

% point O
uPjO1=[0;0];
uPjO2=[-L2/2;0];

% point B
uPjB2=[]; % input field
uPjB3=[]; % input field

% point C
uPjC3=[]; % input field
uPjC5=[]; % input field

% point D
uPjD3=[]; % input field
uPjD4=[]; % input field

% point F
uPjF5=[]; % input field
uPjF6=[]; % input field

%==============%   
%Initial Values%
%==============%`
q=zeros(18,1);
q6init=pi/6;
%==============%

t=0:0.001:2;

NumData=size(t,2);
q_alltime=zeros(18,NumData);
v_alltime=zeros(18,NumData);
a_alltime=zeros(18,NumData);

for j=1:NumData
 
    epsilon=1e-12;
    delta_q_norm=1;
    i_newton=1;

    while abs(delta_q_norm)>epsilon

    %Transformation Matrix & Its derivative%
    A1=[cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))]; % link 1
    dA1t1=[-sin(q(3)) -cos(q(3)); cos(q(3)) -sin(q(3))]; % link 1
    A2=[]; % link 2, input field
    dA2t2=[]; % link 2, input field
    A3=[]; % link 3, input field
    dA3t3=[]; % link 3, input field
    A4=[]; % link 4, input field
    dA4t4=[]; % link 4, input field
    A5=[]; % link 5, input field
    dA5t5=[]; % link 5, input field
    A6=[]; % link 6, input field
    dA6t6=[]; % link 6, input field

    %Constraint Equation%
    C=zeros(18,1);
    C45=[q(1);q(2)]+A1*uPjO1-[q(4);q(5)]-A2*uPjO2; % point O, Constrain Equation Matrix row 4-5,
    C67=[q(4);q(5)]+A2*uPjB2-[q(7);q(8)]-A3*uPjB3; % point B, Constrain Equation Matrix row 6-7
    C89= % point D, Constrain Equation Matrix row 8-9, input field
    C1011= % point C, Constrain Equation Matrix row 10-11, input field
    C1213= % point F, Constrain Equation Matrix row 12-13, input field
    C1415= % slider 4, Constrain Equation Matrix row 14-15, input field
    C1617= %slider 6, Constrain Equation Matrix row 16-17, input field
    C12= % link 2, Constrain Equation Matrix row 18, input field
    
    %Constrain Equation Matrix%
    C=[q(1);q(2);q(3);C45;C67;C89;C1011;C1213;C1415;C1617;C12];
    
    C_norm=sqrt(sum(C(1:18).^2))/18;

    %Constrain Jacobian Matrix%
    unit22=[1 0; 0 1];
    
    %Zeros 2x2 Matrix
    zeros22=[0 0; 0 0];

    %Ground Constraint%
    C0q=[]; % Constrain Jacobian Matrix row 1-3, input field

    %Pin Join%
    CPjO=[zeros22 dA1t1*uPjO1 -unit22 -dA2t2*uPjO2 zeros(2,12)]; % Constrain Jacobian Matrix row 4-5
    CPjB=[]; % Constrain Jacobian Matrix row 6-7, input field
    CPjD=[]; % Constrain Jacobian Matrix row 8-9, input field
    CPjC=[]; % Constrain Jacobian Matrix row 10-11, input field
    CPjF=[]; % Constrain Jacobian Matrix row 12-13, input field

    %Slider%
    Cslider4=[zeros(1,10) 1 zeros(1,7); zeros(1,11) 1 zeros(1,6)]; % Constrain Jacobian Matrix row 14-15
    Cslider6=[]; % % Constrain Jacobian Matrix row 16-17, input field

    %Driving Constraint, Link 2% 
    Cdrive=[]; % Constrain Jacobian Matrix row 18, input field
 
    % Final Constrain Jacobian Matrix
    Cq=[C0q;CPjO;CPjB;CPjD;CPjC;CPjF;Cslider4;Cslider6;Cdrive];
   
    delta_q=inv(Cq)*(-C);
    delta_q_norm=sqrt(sum(delta_q(1:12).^2))/12;
    
    q=q+delta_q;

    i_newton=i_newton+1;

      if  i_newton > 30 %limit newton rhapson iteration in case non-convergence%
        break
      end  
    end

    Ct=[zeros(17,1);-w2];
    velocity=inv(Cq)*(-Ct);
    
    QdPjO=A1*uPjO1*(velocity(3,1))^2-A2*uPjO2*(velocity(6,1))^2;
    QdPjB=; % input field
    QdPjD=; % input field
    QdPjC=; % input field
    QdPjF=; % input field
    Qd=[zeros(3,1);QdPjO;QdPjB;QdPjD;QdPjC;QdPjF;zeros(5,1)];

    accel=inv(Cq)*Qd;

    q_alltime(:,j)=q;
    v_alltime(:,j)=velocity;
    a_alltime(:,j)=accel;

end

C_norm;
delta_q_norm;
i_newton;


figure(1);
plot(t,q_alltime(16,:));
grid on;
xlabel('Time [s]');
ylabel('Slider Position [m]');
title('Position of Slider 6 vs Time');

figure(2);
plot(t,v_alltime(16,:));
grid on;
xlabel('Time [s]');
ylabel('Slider Velocity [m/s]');
title('Velocity of Slider 6 vs Time');

figure(3);
plot(t,a_alltime(16,:));
grid on;
xlabel('Time [s]');
ylabel('Slider Velocity [m/s]');
title('Acceleration of Slider 6 vs Time');

figure(4);
plot(t,v_alltime(15,:));
grid on;
xlabel('Time [s]');
ylabel('Angular Velocity [m/s]');
title('Angular Velocity of Link 5 vs Time');

figure(5);
plot(t,a_alltime(15,:));
grid on;
xlabel('Time [s]');
ylabel('Angular Acceleration [m/s]');
title('Angular Acceleration of Link 5 vs Time');
