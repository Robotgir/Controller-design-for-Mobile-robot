clear
n=500;            %Number of time steps  
tlim=60;          %time limit

% control law parameters section
alpha0=2.7;  
alpha1=0.2;
eps1=0.01;
eps2=0.02;
k1=0.5;
k2=0.5;
k3=1;
k4=5;
rho=1;
phi=-7*pi/12;
%mobile robot dimensions and specifications
m =12;            %mass in kg
I = 0.18;         %moment of inertia of the mobile robot kg/m^2  
L=0.000065;      %inductance in H
R=0.032;         %resistance in ohm
j1=-0.1;          %ICR projection along x direction in the robot frame or local frame
a=0.3;            %should be refered to in fig:4.1(b)in meters
b=0.3;            %should be refered to in fig:4.1(b)in meters
r = 0.0765;       %radiusof wheel in meters
c = 0.154;        %should be refered to in fig:4.1(b)in meters
J =[0 -1; 1 0];
g=9.8;            %acceleration of gravity in m/sec^2

%time steps
t(1,n+1)=0;       %Initializing time
for i=1:n+1
t(1,i)=tlim/n*(i-1);
end
dt=t(1,2)-t(1,1); %size of time step  



% Normal forces on each wheel------
N1=(m*g*b)/(2*(a+b));
N4=(m*g*b)/(2*(a+b));
N2=(m*g*a)/(2*(a+b));
N3=(m*g*a)/(2*(a+b));
ul=0.15;                   %lateral friction coefficent
us=0.34;                   %longitudinal orslip friction
N=12;                      %because of the gear reduction ratio of 12:1
ki=0.018;                  %estimated motor current constant 
ke=ki/sqrt(3);             %estimated motor voltage constant 

%initialization---------------------
deltad(1,n)=0;  
deltad_dot(1,1)=-alpha1*(deltad(1,1)-eps1);  %rate of change of intermediate auxilary term deltad
ref_velB(1:2,n+1)=0;       %reference velocity
q(1:3,1)=[0 0 0]';         %the actual initial position of the robot
q_dot(1:3,1)=0;            %global frame velocity of the robot center
qr=[0 1 0]';               %the final goal position
Tq(1:2,1)=[1 1]';          %torque in Nm
Tqd_dot(1:2,1)=0;          %rate of change of torque 
velB(1:2,1)=0;             %local frame velocity of robot
ud_dot(1:2,1)=0;           %rate of change of control signal ud
l_dot(1,1)=0;              %rate of change of intermediate auxilary term l
T_dot(1:2,1:2,1)=0;        %rate of change of intermediate auxilary term T
PP_dot(1:2,1)=0;           %rate of change of intermediate auxilary term PP
Sq_dot(1:3,1:2,1)=0;       %rate of change of intermediate auxilary term Sq
q_hat(1:3,1)=0;            %pose error 
l(1,1)=0;                  %auxilary term l
    T(1:2,1:2,1)=0;            %auxilary matrix
    P(1:3,1:3,1)=0;            %state transformation matrix
    auxerror(1:3,1)=0;         %Transformed error
    z(1:2,1)=0;                %transformed error z1 z2
    PP(1:2,1)=0;              %aux term
    PCoeff(1,1)=0;           %aux term
    f(1,1)=0;               %drift term
    QCoeff(1,1)=0;           %intermediate auxilary term QCoeff
    Om1(1,1)=0;             %intermediate auxilary term Om1
    v=0;
%     zd1Sol(v)={}; 
%     zd2Sol(v)={};
    zD1(1,1)=0;
    zD2(1,1)=0; 
    zd(1:2,1)=0;
    z_hat(1:2,1)=0;
    ua(1,1)=0;
    ua(2,1)=0;%time varying feedback modulated by zd
    ud(1,1)=0;
    ud(2,1)=0;
    %friction forces ref fig 4.4 and fig 4.3 -
    fl1(1,1)=0;%longitudinal friction force component
    fl2(1,1)=0;
    fl3(1,1)=0;
    fl4(1,1)=0;
    fs1(1,1)=0;%lateral friction force component
    fs2(1,1)=0;
    fs3(1,1)=0;
    fs4(1,1)=0;
    Frx(1,1)=0;
    Fry(1,1)=0;
    Mr(1,1)=0;
%-----------------------------------
    wv(1:4,1)= 0;%wheelvelocities of the robot
    awv(1:2,1)=0;%angular velocities of left and right side wheels
    vx(1:4,1)=0;%xcomponent of the wheel velocity
    vy(1:4,1)=0;%ycomponent of the wheel velocity
%M_,C_,B_,R_-----------------------------
    M(1:3,1:3,1)=0;
    Sq(1:3,1:2,1)=0;
    Sq_dot(1:3,1:2,1)=0;
    C_(1:2,1:2,1)= 0;
    M_(1:2,1:2,1)=0;
    R_(1:2,1)=0; 
    B_(1:2,1:2,1)=0;
    B_Tq(1:2,1)=0;
%M__,C__,R__,B__ -------------------------
    M__(1:2,1:2,1)=0;
    C__(1:2,1:2,1)=0;
    R__(1:2,1)=0;
    B__(1:2,1:2,1)=0;
%solving for actual u signal, as a result actual Tq (torque) from motor---
    InvM__(1:2,1:2,1)=0;
    ac(1:2,1:2,1)=0;
    br(1:2,1)=0; 
    cb(1:2,1)=0;
    g=0;
%     u1Sol={};
%     u2Sol={}; 
    U1(1,1)=0;
    U2(1,1)=0; 
    u(1:2,1)=0;
    u_hat(1:2,1)=0;           
%error -difference between actual u signal and desired u (ud) signal 
%dynamic control law implementation---------
    ud_dot(1:2,1)=0;%rate of change of desired u signal
    Y(1:2,1:4,1)=0;                         %Y is regression matrix
    YV1(1:2,1)=0;  
%divided YV signal into  YV1 and YV2 to observe influence of each part on YV
    YV2(1:2,1)=0;
    YV(1:2,1)=0; %Y is regression matrix and V is vector of dynamical properties
    V=[m I us*m ul*m]'; 
    magnt(1,1)=0;%auxilary term 
    taua1(1:2,1)=0;
%auxilary term taua sectioned to observe its influence
    taua(1:2,1)= 0;
    %auxilary term taua
    InvB__(1:2,1:2,1)=0;
    Tqd1(1:2,1)=0;  
    %Tqd sectioned to observe its terms influence
    Tqd2(1:2,1)=0;
    Tqd3(1:2,1)=0;
    Tqd4(1:2,1)=0;
    Tqd5(1:2,1)=0;

    Tqd(1:2,1)= 0; %desired torque to be generated on the wheel

%motor level voltage control law------------
    Tqd_dot(1:2,1)=0;
    T_hat(1:2,1)=0;                         
    %error - difference between desired torque and actual torque

    Ad(1,1)=0;                        
    %desired current in each motor to produce appropriate torque Tq 

    Ad(2,1)=0;

    angvelwheelRL(1:2,1)=0; %angular velocities of wheels on left and right side of mobile robot
    Vd1(1:2,1)=0;%Vd sectioned into Vd1, Vd2, Vd3 to observe its terms influence
    Vd2(1:2,1)=0;
    Vd3(1:2,1)=0;
    Vd(1:2,1)=0;
%---------------------------------------
for i=1:n+1
    deltad(1,i)=alpha0*exp(-alpha1*t(1,i))+eps1; %auxilary term
end
deltad_dot(1,2:n+1)=diff(deltad)/dt;   
%rate of change of intermediate auxilary term deltad

figure ;%start figure window
hold on;

for j=1:n
    q_hat(1:3,j)=q(1:3,j)-qr;       %error signal 
    l(1,j)=q_hat(1,j)*sin(q(3,j))-q_hat(2,j)*cos(q(3,j)); %auxilary term l
    T(1:2,1:2,j)= [l(1,j) 1;1 0];   %auxilary term T
    if j>=2
    l_dot(1,j)=(l(1,j)-l(1,j-1))/dt;                              
    %rate of change of intermediate auxilary term l
    T_dot(1:2,1:2,j)=(T(1:2,1:2,j)-T(1:2,1:2,j-1))/dt;            
    %rate of change of intermediate auxilary term T
    end
    P(1:3,1:3,j) =[-q_hat(3,j)*cos(q(3,j))+2*sin(q(3,j)) -q_hat(3,j)*sin(q(3,j))-2*cos(q(3,j)) -2*j1;
        0 0 1;cos(q(3,j)) sin(q(3,j)) 0];                     
    %state transformation matrix for more theory ref section 4.2
    auxerror(1:3,j)=P(1:3,1:3,j)*q_hat(1:3,j);                    
    % Transformed error
    z(1:2,j)=auxerror(2:3,j);                       
    PP(1:2,j)=[cos(auxerror(2,j)),(-j1*sin(auxerror(2,j))+l(1,j));0,1]*ref_velB(1:2,j); %intermediate auxilary term PP
    if j>=2
     PP_dot(1:2,j)=(PP(1:2,j)-PP(1:2,j-1))/dt;                 
     %rate of change of intermediate auxilary term PP
    end

%kinematic control law implementation--
    PCoeff(1,j)=deltad_dot(1,j)/deltad(1,j);                      
    %intermediate auxilary term PCoeff
    f(1,j)=2*[-sin(auxerror(2,j))  (j1+auxerror(3,j)-j1*cos(auxerror(2,j)))]*ref_velB(1:2,j);%drift term 
    QCoeff(1,j)=((k1*auxerror(1,j)+f(1,j))/deltad(1,j)^2);        
    %intermediate auxilary term QCoeff
    Om1(1,j)= k2+PCoeff(1,j)+auxerror(1,j)*QCoeff(1,j);           
    %intermediate auxilary term Om1

    syms zd1(v) zd2(v) 
     zd1Sol= symvar(zd1(v)); 
     zd2Sol= symvar(zd2(v));                                         
    %solving nonstationary differential equation
    eqns = [diff(zd1,v)==PCoeff(1,j)*zd1-(QCoeff(1,j)+auxerror(1,j)*Om1(1,j))*zd2, diff(zd2,v)==PCoeff(1,j)*zd2+(QCoeff(1,j)+auxerror(1,j)*Om1(1,j))*zd1];
    cond=[zd1(0)==deltad(1,1)*cos(phi);zd2(0)==deltad(1,1)*sin(phi)];%deltad(1,1)*cos(phi)for zd1(0);  for zd2(0)deltad(1,1)*sin(phi);
    [zd1Sol(v), zd2Sol(v)] = dsolve(eqns,cond);
    g1=matlabFunction(zd1Sol(v));
    g2=matlabFunction(zd2Sol(v));
    zD1(1,j)=g1(t(1,j));
    zD2(1,j)=g2(t(1,j)); 
    zd(1:2,j)=[zD1(1,j);zD2(1,j)];%desired auxilary signal zd

    z_hat(1:2,j)=zd(1:2,j)-z(1:2,j);%error between desired aux signal obtained from a tunable oscillator and transformed error z
    ua(1,j)=-PCoeff(1,j)*zd(2,j)+Om1(1,j)*zd(1,j);
    ua(2,j)=PCoeff(1,j)*zd(1,j)+Om1(1,j)*zd(2,j);%time varying feedback modulated by zd
    ud(1,j)=ua(1,j)-k2*auxerror(2,j);
    ud(2,j)=ua(2,j)-k2*auxerror(3,j);

%--------------------------------------
    wv(1:4,j)= [1 -c;1 c;0 -j1+b;0 -j1-a]*velB(1:2,j);%wheelvelocities of the robot
    awv(1:2,j)=(1/r)*[wv(1,j);wv(2,j) ];%angular velocities of left and right side wheels

    vx(1:4,j)=[wv(1,j); wv(1,j); wv(2,j); wv(2,j)];%xcomponent of the wheel velocity
    vy(1:4,j)=[wv(4,j); wv(3,j); wv(3,j); wv(4,j)];%ycomponent of the wheel velocity


%friction forces ref fig 4.4 and fig 4.3 -
    fl1(1,j)=ul*N1*sign(vy(1,j));%longitudinal friction force component
    fl2(1,j)=ul*N2*sign(vy(2,j));
    fl3(1,j)=ul*N3*sign(vy(3,j));
    fl4(1,j)=ul*N4*sign(vy(4,j));

    fs1(1,j)=us*N1*sign(vx(1,j));%lateral friction force component
    fs2(1,j)=us*N2*sign(vx(2,j));
    fs3(1,j)=us*N3*sign(vx(3,j));
    fs4(1,j)=us*N4*sign(vx(4,j));

%resistive forces-------------------------
    Frx(1,j)=cos(q(3,j))*(fs1(1,j)+fs2(1,j)+fs3(1,j)+fs4(1,j))-sin(q(3,j))*(fl1(1,j)+fl2(1,j)+fl3(1,j)+fl4(1,j));
    Fry(1,j)=sin(q(3,j))*(fs1(1,j)+fs2(1,j)+fs3(1,j)+fs4(1,j))+cos(q(3,j))*(fl1(1,j)+fl2(1,j)+fl3(1,j)+fl4(1,j));
    Mr(1,j)=-a*(fl1(1,j)+fl4(1,j))+b*(fl2(1,j)+fl3(1,j))+c*(-(fs1(1,j)+fs2(1,j))+(fs3(1,j)+fs4(1,j)));

%M_,C_,B_,R_-----------------------------
    M(1:3,1:3,j)=[m 0 0;0 m 0;0 0 I];
    Sq(1:3,1:2,j)=S(q(3,j));
    if j>=2
        Sq_dot(1:3,1:2,j)=(Sq(1:3,1:2,j)-Sq(1:3,1:2,j-1))/dt;
    end
    C_(1:2,1:2,j)= Sq(1:3,1:2,j)'*M(1:3,1:3,j)*Sq_dot(1:3,1:2,j);
    M_(1:2,1:2,j)=[m 0;0 m*j1^2+I];
    R_(1:2,j)=[Frx(1,j);(j1*Fry(1,j))+Mr(1,j)]; 
    B_(1:2,1:2,j)=1/r*[1 1; -c c];

    B_Tq(1:2,j)=B_(1:2,1:2,j)*Tq(1:2,j);

%M__,C__,R__,B__ -------------------------
    M__(1:2,1:2,j)= T(1:2,1:2,j)'*M_(1:2,1:2,j)*T(1:2,1:2,j);
    C__(1:2,1:2,j)=T(1:2,1:2,j)'*(C_(1:2,1:2,j)*T(1:2,1:2,j)+M_(1:2,1:2,j)*T_dot(1:2,1:2,j));
    R__(1:2,j)=T(1:2,1:2,j)'*(C_(1:2,1:2,j)*PP(1:2,j)+M_(1:2,1:2,j)*PP_dot(1:2,j)+R_(1:2,j));
    B__(1:2,1:2,j)=T(1:2,1:2,j)'*B_(1:2,1:2,j);

%solving for actual u signal, as a result actual Tq (torque) from motor---
    InvM__(1:2,1:2,j)=pinv(M__(1:2,1:2,j));

    ac(1:2,1:2,j)=InvM__(1:2,1:2,j)*C__(1:2,1:2,j);
    br(1:2,j)=InvM__(1:2,1:2,j)*R__(1:2,j); 
    cb(1:2,j)=InvM__(1:2,1:2,j)*B__(1:2,1:2,j)*Tq(1:2,j);

    syms u1(g) u2(g) 
    u1Sol=symvar(u1(g));
    u2Sol=symvar(u2(g));
    eqns = [diff(u1,g)==ac(1,1,j)*u1+ac(1,2,j)*u2+cb(1,j)+br(1,j), diff(u2,g)==ac(2,1,j)*u1+ac(2,2,j)*u2+cb(2,j)+br(2,j)];
    cond=[u1(0)==ud(1,1);u2(0)==ud(2,1)];
    [u1Sol(g), u2Sol(g)] = dsolve(eqns,cond);
    r1=matlabFunction(u1Sol(g));
    r2=matlabFunction(u2Sol(g));

    if j==1
        U1(1,1)=ud(1,1);
        U2(1,1)=ud(1,1);
        u(1:2,1)=[U1(1,1);U2(1,1)];
    end
    if j>=2
        U1(1,j)=r1(t(1,j));
        U2(1,j)=r2(t(1,j)); 
        u(1:2,j)=[U1(1,j);U2(1,j)];   
    end

    u_hat(1:2,j)=ud(1:2,j)-u(1:2,j);           
%error -difference between actual u signal and desired u (ud) signal 
 
%dynamic control law implementation---------
    if j>=2 
        ud_dot(1:2,j)=(ud(1:2,j)-ud(1:2,j-1))/dt;%rate of change of desired u signal
    end
    YV1(1:2,j)=M__(1:2,1:2,j)*ud_dot(1:2,j);  
    %divided YV signal into  YV1 and YV2 to observe influence of each part on YV
    YV2(1:2,j)=C__(1:2,1:2,j)*ud(1:2,j);
    YV(1:2,j)=M__(1:2,1:2,j)*ud_dot(1:2,j)+C__(1:2,1:2,j)*ud(1:2,j)+R__(1:2,j); %Y is regression matrix and V is vector of dynamical properties
    V=[m I us*m ul*m]';
% V=[m I us*m ul*m]';
% Xregg(1:(net-1),1:4)=repelem(V',net-1,1);
% Yregg(1:(net-1),1:2)= YV(1:2,1:net-1)';
% y=Yregg(1:(net-1),2);
% X = [ Xregg(1:(net-1),1) Xregg(1:(net-1),2) Xregg(1:(net-1),3) Xregg(1:(net-1),4)];
% b = regress(y,X);
%regression matrix obtained by running above commented code,where YV there is obtained by ud from kinematic control law and some assumptions
     beta=[  -0.5745   1.8878                  
         0           0
         0           0
         0            0];
    Y(1:2,1:4,j)=beta';                         %Y is regression matrix 
    magnt(1,j)=norm(Y(1:2,1:4,j)'*u_hat(1:2,j));%auxilary term 
    taua1(1:2,j)=(Y(1:2,1:4,j)*(rho^2)*Y(1:2,1:4,j)'*u_hat(1:2,j));
    %auxilary term taua sectioned to observe its influence
    taua(1:2,j)= (Y(1:2,1:4,j)*(rho^2)*Y(1:2,1:4,j)'*u_hat(1:2,j))/(magnt(1,j)*rho+eps2);
    %auxilary term taua
    InvB__(1:2,1:2,j)=pinv(B__(1:2,1:2,j));
    Tqd1(1:2,j)=pinv(B__(1:2,1:2,j))*(auxerror(1,j)*J*z(1:2,j));  
    %Tqd sectioned to observe its terms influence
    Tqd2(1:2,j)=pinv(B__(1:2,1:2,j))*z_hat(1:2,j);
    Tqd3(1:2,j)=pinv(B__(1:2,1:2,j))*YV(1:2,j);
    Tqd4(1:2,j)=pinv(B__(1:2,1:2,j))*taua(1:2,j);
    Tqd5(1:2,j)=pinv(B__(1:2,1:2,j))*k3*u_hat(1:2,j);

    Tqd(1:2,j)= pinv(B__(1:2,1:2,j))*(auxerror(1,j)*J*z(1:2,j)+z_hat(1:2,j)+YV(1:2,j)+taua(1:2,j)+k3*u_hat(1:2,j)); %desired torque to be generated on the wheel

%motor level voltage control law-------------
    if j>=2
        Tqd_dot(1:2,j)=(Tqd(1:2,j)-Tqd(1:2,j-1))/dt;
    end
    T_hat(1:2,j)=Tqd(1:2,j)-Tq(1:2,j);                         
    %error - difference between desired torque and actual torque

    Ad(1,j)=(55.556*Tq(1,j)+2.457)/N;                        
    %desired current in each motor to produce appropriate torque Tq 

    Ad(2,j)=(55.556*Tq(2,j)+2.457)/N;

    angvelwheelRL(1:2,j)=[(velB(1,j)+velB(2,j)*c)/r ; (velB(1,j)-velB(2,j)*c)/r ]; %angular velocities of wheels on left and right side of mobile robot
    Vd1(1:2,j)=(L*(ki*N)^-1*(Tqd_dot(1:2,j)+k4*T_hat(1:2,j)+B__(1:2,1:2,j)'*u_hat(1:2,j)));%Vd sectioned into Vd1, Vd2, Vd3 to observe its terms influence
    Vd2(1:2,j)=R*Ad(1:2,j);
    Vd3(1:2,j)=ke*N*angvelwheelRL(1:2,j);
    Vd(1:2,j)=(L*(ki*N)^-1*(Tqd_dot(1:2,j)+k4*T_hat(1:2,j)+B__(1:2,1:2,j)'*u_hat(1:2,j))+1*(R*Ad(1:2,j)+ke*N*angvelwheelRL(1:2,j)));
    %desired voltage signal to make the robot track or reach the desired goal path or point 
    Tq(1:2,j+1)=(((Vd(1:2,j)-(angvelwheelRL(1:2,j)*ke*N))*ki)/R)*N; %actual torque generated by the motor on the wheel

    velB(1:2,j+1)=T(1:2,1:2,j)*ud(1:2,j)+PP(1:2,j); %obtaining actual velocity of the mobile robot in local frame
%obtating atual pose by integrating velocity using eulers backward numerical method
    q_dot(3,j+1)=velB(2,j+1);
    q(3,j+1)=q(3,j)+q_dot(3,j+1)*dt;
    Sq(1:3,1:2,j+1)=S(q(3,j+1));
    q_dot(1:3,j+1)=Sq(1:3,1:2,j+1)*velB(1:2,j+1);  
    q(1:3,j+1)=q(1:3,j)+q_dot(1:3,j+1)*dt;
%range is the distance between actual point and goal 
    net =j-1;  %auxilary term to plot graphs of recorded data
    range=sqrt((qr(1,1)-q(1,j))^2+(qr(2,1)-q(2,j))^2) %#ok<NOPTS> %for constant ref point
    angularvel=velB(2,j) %#ok<NOPTS>
% prints parameter details o the simulation plot window        
    plot(q(1,j),q(2,j),'.') 
    plot(qr(1,1),qr(2,1),'go')
    xlabel('x');
    ylabel('y');
    ylim([-2 2])
    xlim([-2 2])
    str=sprintf('n:%d  j1:%d \n alpha0:%d alpha1:%d \n eps1:%d eps2:%d \n k1:%d k2:%d k3:%d k4:%d \n phi:%s', n,j1,alpha0,alpha1,eps1,eps2,k1,k2,k3,k4,phi);
    annotation('textbox',[0.5 0.6 0.5 0.4],'String',str,'FitBoxToText','on');
%condition to stop the simulation if the robot reaches proximity of goal
    drawnow()
    if range<0.1 
      break
    end
end
f1=figure;
%plots actual position wrt  time
plot(t(1,1:net),q(1,1:net),'--b', t(1,1:net), q(2,1:net), '--g', t(1,1:net), q(3,1:net), '--r');
hold on 
xlabel('time');
ylabel('q.- ');
lgd = legend;
lgd.NumColumns = 2;
legend('q1','q2','q3');
hold off

f2=figure;
%plots evolution of signals z and zd
plot3(z(1,1:net),z(2,1:net),t(1,1:net),'b');
hold on
plot3(zd(1,1:net),zd(2,1:net),t(1,1:net),'g');
xlabel('z(1),zd(1)');
ylabel('z(2),zd(2)');
zlabel('Time');
legend('z(1),z(2)','zd(1),zd(2)');
str = {['n = ' num2str(n) ' ' '$ \alpha 0 =\mbox{ }$' num2str(alpha0) ' ' '$ \alpha 1 =\mbox{ }$' num2str(alpha1)],['k1= ' num2str(k1) ' ' 'k2= ' num2str(k2) ' ' 'k3= ' num2str(k3) ' ' 'k4= ' num2str(k4) ], ['$ \epsilon 1 =\mbox{ }$' num2str(eps1) ' ' '$ \epsilon 2 =\mbox{ }$' num2str(eps2)] , ['$ \phi =\mbox{ }$' num2str(phi)] };
dim=[0.6 0.007 1 1];
annotation('textbox',dim,'string',str,'FitBoxToText','on','Interpreter','latex');

hold off
f3=figure;
%plots evolution of the pose error-needs to converge close to zero
plot(t(1,1:net),q_hat(1,1:net), 'r', t(1,1:net), q_hat(2,1:net), 'g', t(1,1:net), q_hat(3,1:net), 'b');
xlabel('time');
ylabel('q_hat');
legend('q_hat(1)','q_hat(2)','q_hat(3)');
% ylim([-1 2])

f4=figure;
%plots initial and goal points and path traversed by the robot center
plot(q(1,1:net),q(2,1:net))
hold on
plot(qr(1,1),qr(2,1),'go')
xlabel('x');
ylabel('y');
ylim([-5 5])
xlim([-5 5])
legend('path','goal');

f5=figure;
%plots angular velosities of wheels on left and right sides of mobile robot wrt time
plot(t(1,1:net),angvelwheelRL(1,1:net), 'r', t(1,1:net), angvelwheelRL(2,1:net), 'g');
xlabel('time');
ylabel('angular velocity in rad/sec');
legend('angvelL','angvelR');

f6=figure;
%plots mobile robot velocity wrt time in local and inertial frame
plot(t(1,1:net),velB(1,1:net), 'r',t(1,1:net),q_dot(1,1:net),'g',t(1,1:net),q_dot(2,1:net),'b');
xlabel('time');
ylabel('mobile robot velocity  m/sec');
legend('local frame','Inertial frame x vel','Inertial frame y vel');

f7=figure;
%plots mobile robot angular velocity wrt time 
 plot(t(1,1:net), velB(2,1:net), 'g');
 xlabel('time');
ylabel('mobile robot angular velocity  rad/sec');

f8=figure;
%plots angle of the mobile robot and rate of change of angle wrt time
plot(t(1,1:net),q(3,1:net), 'r',t(1,1:net), velB(2,1:net), 'g'); 
legend('direction in radians','angular velocity of body localframe in rad/sec');

f9=figure;
%plots graphs of desired torque and actual torqe wrt time
plot(t(1,1:net),Tqd(1,1:net), 'r', t(1,1:net), Tqd(2,1:net), 'g',t(1,1:net),Tq(1,1:net), '-.r', t(1,1:net), Tq(2,1:net), '-.g');
xlabel('time [s]');
ylabel('desired Torque dTR,dTL & actual Torque TR,TL');
legend('dTR','dTL','TR','TL');
str = {['n = ' num2str(n) ' ' '$ \alpha 0 =\mbox{ }$' num2str(alpha0) ' ' '$ \alpha 1 =\mbox{ }$' num2str(alpha1)],['k1= ' num2str(k1) ' ' 'k2= ' num2str(k2) ' ' 'k3= ' num2str(k3) ' ' 'k4= ' num2str(k4) ], ['$ \epsilon 1 =\mbox{ }$' num2str(eps1) ' ' '$ \epsilon 2 =\mbox{ }$' num2str(eps2)] , ['$ \phi =\mbox{ }$' num2str(phi)] };
dim=[0.6 0.007 1 1];
annotation('textbox',dim,'string',str,'FitBoxToText','on','Interpreter','latex');

f10=figure;
%plots current consumption by motors on left and rights side 
plot(t(1,1:net),Ad(1,1:net), 'r', t(1,1:net), Ad(2,1:net));
xlabel('time [s]');
ylabel('current in  [A]');
legend('AR','AL');
str = {['n = ' num2str(n) ' ' '$ \alpha 0 =\mbox{ }$' num2str(alpha0) ' ' '$ \alpha 1 =\mbox{ }$' num2str(alpha1)],['k1= ' num2str(k1) ' ' 'k2= ' num2str(k2) ' ' 'k3= ' num2str(k3) ' ' 'k4= ' num2str(k4) ], ['$ \epsilon 1 =\mbox{ }$' num2str(eps1) ' ' '$ \epsilon 2 =\mbox{ }$' num2str(eps2)] , ['$ \phi =\mbox{ }$' num2str(phi)] };
dim=[0.6 0.007 1 1];
annotation('textbox',dim,'string',str,'FitBoxToText','on','Interpreter','latex');

f11=figure;
%plots voltage requirement by motors on left and rights side 
plot(t(1,1:net),Vd(1,1:net), 'r', t(1,1:net), Vd(2,1:net));
xlabel('time [s]');
ylabel('Voltage in [v]');
legend('VR','VL');
str = {['n = ' num2str(n) ' ' '$ \alpha 0 =\mbox{ }$' num2str(alpha0) ' ' '$ \alpha 1 =\mbox{ }$' num2str(alpha1)],['k1= ' num2str(k1) ' ' 'k2= ' num2str(k2) ' ' 'k3= ' num2str(k3) ' ' 'k4= ' num2str(k4) ], ['$ \epsilon 1 =\mbox{ }$' num2str(eps1) ' ' '$ \epsilon 2 =\mbox{ }$' num2str(eps2)] , ['$ \phi =\mbox{ }$' num2str(phi)] };
dim=[0.6 0.007 1 1];
annotation('textbox',dim,'string',str,'FitBoxToText','on','Interpreter','latex');

%function for kinematic transformation
function s=S(x)
j1=-0.1;
s=[cos(x) j1*sin(x);sin(x) -j1*cos(x);0 1];
end