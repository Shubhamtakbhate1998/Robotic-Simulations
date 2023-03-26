clc
clear
%%
% This is part one of the final project
%This part contains intial info about end effoctor trajectory
disp('Run Robotics Capstorne project')
%generate trajectory using trajectorygenerator
%first we write configuration conditions for the function
%Change this accordingly for cube configuration or position
Tse_init=[0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Tsc_init=[1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
Tsc_final=[0 1 0 1; -1 0 0 -1.5; 0 0 1 0.025; 0 0 0 1];


%grasp and standoff configurations, gripper moves forward in x direction
%.017 for better grip, it hovers over at 20cm at standoff
%since the the standoff has to be above.
Tce_grasp=[0 0 1 0.017; 0 1 0 0; -1 0 0 0; 0 0 0 1];
Tce_so=[0 0 1 0.017; 0 1 0 0; -1 0 0 0.20; 0 0 0 1];
%time intervals for each movement are set to be reasonable. Some time
%intervals equal others so there are only 4 total
timestep=0.01;
tmove1=6;
tdown_up=3;
topen_close=1;
tmove2=6;
ttotal=tmove1+tmove2+4*tdown_up+2*topen_close;
k=10; %number of reference trajectories per time step
%%
%%Part 2
%base information
r=(0.0475); %wheel radius
w=(0.3/2); %half vehicle track 
l=(0.47/2); %half wheelbase
F=r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
F6=[0 0 0 0; 0 0 0 0; F; 0 0 0 0];
Blist = [[0; 0; 1; 0; 0.033; 0], [0; -1; 0; -0.5076; 0; 0], ...
         [0; -1; 0; -0.3526; 0; 0], [0; -1; 0; -0.2176; 0; 0], ...
         [0; 0; 1; 0; 0; 0]];%screw axes
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M0e=[1 0 0 0.033; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];

robot_init_config=[0 -0.5 -0.5, 0 0 0 0 0, 0 0 0 0, 0]; %inital configuration of robot joints, chassis, and gripper
%%
%initialize end effector configuration
thetalist=zeros(5,1);
phi = robot_init_config(1,1);
x = robot_init_config(1,2);
y = robot_init_config(1,3);
T0e = FKinBody(M0e, Blist, thetalist);
Tsb = [cos(phi) -sin(phi) 0    x;
       sin(phi) cos(phi)  0    y;
       0        0         1    0.0963;
       0        0         0    1];
tmp = Tsb*Tb0*T0e; 

disp('Refrence trajectory function running')
%generate trajectory
[Tse_mat] = trajectorygenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_so, k, tmove1, tmove2, tdown_up, topen_close, timestep);
csvwrite('refrencetrajectorynewtask.csv', Tse_mat)
N=size(Tse_mat, 1);

Xlist=zeros(N, 12); %initialize youbot actual configuration
maxspeed=20; %arbitrary high angular speed
Kp=1*eye(6); %control gains, Proportional
Ki=0*eye(6); %integral gain
Xlist(1,:)=[tmp(1,1:3),tmp(2,1:3),tmp(3,1:3),tmp(1:3,4)']; %first configuration for end effector of youbot

%initialize joint & wheel angle list, chassis list, gripper state list and
%separate list of Errors
config=zeros(N,13);
config(1,:)=robot_init_config;
Xerr=[];
Xerrt=0;
config_list=robot_init_config;
%%
%find largest absolute multiple of 2*pi in the joint/wheel configurations
M=20; 

disp('Generating Animation csv file for copasim')
for i = 1:(N-1)
    %run this loop to eliminate any rotational positions that exceed in absolute value of 2*pi
    for j = 1:12
        for jj = M:-1:1
            if (j==2 || j==3)
                config(i,j)=config(i,j);            
            elseif config(i,j) > jj*2*pi
                config(i,j)=config(i,j)-jj*2*pi;
            elseif config(i,j) < -jj*2*pi
                config(i,j)=config(i,j)+jj*2*pi;
            end
        end
    end
    %construct endeffector configuration to plug into "nextstate"
    if i == 1
        Xlist(i,:)=Xlist(i,:);
    else
        phi=config(i,1); %chassis head angle
        Tsb=[cos(phi) -sin(phi) 0 config(i,2)
            sin(phi) cos(phi) 0 config(i,3)
            0 0 1 0.0963;
            0 0 0 1];
        thetalist=config(i,4:8)';
        T0e = FKinBody(M0e, Blist, thetalist);
        Tse=Tsb*Tb0*T0e;
        Xlist(i,1:3)=Tse(1,1:3);
        Xlist(i,4:6)=Tse(2,1:3);
        Xlist(i,7:9)=Tse(3,1:3);
        Xlist(i,10:12)=Tse(1:3,4);
    end
    %%
    %Feedback controller running
    [V, Xerrtemp, Xerrt] = feedbackcontrol(Tse_mat(i,1:12), Tse_mat(i+1,1:12), Xlist(i,:) , Kp, Ki, timestep/k, Xerrt); 
    Jarm = JacobianBody(Blist, thetalist); %arm jacobian
    AdT=Adjoint(TransInv(T0e)*TransInv(Tb0));
    Jbase=AdT*F6; %base jacobian
    Je=[Jbase, Jarm]; %concatenate for end effector jacobian
    speeds=pinv(Je, 1e-4)*V; %wheel and arm speeds
    next_config=nextstate(config(i,1:12)', speeds, timestep/k, maxspeed); %make new configuration
    next_config(NearZero(next_config))=0;
    config(i+1,1:12) = next_config; %assign new configuration to config list
    config(i+1,13)=Tse_mat(i+1,13); %give new configuration gripper state the gripper state for the reference trajectory
    %assign configuration to config list accordingly to "k"
    %Add Xerr to Xerr list if k configurations have been passed
    if rem(i,k) == 0
        config_list=[config_list; config(i+1,:)];
        Xerr=[Xerr, Xerrtemp];
    end
end

csvwrite('new.csv', config_list)
csvwrite('Xerrnew.csv', Xerr)

%

disp('Done with file genration')
%plot Xerr
t=linspace(0, ttotal, N/k-1) ;
figure
plot(t, Xerr(1,:))
hold on
plot(t, Xerr(2,:))
plot(t, Xerr(3,:))
plot(t, Xerr(4,:))
plot(t, Xerr(5,:))
plot(t, Xerr(6,:))
xlabel('time (s)')
ylabel('Error')
title('Xerr vs. Time')
legend('1','2', '3', '4', '5', '6', 'location', 'northeast')
axis([0 ttotal -4 4])

disp('Done with code')

%}
