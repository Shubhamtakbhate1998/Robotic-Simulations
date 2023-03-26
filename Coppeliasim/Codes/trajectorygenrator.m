function [Tse_mat] = trajectorygenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, Tce_so, k, tmove1, tmove2, tdown_up, topen_close, timestep)
%INPUT: Tse_init- inital end effector configuration
%Tsc_init- initial cube configuration
%Tsc_final- final cube configuration
%Tce_grasp- end effector relative to the cube when grasping
%Tce_so- end effector relative to the cube when in standoff position
%k- number of trajectory configurations per time step (e.g. 10 per 0.01
%seconds)
%OUTPUT: Tse_mat- end effector position at all time increments in a matrix
%and the gripper state

%1 move from home configuration to standoff
method=3;
N1=tmove1*k/timestep; %number of steps from rest to rest
traj = ScrewTrajectory(Tse_init, Tsc_init*Tce_so, tmove1, N1, method);
concat1=cell2mat(traj');

%

%2 move from standoff to cube but open
N2=tdown_up*k/timestep;
traj = ScrewTrajectory(Tsc_init*Tce_so, Tsc_init*Tce_grasp, tdown_up, N2, method);
concat2=cell2mat(traj');
%3 open to close gripper
N3=topen_close*k/timestep;
traj = ScrewTrajectory(Tsc_init*Tce_grasp, Tsc_init*Tce_grasp, topen_close, N3, method);
concat3=cell2mat(traj');
%4 move from cube back to standoff
N4=tdown_up*k/timestep;
traj = ScrewTrajectory(Tsc_init*Tce_grasp, Tsc_init*Tce_so, tdown_up, N4, method);
concat4=cell2mat(traj');
%5 move from standoff to standoff point 2
N5=tmove2*k/timestep;
traj = ScrewTrajectory(Tsc_init*Tce_so, Tsc_final*Tce_so, tmove2, N5, method);
concat5=cell2mat(traj');
%6 move from standoff to end box configuration
N6=tdown_up*k/timestep;
traj = ScrewTrajectory(Tsc_final*Tce_so, Tsc_final*Tce_grasp, tdown_up, N6, method);
concat6=cell2mat(traj');
%7 close to open gripper
N7=topen_close*k/timestep;
traj = ScrewTrajectory(Tsc_final*Tce_grasp, Tsc_final*Tce_grasp, topen_close, N7, method);
concat7=cell2mat(traj');
%8 move away from cube to end standoff configuration
N8=tdown_up*k/timestep;
traj = ScrewTrajectory(Tsc_final*Tce_grasp, Tsc_final*Tce_so, tdown_up, N8, method);
concat8=cell2mat(traj');

%concatenate all matrices
concat1=[concat1; concat2; concat3; concat4; concat5; concat6; concat7; concat8];
N=N1+N2+N3+N4+N5+N6+N7+N8;
concat2=zeros(N,13);

for i = 1:N
    concat2(i,1:3)=concat1(4*i-3,1:3);
    concat2(i,4:6)=concat1(4*i-2,1:3);
    concat2(i,7:9)=concat1(4*i-1,1:3);
    concat2(i,10:12)=[concat1(4*i-3,4) concat1(4*i-2,4) concat1(4*i-1,4)];
end

%13th column is closed for closed states for picking stage
for i = (N1+N2+1):(N1+N2+N3+N4+N5+N6)
    concat2(i,13)=1;
end

%{
concat2=zeros(N1,13);

for i = 1:N1
    concat2(i,1:4)=concat1(4*i-3,:);
    concat2(i,5:8)=concat1(4*i-2,:);
    concat2(i,9:12)=concat1(4*i-1,:);
end
%}

Tse_mat=concat2;

csvwrite('endeffectorpath.csv', Tse_mat)
