function [V, Xerr, Xerrt] = feedbackcontrol(Xd, Xdnext, X, Kp, Ki, timestep, Xerrt)
%INPUT: Xd- current end effector reference (desired) configuration
%Xdnext:Next end effector config
%X: Actual end effector config
%Kp:proportional gain
%Ki:integral gain
%timestep: sucessive time step ie 0.01
%Since we know that when we run the trajectory genrator function it
%gentrates the results as a line we need to change it back to matrix form
%hence this step
matrix=zeros(4);
matrix(4,:)=[0 0 0 1];
matrix(1,1:3)=X(1:3);
matrix(2,1:3)=X(4:6);
matrix(3,1:3)=X(7:9);
matrix(1:3,4)=X(10:12);
X=matrix;

matrix=zeros(4);
matrix(4,:)=[0 0 0 1];
matrix(1,1:3)=Xd(1:3);
matrix(2,1:3)=Xd(4:6);
matrix(3,1:3)=Xd(7:9);
matrix(1:3,4)=Xd(10:12);
Xd=matrix;

matrix=zeros(4);
matrix(4,:)=[0 0 0 1];
matrix(1,1:3)=Xdnext(1:3);
matrix(2,1:3)=Xdnext(4:6);
matrix(3,1:3)=Xdnext(7:9);
matrix(1:3,4)=Xdnext(10:12);
Xdnext=matrix;

%find error and twist vectors
%This is already mentioned on the wiki page and also lot of study is
%reuired for this
Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd)); %error twist
Xerrt=Xerrt+Xerr*timestep; %increment to the numerical integral of the error
Vd=se3ToVec(1/timestep*MatrixLog6(TransInv(Xd)*Xdnext)); %feedforward reference twist
V=Adjoint(TransInv(X)*Xd)*Vd+Kp*Xerr+Ki*Xerrt;  %commanded end effector twist

end




