function [confignext] = nextstate(config, speeds, timestep, maxspeed)
% INPUT: config- 12-vector of robot current configuration [3 chassis, 5 arm, 4 wheels]
% speeds: wheel and arm speeds
%a small time step

r=.0475; %wheel radius (m)
l=0.47/2; %half wheel base (m)
w=0.3/2; %half wheel track (m)

%if any of the joint or wheel speeds are greater than the maxspeed in
%absolute value, set that speed to the max speed in absolute value
for i = 1:length(speeds)
    if speeds(i) > maxspeed
        speeds(i) = maxspeed;
    elseif speeds(i) < -maxspeed
        speeds(i) = -maxspeed;
    end
end

arm_angles_old=config(4:8); %old arm joint angles
wheel_angles_old=config(9:12); %old wheel angles
joint_speeds=speeds(5:9); %arm joint speeds
wheel_speeds=speeds(1:4); %wheel speeds
chassis_init=config(1:3); %initial chassis configuration (x y phi)

arm_angles_new=arm_angles_old+joint_speeds*timestep;
wheel_angles_new=wheel_angles_old+wheel_speeds*timestep;

F=r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w);
    1 1 1 1; -1 1 -1 1]; %pseudoinverse of H(0) for 4 wheel mecanum robot
Vb=F*wheel_speeds; %body twist chassis

%Vb6=[0; 0; Vb; 0]; %6-vector of body twist
%Tbbp = VecTose3(Vb6); %new chassis frame
%V = se3ToVec(Tbbp);
wbz=Vb(1);
vbx=Vb(2);
vby=Vb(3);
if NearZero(wbz)
   dqb = [0; vbx; vby];
else
   dqb = [wbz; (vbx*sin(wbz)+vby*(cos(wbz)-1))/wbz; (vby*sin(wbz)+vbx*(1-cos(wbz)))/wbz]; %change in coordinates body frame
end
phik=config(1); %chassis angle 
dq=[1 0 0; 0 cos(phik) -sin(phik); 0 sin(phik) cos(phik)]*dqb; %change in coordinates {s} frame

chassis_new=chassis_init+dq*timestep;

confignext=[chassis_new' arm_angles_new' wheel_angles_new'];




