clf
sub = rossubscriber('/accel');
sub1 = rossubscriber('/encoders');
%Define the functions 
d = 0.83;
lambda = 1/4;
neatoangle = (15*pi/180);
%Making our ros objects 
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
%accelMessage = receive(sub);
%time object
run = 1;
phi = pi;
psi = 0;
theta = pi;
heading = [1;0;0];
pos = [0;0;0];
while run == 1
%define t as our current time
        %Our first step rotates our robot about its center. 
        %Receives accelerometer data
        accelMessage = receive(sub);
        gv = accelMessage.Data;
        %calculates angles based on accel data
        phi = atan(-gv(2)/gv(3))
        theta = atan(-gv(1)/sqrt(gv(2)^2 + gv(3)^2)) - neatoangle;
        %we are just using our roll angle to estimate our yaw angle
        psi = phi
        %rotation matrix
        Rotx = [cos(theta)*cos(psi); cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi);cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi)];
        Roty = [cos(theta)*sin(psi); cos(phi)*cos(psi) + sin(theta)*sin(phi)*sin(psi); cos(phi)*sin(theta)*sin(psi) - cos(psi)*sin(phi)];
        Rotz = [-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];
        Rotm = [Rotx, Roty, Rotz]
        %calculating angular velocity to tell it how much to turn
        omega = (phi/lambda)*(d/2);
        Vl = omega
        Vr = -omega
        msg.Data = [Vl, Vr];
        send(pub, msg);
        pause(lambda)
        
        %Our second step moves the robot a fixed distance 
        encoders = receive(sub1);
        data1 = encoders.Data;
        Vl = 0.09;
        Vr = 0.09;
    
        % average distance

        msg.Data = [Vl, Vr];
        send(pub, msg);
        pause(lambda/4)
        encoders = receive(sub1);
        data2 = encoders.Data;
        dis_l = data1(1) -data2(1);
        dis_r = data1(2) - data2(2);
        %Plots a point that is the set distance along the neato's heading
        heading = Rotm*heading
        tot_dis = (dis_l + dis_r) / 2
        pos = pos + (tot_dis)*((heading-pos)/norm(heading-pos));

        % plots the position of neato
        plot3(pos(1), pos(2), pos(3), 'b^');
        hold on
        
end
msg.Data = [0, 0];
send(pub, msg);