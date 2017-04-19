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
        
        accelMessage = receive(sub);
        gv = accelMessage.Data;
           
        phi = atan(-gv(2)/gv(3))
        theta = atan(-gv(1)/sqrt(gv(2)^2 + gv(3)^2)) - neatoangle;
        psi = phi
        Rotx = [cos(theta)*cos(psi); cos(psi)*sin(theta)*sin(phi)-cos(phi)*sin(psi);cos(phi)*cos(psi)*sin(theta) + sin(phi)*sin(psi)];
        Roty = [cos(theta)*sin(psi); cos(phi)*cos(psi) + sin(theta)*sin(phi)*sin(psi); cos(phi)*sin(theta)*sin(psi) - cos(psi)*sin(phi)];
        Rotz = [-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];
        Rotm = [Rotx, Roty, Rotz]
        omega = (phi/lambda)*(d/2);
        Vl = omega
        Vr = -omega
        msg.Data = [Vl, Vr];
        send(pub, msg);
        pause(lambda)
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
        heading = Rotm*heading
        tot_dis = (dis_l + dis_r) / 2
        pos = pos + (tot_dis)*((heading-pos)/norm(heading-pos));

        % plots the position of neato
        plot3(pos(1), pos(2), pos(3), 'b^');
        hold on
        
end
msg.Data = [0, 0];
send(pub, msg);