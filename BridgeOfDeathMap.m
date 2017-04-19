clf
pub = rospublisher('/raw_vel');
sub = rossubscriber('/encoders');

strtmsg = rosmessage(pub);
stopmsg = rosmessage(pub);

d = 0.25
% will cause neato to stop
stopmsg.Data = [0,0];

% using Paul's code from "symbolicFunExample" as base for this part
syms u;
% set the contant that will slow down the neato's speed
k = 0.2;
% makes it so that matlab knows u is a real number
assume(u,'real');

% contants for the equations
l = 0.4;
a = 0.4;
% symbolic equations for the path of neato
% R = sym([0.5*cos(u*k); (3/4)*sin(u*k); 0]); %part 8
card = sym([-2.*a.*((l-cos(u.*k)).*cos(u.*k)+(1-l));
2.*a.*(l-cos(u.*k)).*sin(u.*k); 0]); % part 9

% tangent vector
T = diff(card);
% computing T hat - using simplify to keep the symbolic equations from
% getting too crazy
That = simplify(T ./ norm(T));
% caclulate N hat
Nhat = simplify(diff(That)./norm(diff(That)));
% calculate B hat
Bhat = simplify(cross(That, Nhat));

% calculating the angular velocity
w = cross(That, diff(That));
% calculating the velocity of neato
V = norm(diff(card));

% breaking the velocity into its right and left wheel components
v_r = V + w(3)*d/2;
v_l = V - w(3)*d/2;
% turning the symbolic equations back into vectors/something matlab can use
% to compute what we need
V = matlabFunction([v_l,v_r]);
Px = matlabFunction(card(1));
Py = matlabFunction(card(2));

% starts the timer
time = tic;

% retrievs velocity
encoders = receive(sub);
old_data = encoders.Data;

% sets variables that will be used in loop
x = 0;
y = 0;
theta = 0;
hold on
while 1
    t = toc(time);
   
    % recieves velocity again, will be same as the data outside the loop
    % for the first run but will change every other iteration
    encoders = receive(sub);
    new_data = encoders.Data;
    % calculates distance traveled by each wheel
    dis_l = old_data(1) - new_data(1)
    dis_r = old_data(2) - new_data(2)
    % average distance
    tot_dis = (dis_l + dis_r) / 2;
    % Calculating the ICC
    R = (d/2)*((dis_l + dis_r)/(dis_r - dis_l))
    ICC = [Px(t) - R*cos(theta); Py(t)+R*sin(theta)]
        delta_theta = (dis_r - dis_l) / 0.25
    theta = theta + delta_theta;

    %our rotation matrix
    Rotmat = [cos(delta_theta) -sin(delta_theta); sin(delta_theta) cos(delta_theta)]
    new_positions = Rotmat*([Px(t);Py(t)] - ICC) + ICC
    %Makes sure the ICC exists, if it doesnt then it drives straight. 
    if isnan(R)
      new_positions = dis_l*[cos(theta); sin(theta)] + [Px(t); Py(t)]
    end
    % path of the wheels 
    x = new_positions(1)
    y = new_positions(2)
    
    % sends velocity
    strtmsg.Data = V(t);
    send(pub, strtmsg);
    
    % makes neato stop after 20 seconds
    if t > 20
        send(pub, stopmsg)
        break
    end
    %sets the old data to equal new data so that the velocity changes
    %correctly for each iteration
    old_data = new_data;
    % plots the position of neato
    plot(x, y, 'r*');
   
end
    test = linspace(0,20);
    plot(Px(test), Py(test), 'b')