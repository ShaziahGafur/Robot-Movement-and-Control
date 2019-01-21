load('robpos.mat')
time=robpos(:,1);  
x=robpos(:,2);
y=robpos(:,3);

%______________________________________________________________________
% (A) Piecewise Linear Approximation
figure(1)

% piecewise linear approximation of  position
% plot of x posiiton (x-axis)and y posiiton (y-axis) 

plot(x,y)
xlabel('x Position')
ylabel('y Position')
title('Plot of x vs y coordinates of the Robot')

figure(2)

t1 = time(1:60);

v = [];

for  i = 1:length(time)-1
    v(i)= sqrt(power((x(i+1)-x(i)),2) +  power((y(i+1)-y(i)),2)); % Calculates approximated speed
    v_theta(i) = atan((y(i+1)-y(i))/(x(i+1)-x(i))); % Calculates the direction (theta) of travel. This, in conjunction with speed, determines the velocity 
end

% Note: vector v includes speed for t = 0 to t = 59
% Where v(1) @ t = 0

plot(t1,v)
xlabel('Time Elapsed')
ylabel('Speed')
title('Time Elapsed vs Speed of the Robot (Original Plot)')


figure(3)

t2=time(2:60);

a = [];
for  i = 2:1:length(time)-1
    a(i-1)= v(i) - v(i-1); % Calculates approximated acceleration
end 

% Note:  vector a includes acceleration for t = 1 to t = 59
% Where a(1) @ t = 1


plot(t2,a)
xlabel('Time Elapsed')
ylabel('Acceleration')
title('Time Elapsed vs Acceleration of the Robot (Original Plot)')

%______________________________________________________________________
% (B) Curve-fitted Models
% Deciding the best plot for time VS x-position
%Degree 8 polynomial is chosen as the best curve for time vs x position

figure(4)
hold all
time_a =linspace(0,60,1001);

plot (time, x,'.-')
 coefs=polyfit(time, x,8);
 x_fit=polyval(coefs, time_a);
 plot(time_a,x_fit)

xlabel('Time Elapsed')
ylabel('x Position')
title('Time Elapsed vs x Position of the Robot (Curve-fitted model)')
legend('Original Graph','Degree 8 Polynomial');

hold off

% Deciding the best plot for time VS y-position
%Degree 8 polynomial is chosen as the best curve

figure(5)
hold all
time_a =linspace(0,60,1001);

plot (time, y,'.-')
 coefs=polyfit(time, y,8);
 y_fit=polyval(coefs, time_a);
 plot(time_a,y_fit)
 
xlabel('Time Elapsed')
ylabel('y Position')
title('Time Elapsed vs y Position of the Robot (Curve-fitted model)')
legend('Original Graph','Degree 8 Polynomial');

hold off

%______________________________________________________________
% Curve-fitted model of speed 

figure(6)

incrm = 60/1000; %1000 points used in 60 seconds, thus the increment length is 60/1000


for i=1:length(time_a)-1
    vx_delta(i) = (x_fit(i+1) - x_fit(i))/incrm; %change of speed in x direction
    vy_delta(i) = (y_fit(i+1) - y_fit(i))/incrm; %change of speed in y direction
    vd_theta(i) = atan((vx_delta(i))/(vy_delta(i))); %direction of theta (radians) of speed (for velocity)
    v_delta(i) =  sqrt ( (vx_delta(i)).^2 + (vy_delta(i)).^2 ) ; %magnitude of speed
    
end

plot(time_a(1:1000), v_delta);
xlabel('Time Elapsed')
ylabel('Speed')
title('Time Elapsed vs Speed of the Robot (Curve-fitted model)')

%______________________________________________________________
% Curve-fitted model of acceleration 

figure(7)

for i=1:length(v_delta)-1
    ax_delta(i) = (vx_delta(i+1) - vx_delta(i))/incrm ;%change of acceleration in x direction
    ay_delta(i) = (vy_delta(i+1) - vy_delta(i))/incrm; %change of acceleration in y direction
    ad_theta(i) = atan((ax_delta(i))/(ay_delta(i))); %direction of theta (radians) of acceleration
    a_delta(i) =  sqrt ( (ax_delta(i)).^2 + (ay_delta(i)).^2 ); %magnitude of total acceleration
end

plot(time_a(1:999), a_delta)
xlabel('Time Elapsed')
ylabel('Acceleration')
title('Time Elapsed vs Acceleration of the Robot (Curve-fitted model)')


%______________________________________________________________________
% (C) Path Control

%new data points to be generated

k = 100; % number of desired points on circular path (must be divisble by 4)

x_circle= zeros(1,k+1); %x-coordinates of circular path
y_circle= zeros(1,k+1); %y-coordinates of circular path

%x_circle(1) and y_circle(1) sync @ t=0

% for loop assigns x coordinates and y coordinates of circle 
%(equation: x^2+(y-1.2_^2=1.2^2) into x_circle and y_circle

for i=1:(k/4)
    theta = (3*pi/2) + (2*pi*(i-1)/k);
    x_circle(i)=sqrt((1.2^2)/(1+(tan(theta))^2));
    
    y_circle(i)= 1.2+(-1)*sqrt((1.2^2)-((x_circle(i))^2));
    x_circle(i+(k/2))=(-1)*x_circle(i);
    y_circle(i+(k/2))=2.4-y_circle(i);
end

for i=(k/4)+1:(k/2)+1
    theta= (3*pi/2) + (2*pi*(i-1)/k);
    x_circle(i)=sqrt((1.2^2)/(1+(tan(theta))^2));
    
    y_circle(i)= 1.2+sqrt((1.2^2)-((x_circle(i))^2));
    x_circle(i+(k/2))=(-1)*x_circle(i);
    y_circle(i+(k/2))=2.4-y_circle(i);
end


% Path Control: Determining velocity 

figure(8)
time_a= linspace(0,60,101);
incrm = 60/100;

for i=1:k
    vx_circle(i) = (x_circle(i+1) - x_circle(i))/incrm; %change of speed in x direction
    vy_circle(i) = (y_fit(i+1) - y_fit(i))/incrm; %change of speed in y direction
    vc_theta(i) = atan((vx_circle(i))/(vy_circle(i))); %direction of theta (radians) of speed (for velocity)
    v_circle(i) =  sqrt ((vx_circle(i)).^2 + (vy_circle(i)).^2 ); %magnitude of speed
    
end

plot(time_a(1:100), v_circle);
xlabel('Time Elapsed')
ylabel('Speed')
title('Time Elapsed vs Speed of the Robot (During Circular Path)')

% Path Control: Determining acceleration 

figure(9)

for i=1:k-1
    ax_circle(i) = (vx_circle(i+1) - vx_circle(i))/incrm ;%change of acceleration in x direction
    ay_circle(i) = (vy_circle(i+1) - vy_circle(i))/incrm; %change of acceleration in y direction
    ac_theta(i) = atan((ax_circle(i))/(ay_circle(i))); %direction of theta (radians) of acceleration
    a_circle(i) =  sqrt ( (ax_circle(i)).^2 + (ay_circle(i)).^2 ); %magnitude of total acceleration
end

plot(time_a(1:99), a_circle)
xlabel('Time Elapsed')
ylabel('Acceleration')
title('Time Elapsed vs Acceleration of the Robot (During Circular Path)')

