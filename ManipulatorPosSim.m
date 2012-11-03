% Justinas Miseikis

% Clear the workspace
clear
clc

% Default parameters
r = 2;
a1 = 3;
a2 = 4;
centre = [5 0];

beta = 0.01;
%beta = 0.8;
pointNum = 50;
%pointNum = 10;

%% Generate a circular trajectory with 10 or 50 points

% Get theta angles
theta_step = 2*pi / (pointNum-1)
theta = 0:theta_step:2*pi

ctr = 1;
% Calculate x and y coordinates for each point
for currTh=theta
    x(ctr) = 5+r*cos(currTh);
    y(ctr) = r*sin(currTh);
    ctr = ctr + 1;
end

curr_th1 = 0;
curr_th2 = 0;

ctr = 1;


%% Calculate joint angles
for i=1:pointNum
    fprintf('Approaching point number: %d, located at coords (%f %f) \n', i, x(i), y(i));
    
    c2 = (x(i)^2 +  y(i)^2 - a1^2 - a2^2) / (2 * a1 * a2);
    s2 = sqrt(1-c2^2);
    
	% Calculate thetas for both joints
    th2(i) = atan2(s2,c2);
    diff(th2(i));
	th1(i) = atan2(y(i),x(i)) - atan2(a2*sin(th2(i)), (a1 + a2 * cos(th2(i))));
    
    % Get the error
    err(1) = th1(i) - curr_th1;
	err(2) = th2(i) - curr_th2;

    % For every angle
    while abs(err(1)) > 0.01 || abs(err(2)) > 0.01
        
		% Update thetas
        curr_th1 = curr_th1 + beta * err(1);
        curr_th2 = curr_th2 + beta * err(2);
        
		% Update errors
        err(1) = th1(i) - curr_th1;
        err(2) = th2(i) - curr_th2;
        
        % Store desired thetas
        des_th1(ctr) = th1(i);
        des_th2(ctr) = th2(i);

        % Store actual thetas
        act_th1(ctr) = curr_th1;
        act_th2(ctr) = curr_th2;
        
        % Store desired position
        des_x(ctr) = a1*cos(th1(i)) + a2*cos(th1(i)+th2(i));
        des_y(ctr) = a1*sin(th1(i)) + a2*sin(th1(i)+th2(i));
        
        % Store actual position
        act_x(ctr) = a1*cos(curr_th1) + a2*cos(curr_th1+curr_th2);
        act_y(ctr) = a1*sin(curr_th1) + a2*sin(curr_th1+curr_th2);
        
        
        ctr = ctr + 1;
    end
end


%% Plot robot arm in all the positions

for i=1:pointNum
    % Plot point to reach
    plot(x(i), y(i), 'xg');
    hold on;
    
    % Plot a1
    tempX = a1*cos(th1(i));
    tempY = a1*sin(th1(i));
    
    prevX = tempX;
    prevY = tempY;

    plot([0 tempX], [0 tempY], 'r');
    hold on;
    
    % Plot a2
    tempX = prevX + a2*cos(th1(i)+th2(i));
    tempY = prevY + a2*sin(th1(i)+th2(i));
    
    
    plot([prevX tempX], [prevY tempY], 'k');
    hold on;
    
end

title('Robot configuration when desired points are reached. Red line - first part of manipulator, black line - second part');



%% Plot actual position vs desired position
figure(2);

% Plot the desired position
plot(des_x, des_y, 'rx', 'LineWidth',4);
hold on;

% Plot an actual position
plot(act_x, act_y, 'b.');
axis([3 7 -2 2])
xlabel('X position')
ylabel('Y');
title('Desired and actual X, Y positions. Red X - desired, Blue dots - actual');

figure(3)
plot(1:ctr-1, des_th1, 'r', 'LineWidth',2);
hold on;
plot(1:ctr-1, des_th2, 'b', 'LineWidth',2);
hold on;
plot(1:ctr-1, act_th1, 'k', 'LineWidth',2);
hold on;
plot(1:ctr-1, act_th2, 'm', 'LineWidth',2);

xlabel('Iterations');
ylabel('atehT');
title('Joint angles vs no. iterations. Red - desired th1, Blue - desired th2, Black - actual th1, Magenta - actual th2')
