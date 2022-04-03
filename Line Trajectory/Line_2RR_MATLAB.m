clc; clear all; clf;

a1 = 1.5;
a2 = 2;
n = 25;     % no of steps
pf = [2;2]; %final position of end-effector
pi = [-1.5;2];  % initial position of end-effector
deltap = pf - pi;
deltat = linspace(0,1,n); 


pxarr = zeros(n,1); pyarr = zeros(n,1);
axarr = zeros(n,1);ayarr = zeros(n,1);
theta1arr = zeros(n,1); theta2arr = zeros(n,1);
omega1arr = zeros(n,1); omega2arr = zeros(n,1);
pxarr(1) = pi(1);
pyarr(1) = pi(2);

%trajectory planner

for i=2:n 
    t = deltat(i);
    p = pi + t* deltap;
    pxarr(i) = p(1);
    pyarr(i) = p(2);

end

%call function to plot initial pose
[theta1_1,theta2_1,theta1_2,theta2_2] = R2Ikin(a1, a2, pxarr(1),pyarr(1));

theta1prev = theta1_1;
theta2prev = theta2_1;

theta1arr(1) = theta1prev;
theta2arr(1) = theta2prev;
omega1arr(1) = 0;
omega2arr(1) = 0;



% joint angles array using Ikin
for i=2:n
    [theta1_1,theta2_1,theta1_2,theta2_2] = R2Ikin(a1, a2, pxarr(i),pyarr(i));
    
    
    if ((theta1_1 - theta1prev)^2 + (theta2_1 - theta2prev)^2) < ((theta1_2 - theta1prev)^2 + (theta2_2 - theta2prev)^2)
        theta1 = theta1_1;
        theta2 = theta2_1;

    else
        theta1 = theta1_2;
        theta2 = theta2_2;

    end  
    theta1arr(i) = theta1;
    theta2arr(i)= theta2;
    [omega1,omega2] = AngularCalc(theta1arr(i) , theta2arr(i), theta1prev, theta2prev);
    omega1arr(i) = omega1;
    omega2arr(i) = omega2;
    axarr(i) = a1*cos(theta1);
    ayarr(i) = a1*sin(theta1);
    theta1prev = theta1;
    theta2prev = theta2;
    

end

Xaxis1Coord = [-3,3]; Yaxis1Coord = [0,0];
Yaxis2Coord = [-3,3]; Xaxis2Coord = [0,0];



for i=1:n
    ax = axarr(i);ay = ayarr(i);
    bx = pxarr(i); by = pyarr(i);
    traceXA(i) = bx;
    traceYA(i) = by;
    link1XCoord = [0, ax];
    link1YCoord = [0, ay];

    link2XCoord = [ax , bx];
    link2YCoord = [ay , by];

    plot(Xaxis1Coord , Yaxis1Coord,'r',Xaxis2Coord,Yaxis2Coord,'r')
    hold on;

    plot(link1XCoord, link1YCoord,'b');
    plot(link2XCoord,link2YCoord,'g');
    hold on;
    plot(traceXA,traceYA,'k');
    hold off;
    pause(0.05);




end
