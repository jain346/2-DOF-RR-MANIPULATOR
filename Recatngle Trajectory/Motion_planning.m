clc; clear all; clf;

a1 = 1.5;
a2 = 2;
n = 15;     % no of steps
ff = [-1.5;3];
df = [0;3];
cf = [0 ;2];

pf = [-1.5;2]; %final position of end-effector
pi = [0;2];  % initial position of end-effector
deltap = pf - pi;
deltaf = ff -pf;
deltad = df - ff;
deltac = cf - df;
deltat = linspace(0,1,n); 


pxarr = zeros(n,1); pyarr = zeros(n,1);
fxarr = zeros(n,1); fyarr = zeros(n,1);
dxarr = zeros(n,1); dyarr = zeros(n,1);
cxarr = zeros(n,1); cyarr = zeros(n,1);

axarr = zeros(n,1);ayarr = zeros(n,1);
theta1arr = zeros(n,1); theta2arr = zeros(n,1);
omega1arr = zeros(n,1); omega2arr = zeros(n,1);

%assigning first element value  to intitialposition



%trajectory planner

for i=1:n 
    t = deltat(i);
    p = pi + t* (deltap);
    f = pf + t*(deltaf);
    d = ff + t*(deltad);
    c = df + t*(deltac);
    cxarr(i) = c(1);
    cyarr(i) = c(2);
    dxarr(i) = d(1);
    dyarr(i) = d(2);
    fxarr(i) = f(1);
    fyarr(i) = f(2);
    pxarr(i) = p(1);
    pyarr(i) = p(2);

end

txarr = cat(1,pxarr,fxarr,dxarr, cxarr);  % used for making single array 
tyarr = cat(1,pyarr,fyarr,dyarr,cyarr);    % used for making single array


%call function to plot initial pose
[theta1_1,theta2_1,theta1_2,theta2_2] = R2Ikin(a1, a2, txarr(1),tyarr(1));

theta1prev = theta1_1;
theta2prev = theta2_1;

theta1arr(1) = theta1prev;
theta2arr(1) = theta2prev;
omega1arr(1) = 0;
omega2arr(1) = 0;

t= 4*n;

% joint angles array using Ikin
for i=2:t 
    [theta1_1,theta2_1,theta1_2,theta2_2] = R2Ikin(a1, a2, txarr(i),tyarr(i));
    
    
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




Xaxis1Coord = [-4,4]; Yaxis1Coord = [0,0];
Yaxis2Coord = [-4,4]; Xaxis2Coord = [0,0];



for i=1:t
    ax = axarr(i);ay = ayarr(i);
    bx = txarr(i); by = tyarr(i);
    
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
