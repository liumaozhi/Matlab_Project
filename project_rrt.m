% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 3 that denotes question
%                       number to run.
function project_rrt

%     close all;
    
    
    
    % set up p560ot and initial joint configuration
	p560=p560otCreate();
	qStart = [0 pi/2 pi 0 0 0];
	xGoal = [0.45;0.0;-0.5];
    sphereCenter = [0.5;0;0];
    sphereCenter2 = [0.25;-0.3;0.25];
    sphereCenter3 = [0;0.3;0];
    sphereRadius = 0.2;
    
    % plot p560ot and sphere
    p560.plot(qStart);
    hold on;	
    drawSphere(sphereCenter,sphereRadius);
    hold on;	
    drawSphere(sphereCenter2,sphereRadius);
    hold on;	
    drawSphere(sphereCenter3,sphereRadius);
    
    qMilestones = milestones_fd(p560,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius,qStart,xGoal);

    qMilestonesSmoothed = smooth(p560,qMilestones,sphereCenter,sphereCenter2,sphereCenter3,sphereRadius)    
    % interpolate and plot direct traj from start to goal
    
    qTraj = interpMilestones(qMilestonesSmoothed);
    p560.plot(qTraj);
   
    
    

end

function traj = interpMilestones(qMilestones)

    d = 0.05;
%     traj = qMilestones(1,:);
    traj = [];
    for i=2:size(qMilestones,1)
        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function qPath = getPath(tree)

    m = 10;
    idx = size(tree,1);
    path = tree(end,1:end-1);
    
    while(idx ~= 1)
        
        curr = tree(idx,1:end-1);
        idx = tree(idx,end);
        next = tree(idx,1:end-1);
        path = [path;[linspace(curr(1),next(1),m)' linspace(curr(2),next(2),m)' linspace(curr(3),next(3),m)' linspace(curr(4),next(4),m)']];
        
    end
    qPath = path(end:-1:1,:);
    
end

function p560=p560otCreate()
%MDL_PUMA560 Create model of Puma 560 manipulator


clear L
deg = pi/180;

% joint angle limits from 
% A combined optimization method for solving the inverse kinematics pp560lem...
% Wang & Chen
% IEEE Trans. RA 7(4) 1991 pp 489-
L(1) = Revolute('d', 0, 'a', 0, 'alpha', pi/2, ...
    'I', [0, 0.35, 0, 0, 0, 0], ...
    'r', [0, 0, 0], ...
    'm', 0, ...
    'Jm', 200e-6, ...
    'G', -62.6111, ...
    'B', 1.48e-3, ...
    'Tc', [0.395 -0.435], ...
    'qlim', [-160 160]*deg );

L(2) = Revolute('d', 0, 'a', 0.4318, 'alpha', 0, ...
    'I', [0.13, 0.524, 0.539, 0, 0, 0], ...
    'r', [-0.3638, 0.006, 0.2275], ...
    'm', 17.4, ...
    'Jm', 200e-6, ...
    'G', 107.815, ...
    'B', .817e-3, ...
    'Tc', [0.126 -0.071], ...
    'qlim', [-45 225]*deg );

L(3) = Revolute('d', 0.15005, 'a', 0.0203, 'alpha', -pi/2,  ...
    'I', [0.066, 0.086, 0.0125, 0, 0, 0], ...
    'r', [-0.0203, -0.0141, 0.070], ...
    'm', 4.8, ...
    'Jm', 200e-6, ...
    'G', -53.7063, ...
    'B', 1.38e-3, ...
    'Tc', [0.132, -0.105], ...
    'qlim', [-225 45]*deg );

L(4) = Revolute('d', 0.4318, 'a', 0, 'alpha', pi/2,  ...
    'I', [1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0], ...
    'r', [0, 0.019, 0], ...
    'm', 0.82, ...
    'Jm', 33e-6, ...
    'G', 76.0364, ...
    'B', 71.2e-6, ...
    'Tc', [11.2e-3, -16.9e-3], ...
    'qlim', [-110 170]*deg);

L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2,  ...
    'I', [0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0], ...
    'r', [0, 0, 0], ...
    'm', 0.34, ...
    'Jm', 33e-6, ...
    'G', 71.923, ...
    'B', 82.6e-6, ...
    'Tc', [9.26e-3, -14.5e-3], ...
    'qlim', [-100 100]*deg );


L(6) = Revolute('d', 0, 'a', 0, 'alpha', 0,  ...
    'I', [0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0], ...
    'r', [0, 0, 0.032], ...
    'm', 0.09, ...
    'Jm', 33e-6, ...
    'G', 76.686, ...
    'B', 36.7e-6, ...
    'Tc', [3.96e-3, -10.5e-3], ...
    'qlim', [-266 266]*deg );

p560 = SerialLink(L, 'name', 'Puma 560', ...
    'manufacturer', 'Unimation', 'ikine', 'puma', 'comment', 'viscous friction; params of 8/95');


p560.model3d = 'UNIMATE/puma560';

%
% some useful poses
%
qz = [0 0 0 0 0 0]; % zero angles, L shaped pose
qr = [0 pi/2 -pi/2 0 0 0]; % ready pose, arm up
qs = [0 0 -pi/2 0 0 0];
qn=[0 pi/4 pi 0 pi/4  0];


clear L
end


function drawSphere(position,diameter)

%     diameter = 0.1;
    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);
    %~ shading flat

end


