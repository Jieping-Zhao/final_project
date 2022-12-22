clear;
clc;

ur5 = ur5_interface();
home = [0; -9*pi/20; -pi/20; -11*pi/20; pi/20; 0];
r = 0.05;
pause_short = 2.2;
para = [0.5 0.1 2];

input_mode = 0;

if input_mode == 0
    gst_target = [-0.8188 -0.5564   -0.1414    0.1135; -0.5707    0.8155   -0.0962    0.5332;  -0.0618   -0.1595   -0.9853    0.3557; 0 0 0 1];
    
    inv_target = ur5InvKin(gst_target);
    
    norm_q_target = [0 0 0 0 0 0 0 0];

    for i = 1:8
        norm_q_target(i) = norm(inv_target(i));
    end
    [~, j] = min(norm_q_target);
    q2 = inv_target(:,i);

end

if input_mode == 1

    disp("Move robot to target position. Press any key to continue.");
    waitforbuttonpress 
    q2 = ur5.get_current_joints();
    gst_target = ur5FwdKin(q2); 
    disp("Target position collected.")

end

% q2(2) = q2(2) - 0.2*sign(q2(2));

ur5.move_joints(ur5.home, 8)
pause(8.8)

ur5.move_joints(home, 2);
pause(pause_short)

disp('move to up up')
ur5.move_joints(q2, 5);
pause(5)

disp('move to up')
traj = circleGen(ur5FwdKin(ur5.get_current_joints()), r);
if(mymove(traj, 1, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short);

function traj = circleGen(g_target, r)

    s = linspace(0, 2*pi + 0.01, 25);
    p = zeros(3, 1, length(s));
    R = zeros(3, 3, length(3));
    
    for i=1:length(s)

        p(:,:,i) = g_target(1:3,4) + [0;r;0]*cos(s(i)) + [r;0;0]*sin(s(i));
        R(:,:,i) = g_target(1:3,1:3);

    end

    traj = [R,p; zeros(1,3,length(s)),ones(1,1,length(s))];

end

