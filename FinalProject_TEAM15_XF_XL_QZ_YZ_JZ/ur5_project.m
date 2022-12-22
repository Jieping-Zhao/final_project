%% Notations

% group work of Team 15

%% Read me
% First, choose input_mode, 0 -> auto, 1 -> manual

% if input_mode == 0,
% make sure 'gst_start' and 'gst_target' are both proper locations
% hit 'Run'
% input selection of control mode (1, 2, 3), then enter
% program starts

% if input_mode == 1,
% hit 'Run'
% select control mode (1, 2, 3), then enter
% move ur5 to start location, press any botton
% move ur5 to target location, press any botton
% program starts

%% Initialization and input
clear; clc;

%%default intermediate position
home = [1.14 -1.50 1.01 -1.86 -1.52 0.045]';
%%descend distance
H = 0.1;

%%[T_step for InKinControl, gain for InvJacControl, gain for TransJacControl]
para = [5 0.1 0.1];

%% switching mode, o -> auto, 1 -> manual
input_mode = 0;

%%pause short
pause_short = 2.5;

ur5 = ur5_interface();

%control mode selection
wait_input = "\n Enter a value to select mode.\n" + ...
    ">>>>>>> press 1 to use inverse kinematics control \n" + ...
    ">>>>>>> press 2 to use RRcontrol \n" + ...
    ">>>>>>> press 3 for transpose Jacobian \n \n" + ...
    "Enter control mode : " ;
mode = input(wait_input);

%%  setting
% 0 is default, 1 is manual

% Here is default mode, moves to the following locations autoly
if input_mode == 0
    
    %% change start and target location here
    gst_start = [0 -1 0 0.43; -1 0 0 0.43; 0 0 -1 0.21; 0 0 0 1];
    gst_target = [0 -1 0 -0.42; -1 0 0 0.45; 0 0 -1 0.21; 0 0 0 1];
    %% 
    
    inv_start = ur5InvKin(gst_start);
    inv_target = ur5InvKin(gst_target);
    
    norm_q_start = [0 0 0 0 0 0 0 0];
    norm_q_target = [0 0 0 0 0 0 0 0];

    for i = 1:8
        norm_q_start(i) = norm(inv_start(i));
        norm_q_target(i) = norm(inv_target(i));
    end

    [~, i] = min(norm_q_start);
    [~, j] = min(norm_q_target);

    q1 = inv_start(:,i);
    q2 = inv_target(:,i);
end

%% Here is manual mode
if input_mode == 1

    disp('Move to g_start, press any after you finish');
    waitforbuttonpress
    q1 = ur5.get_current_joints();
    gst_start = ur5FwdKin(q1);
    disp('done')

    pause(pause_short)

    disp('Move to g_start, press any after you finish');
    waitforbuttonpress 
    q2 = ur5.get_current_joints();
    gst_target = ur5FwdKin(q2); 
    disp('done')

end

gst_start_up = gst_start + [0 0 0 0; 0 0 0 0; 0 0 0 H; 0 0 0 0];
gst_target_up = gst_target + [0 0 0 0; 0 0 0 0; 0 0 0 H; 0 0 0 0];

%% Execution
% move to real home, home, upup, start_up, start, start_up, target_upup,
% target_up, target, target_up
ur5.move_joints(ur5.home,8)
pause(8.8)

% select a home
if (gst_start_up(1,4)>0) && (gst_start_up(2,4)>0)
    home = [-1.78,-1.76,-0.53,-2.39,1.46,0.01]';
end
if (gst_start_up(1,4)>0) && (gst_start_up(2,4)<0)
    home = [-3.51,-1.85,-0.53,-2.30,1.51,0.01]';
end
if (gst_start_up(1,4)<0) && (gst_start_up(2,4)>0)
    home = [-0.2,-1.85,-0.53,-2.30,1.51,0.01]';
end
if (gst_start_up(1,4)<0) && (gst_start_up(2,4)<0)
    home = [1.20,-1.88,-0.53,-2.23,1.51,0.01]';
end

ur5.move_joints(home, 4);
pause(2*pause_short)

disp('move to start up')
traj = trajGen(ur5FwdKin(ur5.get_current_joints()), gst_start_up );
if(mymove(traj, mode, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short);


disp(' ')
disp('move to start')
traj = trajGen(gst_start_up, gst_start);
if(mymove(traj, mode, para, ur5) == -1)
    rosshutdown
    return
end
pause(pause_short)
myError(ur5, gst_start);
disp(ur5.get_current_joints()*180/pi)


disp('move to start up')
traj = trajGen(gst_start, gst_start_up);
if(mymove(traj, mode, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short)
myError(ur5, gst_start_up); 
disp(ur5.get_current_joints()*180/pi)

% select a home
if (gst_target_up(1,4)>0) && (gst_target_up(2,4)>0)
    home = [-1.78,-1.76,-0.53,-2.39,1.46,0.01]';
end
if (gst_target_up(1,4)>0) && (gst_target_up(2,4)<0)
    home = [-3.51,-1.85,-0.53,-2.30,1.51,0.01]';
end
if (gst_target_up(1,4)<0) && (gst_target_up(2,4)>0)
    home = [-0.2,-1.85,-0.53,-2.30,1.51,0.01]';
end
if (gst_target_up(1,4)<0) && (gst_target_up(2,4)<0)
    home = [1.20,-1.88,-0.53,-2.23,1.51,0.01]';
end
ur5.move_joints(home,8)
pause(8.8)

disp('move to target up')
traj = trajGen(ur5FwdKin(ur5.get_current_joints()), gst_target_up);
if(mymove(traj, mode, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short)


disp('move to target')
traj = trajGen(gst_target_up, gst_target);
if(mymove(traj, mode, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short)
myError(ur5, gst_target);
disp(ur5.get_current_joints()*180/pi)


disp('move to target up')
traj = trajGen(gst_target, gst_target_up);
if(mymove(traj, mode, para, ur5) == -1)
    rosshutdown
    return
end
pause(pause_short)
myError(ur5, gst_target_up);
disp(ur5.get_current_joints()*180/pi)

disp('done')