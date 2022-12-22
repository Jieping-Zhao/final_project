clear;
clc;

ur5 = ur5_interface();
% home = [1.1246
%    -1.2998
%     0.5345
%    -1.0115
%    -1.5531
%    -0.0074]';
home = [1.1246; -1.2998; 0.53; -1; - 1.55; -0.007];
r = 0.05;
pause_short = 2.2;
para = [0.5 0.1 2];

input_mode = 0;

if input_mode == 0
    gst_target = [-0.8381   -0.5444    0.0364    0.1403; -0.5455    0.8372   -0.0385    0.5781;  -0.0095   -0.0521   -0.9986    0.3525; 0 0 0 1];
    
    inv_target = ur5InvKin(gst_target);
    
    norm_q_target = [0 0 0 0 0 0 0 0];

    for i = 1:8
        norm_q_target(i) = norm(inv_target(i));
    end
    [~, j] = min(norm_q_target);
    q2 = inv_target(:,i);

end

gst_1 = [ -0.9426    0.3314    0.0400     0.1277;  0.3313    0.9435   -0.0084     0.3996; -0.0405    0.0053   -0.9992    0.3525;0 0 0 1];
gst_2 = [-0.9426    0.3314    0.0400    0.1155; 0.3313    0.9435   -0.0084   0.4259;-0.0405    0.0053   -0.9992     0.3525;0 0 0 1];
gst_3 = [-0.9426    0.3314    0.0400    0.1105; 0.3313    0.9435   -0.0084     0.3997;  -0.0405    0.0053   -0.9992     0.3525; 0  0 0 1];
gst_4 = [ -0.9426    0.3314    0.0400   0.1333;0.3313    0.9435   -0.0084    0.4168;-0.0405    0.0053   -0.9992   0.3525;0 0 0 1];
gst_5 =  [ -0.9426    0.3314    0.0400    0.0868;0.3313    0.9435   -0.0084     0.4161; -0.0405    0.0053   -0.9992     0.3525;0 0 0 1 ];

ur5.move_joints(ur5.home, 8)
pause(8.8)

ur5.move_joints(home, 2);
pause(pause_short)

disp('move to gst_1')
ur5.move_joints(ur5InvKin(gst_1), 5);
pause(5)

disp('move to gst_2')
traj = lineGen(gst_1, gst_2);
if(mymove(traj, 1, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short);


disp('move to gst_3')
traj = lineGen(gst_2, gst_3);
if(mymove(traj, 1, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short);

disp('move to gst_4')
traj = lineGen(gst_3, gst_4);
if(mymove(traj, 1, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short);

disp('move to gst_5')
traj = lineGen(gst_4, gst_5);
if(mymove(traj, 1, para, ur5)==-1)
    rosshutdown
    return
end
pause(pause_short);


disp('move to gst_1')
traj = lineGen(gst_5, gst_1);
if(mymove(traj, 1, para, ur5)==-1)
    rosshutdown
    return
end


pause(pause_short);

