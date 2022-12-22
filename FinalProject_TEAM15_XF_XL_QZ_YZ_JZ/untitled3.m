ur5 = ur5_interface();
while 1
    q = ur5.get_current_joints()
    ur5FwdKin(q)
    pause(1)
end
ur5FwdKin(q)

