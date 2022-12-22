% Error Reporting
function myError(ur5,gdesired)

    gst = ur5FwdKin( ur5.get_current_joints() );
    R = gst(1:3,1:3);
    p = gst(1:3,4);

    Rd = gdesired(1:3,1:3);
    pd = gdesired(1:3,4);

    dr3 = norm(p-pd);
    dso3 = sqrt(trace((R-Rd)*(R-Rd)'));

    fprintf("dr3: %.2f mm\n", dr3*1000);
    fprintf("dso3: %.2f degrees\n \n", dso3/pi*180);

end