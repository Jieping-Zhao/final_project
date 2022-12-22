function finalerr = validcheck(ur5)

    q=ur5.get_current_joints();
    gst=ur5FwdKin(q);
    Jb=ur5BodyJacobian(q);
    det_Jb=manipulability(Jb, 'detjac');
    gsg = gst;

    finalerr = 1;

    if(abs(det_Jb) < 1e-7)
        finalerr = -1;
        warning('Abort: Singularity detected.')
        return
    end

    if (abs(q) > 2*pi)
        finalerr = -1;
        warning('Abort: Joint limits exceeded.')
        return
    end  

    if (gsg(3,4) <= 0.03)
        finalerr = -1;
        warning('Abort: Bottom surface impact detected.')
        return
    end

end