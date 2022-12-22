function finalerr=transJacControl(gdesired,K,ur5)

    T = 1;

    xi = [1;1;1;1;1;1];
    n = 0;
    
    while (norm(xi(1:3))> 0.05 || norm(xi(4:6)) > 0.1)

        q = ur5.get_current_joints();
        gst = ur5FwdKin(q);
        Jb = ur5BodyJacobian(q);
        xi = getXi(gdesired\gst);
        
        
        n = n+1
        
        alpha = (norm(Jb.' * xi) / norm(Jb*Jb.'*xi))^2;

        dq = -transpose(Jb) * K * xi * alpha;
        
        disp(norm(dq))
        if norm (dq)<0.02
            T = 0.3;
        end
        if norm(dq)< 0.01
            T = 0.2;
        end

       for i = 1:6
            if abs(dq(i))/T > pi * ur5.speed_limit * 0.7
                dq(i) = 0.7 * sign(dq(i))*pi * ur5.speed_limit * T;
            end
        end

        q = q + dq;

        ur5.move_joints(q, T);
        pause(1.2*T);


    end
    err = gdesired\gst;
    finalerr = norm(err(1:3,4));
    
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
end