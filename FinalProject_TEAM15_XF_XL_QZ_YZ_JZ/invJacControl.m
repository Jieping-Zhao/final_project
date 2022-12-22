function finalerr=invJacControl(gdesired,K,ur5)

    T = 1.5;

    xi = [1;1;1;1;1;1];
    trytime = 0;
    n = 0;
    while (norm(xi(1:3))> 0.05 || norm(xi(4:6)) > 0.1)
        n = n + 1
        disp(norm(xi))
        if norm(xi)<0.3
            T = 0.4;
        end
        q = ur5.get_current_joints();
        gst = ur5FwdKin(q);
        Jb = ur5BodyJacobian(q); 
        xi = getXi( gdesired \ gst );
        
        dq = -pinv(Jb) * K * xi;

%         disp(dq)

        over_speed = abs(dq) > pi*ur5.speed_limit * T * 0.5;
        dq(over_speed) = 0.5 * sign(dq(over_speed))* pi * ur5.speed_limit * T;

        q_t = dq+q;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         gst_start = [0 -1 0 0.1; -1 0 0 -0.1; 0 0 -1 0.21; 0 0 0 1];
        q_t = q_t * 180/pi;

        if q_t(2) < -179 || q_t(2) > 0
            dq(2) = 0;
        end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        q = q + dq;

        if abs(q(1))>pi
            fail = 1;

        elseif abs(q(6))>pi
        
            fail = 6;
        else
            fail = 0;
        end

        if fail >= 1
            trytime = trytime+1;
        else
            trytime = 0;
        end

        if trytime > 30
            q(fail) = pi/2;
            ur5.move_joints(q,5);
            pause(5)
            trytime = 0;
        end

        

        ur5.move_joints(q, T);
        pause(T);

        disp(q*180/pi)

    end
    err = gdesired \gst;
    finalerr = norm(err(1:3,4));
    
end
     
                
    
