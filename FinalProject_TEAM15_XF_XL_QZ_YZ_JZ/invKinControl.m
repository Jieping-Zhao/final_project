function finalerr = invKinControl(gdesired,T, ur5)

    q = ur5.get_current_joints();

    dq = 1;
    
    %%%%%%%%%%%%%%%%%
    n = 0;

    while dq > 1e-2

        q_inv = ur5InvKin(gdesired);
        qnorm = zeros(1, 8);
    
        for i=1:8
            qnorm(i) = norm(q_inv(:, i) - q);
        end
    
        [~,i] = min(qnorm);
        
        q_desired = q_inv(:,i);
        dq = q_desired - q;
    
        for i = 1:length(dq)
            if abs( dq(i) ) / T > pi * ur5.speed_limit
                dq(i) = 0.9 * sign(dq(i)) * pi * ur5.speed_limit * T;
            end
        end

        q_desired = q+dq;

        ur5.move_joints(q_desired, T); 
        pause(1.01 * T);
    
        finalerr = 0;
        
        %%%%
        n = n +1 
    end
end