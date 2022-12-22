function xi = getXi(g)

    R = g(1:3, 1:3);
    p = g(1:3, 4);
    
    theta = acos((trace(R)-1)/2);
    
    % theta = 0, w = 0, v = p
    if (abs(theta) < 0.001)
        w = [0; 0; 0];
        v = p;
        xi = [v; w];

    % when theta = pi, sin(theta) = 0
    elseif (abs(theta - pi) < 0.001)
        w_hatsqu = 0.5*(R-eye(3,3));
        w = sqrt(diag(w_hatsqu));

        % symbol of w do not change
        if w_hatsqu(3, 2) < 0
            w(1) = -w(1);
        end

        if w_hatsqu(3, 1) < 0
            w(2) = -w(2);
        end

        if w_hatsqu(2, 1) < 0
            w(3) = -w(3);
        end

        v = (eye(3,3)*theta+2*skew3(w)+ thetaw_hatsqu) \ p;
        xi = theta*[v; w];
    
    % common case
    else
        w = (1/(2*sin(theta)))*invskew(R-R.');
        v = ((eye(3,3)-expm(theta*skew3(w)))*skew3(w)+w*w.' * theta)\p;
        xi = theta*[v; w];
    end    
end

function y = invskew(x)
    
    y = [x(3, 2); x(1, 3); x(2, 1)];

end