% Trajectory Generation
% Hard to say
function traj = lineGen(g_start, g_end)

    s = linspace(0, 1+0.1, 10);
    p = zeros(3, 1, length(s));
    R = zeros(3, 3, length(s));
    
    for i=1:length(s)
        p(:,:,i) = g_start(1:3,4) + s(i)*(g_end(1:3,4)-g_start(1:3,4));
        R(:,:,i) = g_start(1:3,1:3);
    end

    traj = [R,p; zeros(1,3,length(s)),ones(1,1,length(s))];

end