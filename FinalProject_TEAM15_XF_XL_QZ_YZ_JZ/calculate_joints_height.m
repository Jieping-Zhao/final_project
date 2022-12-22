function joints_height = calculate_joints_height(q)
    joints_height  = zeros(1,6);

%     q = q + [0; pi/2; 0; pi/2; 0; 0 ];

    % revolute joint
    w1 = [0 0 1].';
    w2 = [0 1 0].';
    w3 = [0 1 0].';
    w4 = [0 1 0].';
    w5 = [0 0 1].';
    w6 = [0 1 0].';

    w_ = {w1, w2, w3, w4, w5, w6};
    
    % length of arms
    l1 = 0.425;
    l2 = 0.392;
    l3 = 0.1093;
    l4 = 0.09475;
    l5 = 0.0825;
    l6 = 0.0892;

    % points q
    q1 = [0 0 0].';
    q2 = [0 0 l6].';
    q3 = [0 0 l6+l1].';
    q4 = [0 0 l6+l1+l2].';
    q5 = [0 l3 0].';
    q6 = [0 l3+l5 l1+l2+l4+l6].';

    q_ = {q1, q2, q3, q4, q5, q6};
    
    % gst0
    gst0 = [expm(-skew3([1; 0; 0])*pi/2), [0,l3+l5,l1+l2+l4+l6]';0, 0, 0, 1];

    % compute xi
    xi = {0 0 0 0 0 0};
    for i = 1:6
       xi{i} = revoluteTwist(q_{i}, w_{i});
       y = eye(4);
           
       for 1:i
           y = y*expm(skew6(xi{i})*q(i));
       end
       y = y*gst0;
       joints_height(i) = y(3,4);
    end
end