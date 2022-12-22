function y = myexp(xi,a,b,q)

    y = eye(4,4);
    for i = a:b
        skew6(xi{i});
        y = y*expm(skew6(xi{i})*q(i));
    end
end