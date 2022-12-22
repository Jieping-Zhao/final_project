function y = skew6(xi)
    w = xi(4:6);
    v = xi(1:3);
    y = [skew3(w), v; 0, 0, 0, 0];
end