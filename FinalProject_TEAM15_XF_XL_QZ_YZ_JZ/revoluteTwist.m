function y = revoluteTwist(q, w)
    y = [cross(-w, q); w];
end