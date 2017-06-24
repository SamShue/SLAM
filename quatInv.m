function q2 = quatInv(q1)
    q2 = q1.*[1, -1, -1, -1]./sum(q1.*q1);
end

