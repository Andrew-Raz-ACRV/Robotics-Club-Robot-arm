function q_ = applyJointLimits(q,qL,qU)
%Given the lower and upper joint limits in qL and qU respectively, this
%restricts the values of q to be in that range i.e. qL(i) < q(i) < qU(i)

%by default
q_ = q;

%Check limits in each value q
for ii = 1:length(q)
    if q(ii)<qL(ii)
        q_(ii)=qL(ii);
    elseif q(ii)>qU(ii)
        q_(ii)=qU(ii);
    end
end

end