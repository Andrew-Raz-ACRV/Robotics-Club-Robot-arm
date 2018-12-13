function v_ = cap_mag(v,magnitude)
% Given a vector v and the magnitude required. This makes the magnitude of
% the vector v become magnitude based on the unit vector u = v/magnitude

actual_magnitude = norm(v);

v_ = magnitude * (v/actual_magnitude);

end