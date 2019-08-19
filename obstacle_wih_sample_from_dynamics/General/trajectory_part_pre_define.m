% Return the reward obtained along trajectory determined by given inputs,
% as well as the gradient of the reward with respect to those inputs.
function val = trajectory_part_pre_define(u)

    rho= u(1);
    sf = u(end);
    val = (rho - 4)^2 + (sf - 1.3)^2;
    
end