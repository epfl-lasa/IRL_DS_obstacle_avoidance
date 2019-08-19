function po = rot(p, a, flag)

theta = pi/4;
if flag == 'ac'
    R = makehgtform('zrotate', -theta);
    R = R(1:3, 1:3);
else
    R = makehgtform('zrotate', theta);
    R = R(1:3, 1:3);
end

po = R*p';

if a == 1
    theta = 5/180*pi;

else
    theta = -40/180*pi;
end



if flag == 'ac'
    R = makehgtform('yrotate', theta);    
else
    R = makehgtform('yrotate', theta);    
end

R = R(1:3, 1:3);
po = R*po;