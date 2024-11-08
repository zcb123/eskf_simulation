function quat = Quaternion_from_AxisAngle_3arg(angle,ZERO,axis)
    
    if nargin == 1
        if length(angle) == 3
            angle_len = norm(angle);
             
            if angle_len < 1e-6
                    quat = single([1 0 0 0]');
                return;
            end
            x = angle(1)/angle_len;
            y = angle(2)/angle_len;
            z = angle(3)/angle_len;
            theta = angle_len/2;
            quat = single([cos(theta) ;sin(theta)*x ;sin(theta)*y ;sin(theta)*z]);
        end
    elseif nargin == 2
        if length(angle) == 3
            angle_len = norm(angle);
            theta = angle_len;
            if theta < ZERO
                    quat = single([1 0 0 0]');
                return ;
            end
            x = angle(1)/angle_len;
            y = angle(2)/angle_len;
            z = angle(3)/angle_len;
            theta = angle_len/2;
            quat = single([cos(theta) ;sin(theta)*x ;sin(theta)*y ;sin(theta)*z]);

        end
    elseif nargin == 3
        len = sqrt(axis(1)^2+axis(2)^2+axis(3)^2);
        norm_axis = axis/len;
        
        theta = angle/2;
    
        if theta<ZERO
            quat = single([1 0 0 0]');
        else
            quat = single([cos(theta); sin(theta)*norm_axis(1); sin(theta)*norm_axis(2); sin(theta)*norm_axis(3)]);
        end
    end
end