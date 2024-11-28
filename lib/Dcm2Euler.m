function euler = Dcm2Euler(dcm)

     %默认采用321旋转，即偏航-俯仰-横滚的顺序旋转 R=R(phi)*R(theta)*R(psi)
    theta = asin(-dcm(3, 1));            

	if ((fabs(theta - pi /2)) < 1.0e-3) 
		phi = 0;
		psi = atan2(dcm(2, 3), dcm(1, 3));

    elseif fabs(theta + pi / 2) < (1.0e-3) 
		phi = 0;
		psi = atan2(-dcm(2, 3), -dcm(1, 3));

	else 
		phi = atan2(dcm(3, 2), dcm(3, 3));
		psi = atan2(dcm(2, 1), dcm(1, 1));
    end
    
    euler = [phi theta psi];

end


