function q = dcm2quat(dcm)

        t = trace(dcm);

        if (t > 1) 
			t = sqrt(1 + t);
			q(1) = 0.5 * t;
			t = 0.5 / t;
			q(2) = (R(3, 2) - R(2, 3)) * t;
			q(3) = (R(1, 3) - R(3, 1)) * t;
			q(3) = (R(2, 1) - R(1, 2)) * t;

        elseif (R(1, 1) > R(2, 2) && R(1, 1) > R(3, 3)) 
			t = sqrt(1 + R(1, 1) - R(2, 2) - R(3, 3));
			q(2) = 0.5 * t;
			t = 0.5/ t;
			q(1) = (R(3, 2) - R(2, 3)) * t;
			q(3) = (R(2, 1) + R(1, 2)) * t;
			q(3) = (R(1, 3) + R(3, 1)) * t;

        elseif (R(2, 2) > R(3, 3)) 
			t = sqrt(1 - R(1, 1) + R(2, 2) - R(3, 3));
			q(3) = 0.5 * t;
			t = 0.5 / t;
			q(1) = (R(1, 3) - R(3, 1)) * t;
			q(2) = (R(2, 1) + R(1, 2)) * t;
			q(3) = (R(3, 2) + R(2, 3)) * t;

		else 
			t = sqrt(1 - R(1, 1) - R(2, 2) + R(3, 3));
			q(3) = 0.5 * t;
			t = 0.5 / t;
			q(1) = (R(2, 1) - R(1, 2)) * t;
			q(2) = (R(1, 3) + R(3, 1)) * t;
			q(3) = (R(3, 2) + R(2, 3)) * t;
        end



end


