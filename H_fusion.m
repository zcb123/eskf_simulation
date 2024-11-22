             		%% X	
% Observation Jacobians
				Hfusion = zeros(23,1);

				Hfusion(2) = HKX8*(HKX3 - HKX7) + gyro_unbias(1)*(-HKX10 + HKX16) - gyro_unbias(2)*(HKX9*(-HKX4 + HKX5) - x_offset_body*(HKX13 + HKX17));
				Hfusion(3) = -HKX25;
				Hfusion(4) = 1;
				Hfusion(10) = -HKX30*(HKX27 + HKX28*z_offset_body);
				Hfusion(11) = HKX33;
				Hfusion(12) = -HKX29*(-HKX28*HKX31 + HKX32*y_offset_body);
%% Y
				% Observation Jacobians
				Hfusion = zeros(23,1);

                Hfusion(1) = HKY11;
				Hfusion(3) = HKY18*(HKY13 - HKY17) - gyro_unbias(2)*(HKY20 + z_offset_body*(HKY10 + HKY21)) + gyro_unbias(3)*(HKY22 - HKY25);
				Hfusion(5) = 1;
				Hfusion(10) = -HKY26*(-HKY27*HKY28 + HKY29*z_offset_body);
				Hfusion(11) = -HKY31;
				Hfusion(12) = HKY26*(HKY29*x_offset_body + HKY32);


                %% Z 
Hfusion = zeros(23,1);



Hfusion(1) = HKZ18;
Hfusion(2) = HKZ30;
Hfusion(6) = 2;
Hfusion(10) = HKZ36;
Hfusion(11) = -HKZ40;
Hfusion(12) = -HKZ43;