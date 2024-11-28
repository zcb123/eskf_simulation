% Equations for covariance matrix prediction, without process noise~
	PS0 = 2*powf(q2, 2);
	PS1 = 2*powf(q3, 2) - 1;
	PS2 = PS0 + PS1;
	PS3 = q0*q2;
	PS4 = q1*q3;
	PS5 = 2*PS3 + 2*PS4;
	PS6 = q0*q3;
	PS7 = q1*q2;
	PS8 = 2*PS6 - 2*PS7;
	PS9 = PS2*P(9,9) - PS5*P(9,11) + PS8*P(9,10) + P(0,9);
	PS10 = PS2*P(9,11) - PS5*P(11,11) + PS8*P(10,11) + P(0,11);
	PS11 = PS2*P(9,10) - PS5*P(10,11) + PS8*P(10,10) + P(0,10);
	PS12 = PS2*P(0,9) - PS5*P(0,11) + PS8*P(0,10) + P(0,0);
	PS13 = 2*powf(q1, 2);
	PS14 = PS1 + PS13;
	PS15 = PS6 + PS7;
	PS16 = 2*PS9;
	PS17 = q0*q1;
	PS18 = q2*q3;
	PS19 = 2*PS17 - 2*PS18;
	PS20 = PS2*P(1,9) - PS5*P(1,11) + PS8*P(1,10) + P(0,1);
	PS21 = PS0 + PS13 - 1;
	PS22 = 2*PS17 + 2*PS18;
	PS23 = PS3 - PS4;
	PS24 = PS2*P(2,9) - PS5*P(2,11) + PS8*P(2,10) + P(0,2);
	PS25 = PS2*P(9,12);
	PS26 = PS25 - PS5*P(11,12) + PS8*P(10,12) + P(0,12);
	PS27 = dvy - dvy_b; %dVel_y - dv_by;
	PS28 = PS22*PS27;
	PS29 = dvx - dvx_b; %dVel_x - dv_bx;
	PS30 = 2*PS29;
	PS31 = PS23*PS30;
	PS32 = dvz - dvz_b; %dVel_z - dv_bz;
	PS33 = PS21*PS32;
	PS34 = -PS28 + PS31 + PS33;
	PS35 = PS15*PS30;
	PS36 = PS19*PS32;
	PS37 = PS14*PS27;
	PS38 = PS35 - PS36 - PS37;
	PS39 = -PS5*P(11,14);
	PS40 = PS2*P(9,14) + PS39 + PS8*P(10,14) + P(0,14);
	PS41 = PS8*P(10,13);
	PS42 = PS2*P(9,13) + PS41 - PS5*P(11,13) + P(0,13);
	PS43 = PS2*P(3,9) - PS5*P(3,11) + PS8*P(3,10) + P(0,3);
	PS44 = PS32*PS5;
	PS45 = PS27*PS8;
	PS46 = PS2*PS29;
	PS47 = -PS44 + PS45 + PS46;
	PS48 = PS28 - PS31 - PS33;
	PS49 = 2*PS26;
	PS50 = PS2*P(4,9) - PS5*P(4,11) + PS8*P(4,10) + P(0,4);
	PS51 = -PS35 + PS36 + PS37;
	PS52 = PS44 - PS45 - PS46;
	PS53 = PS2*P(5,9) - PS5*P(5,11) + PS8*P(5,10) + P(0,5);
	PS54 = 2*PS15;
	PS55 = PS14*P(10,10) + PS19*P(10,11) - PS54*P(9,10) + P(1,10);
	PS56 = PS14*P(9,10) + PS19*P(9,11) - PS54*P(9,9) + P(1,9);
	PS57 = PS14*P(10,11) + PS19*P(11,11) - PS54*P(9,11) + P(1,11);
	PS58 = PS14*P(1,10) + PS19*P(1,11) - PS54*P(1,9) + P(1,1);
	PS59 = 2*PS23;
	PS60 = PS14*P(2,10) + PS19*P(2,11) - PS54*P(2,9) + P(1,2);
	PS61 = -PS54*P(9,12);
	PS62 = PS14*P(10,12) + PS19*P(11,12) + PS61 + P(1,12);
	PS63 = PS19*P(11,14);
	PS64 = PS14*P(10,14) - PS54*P(9,14) + PS63 + P(1,14);
	PS65 = PS14*P(10,13);
	PS66 = PS19*P(11,13) - PS54*P(9,13) + PS65 + P(1,13);
	PS67 = PS14*P(3,10) + PS19*P(3,11) - PS54*P(3,9) + P(1,3);
	PS68 = PS14*P(0,10) + PS19*P(0,11) - PS54*P(0,9) + P(0,1);
	PS69 = PS14*P(4,10) + PS19*P(4,11) - PS54*P(4,9) + P(1,4);
	PS70 = PS14*P(5,10) + PS19*P(5,11) - PS54*P(5,9) + P(1,5);
	PS71 = PS21*P(11,11) - PS22*P(10,11) + PS59*P(9,11) + P(2,11);
	PS72 = PS21*P(10,11) - PS22*P(10,10) + PS59*P(9,10) + P(2,10);
	PS73 = PS21*P(9,11) - PS22*P(9,10) + PS59*P(9,9) + P(2,9);
	PS74 = PS21*P(2,11) - PS22*P(2,10) + PS59*P(2,9) + P(2,2);
	PS75 = PS59*P(9,12);
	PS76 = PS21*P(11,12) - PS22*P(10,12) + PS75 + P(2,12);
	PS77 = PS21*P(1,11) - PS22*P(1,10) + PS59*P(1,9) + P(1,2);
	PS78 = PS21*P(11,14);
	PS79 = -PS22*P(10,14) + PS59*P(9,14) + PS78 + P(2,14);
	PS80 = -PS22*P(10,13);
	PS81 = PS21*P(11,13) + PS59*P(9,13) + PS80 + P(2,13);
	PS82 = PS21*P(3,11) - PS22*P(3,10) + PS59*P(3,9) + P(2,3);
	PS83 = PS21*P(0,11) - PS22*P(0,10) + PS59*P(0,9) + P(0,2);
	PS84 = PS21*P(4,11) - PS22*P(4,10) + PS59*P(4,9) + P(2,4);
	PS85 = PS21*P(5,11) - PS22*P(5,10) + PS59*P(5,9) + P(2,5);
	PS86 = PS2*P(12,12) - PS34*P(1,12) - PS38*P(2,12) - PS5*P(12,14) + PS8*P(12,13) + P(3,12);
	PS87 = PS2*P(1,12) - PS34*P(1,1) - PS38*P(1,2) - PS5*P(1,14) + PS8*P(1,13) + P(1,3);
	PS88 = PS2*P(2,12) - PS34*P(1,2) - PS38*P(2,2) - PS5*P(2,14) + PS8*P(2,13) + P(2,3);
	PS89 = PS2*P(12,14) - PS34*P(1,14) - PS38*P(2,14) - PS5*P(14,14) + PS8*P(13,14) + P(3,14);
	PS90 = PS2*P(12,13) - PS34*P(1,13) - PS38*P(2,13) - PS5*P(13,14) + PS8*P(13,13) + P(3,13);
	PS91 = PS2*P(3,12) - PS34*P(1,3) - PS38*P(2,3) - PS5*P(3,14) + PS8*P(3,13) + P(3,3);
	PS92 = PS2*P(0,12) - PS34*P(0,1) - PS38*P(0,2) - PS5*P(0,14) + PS8*P(0,13) + P(0,3);
	PS93 = PS2*P(4,12) - PS34*P(1,4) - PS38*P(2,4) - PS5*P(4,14) + PS8*P(4,13) + P(3,4);
	PS94 = PS2*P(5,12) - PS34*P(1,5) - PS38*P(2,5) - PS5*P(5,14) + PS8*P(5,13) + P(3,5);
	PS95 = PS14*P(13,13) + PS19*P(13,14) - PS47*P(2,13) - PS48*P(0,13) - PS54*P(12,13) + P(4,13);
	PS96 = PS14*P(0,13) + PS19*P(0,14) - PS47*P(0,2) - PS48*P(0,0) - PS54*P(0,12) + P(0,4);
	PS97 = PS14*P(12,13) + PS19*P(12,14) - PS47*P(2,12) - PS48*P(0,12) - PS54*P(12,12) + P(4,12);
	PS98 = PS14*P(13,14) + PS19*P(14,14) - PS47*P(2,14) - PS48*P(0,14) - PS54*P(12,14) + P(4,14);
	PS99 = PS14*P(4,13) + PS19*P(4,14) - PS47*P(2,4) - PS48*P(0,4) - PS54*P(4,12) + P(4,4);
	PS100 = PS14*P(5,13) + PS19*P(5,14) - PS47*P(2,5) - PS48*P(0,5) - PS54*P(5,12) + P(4,5);
	PS101 = PS21*P(14,14) - PS22*P(13,14) - PS51*P(0,14) - PS52*P(1,14) + PS59*P(12,14) + P(5,14);
	PS102 = PS21*P(13,14) - PS22*P(13,13) - PS51*P(0,13) - PS52*P(1,13) + PS59*P(12,13) + P(5,13);
	PS103 = PS21*P(12,14) - PS22*P(12,13) - PS51*P(0,12) - PS52*P(1,12) + PS59*P(12,12) + P(5,12);
	PS104 = PS21*P(5,14) - PS22*P(5,13) - PS51*P(0,5) - PS52*P(1,5) + PS59*P(5,12) + P(5,5);

	% covariance update
	SquareMatrix23f nextP;

	% calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
	% 计算四元数、速度、位置、陀螺仪零偏状态的上三角对角阵
	nextP(0,0) = -PS10*PS5 + PS11*PS8 + PS12 + PS2*PS9;
	nextP(0,1) = PS10*PS19 + PS11*PS14 - PS15*PS16 + PS20;
	nextP(1,1) = PS14*PS55 + PS19*PS57 - PS54*PS56 + PS58;
	nextP(0,2) = PS10*PS21 - PS11*PS22 + PS16*PS23 + PS24;
	nextP(1,2) = PS21*PS57 - PS22*PS55 + PS56*PS59 + PS60;
	nextP(2,2) = PS21*PS71 - PS22*PS72 + PS59*PS73 + PS74;
	nextP(0,3) = PS2*PS26 - PS20*PS34 - PS24*PS38 - PS40*PS5 + PS42*PS8 + PS43;
	nextP(1,3) = PS2*PS62 - PS34*PS58 - PS38*PS60 - PS5*PS64 + PS66*PS8 + PS67;
	nextP(2,3) = PS2*PS76 - PS34*PS77 - PS38*PS74 - PS5*PS79 + PS8*PS81 + PS82;
	nextP(3,3) = PS2*PS86 - PS34*PS87 - PS38*PS88 - PS5*PS89 + PS8*PS90 + PS91;
	nextP(0,4) = -PS12*PS48 + PS14*PS42 - PS15*PS49 + PS19*PS40 - PS24*PS47 + PS50;
	nextP(1,4) = PS14*PS66 + PS19*PS64 - PS47*PS60 - PS48*PS68 - PS54*PS62 + PS69;
	nextP(2,4) = PS14*PS81 + PS19*PS79 - PS47*PS74 - PS48*PS83 - PS54*PS76 + PS84;
	nextP(3,4) = PS14*PS90 + PS19*PS89 - PS47*PS88 - PS48*PS92 - PS54*PS86 + PS93;
	nextP(4,4) = PS14*PS95 + PS19*PS98 - PS47*(PS14*P(2,13) + PS19*P(2,14) - PS47*P(2,2) - PS48*P(0,2) - PS54*P(2,12) + P(2,4)) - PS48*PS96 - PS54*PS97 + PS99;
	nextP(0,5) = -PS12*PS51 - PS20*PS52 + PS21*PS40 - PS22*PS42 + PS23*PS49 + PS53;
	nextP(1,5) = PS21*PS64 - PS22*PS66 - PS51*PS68 - PS52*PS58 + PS59*PS62 + PS70;
	nextP(2,5) = PS21*PS79 - PS22*PS81 - PS51*PS83 - PS52*PS77 + PS59*PS76 + PS85;
	nextP(3,5) = PS21*PS89 - PS22*PS90 - PS51*PS92 - PS52*PS87 + PS59*PS86 + PS94;
	nextP(4,5) = PS100 + PS21*PS98 - PS22*PS95 - PS51*PS96 - PS52*(PS14*P(1,13) + PS19*P(1,14) - PS47*P(1,2) - PS48*P(0,1) - PS54*P(1,12) + P(1,4)) + PS59*PS97;
	nextP(5,5) = PS101*PS21 - PS102*PS22 + PS103*PS59 + PS104 - PS51*(PS21*P(0,14) - PS22*P(0,13) - PS51*P(0,0) - PS52*P(0,1) + PS59*P(0,12) + P(0,5)) - PS52*(PS21*P(1,14) - PS22*P(1,13) - PS51*P(0,1) - PS52*P(1,1) + PS59*P(1,12) + P(1,5));
	nextP(0,6) = PS2*P(6,9) + PS43*dt - PS5*P(6,11) + PS8*P(6,10) + P(0,6);
	nextP(1,6) = PS14*P(6,10) + PS19*P(6,11) - PS54*P(6,9) + PS67*dt + P(1,6);
	nextP(2,6) = PS21*P(6,11) - PS22*P(6,10) + PS59*P(6,9) + PS82*dt + P(2,6);
	nextP(3,6) = PS2*P(6,12) - PS34*P(1,6) - PS38*P(2,6) - PS5*P(6,14) + PS8*P(6,13) + PS91*dt + P(3,6);
	nextP(4,6) = PS14*P(6,13) + PS19*P(6,14) - PS47*P(2,6) - PS48*P(0,6) - PS54*P(6,12) + P(4,6) + dt*(PS14*P(3,13) + PS19*P(3,14) - PS47*P(2,3) - PS48*P(0,3) - PS54*P(3,12) + P(3,4));
	nextP(5,6) = PS21*P(6,14) - PS22*P(6,13) - PS51*P(0,6) - PS52*P(1,6) + PS59*P(6,12) + P(5,6) + dt*(PS21*P(3,14) - PS22*P(3,13) - PS51*P(0,3) - PS52*P(1,3) + PS59*P(3,12) + P(3,5));
	nextP(6,6) = P(3,6)*dt + P(6,6) + dt*(P(3,3)*dt + P(3,6));
	nextP(0,7) = PS2*P(7,9) - PS5*P(7,11) + PS50*dt + PS8*P(7,10) + P(0,7);
	nextP(1,7) = PS14*P(7,10) + PS19*P(7,11) - PS54*P(7,9) + PS69*dt + P(1,7);
	nextP(2,7) = PS21*P(7,11) - PS22*P(7,10) + PS59*P(7,9) + PS84*dt + P(2,7);
	nextP(3,7) = PS2*P(7,12) - PS34*P(1,7) - PS38*P(2,7) - PS5*P(7,14) + PS8*P(7,13) + PS93*dt + P(3,7);
	nextP(4,7) = PS14*P(7,13) + PS19*P(7,14) - PS47*P(2,7) - PS48*P(0,7) - PS54*P(7,12) + PS99*dt + P(4,7);
	nextP(5,7) = PS21*P(7,14) - PS22*P(7,13) - PS51*P(0,7) - PS52*P(1,7) + PS59*P(7,12) + P(5,7) + dt*(PS21*P(4,14) - PS22*P(4,13) - PS51*P(0,4) - PS52*P(1,4) + PS59*P(4,12) + P(4,5));
	nextP(6,7) = P(3,7)*dt + P(6,7) + dt*(P(3,4)*dt + P(4,6));
	nextP(7,7) = P(4,7)*dt + P(7,7) + dt*(P(4,4)*dt + P(4,7));
	nextP(0,8) = PS2*P(8,9) - PS5*P(8,11) + PS53*dt + PS8*P(8,10) + P(0,8);
	nextP(1,8) = PS14*P(8,10) + PS19*P(8,11) - PS54*P(8,9) + PS70*dt + P(1,8);
	nextP(2,8) = PS21*P(8,11) - PS22*P(8,10) + PS59*P(8,9) + PS85*dt + P(2,8);
	nextP(3,8) = PS2*P(8,12) - PS34*P(1,8) - PS38*P(2,8) - PS5*P(8,14) + PS8*P(8,13) + PS94*dt + P(3,8);
	nextP(4,8) = PS100*dt + PS14*P(8,13) + PS19*P(8,14) - PS47*P(2,8) - PS48*P(0,8) - PS54*P(8,12) + P(4,8);
	nextP(5,8) = PS104*dt + PS21*P(8,14) - PS22*P(8,13) - PS51*P(0,8) - PS52*P(1,8) + PS59*P(8,12) + P(5,8);
	nextP(6,8) = P(3,8)*dt + P(6,8) + dt*(P(3,5)*dt + P(5,6));
	nextP(7,8) = P(4,8)*dt + P(7,8) + dt*(P(4,5)*dt + P(5,7));
	nextP(8,8) = P(5,8)*dt + P(8,8) + dt*(P(5,5)*dt + P(5,8));
	nextP(0,9) = PS9;
	nextP(1,9) = PS56;
	nextP(2,9) = PS73;
	nextP(3,9) = PS25 - PS34*P(1,9) - PS38*P(2,9) - PS5*P(9,14) + PS8*P(9,13) + P(3,9);
	nextP(4,9) = PS14*P(9,13) + PS19*P(9,14) - PS47*P(2,9) - PS48*P(0,9) + PS61 + P(4,9);
	nextP(5,9) = PS21*P(9,14) - PS22*P(9,13) - PS51*P(0,9) - PS52*P(1,9) + PS75 + P(5,9);
	nextP(6,9) = P(3,9)*dt + P(6,9);
	nextP(7,9) = P(4,9)*dt + P(7,9);
	nextP(8,9) = P(5,9)*dt + P(8,9);
	nextP(9,9) = P(9,9);
	nextP(0,10) = PS11;
	nextP(1,10) = PS55;
	nextP(2,10) = PS72;
	nextP(3,10) = PS2*P(10,12) - PS34*P(1,10) - PS38*P(2,10) + PS41 - PS5*P(10,14) + P(3,10);
	nextP(4,10) = PS19*P(10,14) - PS47*P(2,10) - PS48*P(0,10) - PS54*P(10,12) + PS65 + P(4,10);
	nextP(5,10) = PS21*P(10,14) - PS51*P(0,10) - PS52*P(1,10) + PS59*P(10,12) + PS80 + P(5,10);
	nextP(6,10) = P(3,10)*dt + P(6,10);
	nextP(7,10) = P(4,10)*dt + P(7,10);
	nextP(8,10) = P(5,10)*dt + P(8,10);
	nextP(9,10) = P(9,10);
	nextP(10,10) = P(10,10);
	nextP(0,11) = PS10;
	nextP(1,11) = PS57;
	nextP(2,11) = PS71;
	nextP(3,11) = PS2*P(11,12) - PS34*P(1,11) - PS38*P(2,11) + PS39 + PS8*P(11,13) + P(3,11);
	nextP(4,11) = PS14*P(11,13) - PS47*P(2,11) - PS48*P(0,11) - PS54*P(11,12) + PS63 + P(4,11);
	nextP(5,11) = -PS22*P(11,13) - PS51*P(0,11) - PS52*P(1,11) + PS59*P(11,12) + PS78 + P(5,11);
	nextP(6,11) = P(3,11)*dt + P(6,11);
	nextP(7,11) = P(4,11)*dt + P(7,11);
	nextP(8,11) = P(5,11)*dt + P(8,11);
	nextP(9,11) = P(9,11);
	nextP(10,11) = P(10,11);
	nextP(11,11) = P(11,11);

    
	 % delta angle noise
	 % R * diag(nAng_x*nAng_x, nAng_y*nAng_y, nAng_z*nAng_z) * R'
	
	mR00 = R_to_earth(0,0); mR01 = R_to_earth(0,1); mR02 = R_to_earth(0,2);
	mR10 = R_to_earth(1,0); mR11 = R_to_earth(1,1); mR12 = R_to_earth(1,2);
	mR20 = R_to_earth(2,0); mR21 = R_to_earth(2,1); mR22 = R_to_earth(2,1);

        r00_sx = mR00 * daxVar;
        r01_sy = mR01 * dayVar;
        r02_sz = mR02 * dazVar;
        r10_sx = mR10 * daxVar;
        r11_sy = mR11 * dayVar;
        r12_sz = mR12 * dazVar;
	r20_sx = mR20 * daxVar;
	r21_sy = mR21 * dayVar;
	r22_sz = mR22 * dazVar;
        r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12 = r00_sx * mR10 + r01_sy * mR11 + r02_sz * mR12;
        r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22 = r00_sx * mR20 + r01_sy * mR21 + r02_sz * mR22;
        r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz = r10_sx * mR20 + r11_sy * mR21 + r12_sz * mR22;

	nextP(0,0) = kahanSummation(nextP(0,0), mR00 * r00_sx + mR01 * r01_sy + mR02 * r02_sz, delta_angle_var_accum(0));
	nextP(0,1) += r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12;
	nextP(0,2) += r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22;
	nextP(1,1) = kahanSummation(nextP(1,1), mR10 * r10_sx + mR11 * r11_sy + mR12 * r12_sz, delta_angle_var_accum(1));
	nextP(1,2) += r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz;
	nextP(2,2) = kahanSummation(nextP(2,2), mR20 * r20_sx + mR21 * r21_sy + mR22 * r22_sz, delta_angle_var_accum(2));

	
	 % delta velocity noise
	 % R * diag(nVel_x, nVel_y, nVel_z) * R'
	
        r00_sx = mR00 * dvxVar;
        r01_sy = mR01 * dvyVar;
        r02_sz = mR02 * dvzVar;
        r10_sx = mR10 * dvxVar;
        r11_sy = mR11 * dvyVar;
        r12_sz = mR12 * dvzVar;
	r20_sx = mR20 * dvxVar;
	r21_sy = mR21 * dvyVar;
	r22_sz = mR22 * dvzVar;
        r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12 = r00_sx * mR10 + r01_sy * mR11 + r02_sz * mR12;
        r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22 = r00_sx * mR20 + r01_sy * mR21 + r02_sz * mR22;
        r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz = r10_sx * mR20 + r11_sy * mR21 + r12_sz * mR22;

	nextP(3,3) = kahanSummation(nextP(3,3), mR00 * r00_sx + mR01 * r01_sy + mR02 * r02_sz, delta_vel_var_accum(0));
	nextP(3,4) += r00_sx_r10_plus_r01_sy_r11_plus_r02_sz_r12;
	nextP(3,5) += r00_sx_r20_plus_r01_sy_r21_plus_r02_sz_r22;
	nextP(4,4) = kahanSummation(nextP(4,4), mR10 * r10_sx + mR11 * r11_sy + mR12 * r12_sz, delta_vel_var_accum(1));
	nextP(4,5) += r10_r20_sx_plus_r11_r21_sy_plus_r12_r22_sz;
	nextP(5,5) = kahanSummation(nextP(5,5), mR20 * r20_sx + mR21 * r21_sy + mR22 * r22_sz, delta_vel_var_accum(2));

	% process noise contribution for delta angle states can be very small compared to
	% the variances, therefore use algorithm to minimise numerical error
	noise_delta_ang_bias = sq(d_ang_bias_sig);
	for i = 9:11 
		index = i - 9;
		nextP(i, i) = kahanSummation(nextP(i, i), noise_delta_ang_bias, delta_angle_bias_var_accum(index));
	end

	noise_delta_vel_bias = sq(d_vel_bias_sig);
	if ~accel_bias_inhibit(0) 
		% calculate variances and upper diagonal covariances for IMU X axis delta velocity bias state
		nextP(0,12) = PS26;
		nextP(1,12) = PS62;
		nextP(2,12) = PS76;
		nextP(3,12) = PS86;
		nextP(4,12) = PS97;
		nextP(5,12) = PS103;
		nextP(6,12) = P(3,12)*dt + P(6,12);
		nextP(7,12) = P(4,12)*dt + P(7,12);
		nextP(8,12) = P(5,12)*dt + P(8,12);
		nextP(9,12) = P(9,12);
		nextP(10,12) = P(10,12);
		nextP(11,12) = P(11,12);
		nextP(12,12) = P(12,12);

		% add process noise that is not from the IMU
		% process noise contribution for delta velocity states can be very small compared to
		% the variances, therefore use algorithm to minimise numerical error
		nextP(12, 12) = kahanSummation(nextP(12, 12), noise_delta_vel_bias, delta_vel_bias_var_accum(0));

	else 
		nextP.uncorrelateCovarianceSetVariance<1>(12, prev_dvel_bias_var(0));
		delta_vel_bias_var_accum(0) = 0;

	end

	if ~accel_bias_inhibit(1) 
		% calculate variances and upper diagonal covariances for IMU Y axis delta velocity bias state
		nextP(0,13) = PS42;
		nextP(1,13) = PS66;
		nextP(2,13) = PS81;
		nextP(3,13) = PS90;
		nextP(4,13) = PS95;
		nextP(5,13) = PS102;
		nextP(6,13) = P(3,13)*dt + P(6,13);
		nextP(7,13) = P(4,13)*dt + P(7,13);
		nextP(8,13) = P(5,13)*dt + P(8,13);
		nextP(9,13) = P(9,13);
		nextP(10,13) = P(10,13);
		nextP(11,13) = P(11,13);
		nextP(12,13) = P(12,13);
		nextP(13,13) = P(13,13);

		% add process noise that is not from the IMU
		% process noise contribution for delta velocity states can be very small compared to
		% the variances, therefore use algorithm to minimise numerical error
		nextP(13, 13) = kahanSummation(nextP(13, 13), noise_delta_vel_bias, delta_vel_bias_var_accum(1));

	else 
		nextP.uncorrelateCovarianceSetVariance<1>(13, prev_dvel_bias_var(1));
		delta_vel_bias_var_accum(1) = 0;

	end

	if ~accel_bias_inhibit(2) 
		% calculate variances and upper diagonal covariances for IMU Z axis delta velocity bias state
		nextP(0,14) = PS40;
		nextP(1,14) = PS64;
		nextP(2,14) = PS79;
		nextP(3,14) = PS89;
		nextP(4,14) = PS98;
		nextP(5,14) = PS101;
		nextP(6,14) = P(3,14)*dt + P(6,14);
		nextP(7,14) = P(4,14)*dt + P(7,14);
		nextP(8,14) = P(5,14)*dt + P(8,14);
		nextP(9,14) = P(9,14);
		nextP(10,14) = P(10,14);
		nextP(11,14) = P(11,14);
		nextP(12,14) = P(12,14);
		nextP(13,14) = P(13,14);
		nextP(14,14) = P(14,14);
		% add process noise that is not from the IMU
		% process noise contribution for delta velocity states can be very small compared to
		% the variances, therefore use algorithm to minimise numerical error
		nextP(14, 14) = kahanSummation(nextP(14, 14), noise_delta_vel_bias, delta_vel_bias_var_accum(2));

	else 
		nextP.uncorrelateCovarianceSetVariance<1>(14, prev_dvel_bias_var(2));
		delta_vel_bias_var_accum(2) = 0;
	end

	% Don't do covariance prediction on magnetic field states unless we are using 3-axis fusion
	if (control_status.flags.mag_3D) 
		% calculate variances and upper diagonal covariances for earth and body magnetic field states

		nextP(0,15) = PS2*P(9,15) - PS5*P(11,15) + PS8*P(10,15) + P(0,15);
		nextP(1,15) = PS14*P(10,15) + PS19*P(11,15) - PS54*P(9,15) + P(1,15);
		nextP(2,15) = PS21*P(11,15) - PS22*P(10,15) + PS59*P(9,15) + P(2,15);
		nextP(3,15) = PS2*P(12,15) - PS34*P(1,15) - PS38*P(2,15) - PS5*P(14,15) + PS8*P(13,15) + P(3,15);
		nextP(4,15) = PS14*P(13,15) + PS19*P(14,15) - PS47*P(2,15) - PS48*P(0,15) - PS54*P(12,15) + P(4,15);
		nextP(5,15) = PS21*P(14,15) - PS22*P(13,15) - PS51*P(0,15) - PS52*P(1,15) + PS59*P(12,15) + P(5,15);
		nextP(6,15) = P(3,15)*dt + P(6,15);
		nextP(7,15) = P(4,15)*dt + P(7,15);
		nextP(8,15) = P(5,15)*dt + P(8,15);
		nextP(9,15) = P(9,15);
		nextP(10,15) = P(10,15);
		nextP(11,15) = P(11,15);
		nextP(12,15) = P(12,15);
		nextP(13,15) = P(13,15);
		nextP(14,15) = P(14,15);
		nextP(15,15) = P(15,15);
		nextP(0,16) = PS2*P(9,16) - PS5*P(11,16) + PS8*P(10,16) + P(0,16);
		nextP(1,16) = PS14*P(10,16) + PS19*P(11,16) - PS54*P(9,16) + P(1,16);
		nextP(2,16) = PS21*P(11,16) - PS22*P(10,16) + PS59*P(9,16) + P(2,16);
		nextP(3,16) = PS2*P(12,16) - PS34*P(1,16) - PS38*P(2,16) - PS5*P(14,16) + PS8*P(13,16) + P(3,16);
		nextP(4,16) = PS14*P(13,16) + PS19*P(14,16) - PS47*P(2,16) - PS48*P(0,16) - PS54*P(12,16) + P(4,16);
		nextP(5,16) = PS21*P(14,16) - PS22*P(13,16) - PS51*P(0,16) - PS52*P(1,16) + PS59*P(12,16) + P(5,16);
		nextP(6,16) = P(3,16)*dt + P(6,16);
		nextP(7,16) = P(4,16)*dt + P(7,16);
		nextP(8,16) = P(5,16)*dt + P(8,16);
		nextP(9,16) = P(9,16);
		nextP(10,16) = P(10,16);
		nextP(11,16) = P(11,16);
		nextP(12,16) = P(12,16);
		nextP(13,16) = P(13,16);
		nextP(14,16) = P(14,16);
		nextP(15,16) = P(15,16);
		nextP(16,16) = P(16,16);
		nextP(0,17) = PS2*P(9,17) - PS5*P(11,17) + PS8*P(10,17) + P(0,17);
		nextP(1,17) = PS14*P(10,17) + PS19*P(11,17) - PS54*P(9,17) + P(1,17);
		nextP(2,17) = PS21*P(11,17) - PS22*P(10,17) + PS59*P(9,17) + P(2,17);
		nextP(3,17) = PS2*P(12,17) - PS34*P(1,17) - PS38*P(2,17) - PS5*P(14,17) + PS8*P(13,17) + P(3,17);
		nextP(4,17) = PS14*P(13,17) + PS19*P(14,17) - PS47*P(2,17) - PS48*P(0,17) - PS54*P(12,17) + P(4,17);
		nextP(5,17) = PS21*P(14,17) - PS22*P(13,17) - PS51*P(0,17) - PS52*P(1,17) + PS59*P(12,17) + P(5,17);
		nextP(6,17) = P(3,17)*dt + P(6,17);
		nextP(7,17) = P(4,17)*dt + P(7,17);
		nextP(8,17) = P(5,17)*dt + P(8,17);
		nextP(9,17) = P(9,17);
		nextP(10,17) = P(10,17);
		nextP(11,17) = P(11,17);
		nextP(12,17) = P(12,17);
		nextP(13,17) = P(13,17);
		nextP(14,17) = P(14,17);
		nextP(15,17) = P(15,17);
		nextP(16,17) = P(16,17);
		nextP(17,17) = P(17,17);
		nextP(0,18) = PS2*P(9,18) - PS5*P(11,18) + PS8*P(10,18) + P(0,18);
		nextP(1,18) = PS14*P(10,18) + PS19*P(11,18) - PS54*P(9,18) + P(1,18);
		nextP(2,18) = PS21*P(11,18) - PS22*P(10,18) + PS59*P(9,18) + P(2,18);
		nextP(3,18) = PS2*P(12,18) - PS34*P(1,18) - PS38*P(2,18) - PS5*P(14,18) + PS8*P(13,18) + P(3,18);
		nextP(4,18) = PS14*P(13,18) + PS19*P(14,18) - PS47*P(2,18) - PS48*P(0,18) - PS54*P(12,18) + P(4,18);
		nextP(5,18) = PS21*P(14,18) - PS22*P(13,18) - PS51*P(0,18) - PS52*P(1,18) + PS59*P(12,18) + P(5,18);
		nextP(6,18) = P(3,18)*dt + P(6,18);
		nextP(7,18) = P(4,18)*dt + P(7,18);
		nextP(8,18) = P(5,18)*dt + P(8,18);
		nextP(9,18) = P(9,18);
		nextP(10,18) = P(10,18);
		nextP(11,18) = P(11,18);
		nextP(12,18) = P(12,18);
		nextP(13,18) = P(13,18);
		nextP(14,18) = P(14,18);
		nextP(15,18) = P(15,18);
		nextP(16,18) = P(16,18);
		nextP(17,18) = P(17,18);
		nextP(18,18) = P(18,18);
		nextP(0,19) = PS2*P(9,19) - PS5*P(11,19) + PS8*P(10,19) + P(0,19);
		nextP(1,19) = PS14*P(10,19) + PS19*P(11,19) - PS54*P(9,19) + P(1,19);
		nextP(2,19) = PS21*P(11,19) - PS22*P(10,19) + PS59*P(9,19) + P(2,19);
		nextP(3,19) = PS2*P(12,19) - PS34*P(1,19) - PS38*P(2,19) - PS5*P(14,19) + PS8*P(13,19) + P(3,19);
		nextP(4,19) = PS14*P(13,19) + PS19*P(14,19) - PS47*P(2,19) - PS48*P(0,19) - PS54*P(12,19) + P(4,19);
		nextP(5,19) = PS21*P(14,19) - PS22*P(13,19) - PS51*P(0,19) - PS52*P(1,19) + PS59*P(12,19) + P(5,19);
		nextP(6,19) = P(3,19)*dt + P(6,19);
		nextP(7,19) = P(4,19)*dt + P(7,19);
		nextP(8,19) = P(5,19)*dt + P(8,19);
		nextP(9,19) = P(9,19);
		nextP(10,19) = P(10,19);
		nextP(11,19) = P(11,19);
		nextP(12,19) = P(12,19);
		nextP(13,19) = P(13,19);
		nextP(14,19) = P(14,19);
		nextP(15,19) = P(15,19);
		nextP(16,19) = P(16,19);
		nextP(17,19) = P(17,19);
		nextP(18,19) = P(18,19);
		nextP(19,19) = P(19,19);
		nextP(0,20) = PS2*P(9,20) - PS5*P(11,20) + PS8*P(10,20) + P(0,20);
		nextP(1,20) = PS14*P(10,20) + PS19*P(11,20) - PS54*P(9,20) + P(1,20);
		nextP(2,20) = PS21*P(11,20) - PS22*P(10,20) + PS59*P(9,20) + P(2,20);
		nextP(3,20) = PS2*P(12,20) - PS34*P(1,20) - PS38*P(2,20) - PS5*P(14,20) + PS8*P(13,20) + P(3,20);
		nextP(4,20) = PS14*P(13,20) + PS19*P(14,20) - PS47*P(2,20) - PS48*P(0,20) - PS54*P(12,20) + P(4,20);
		nextP(5,20) = PS21*P(14,20) - PS22*P(13,20) - PS51*P(0,20) - PS52*P(1,20) + PS59*P(12,20) + P(5,20);
		nextP(6,20) = P(3,20)*dt + P(6,20);
		nextP(7,20) = P(4,20)*dt + P(7,20);
		nextP(8,20) = P(5,20)*dt + P(8,20);
		nextP(9,20) = P(9,20);
		nextP(10,20) = P(10,20);
		nextP(11,20) = P(11,20);
		nextP(12,20) = P(12,20);
		nextP(13,20) = P(13,20);
		nextP(14,20) = P(14,20);
		nextP(15,20) = P(15,20);
		nextP(16,20) = P(16,20);
		nextP(17,20) = P(17,20);
		nextP(18,20) = P(18,20);
		nextP(19,20) = P(19,20);
		nextP(20,20) = P(20,20);

		mag_noise = sq(mag_I_sig);
		mag_bias_noise = sq(mag_B_sig);
		% add process noise that is not from the IMU
		for i = 15: 17 
			nextP(i, i) = nextP(i, i) + mag_noise;
		end
		for i = 18: 20 
			nextP(i, i) = nextP(i, i) + mag_bias_noise;
		end

	end

	% Don't do covariance prediction on wind states unless we are using them
	if (control_status.flags.wind) 

		% calculate variances and upper diagonal covariances for wind states
		nextP(0,21) = PS2*P(9,21) - PS5*P(11,21) + PS8*P(10,21) + P(0,21);
		nextP(1,21) = PS14*P(10,21) + PS19*P(11,21) - PS54*P(9,21) + P(1,21);
		nextP(2,21) = PS21*P(11,21) - PS22*P(10,21) + PS59*P(9,21) + P(2,21);
		nextP(3,21) = PS2*P(12,21) - PS34*P(1,21) - PS38*P(2,21) - PS5*P(14,21) + PS8*P(13,21) + P(3,21);
		nextP(4,21) = PS14*P(13,21) + PS19*P(14,21) - PS47*P(2,21) - PS48*P(0,21) - PS54*P(12,21) + P(4,21);
		nextP(5,21) = PS21*P(14,21) - PS22*P(13,21) - PS51*P(0,21) - PS52*P(1,21) + PS59*P(12,21) + P(5,21);
		nextP(6,21) = P(3,21)*dt + P(6,21);
		nextP(7,21) = P(4,21)*dt + P(7,21);
		nextP(8,21) = P(5,21)*dt + P(8,21);
		nextP(9,21) = P(9,21);
		nextP(10,21) = P(10,21);
		nextP(11,21) = P(11,21);
		nextP(12,21) = P(12,21);
		nextP(13,21) = P(13,21);
		nextP(14,21) = P(14,21);
		nextP(15,21) = P(15,21);
		nextP(16,21) = P(16,21);
		nextP(17,21) = P(17,21);
		nextP(18,21) = P(18,21);
		nextP(19,21) = P(19,21);
		nextP(20,21) = P(20,21);
		nextP(21,21) = P(21,21);
		nextP(0,22) = PS2*P(9,22) - PS5*P(11,22) + PS8*P(10,22) + P(0,22);
		nextP(1,22) = PS14*P(10,22) + PS19*P(11,22) - PS54*P(9,22) + P(1,22);
		nextP(2,22) = PS21*P(11,22) - PS22*P(10,22) + PS59*P(9,22) + P(2,22);
		nextP(3,22) = PS2*P(12,22) - PS34*P(1,22) - PS38*P(2,22) - PS5*P(14,22) + PS8*P(13,22) + P(3,22);
		nextP(4,22) = PS14*P(13,22) + PS19*P(14,22) - PS47*P(2,22) - PS48*P(0,22) - PS54*P(12,22) + P(4,22);
		nextP(5,22) = PS21*P(14,22) - PS22*P(13,22) - PS51*P(0,22) - PS52*P(1,22) + PS59*P(12,22) + P(5,22);
		nextP(6,22) = P(3,22)*dt + P(6,22);
		nextP(7,22) = P(4,22)*dt + P(7,22);
		nextP(8,22) = P(5,22)*dt + P(8,22);
		nextP(9,22) = P(9,22);
		nextP(10,22) = P(10,22);
		nextP(11,22) = P(11,22);
		nextP(12,22) = P(12,22);
		nextP(13,22) = P(13,22);
		nextP(14,22) = P(14,22);
		nextP(15,22) = P(15,22);
		nextP(16,22) = P(16,22);
		nextP(17,22) = P(17,22);
		nextP(18,22) = P(18,22);
		nextP(19,22) = P(19,22);
		nextP(20,22) = P(20,22);
		nextP(21,22) = P(21,22);
		nextP(22,22) = P(22,22);

		noise_wind = sq(wind_vel_sig);
		% add process noise that is not from the IMU
		for i = 21:22 
			nextP(i, i) = nextP(i, i) + noise_wind;
		end

    end
    % stop position covariance growth if our total position variance reaches 100m
	% this can happen if we lose gps for some time
	if ((P(6, 6) + P(7, 7)) > 1e4f) 
		for (uint8_t i = 6; i <= 7; i++) 
			for (uint8_t j = 0; j < k_num_states; j++) 
				nextP(i, j) = P(i, j);
				nextP(j, i) = P(j, i);
			end
		end
	end

	% covariance matrix is symmetrical, so copy upper half to lower half
	for row = 1: k_num_states
		for (unsigned column = 0 ; column < row; column++) 
			P(row, column) = P(column, row) = nextP(column, row);
		end
	end

	% copy variances (diagonals)
	for (unsigned i = 0; i < k_num_states; i++) 
		P(i, i) = nextP(i, i);
	end

	% fix gross errors in the covariance matrix and ensure rows and
	% columns for un-used states are zero
	fixCovarianceErrors(false);

