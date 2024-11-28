function ret = ahrsPredictRotMat(R,g)
    
    global FLT_EPSILON;
    ret = R;
    ret(1,1) = ret(1,1) + R(1,2) * g(3) - R(1,3) * g(2);
	ret(1,2) = ret(1,2)+ R(1,3) * g(1) - R(1,1) * g(3);
	ret(1,3) = ret(1,3)+ R(1,1) * g(2) - R(1,2) * g(1);
	ret(2,1) = ret(2,1) + R(2,2) * g(3) - R(2,3) * g(2);
	ret(2,2) = ret(2,2) +R(2,3) * g(1) - R(2,1) * g(3);
	ret(2,3) = ret(2,3) +R(2,1) * g(2) - R(2,2) * g(1);
	ret(3,1) = ret(3,1) + R(3,2) * g(3) - R(3,3) * g(2);
	ret(3,2) = ret(3,2) + R(3,3) * g(1) - R(3,1) * g(3);
	ret(3,3) = ret(3,3) + R(3,1) * g(2) - R(3,2) * g(1);

    %这里将g转化成反对称矩阵，R+R*[g]x
    for r = 1:3
		rowLengthSq = norm(ret(r,:))^2;

		if (rowLengthSq > FLT_EPSILON) 
			% Use linear approximation for inverse sqrt taking advantage of the row length being close to 1.0
			rowLengthInv = 1.5 - 0.5 * rowLengthSq;
			ret(r,:) = ret(r,:) * rowLengthInv;
        end
	end
end