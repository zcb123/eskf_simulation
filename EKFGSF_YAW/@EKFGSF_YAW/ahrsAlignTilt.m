function obj = ahrsAlignTilt(obj)
 	down_in_bf = -obj.delta_vel/norm(obj.delta_vel);        %delta_vel归一化

	% Calculate earth frame North axis unit vector rotated into body frame, orthogonal to 'down_in_bf'
	i_vec_bf = [1, 0, 0]';

	north_in_bf = i_vec_bf - down_in_bf * (sum(i_vec_bf.*(down_in_bf)));
	north_in_bf = north_in_bf/norm(north_in_bf);

	% Calculate earth frame East axis unit vector rotated into body frame, orthogonal to 'down_in_bf' and 'north_in_bf'
	east_in_bf = cross(down_in_bf, north_in_bf);

	% Each column in a rotation matrix from earth frame to body frame represents the projection of the
	% corresponding earth frame unit vector rotated into the body frame, eg 'north_in_bf' would be the first column.
	% We need the rotation matrix from body frame to earth frame so the earth frame unit vectors rotated into body
	% frame are copied into corresponding rows instead.
    % 从地球坐标系到身体坐标系的旋转矩阵中的每一列都表示旋转到身体坐标中的相应地球坐标系单位向量的投影，例如'north_in_bf'将是第一列。
	R = zeros(3,3);
    R(1,:) = north_in_bf;
    R(2,:) = east_in_bf;
    R(3,:) = down_in_bf;
    
% 	R.setRow(1, north_in_bf);
% 	R.setRow(2, east_in_bf);
% 	R.setRow(3, down_in_bf);

	for model_index = 1:5
		obj.ahrs_ekf_gsf(model_index,1).R = R;
    end

end