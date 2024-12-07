function resetVerticalVelocityToGps(gps_sample_delayed)

    resetVerticalVelocityTo(gps_sample_delayed.vel(3));

	%the state variance is the same as the observation
	uncorrelateCovarianceSetVariance(1,6, sq(1.5 * gps_sample_delayed.sacc));
    
end