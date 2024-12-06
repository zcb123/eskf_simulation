function resetVerticalVelocityToZero()

    
    resetVerticalVelocityTo(0.0);

	% Set the variance to a value large enough to allow the state to converge quickly
	% that does not destabilise the filter
	uncorrelateCovarianceSetVariance(1,6,10);


end


