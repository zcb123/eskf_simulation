function resetHorizontalPositionToLastKnown()

    global params last_known_posNE;
    %information_events.flags.reset_pos_to_last_known = true;
	disp("reset position to last known position");
	% Used when falling back to non-aiding mode of operation
	resetHorizontalPositionTo(last_known_posNE);
    noise = sq(params.pos_noaid_noise);
    uncorrelateCovarianceSetVariance(2,7,[noise noise]')
	

end

