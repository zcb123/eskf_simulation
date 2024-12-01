function ret = isMeasuredMatchingExpected(measured, expected, gate)



    ret = (measured >= expected - gate)...
	       && (measured <= expected + gate);

end


