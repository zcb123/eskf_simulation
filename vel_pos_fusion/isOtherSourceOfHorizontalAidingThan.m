function ret = isOtherSourceOfHorizontalAidingThan(aiding_flag)


    nb_sources = getNumberOfActiveHorizontalAidingSources();

    if aiding_flag
        ret = nb_sources>1;
    else
        ret = nb_sources>0;
    end


end
