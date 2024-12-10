function res = toAxisAngle(dTheta)

    len = norm(dTheta);
    res = [dTheta(1)/len dTheta(2)/len dTheta(3)/len]';

end

