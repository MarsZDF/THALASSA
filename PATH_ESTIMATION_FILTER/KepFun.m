function E = KepFun(M,e)
clear Eb
Eb(1) = M;
i = 1; 
Eb(i+1) = Eb(i) - (Eb(i) - e*sin(Eb(i)) -M)/(1-e*cos(Eb(i)));
i = 2;
if i > 10000
    return
else
    while abs(Eb(i) - Eb(i-1)) > 1e-05
        %Eb(i+1) = Eb(i) + ((M+e*sin(Eb(i)) - Eb(i))/1 - e*cos(Eb(i)));
        Eb(i+1) = Eb(i) - (Eb(i) - e*sin(Eb(i)) -M)/(1-e*cos(Eb(i)));
        i = i + 1;
    end
end
E = Eb(end);