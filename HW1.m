function TF_with_feedback
    
    s = tf('s');
    G = (0.9) / (s^2-0.03);
    K = 0.44;
    a = 0.392432;
    b= 0;    
    
    Kp = 0.17;
    Ki = 0;
    Kd = 0.44;

    C = Kp + Ki/s + Kd*s;
    
    Gc = (s + a)*(s + b)/(s);
    GPID = G*Gc;
    GPID = GPID*K;
    Gclosed =feedback(GPID,1);
    figure;
    rlocus(Gclosed);
    figure
    step(Gclosed);
    
    
   
end
