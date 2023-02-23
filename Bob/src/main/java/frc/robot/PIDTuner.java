//Otters: 3229 Programming SubTeam

package frc.robot;

public class PIDTuner {
    /*
    u is the control signal
    τdead = transportation lag or dead time: the time taken from the moment the disturbance was introduced to the first sign of change in the output signal
    ε is the difference between the current value and the set point
    Kc is the gain for a proportional controller
    τi is the parameter that scales the integral controller
    τd is the parameter that scales the derivative controller
    t is the time taken for error measurement
    b is the set point value of the signal, also known as bias or offset
    ultimate gain is Ku * Kc 
    Ku is found experimentally by starting from a small value of Kc and adjusting upwards until consistent oscillations are obtained. 
    If the gain is too low, the output signal will be damped and attain equilibrium eventually after the disturbance occurs as shown below.
    On the other hand, if the gain is too high, the oscillations become unstable and grow larger and larger with time as shown below.
    Open loop systems typically use the quarter decay ratio (QDR) for oscillation dampening
    This means that the ratio of the amplitudes of the first overshoot to the second overshoot is 4:
    
    Cohen-Coon Method:
    Kc = (P/N*L)*(1.33+(R/4))
    τi = L*(30+3*R)/(9+20*R)
    τd = 4*L/(11+2*R)

    Variables:
    P is the percent change of the outupt
    N is the percent change of output/τ
    L is τdead
    R is τdead/τ
    τdead = transportation lag or dead time: the time taken from the moment the disturbance was introduced to the first sign of change in the output signal












    
    
    
    */
}
