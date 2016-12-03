from __future__ import division
import seawolf
import math
import time

pulse_cap = 255.0
init_downward_force = .1
ACTIVE_REGION_SIZE = 2 #ft

# TODO: code thruster shutdown if robot ever goes upside down.

def initial_e_dt(integral):
    if(math.fabs(integral) < .00001):
        return 0
    else:
        return init_downward_force / integral

def dataOut(mv):
    out = in_range(0, mv, 255)

    seawolf.notify.send("THRUSTER_REQUEST", "PumpPWM {}".format(out))

def in_range(a,x,b):
    if( x < a ):
        return a
    elif( x > b ):
        return b
    else:
        return x 

def main():
    seawolf.loadConfig("../conf/seawolf.conf")
    seawolf.init("Pulse PID")

    seawolf.var.subscribe("PulsePID.p")
    seawolf.var.subscribe("PulsePID.i")
    seawolf.var.subscribe("PulsePID.d")
    seawolf.var.subscribe("PulsePID.Heading")
    seawolf.var.subscribe("PulsePID.Paused")
    seawolf.var.subscribe("PumpPWM")
    seawolf.var.subscribe("FlowPulses")

    pwm = seawolf.var.get("PumpPWM")
    pulses = seawolf.var.get("FlowPulses")
    paused = seawolf.var.get("PulsePID.Paused")

    pid = seawolf.PID( seawolf.var.get("PulsePID.Heading"), seawolf.var.get("PulsePID.p"), seawolf.var.get("PulsePID.i"), seawolf.var.get("PulsePID.d"))

    e_dt = initial_e_dt( seawolf.var.get("PulsePID.i") )
    
    # set active region (region where response of the robot
    # is practically linear). Outside this region, thrusters
    # would be maxed out, and the ITerm would get staturated.
    # Outside this region, the we use PD control. Inside this
    # region, we use PID control.
    pid.setActiveRegion(ACTIVE_REGION_SIZE)
    
    dataOut(0.0)

    while(True):
        seawolf.var.sync()

        if(seawolf.var.stale("PumpPWM")):
            pwm = seawolf.var.get("PumpPWM")
        
        if(seawolf.var.stale("FlowPulses")):
            pulses = seawolf.var.get("FlowPulses")
			
        if(seawolf.var.stale("PulsePID.p") or seawolf.var.stale("PulsePID.i") or seawolf.var.stale("PulsePID.d")):
            pid.setCoefficients(seawolf.var.get("PulsePID.p"), seawolf.var.get("PulsePID.i"), seawolf.var.get("PulsePID.d"))

            e_dt = initial_e_dt( seawolf.var.get("PulsePID.i") )

        if(seawolf.var.poked("PulsePID.Heading")):
            pid.setSetPoint(seawolf.var.get("PulsePID.Heading"))
            if(paused):
                seawolf.var.set("PulsePID.Paused", 0.0)
                seawolf.var.set("PitchPID.Paused", 0.0)

        if(seawolf.var.stale("PulsePID.Paused")):
            paused = seawolf.var.get("PulsePID.Paused")
            if(paused):
                dataOut(0.0)
                seawolf.notify.send("PIDPAUSED", "Pulse")
                pid.pause()
                e_dt = initial_e_dt( seawolf.var.get("PulsePID.i") )
       
        elif(paused == False):
            mv = pid.update(Pulse)
            mv = in_range(0, mv, pulse_cap)
            dataOut(mv)

    seawolf.close();

if __name__ == "__main__":
    main()
