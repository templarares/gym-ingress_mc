import mc_rtc_rl
from mc_rtc_rl import Configuration, ConfigurationException
"""
convert fsm name to numerical values; initial is 0,righthandtocar is 1, etc...; then normalize it to the obserrvation space
"""
def StateNumber(name):
    stateNumber_=-20
    if (name=="Initial"):
        stateNumber_=0
    elif (name=="IngressFSM::LeftHandToBar"):
        stateNumber_=1
    elif (name=="IngressFSM::LeftHandGrip"):
        stateNumber_=2
    elif (name=="IngressFSM::Grasp"):
        stateNumber_=3
    elif (name=="IngressFSM::RightFootCloseToCarFSM::LiftFoot"):
        stateNumber_=4
    elif (name=="IngressFSM::RightFootCloseToCarFSM::MoveFoot"):
        stateNumber_=5
    elif (name=="IngressFSM::RightFootCloseToCar" 
    or name=="IngressFSM::RightFootCloseToCarFSM::PutFoot" 
    or name=="IngressFSM::RightFootCloseToCarFSM"):
        stateNumber_=6
    elif (name=="IngressFSM::RightFootStepAdmittance"):
        stateNumber_=7    
    elif (name=="IngressFSM::CoMToRightFoot"):
        stateNumber_=8
        """change these state numbers and male sure they are not duplicate when it comes to full ingress"""
    elif (name=="IngressFSM::LandHip"):
        stateNumber_=9
    elif (name=="IngressFSM::LandHipPhase2"):
        stateNumber_=10
    elif (name=="IngressFSM::AdjustCoM"):
        stateNumber_=11
    elif (name=="IngressFSM::PutLeftFoot::LiftFoot"):
        stateNumber_=12
    elif (name=="IngressFSM::PutLeftFoot::MoveFoot"):
        stateNumber_=13
    elif (name=="IngressFSM::PutLeftFoot::PutFoot"
    or name=="IngressFSM::PutLeftFoot"):
        stateNumber_=14
    elif (name=="IngressFSM::PutRightFoot"):
        stateNumber_=15
    elif (name=="IngressFSM::NudgeUp"):
        stateNumber_=16
    elif (name=="IngressFSM::ScootRight"):
        stateNumber_=17
    elif (name=="IngressFSM::SitOnLeft"):
        stateNumber_=18
    elif (name=="IngressFSM::NudgeUpPhase2"):
        stateNumber_=19
    # elif (name=="IngressFSM::CoMToRightFoot"):
    #     stateNumber_=5

    # elif (name=="IngressFSM::PutLeftHand"):
    #     stateNumber_=7
    # elif (name=="IngressFSM::PutLeftFoot"):
    #     stateNumber_=8
    # elif (name=="IngressFSM::PutRightFoot"):
    #     stateNumber_=9
    # elif (name=="IngressFSM::ScootRight"):
    #     stateNumber_=10
    # elif (name=="IngressFSM::ScootRightFoot"):
    #     stateNumber_=11
    # elif (name=="IngressFSM::ScootAdjustHand"):
    #     stateNumber_=12
    # elif (name=="IngressFSM::ScootBody"):
    #     stateNumber_=13
    # elif (name=="IngressFSM::ScootLeftFoot"):
    #     stateNumber_=14
    # elif (name=="IngressFSM::SitPrep"):
    #     stateNumber_=15
    #normalize it to the range [-2,+2]
    #return stateNumber_*0.1
    import numpy as np
    stateVec=np.zeros((20,))
    stateVec[stateNumber_]=1
    return stateVec

def EditTimeout(config,timeout,eval=0.01, speed=0.01):
    OR=config.array("OR")
    EVAL=mc_rtc_rl.Configuration()
    EVAL.add("eval",float(eval))
    OR.push(EVAL)
    ANDconfig=mc_rtc_rl.Configuration()
    AND=ANDconfig.array("AND")
    TIMEOUT=mc_rtc_rl.Configuration()
    TIMEOUT.add("timeout",float(abs(timeout)))
    AND.push(TIMEOUT)
    SPEED=mc_rtc_rl.Configuration()
    SPEED.add("speed",float(speed))
    AND.push(SPEED)
    OR.push(ANDconfig)