"""
convert fsm name to numerical values; initial is 0,righthandtocar is 1, etc...; then normalize it to the obserrvation space
"""
def stateNumber(name):
    stateNumber_=-1
    if (name=="Initial"):
        stateNumber_=0
    elif (name=="IngressFSM::RightHandToSeat"):
        stateNumber_=1
    elif (name=="IngressFSM::RightFootCloseToCarFSM::LiftFoot"):
        stateNumber_=2
    elif (name=="IngressFSM::RightFootCloseToCarFSM::MoveFoot"):
        stateNumber_=3
    elif (name=="IngressFSM::RightFootCloseToCar"):
        stateNumber_=4
    elif (name=="IngressFSM::CoMToRightFoot"):
        stateNumber_=5
    elif (name=="IngressFSM::AdjustCoM"):
        stateNumber_=6
    elif (name=="IngressFSM::PutLeftHand"):
        stateNumber_=7
    elif (name=="IngressFSM::PutLeftFoot"):
        stateNumber_=8
    elif (name=="IngressFSM::PutRightFoot"):
        stateNumber_=9
    elif (name=="IngressFSM::ScootRight"):
        stateNumber_=10
    elif (name=="IngressFSM::ScootRightFoot"):
        stateNumber_=11
    elif (name=="IngressFSM::ScootAdjustHand"):
        stateNumber_=12
    elif (name=="IngressFSM::ScootBody"):
        stateNumber_=13
    elif (name=="IngressFSM::ScootLeftFoot"):
        stateNumber_=14
    elif (name=="IngressFSM::SitPrep"):
        stateNumber_=15
    #normalize it to the range [-2,+2]
    return stateNumber_*0.1