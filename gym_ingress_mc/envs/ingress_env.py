from functools import partial
import sys
import time
from numpy.core.fromnumeric import shape
sys.path.append('/home/templarares/devel/src/bit-car-inout-controller/python/')
import mc_rtc_rl
from mc_rtc_rl import Configuration, ConfigurationException
import numpy as np
import gym
from gym import error, spaces, utils
from gym.spaces import Box
from gym.utils import seeding
from gym_ingress_mc.envs import helper

def do_nothing(name,controller):
    pass

def done_callback(name, controller):
    #print("{} done, robot configuration: {}".format(name, controller.robot().q))
    pass
def start_callback(action, name, controller):
    #print("{} starting to run".format(name))
    if (
        name=="IngressFSM::RightFootCloseToCarFSM::LiftFoot" 
     ):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        com.add("weight",int(2000*(abs(action[0]))))
        right_foot=tasks.add("right_foot")
        target=right_foot.add("target")
        target.add_array("rotation",np.array(action[1:4]*0.2))
        target.add_array("translation",np.array(action[4:7]*0.2+[0.171598,0.845943,0.469149]))
        right_foot.add("weight",int(2000*(abs(action[8]))))
        Completion1=right_foot.add("completion")
        helper.EditTimeout(Completion1,action[9])
        Completion2=com.add("completion")
        helper.EditTimeout(Completion2,action[9])
        return config
    elif(
        name=="IngressFSM::RightFootCloseToCarFSM::MoveFoot" or
        name=="IngressFSM::RightFootCloseToCarFSM::PutFoot"
        ):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        com.add("weight",int(2000*(abs(action[0]))))
        right_foot=tasks.add("right_foot")
        target=right_foot.add("target")
        #target.add_array("rotation",np.array(action[1:4]))
        target.add_array("rotation",np.array([0,0,abs(action[3]*0.6)]))
        target.add_array("translation",np.array(action[4:7]*0.2+[0.350024, 0.402364, 0.401191]))
        right_foot.add("weight",int(2000*(abs(action[8]))))
        Completion1=right_foot.add("completion")
        helper.EditTimeout(Completion1,action[9])
        return config

    elif (name=="IngressFSM::CoMToRightFoot"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        right_hip=tasks.add("right_hip")
        target=right_hip.add("target")
        target.add_array("position",np.array(action[0:3]*0.1+[-0.12,0.57,0.9]))
        right_hip.add("weight",int(2000*(abs(action[8]))))
        Completion1=right_hip.add("completion")
        helper.EditTimeout(Completion1,action[9])
        return config
    elif (name == "IngressFSM::AdjustCoM"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        com.add_array("com",np.array(action[0:3]*0.1+[0.25, 0.1,0.95]))
        com.add("weight",int(2000*(abs(action[8]))))
        Completion1=com.add("completion")
        helper.EditTimeout(Completion1,action[9])

    # add custom codes here. Remove all entries but the "base:" one. Enter them here.
    return mc_rtc_rl.Configuration.from_string("{}")
class IngressEnv(gym.Env):
    "THE MC_RTC global controller"
    #gc = mc_rtc_rl.GlobalController('/home/templarares/.config/mc_rtc/mc_rtc.yaml')
    # @property
    # def action_space(self):
    #     # Do some code here to calculate the available actions
    #     "check current fsm state and modify the action space"
    #     return 1
    gc=1
    "for demonstration purpose, no randomization at initial pose for the 1st episode"
    isFirstEpisode=True
    metadata = {'render.modes': ['human']}
    def __init__(self):
        #self.low=np.array([-1,-1,-1],dtype=np.float32)
        #self.high=np.array([1,1,1],dtype=np.float32)

        self.action_space=spaces.Box(low=-1.0, high=1.0, shape=(12, ),dtype=np.float32)
        "current fsm state"
        #self.currentFSMState = 
        "observation space--need defination"
        self.observation_space=spaces.Box(low=-2.0, high=2.0, shape=(32, ),dtype=np.float32)

        #self.observation_space=
        #self.reset()
        self.gc = mc_rtc_rl.GlobalController('mc_rtc.yaml')
        self.gc.init()
        pass
    def step(self, action):
        "update the fsm state that is immediately to run"
        "this is the preceding state"
        prevState = self.gc.currentState()
        "timer to count execution time of this step"
        "when step() is called for the first time, the next reset() will not be the 1st episode"
        self.isFirstEpisode=False
        #startTime=time.time()
        if (self.gc.currentState()=="IngressFSM::RightFootCloseToCar::LiftFoot"):
            pass
        self.gc.set_rlinterface_done_cb(do_nothing)
        self.gc.set_rlinterface_start_cb(partial(start_callback, action))
        #self.gc.set_rlinterface_start_cb(lambda name, controller: start_callback(action, name, controller))
        #self.currentFSMState = 
        "check if action is in current avaialbe action space"
        #assert ...
        "determine how to modify the mc_rtc config based on current fsm state and action"
        #{}
        "call the reward function"

        "advance mc_rtc until the next fsm state or gc failure"
        #while self.gc.running:
        "if previous state is done, proceed to next state by calling the fsm's next()"
        if (self.gc.ready()):
            self.gc.nextState()
            self.gc.run()
        "fsm state\'s teardown() is immediate followed by next state\'s start()"
        "so use the fsm executor's ready() to check stateDone and forget about done_cb"        
        #while (self.gc.running):        
        while (self.gc.running and (not self.gc.ready())):            
            self.gc.run()
        #now the fsm is ready to proceed to the next state
        "this is the state that has just finished execution"
        currentState = self.gc.currentState()
        "timer to keep track of execution of one step"
        #endTime = time.time()
        # print("execution of this state is %f"%(endTime-startTime))
        # print("Current state is: %s"%currentState)
        # print("Current position of LeftFoot is:", self.gc.EF_trans("LeftFoot"))
        # print("Current orientation of LeftFoot is:", self.gc.EF_rot("LeftFoot"))
        #return observation, reward, done and info

        #observation: location of COM, pose of four EFs, and state number
        LHpose=np.concatenate([self.gc.EF_rot("LeftHand"),self.gc.EF_trans("LeftHand")])
        RHpose=np.concatenate([self.gc.EF_rot("RightHand"),self.gc.EF_trans("RightHand")])
        LFpose=np.concatenate([self.gc.EF_rot("LeftFoot"),self.gc.EF_trans("LeftFoot")])
        RFpose=np.concatenate([self.gc.EF_rot("RightFoot"),self.gc.EF_trans("RightFoot")])
        com=self.gc.com()
        stateNumber=np.concatenate([[helper.StateNumber(name=currentState)],[]])
        observationd=np.concatenate([LHpose,RHpose,LFpose,RFpose,com,stateNumber])
        observation = observationd.astype(np.float32)
        #reward: for grasping state, reward = inverse(distance between ef and bar)-time elapsed+stateDone, using the function from minDist.py
        #done: 
        #info: {}
        done = False
        "for completing a state, the reward is 10 by default; if the controller failed, the punishment is -100"
        if (self.gc.running ):
            reward = 30
        else:
            reward = -200
            done = True
        "negative reward for time elapsed"
        reward-=self.gc.duration()*10.0

        "if last state is done,done is True and reward+=100;also some states are more rewarding than others"
        if (currentState=="IngressFSM::SitPrep"):
            reward += 500
            done = True
        elif (currentState=="IngressFSM::RightFootCloseToCar"):
            reward +=10
        elif (currentState=="IngressFSM::AdjustCoM"):
            reward += 20     
        elif (currentState=="IngressFSM::PutLeftFoot"):
            reward += 50           

        #reward function. currently for the gripping only;
        return observation,float(reward),done,{}

    
    def reset(self):
        #self.gc = mc_rtc_rl.GlobalController('/home/templarares/.config/mc_rtc/mc_rtc.yaml')       
        #self.gc.reset()
        #print("resetting gc...")
        "for demonstration purpose, no randomization at initial pose for the 1st episode"
        if (self.isFirstEpisode):
            self.gc.reset()
        else:
            self.gc.reset_random()
        LHpose=np.concatenate([self.gc.EF_rot("LeftHand"),self.gc.EF_trans("LeftHand")])
        RHpose=np.concatenate([self.gc.EF_rot("RightHand"),self.gc.EF_trans("RightHand")])
        LFpose=np.concatenate([self.gc.EF_rot("LeftFoot"),self.gc.EF_trans("LeftFoot")])
        RFpose=np.concatenate([self.gc.EF_rot("RightFoot"),self.gc.EF_trans("RightFoot")])
        com=self.gc.com()
        observationd=np.concatenate([LHpose,RHpose,LFpose,RFpose,com,[-1.0]])
        observation = observationd.astype(np.float32)
        #self.gc.init()
        return observation
    def render (self, moder='human'):
        pass
    def close(self):
        pass