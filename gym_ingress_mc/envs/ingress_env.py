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

def do_nothing(name,controller):
    pass

def done_callback(name, controller):
    #print("{} done, robot configuration: {}".format(name, controller.robot().q))
    pass
def start_callback(action, name, controller):
    print("{} starting to run".format(name))
    if (name=="IngressFSM::RightFootCloseToCar::LiftFoot"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        com.add("name", "CoMgg")
        return config    
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
    metadata = {'render.modes': ['human']}
    def __init__(self):
        #self.low=np.array([-1,-1,-1],dtype=np.float32)
        #self.high=np.array([1,1,1],dtype=np.float32)

        self.action_space=spaces.Box(low=-1.0, high=2.0, shape=(3, 4))
        "current fsm state"
        #self.currentFSMState = 
        "observation space--need defination"
        self.observation_space=spaces.Box(low=-1.0, high=2.0, shape=(3, 4))

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
        startTime=time.time()
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
        endTime = time.time()
        # print("execution of this state is %f"%(endTime-startTime))
        # print("Current state is: %s"%currentState)
        # print("Current position of LeftFoot is:", self.gc.EF_trans("LeftFoot"))
        # print("Current orientation of LeftFoot is:", self.gc.EF_rot("LeftFoot"))
        #return observation, reward, done and info

        #observation: location of COM, pose of four EFs
        #reward: for grasping state, reward = inverse(distance between ef and bar)-time elapsed+stateDone, using the function from minDist.py
        #done: 
        #info: {}
        done = False
        "for completing a state, the reward is 10 by default; if the controller failed, the punishment is -100"
        if (self.gc.running ):
            reward = 10
        else:
            reward = -100
            done = True

        "if last state is done,done is True and reward+=100;also some states are more rewarding than others"
        if (currentState=="IngressFSM::SitPrep"):
            reward += 100
            done = True
        elif (currentState=="IngressFSM::RightFootCloseToCar"):
            reward +=10
        elif (currentState=="IngressFSM::AdjustCoM"):
            reward += 20        

        #reward function. currently for the gripping only;
        return [0,0,0],reward,done,{}

    
    def reset(self):
        #self.gc = mc_rtc_rl.GlobalController('/home/templarares/.config/mc_rtc/mc_rtc.yaml')       
        #self.gc.reset()
        print("resetting gc...")
        self.gc.reset()
        #self.gc.init()
        pass
    def render (self, moder='human'):
        pass
    def close(self):
        pass