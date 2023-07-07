from ast import Not
from cmath import sqrt
from functools import partial
from genericpath import isfile
import sys
from tabnanny import verbose
import time
from numpy.core.fromnumeric import shape
sys.path.append('/home/templarares/devel/src/JumpingControllerRy/python/')
import mc_rtc_rl
from mc_rtc_rl import Configuration, ConfigurationException
import numpy as np
import gym
from gym import error, spaces, utils
from gym.spaces import Box
from gym.utils import seeding
from gym_ingress_mc.envs import helper
import threading
#from gym_ingress_mc.envs import minDist

def lineseg_dist(p, a, b):
    """helper function that fins min dist from point p to a line segment [a,b]"""

    # normalized tangent vector
    d = np.divide(b - a, np.linalg.norm(b - a))

    # signed parallel distance components
    s = np.dot(a - p, d)
    t = np.dot(p - b, d)

    # clamped parallel distance
    h = np.maximum.reduce([s, t, 0])

    # perpendicular distance component
    c = np.cross(p - a, d)

    return np.hypot(h, np.linalg.norm(c))


def do_nothing(name,controller):
    pass

def done_callback(name, controller):
    #print("{} done, robot configuration: {}".format(name, controller.robot().q))
    pass
def null_callback(action, name, controller):
    return mc_rtc_rl.Configuration.from_string("{}")
def start_callback(action, name, controller):
    # print("{} starting to run".format(name))
    if (
        name=="PrepareJump" 
     ):
        config = mc_rtc_rl.Configuration()
        configs = config.add("configs")
        PrepareJumpCoM = configs.add("PrepareJumpCoM")
        OrientationWaist=configs.add("OrientationWaist")
        OrientationArmLeft = configs.add("OrientationArmLeft")
        OrientationArmRight = configs.add("OrientationArmRight")
        PrepareJumpCoM.add("tasks").add("CoM").add_array("move_com",np.array([0,0,-0.15+action[0]*0.2]))
        OrientationArmLeft.add("tasks").add("OrientationArmLeft").add_array("orientation",np.array([0,1.4+action[1]*0.7,0]))
        OrientationArmRight.add("tasks").add("OrientationArmRight").add_array("orientation",np.array([0,1.4+action[1]*0.7,0]))
        OrientationWaist.add("tasks").add("OrientationWaist").add_array("orientation",np.array([0,0.7+action[2]*0.7,0]))
        # right_foot=configs.add("right_foot")
        # target=right_foot.add("target")
        # target.add_array("rotation",np.array(action[1:4]*0.2))
        # target.add_array("translation",np.array(action[4:7]*0.1+[0.261598,0.845943,0.469149]))
        #right_foot.add("weight",int(2000*(abs(action[8]))))
        #Completion1=right_foot.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        #Completion2=com.add("completion")
        #helper.EditTimeout(Completion2,action[9])
        return config
    elif(name=="JumpMomentum"):
        config = mc_rtc_rl.Configuration()
        config.add("trigger",120+42*action[0])
        config.add("torsoOrient",60+42*action[1])
        task = config.add("task")
        task.add_array("momentum",np.array([0,0,0,0,0,120+42*action[0]+42*np.absolute(action[2])]))
        return config
    elif(name=="PrepareLanding"):
        config = mc_rtc_rl.Configuration()
        config.add("feetOrient",0+21*action[0])
        config.add("torsoOrient",0+21*(2-action[1]))
        config.add("offsetZ",0.5+0.21*action[2])
        config.add("offsetX",-0.01+0.021*action[3])
        return config
    elif(name=="DampingLanding"):
        config = mc_rtc_rl.Configuration()  
        config.add_array("CoM",[0+0.021*action[0],0,0.5+0.21*action[1]])     
        config.add("torsoOrient",0.4+0.21*action[2])
        # config.add("momentumWeight",0.1+action[3])
        config.add("momentumStiff",1+(2-action[3]))        
        return config

     #     #com = tasks.add("com")
    #     #com.add("weight",int(2000*(abs(action[0]))))
    #     left_hand = tasks.add("left_hand")
    #     #left_hand.add("weight",int(2000*(abs(action[0]))))
    #     target=left_hand.add("target")
    #     #action=np.array([ 0.44949245, -0.23903632, -0.0928756,  -0.7057361,  -0.29943895, -0.3467496,  0.29004097,  0.0811981 ])
    #     target.add_array("rotation",np.concatenate([[0],action[1:4]*0.1])+[-0.0768043,0.86447,0.423181,0.260213])
    #     target.add_array("translation",np.array(action[4:7]*0.02+[0.533733,0.69135,1.62638]))        
    #     #left_hand.add("weight",int(2000*(abs(action[8]))))
    #     # Completion1=left_hand.add("completion")
    #     # helper.EditTimeout(Completion1,action[9])
    #     return config
    # elif(name == "IngressFSM::LeftHandGrip"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com = tasks.add("com")
    #     #com.add("weight",int(2000*(abs(action[0]))))
    #     left_hand = tasks.add("left_hand")
    #     #left_hand.add("weight",int(2000*(abs(action[0]))))
    #     target=left_hand.add("target")
    #     #action=np.array([ 0.44949245, -0.23903632, -0.0928756,  -0.7057361,  -0.29943895, -0.3467496,  0.29004097,  0.0811981 ])
    #     target.add_array("rotation",np.concatenate([[0],action[1:4]*0.1])+[-0.106661,0.873789,0.416487,0.227276])
    #     target.add_array("translation",np.array(action[4:7]*0.02+[0.482899,0.623379,1.58913]))        
    #     #left_hand.add("weight",int(2000*(abs(action[8]))))
    #     # Completion1=left_hand.add("completion")
    #     # helper.EditTimeout(Completion1,action[9])
    #     return config
    # elif(
    #     name=="IngressFSM::RightFootCloseToCarFSM::MoveFoot" or
    #     name=="IngressFSM::RightFootCloseToCarFSM::PutFoot"
    #     ):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com = tasks.add("com")
    #     #com.add("weight",int(2000*(abs(action[0]))))
    #     right_foot=tasks.add("right_foot")
    #     target=right_foot.add("target")
    #     #target.add_array("rotation",np.array(action[1:4]))
    #     target.add_array("rotation",np.array([0,0,abs(action[3]*0.6)]))
    #     target.add_array("translation",np.array(action[4:7]*0.1+[0.380024, 0.20364, 0.421191]))
    #     #right_foot.add("weight",int(2000*(abs(action[8]))))
    #     #Completion1=right_foot.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     return config

    # elif (name=="IngressFSM::CoMToRightFoot"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com=tasks.add("com")
    #     com.add_array("move_com",np.array(action[1:4]*0.1+[-0.05,-0.20,0.0]))
    #     #com.add("weight",int(2000*(abs(action[0]))))
    #     #Completion1=com.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     return config
    # elif (name=="IngressFSM::LandHip"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com = tasks.add("com")
    #     #com.add("weight",int(2000*(abs(action[0]))))
    #     right_hip=tasks.add("right_hip")
    #     right_hip.add_array("position",action[1:4]*0.5+[-0.254778,0.845367,0.819587])
    #     #target.add_array("rotation",np.array(action[1:4]))
    #     right_hip_ori=tasks.add("right_hip_ori")
    #     right_hip_ori.add_array("orientation",np.concatenate([[0],action[4:7]*0.5])+[0.370327,0.671353,0.150604,0.624068])
    #     #right_hip.add("weight",int(2000*(abs(action[8]))))
    #     #right_hip_ori.add("weight",int(2000*(abs(action[8]))))
    #     #Completion1=right_hip.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     #Completion2=right_hip_ori.add("completion")
    #     #helper.EditTimeout(Completion2,action[9])
    #     return config
    # elif (name=="IngressFSM::LandHipPhase2"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     #com.add("weight",int(2000*(abs(action[0]))))
    #     right_hip=tasks.add("RightHipRootAdmittance")
    #     right_hip.add_array("stiffness",[1.0,1.0,10.0,10.0,10.0,50+action[1]*10.0])
    #     right_hip.add_array("damping", [6.3, 6.3, 6.3, 6.3, 6.3, 8.1+action[2]*0.5])
    #     right_hip.add_array("admittance", [0.0,0.0,0,0,0,0.009+0.002*action[3]])
    #     #target.add_array("rotation",np.array(action[1:4]))
    #     right_hip_ori=tasks.add("right_hip_ori")
    #     right_hip_ori.add_array("orientation",np.concatenate([[0],action[4:7]*0.5])+[0.235011,0.693461,0.286154,0.618059])
    #     #right_hip.add("weight",int(2000*(abs(action[8]))))
    #     #right_hip_ori.add("weight",int(2000*(abs(action[8]))))
    #     #Completion1=right_hip.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     #Completion2=right_hip_ori.add("completion")
    #     #helper.EditTimeout(Completion2,action[9])
    #     return config
    # elif (name == "IngressFSM::AdjustCoM"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com = tasks.add("com")
    #     com.add_array("com",np.array(action[1:4]*0.5+[0.0642257,-0.336135,0.916262]))
    #     #com.add("weight",int(2000*(abs(action[8]))))
    #     #Completion1=com.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    # elif (name=="IngressFSM::PutLeftFoot::LiftFoot"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com=tasks.add("com")
    #     com.add_array("move_com",np.array(action[1:4]*0.2+[-0.1,-0.6,0.0]))
    #     #com.add("weight",int(1000*(abs(action[0]))))
    #     left_foot=tasks.add("left_foot")
    #     target=left_foot.add("target")
    #     target.add_array("translation",np.array(action[4:7]*0.1+[0.311598, 0.90664, 0.451191]))
    #     #left_foot.add("weight",int(2000*(abs(action[8]))))
    #     #Completion1=com.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     #Completion2=left_foot.add("completion")
    #     #helper.EditTimeout(Completion2,action[9])
    #     return config
    # elif (name=="IngressFSM::PutLeftFoot::MoveFoot"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com=tasks.add("com")
    #     com.add_array("move_com",np.array(action[1:4]*0.1))
    #     #com.add("weight",int(1000*(abs(action[0]))))
    #     left_foot=tasks.add("left_foot")
    #     target=left_foot.add("target")
    #     target.add_array("translation",np.array(action[4:7]*0.1+[0.34, 0.703, 0.421191]))
    #     #left_foot.add("weight",int(2000*(abs(action[8]))))
    #     #Completion1=com.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     #Completion2=left_foot.add("completion")
    #     #helper.EditTimeout(Completion2,action[9])
    #     return config
    # elif (name=="IngressFSM::PutLeftFoot::PutFoot"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com=tasks.add("com")
    #     com.add_array("move_com",np.array(action[1:4]*0.1))
    #     #com.add("weight",int(1000*(abs(action[0]))))
    #     left_foot=tasks.add("left_foot")
    #     target=left_foot.add("target")
    #     target.add_array("translation",np.array(action[4:7]*0.1+[0.326003,0.63986,0.4065]))
    #     #left_foot.add("weight",int(2000*(abs(action[8]))))
    #     #Completion1=com.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     #Completion2=left_foot.add("completion")
    #     #helper.EditTimeout(Completion2,action[9])
    #     return config
    # elif (name=="IngressFSM::NudgeUp"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     waist=tasks.add("waist_pos")
    #     waist.add_array("position",np.array(action[1:4]*0.1)+[0.1173735,0.550307,1.26874])
    #     #waist.add("weight",int(1000*(abs(action[0]))))
    #     #Completion1=waist.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     return config
    # elif (name=="IngressFSM::NudgeUpPhase2"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     waist=tasks.add("waist_pos")
    #     waist.add_array("position",np.array(action[1:4]*0.15)+[0.0573735,0.250307,1.26874])
    #     return config
    # elif (name=="IngressFSM::ScootRight"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     com=tasks.add("com")
    #     com.add_array("com",np.array(action[1:4]*0.15)+[-0.19, 0.19,0.98])
    #     #com.add("weight",int(1000*(abs(action[0]))))
    #     body_pos=tasks.add("body_pos")
    #     body_pos.add_array("position",np.array(action[4:7]*0.15)+[-0.114,0.13,1.26])
    #     body_orie=tasks.add("body_orie")
    #     body_orie.add_array("orientation",np.concatenate([[0],action[7:8]*0.10,[0]])+[0,0.15,0])
    #     #Completion1=com.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     #Completion2=body_pos.add("completion")
    #     #helper.EditTimeout(Completion2,action[9])
    #     return config
    # elif (name=="IngressFSM::SitOnLeft"):
    #     config = mc_rtc_rl.Configuration()
    #     tasks = config.add("tasks")
    #     left_hip=tasks.add("left_hip")
    #     left_hip.add_array("position",np.array(action[4:7]*0.2)+[-0.109182, 0.406125,0.81])
    #     #Completion1=left_hip.add("completion")
    #     #helper.EditTimeout(Completion1,action[9])
    #     return config
    

    # add custom codes here. Remove all entries but the "base:" one. Enter them here.
    return mc_rtc_rl.Configuration.from_string("{}")

class IngressEnvExtensive(gym.Env):
    """A simplified version of IngressEnv, where only targets are adjusted from the default IngressFSM"""
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
    def __init__(self,visualization: bool = False, verbose: bool=False,userl: bool=True):
        #self.low=np.array([-1,-1,-1],dtype=np.float32)
        #self.high=np.array([1,1,1],dtype=np.float32)

        self.action_space=spaces.Box(low=-1.0, high=1.0, shape=(8, ),dtype=np.float32)
        "current fsm state"
        #self.currentFSMState = 
        "observation space--need defination"
        self.observation_space=spaces.Box(low=-10.0, high=10.0, shape=(44, ),dtype=np.float32)
        self.Verbose=verbose
        self.UseRL=userl
        self.failure=False
        #change this along with learning.py if needed; this is for random observation error; probably dont need it;
        self.NLoopsFilename="/home/templarares/devel/src/bit-car-inout-controller/etc/NLoops.yaml"
        #self.observation_space=
        #self.reset()
        #self.gc = mc_rtc_rl.GlobalController('mc_rtc.yaml')
        #self.sim.gc().init()
        self.mjvisual=visualization
        self.sim=mc_rtc_rl.MjSim('mc_rtc.yaml', self.mjvisual)
        #self.sim.gc.init()
        "multi-threaded version for mc_mujoco rendering. doesn't seem to work"
        # self.render_=True
        # def render():
        #     while True:
        #         self.sim.updateScene()
        #         render = self.sim.render()
        # thread = threading.Thread(target = render)
        # thread.start()
        # self.reset()
        pass
    def step(self, action):
        "update the fsm state that is immediately to run"
        "this is the preceding state"
        prevState = self.sim.gc().currentState()
        "timer to count execution time of this step"
        "when step() is called for the first time, the next reset() will not be the 1st episode"
        self.isFirstEpisode=False
        #startTime=time.time()
        # if (self.sim.gc().currentState()=="IngressFSM::RightFootCloseToCar::LiftFoot"):
        #     pass
        self.sim.gc().set_rlinterface_done_cb(do_nothing)
        if self.UseRL:
            self.sim.gc().set_rlinterface_start_cb(partial(start_callback, action))
        else:
            self.sim.gc().set_rlinterface_start_cb(partial(null_callback, action))
        #self.sim.gc().set_rlinterface_start_cb(lambda name, controller: start_callback(action, name, controller))
        #self.currentFSMState = 
        "check if action is in current avaialbe action space"
        #assert ...
        "determine how to modify the mc_rtc config based on current fsm state and action"
        #{}
        "call the reward function"

        "advance mc_rtc until the next fsm state or gc failure"
        #while self.sim.gc().running:
        "if previous state is done, proceed to next state by calling the fsm's next()"
        if (self.sim.gc().ready()):
            self.sim.gc().nextState()
            self.sim.stepSimulation()
            """gc().run() guarantees to make gc().nextState() false, which is necessary for proceeding to the next state; 
            Yet sim.stepSimulation() can't do that. So I have to put gc().run() here. Doesn' seem right, but it does the trick"""
            self.sim.gc().run()
        "fsm state\'s teardown() is immediate followed by next state\'s start()"
        "so use the fsm executor's ready() to check stateDone and forget about done_cb"        
        #while (self.sim.gc().running):
        iter_=0
        render_=True
        # print("gc is running:%s"%self.sim.gc().running)
        # print("gc is ready:%s"%self.sim.gc().ready())        
        while (self.sim.gc().running and render_ and (not self.sim.gc().ready())):  
            try:
                self.sim.stepSimulation()
                if(self.mjvisual):
                    if iter_ % 50 == 0:
                        self.sim.updateScene()
                        render_ = self.sim.render()            
                    iter_+=1
                # print(self.sim.gc().Body_rot("base_link"))
                if self.sim.gc().Body_rot("base_link")[3]<0.5:
                    self.failure=True
                    if self.Verbose:
                        print("body falling over at ", self.sim.gc().currentState())
                    # now step() returns
                    observation=np.zeros(44)
                    reward=0
                    done=True
                    return observation,float(reward),done,{}
            except Exception as e:
                print(e)
                print("QP error, global controller is dead at ",self.sim.gc().currentState())
                # now step() returns
                observation=np.zeros(44)
                reward=0
                done=True
                if self.sim.gc().currentState()=="TerminalState":
                    reward=10000
                return observation,float(reward),done,{}
            #print(iter_)
        if self.Verbose:
            print("advancing to next state")
        #now the fsm is ready to proceed to the next state
        "this is the state that has just finished execution"
        currentState = self.sim.gc().currentState()
        "timer to keep track of execution of one step"
        #endTime = time.time()
        # print("execution of this state is %f"%(endTime-startTime))
        # print("Current state is: %s"%(currentState))
        # print("Current position of LeftFoot is:", self.sim.gc().EF_trans("LeftFoot"))
        # print("Current orientation of LeftFoot is:", self.sim.gc().EF_rot("LeftFoot"))
        # print("Current action is:%s"%action)
        #return observation, reward, done and info

        #observation: location of COM, pose of four EFs, and state number
        # LHpose=np.concatenate([self.sim.gc().EF_rot("LeftGripper"),self.sim.gc().EF_trans("LeftGripper")])
        # RHpose=np.concatenate([self.sim.gc().EF_rot("RightGripper"),self.sim.gc().EF_trans("RightGripper")])
        # LFpose=np.concatenate([self.sim.gc().EF_rot("LeftFoot"),self.sim.gc().EF_trans("LeftFoot")])
        # RFpose=np.concatenate([self.sim.gc().EF_rot("RightFoot"),self.sim.gc().EF_trans("RightFoot")])
        # com=self.sim.gc().com()
        # stateNumber=np.concatenate([[helper.StateNumber(name=currentState)],[]])
        # observationd=np.concatenate([LHpose,RHpose,LFpose,RFpose,com,stateNumber])
        # observation = observationd.astype(np.float32)
        """observation space in in range(-10,+10)"""
        com=self.sim.gc().real_com()#3
        stateNumber=np.zeros((6,))#6
        LF_force_z=np.clip(self.sim.gc().EF_force("LeftFoot")[2],0,400)/40.0#1
        RF_force_z=np.clip(self.sim.gc().EF_force("RightFoot")[2],0,400)/40.0#1
        #RF_trans=self.sim.gc().EF_trans("RightFoot")
        RF_pose=np.concatenate([self.sim.gc().EF_rot("RightFoot"),self.sim.gc().EF_trans("RightFoot")])#7
        LF_pose=np.concatenate([self.sim.gc().EF_rot("LeftFoot"),self.sim.gc().EF_trans("LeftFoot")])#7
        # LH_pose=np.concatenate([self.sim.gc().EF_rot("LeftGripper"),self.sim.gc().EF_trans("LeftGripper")])#7
        # RH_pose=np.concatenate([self.sim.gc().EF_rot("RightHipRoot"),self.sim.gc().EF_trans("RightHipRoot")])#7
        posW_trans=np.clip(self.sim.gc().posW_trans(),-10.0,10.0)#3
        posW_rot=np.clip(self.sim.gc().posW_rot(),-10.0,10.0)#4
        velW_trans=np.clip(self.sim.gc().velW_trans(),-10.0,10.0)#3
        velW_rot=np.clip(self.sim.gc().velW_rot(),-10.0,10.0)#3
        # car_height=np.clip(self.sim.gc().Body_trans_car("springCar")[2],-10.0,10.0)
        accW_trans=np.clip(self.sim.gc().accW_trans(),-10.0,10.0)#3
        accW_rot=np.clip(self.sim.gc().accW_rot(),-10.0,10.0)#3
        # LF_gripper_torque=self.sim.gc().gripper_torque()/20.0#1
        observationd=np.concatenate([com,posW_trans,posW_rot,velW_trans,velW_rot,accW_trans,accW_rot,RF_pose,LF_pose,[LF_force_z],[RF_force_z],stateNumber])
        # observation=np.zeros((44,))
        # #gradually add random error in observation
        # with open(self.NLoopsFilename, 'r') as fp:
        #     first_line=fp.readline().strip('\n')
        #     comma=first_line.find(":")
        #     if comma>0:
        #         NLoops=int(first_line[comma+2:])
        #     else:
        #         NLoops=1
        #     fp.close()
        # NLoops+=13000
        # if NLoops>20000:
        #     NLoops=20000
        # #fix NLoops after learning is complete
        # NLoops=20000
        # for element in observationd:
        #     element=element*(1+(np.random.rand()-0.5)*(sqrt(NLoops)*0.003))
        observation = observationd.astype(np.float32)
        #reward: for grasping state, reward = inverse(distance between ef and bar)-time elapsed+stateDone, using the function from minDist.py
        #done: 
        #info: {}
        done = False
        "for completing a state, the reward is 10 by default"
        if (self.sim.gc().running and render_==True):
            reward = 100
        else:
            reward = 50
            self.failure=True
            done = True
            
        "negative reward for time elapsed"
        #reward-=self.sim.gc().duration()*1.0

        "if last state is done,done is True and reward+=500;also some states are more rewarding than others"
        if (currentState=="TerminalState"):
            reward += 100000
            
            done = True
        # # else:
        # #     reward += 100
        elif (currentState=="JumpMomentum"):
            velW_trans=np.clip(self.sim.gc().velW_trans(),-10.0,10.0)#3
            velW_rot=np.clip(self.sim.gc().velW_rot(),-10.0,10.0)#3
            # reward+=np.clip(velW_trans[2]*10000,0,999999999)
            reward-=np.clip(np.absolute(velW_rot[1])*100,0,1000)
            if (self.Verbose):
                print("velW_trans is: ",velW_trans)
                print("velW_rot is", velW_rot)
        elif (currentState=="DampingLanding"):
            reward+=10000
        elif (currentState=="PrepareLanding"):
            reward+=np.clip((self.sim.gc().duration()-0.5)*100000,0,100000)

        # elif (currentState=="IngressFSM::RightFootCloseToCarFSM::LiftFoot"):
        #     """better reduce the couple on lf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     if (self.Verbose):
        #         print("cost for gripper distance is", np.clip(200.0*(np.exp(50.0*minDist)-1),0,200))
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     """the higher the right foot is lifted, the better"""
        #     RF_trans=self.sim.gc().EF_trans("RightFoot")
        #     if (RF_trans[2]>0.40):
        #         reward+=np.clip(150.0*(np.exp(10.0*(RF_trans[2]-0.40))-1),0,300)
        # elif (currentState=="IngressFSM::RightFootCloseToCarFSM::MoveFoot"):
        #     """better reduce the couple on lf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     RF_trans=self.sim.gc().EF_trans("RightFoot")
        #     """RF should be above the car floor(arround 0.4114 in z direction), but not too much"""
        #     if (RF_trans[2]>0.40):
        #         reward +=100.0*np.exp(-50.0*abs(RF_trans[2]-0.40))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     # """right foot should step lefter a little bit (+y)"""
        #     # if RF_trans[1]>0.24:
        #     #     reward+=np.sqrt((RF_trans[1]-0.24)*12e5)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        # elif (currentState=="IngressFSM::RightFootCloseToCar"):
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     """right foot should step forward a little bit,but not too far"""
        #     RF_trans=self.sim.gc().EF_trans("RightFoot")
        #     if RF_trans[0]>0.3:
        #         reward+=np.sqrt((RF_trans[0]-0.3)*2e5)
        #     if RF_trans[0]>0.4:
        #         reward-=np.sqrt((RF_trans[0]-0.4)*12e5)
        #     # """right foot should step lefter a little bit (+y)"""
        #     # if RF_trans[1]>0.24:
        #     #     reward+=np.sqrt((RF_trans[1]-0.24)*12e5)
        #     #print("RightFoot's x location is:",RF_trans[0])
        #     #print("RF y location:",RF_trans[1])
        #     """Better have some force on RF in its z direction, but not too much"""
        #     if  (self.Verbose):
        #         print("At the end of ",currentState,",Right Foot z-hat force is",RF_force[2])
        #     if (RF_force[2]>0):
        #         reward += np.clip(20*RF_force[2],0,100)
        #     if (RF_force[2]>7):
        #         reward -= np.clip(20*(RF_force[2]-5),0,100)
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """better raise R_hip_3 some height above the car seat"""
        #     RThigh_trans=self.sim.gc().Body_trans("R_hip_3")
        #     if RThigh_trans[0]>0.86:
        #         reward+=np.sqrt((RThigh_trans[0]-0.86)*2e5)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     """not a good state if too much torque in the x direction on RF"""
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     reward -=np.clip(5.0*np.exp(np.sqrt(abs(RF_couple[0]))),0,100)
        # elif (currentState=="IngressFSM::RightFootStepAdmittance"):
        #     """better reduce torque on RF"""
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     """right foot should step forward a little bit,but not too much"""
        #     RF_trans=self.sim.gc().EF_trans("RightFoot")
        #     if RF_trans[0]>0.32:
        #         reward+=np.sqrt((RF_trans[0]-0.32)*2e5)
        #     if RF_trans[0]>0.40:
        #         reward-=np.sqrt((RF_trans[0]-0.39)*9e5)
        #     # """right foot should step lefter a little bit (+y)"""
        #     # if RF_trans[1]>0.26:
        #     #     reward+=np.sqrt((RF_trans[1]-0.26)*2e5)
        #     #print("RF y location:",RF_trans[1])
        #     #print("RightFoot's x location is:",RF_trans[0])
        #     """comment out this line when we are ready for later states"""
        #     #done=True
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """better raise R_hip_3 some height above the car seat"""
        #     RThigh_trans=self.sim.gc().Body_trans("R_hip_3")
        #     if RThigh_trans[2]>0.86:
        #         reward+=np.sqrt((RThigh_trans[2]-0.86)*2e5)
        #     #print("R_hip_3 height is:",RThigh_trans[2])
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #         if self.Verbose:
        #             print("ending state because left hand slipped")
        #     """Better have some force on LF in its z direction, but not too much"""
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     if  (self.Verbose):
        #         print("At the end of ",currentState,",Right Foot z-hat force is",RF_force[2])
        #     if (RF_force[2]>0):
        #         reward += np.clip(5*RF_force[2],0,150)
        #     if (RF_force[2]>35):
        #         reward -= np.clip(20*(RF_force[2]-35),0,150)
        #     """print out right foot location in verbose mode"""
        #     if (self.Verbose):
        #         print("RightFoot's x location is:",RF_trans[0])
        #         print("RightFoot's y location:",RF_trans[1])
        # elif (currentState=="IngressFSM::CoMToRightFoot"):
        #     """better reduce the couple on lf, rfand lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     """right foot should not too close to CarBodyFrontHalf"""
        #     RF_trans=self.sim.gc().EF_trans("RightFoot")
        #     if RF_trans[0]>0.38:
        #         reward-=np.sqrt((RF_trans[0]-0.38)*9e5)
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #         if self.Verbose:
        #             print("ending state because left hand slipped")
        #     """the more the robot is putting its weight on RF, the better"""
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     if  (self.Verbose):
        #         print("At the end of ",currentState,",Right Foot z-hat force is",RF_force[2])
        #     if (RF_force[2]>0):
        #         reward += np.clip(10*RF_force[2],0,1000)
        #     """better have RightHip keep forward and right a bit or it won't be high enough"""
        #     RThigh_trans=self.sim.gc().EF_trans("RightHip")
        #     if (self.Verbose):
        #         print ("RightHip translation is:", RThigh_trans)
        #     # if RThigh_trans[0]>0.08:
        #     #     reward+=np.sqrt((RThigh_trans[0]-0.08)*9e5)
        #     #     #reward+=100.0*np.exp(50.0*np.square(RThigh_trans[0]-0.09))
        #     # if RThigh_trans[0]>0.18:
        #     #     reward-=np.sqrt((RThigh_trans[0]-0.18)*15e5)
        #     # if RThigh_trans[1]>0:
        #     #     reward+=100*np.exp(-0.5*np.sqrt(RThigh_trans[1]))
        #     """the less the robot is putting its weight on LF, the better"""
        #     LF_force=self.sim.gc().EF_force("LeftFoot")
        #     if (LF_force[2]<300):
        #         reward += np.clip((300-LF_force[2]),0,200)
        # elif (currentState=="IngressFSM::LandHip"):
        #     """better reduce the couple on lf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #         if self.Verbose:
        #             print("ending state because left hand slipped")
        #     """better lower RightHip, but not too much"""
        #     #car floor at height 0.8146
        #     RThigh_trans=self.sim.gc().EF_trans("RightHip")
        #     if (self.Verbose):
        #         print("RThigh height is:",RThigh_trans[2])
        #         print("RThigh x-direction is:",RThigh_trans[0])
        #     # if RThigh_trans[2]>0.835:
        #     #     reward+=50.0*np.exp(10.0*(0.835-RThigh_trans[2]))
        #     # """better have RightHip keep forward a bit or it won't be high enough"""
        #     if RThigh_trans[0]>0.05:
        #         reward+=np.clip(np.sqrt((RThigh_trans[0]-0.05)*9e5),0,500)
        #         #reward+=100.0*np.exp(10.0*(RThigh_trans[0]-0.05))
        #     """better make RightHip parallel to the car seat"""
        #     RThigh_rot=self.sim.gc().EF_rot("RightHip")
        #     if (self.Verbose):
        #         print("At the end of ",currentState,", Right thigh orie is:",RThigh_rot)
        #     #rotation[1],[2], i.e., the x,y component in the quarternion, should be close to zero
        #     reward+=200.0*np.exp(-10.0*np.sqrt(np.abs(RThigh_rot[0])))
        #     reward+=200.0*np.exp(-10.0*np.sqrt(np.abs(RThigh_rot[1])))
        #     """have righthip lower its back"""
        #     RThighRear_trans=self.sim.gc().EF_trans("RightHipRoot")
        #     #reward+=np.clip(200*np.exp(100.0*(RThigh_trans[2]-RThighRear_trans[2]-0.01)),0,300)
        #     if (self.Verbose):
        #         print("relative rear height is:",(RThigh_trans[2]-RThighRear_trans[2]-0.01))
        #     reward+=500.0*np.exp(-50.0*np.abs(0.822-RThighRear_trans[2]))
        #     # if RThigh_rot[0]<0 and RThigh_rot[1]>0:
        #     #     reward+=200
        #     # else:
        #     #     reward-=200
        #     # RHip3Trans=self.sim.gc().Body_trans("R_hip_3")
        #     # RKnee1Trans=self.sim.gc().Body_trans("R_knee_1")
        #     # if (RHip3Trans[2]-RKnee1Trans[2])<0.015:
        #     #     reward+=200
        #     LH_force=self.sim.gc().EF_force("LeftGripper")
        #     LH_gripper_torque=self.sim.gc().gripper_torque()
        #     if (self.Verbose):
        #         print("LeftGripper force is: ", LH_force)
        #         print("LeftGripper joint torque is: ",LH_gripper_torque)
        #     """add reward for: large gripping force and small LH_force/gripping force ratio so no sliding"""
        #     LH_force_norm=np.linalg.norm(LH_force)
        #     LH_gripper_force=np.abs(LH_gripper_torque)#Gripper joint is prismatic in urdf
        #     reward+=5*np.abs(LH_gripper_force)*np.exp(-50.0*LH_force_norm/LH_gripper_force)
        # elif (currentState=="IngressFSM::LandHipPhase2"):
        #     reward += 200#reward for completing a milestone state
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """terminate if LH falls off, and take off a bunk from reward"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     """the less force remains on LF, the better"""
        #     LF_force=self.sim.gc().EF_force("LeftFoot")
        #     if (LF_force[2]<250):
        #         reward += np.clip((250-LF_force[2]),0,100)
        #     # """better lower R_hip_3 to be in solid contact with the car seat"""
        #     # RThigh_trans=self.sim.gc().Body_trans("R_hip_3")
        #     # if RThigh_trans[0]<0.91:
        #     #     reward+=np.sqrt((0.91-RThigh_trans[0])*5e5)
        #     """better lower RightHip, but not too much"""
        #     RThigh_trans=self.sim.gc().EF_trans("RightHip")
        #     if (self.Verbose):
        #         print("At the end of ",currentState,",RThigh height is:",RThigh_trans[2])
        #     #reward+=100.0*np.exp(-10.0*np.abs(RThigh_trans[2])-0.8146)
        #     """move righthip forward"""
        #     # if RThigh_trans[0]>0.05:
        #     #     #reward+=np.sqrt((RThigh_trans[0]-0.1)*9e5)
        #     #     reward+=100.0*np.exp(10.0*(RThigh_trans[0]-0.05))
        #     """better make RightHip parallel to the car seat"""
        #     RThigh_rot=self.sim.gc().EF_rot("RightHip")
        #     #rotation[1],[2], i.e., the x,y component in the quarternion, should be close to zero
        #     reward+=200.0*np.exp(-10.0*np.sqrt(np.abs(RThigh_rot[0])))
        #     reward+=200.0*np.exp(-10.0*np.sqrt(np.abs(RThigh_rot[1])))
        #     """have righthip lower its back"""
        #     # if RThigh_rot[0]<0 and RThigh_rot[1]>0:
        #     #     reward+=200
        #     # else:
        #     #     reward-=200
        #     RThighRear_trans=self.sim.gc().EF_trans("RightHipRoot")
        #     #reward+=np.clip(200*np.exp(100.0*(RThigh_trans[2]-RThighRear_trans[2]-0.01)),0,300)
        #     if (self.Verbose):
        #         print("relative rear height is:",(RThigh_trans[2]-RThighRear_trans[2]-0.01))
        #         print("rear height is:",(RThighRear_trans[2]))
        #     reward+=500.0*np.exp(-100.0*np.abs((0.823-RThighRear_trans[2])))
        #     # RHip3Trans=self.sim.gc().Body_trans("R_hip_3")
        #     # RKnee1Trans=self.sim.gc().Body_trans("R_knee_1")
        #     # if (RHip3Trans[2]-RKnee1Trans[2])<0.015:
        #     #     reward+=300
        #     if (self.Verbose):
        #         print("At the end of ",currentState,",Right thigh orie is:",RThigh_rot)
        #         #print("relative height of rhip3 is ",RHip3Trans[2]-RKnee1Trans[2])
        #     """Better have some force on RF in its z direction, but not too much"""
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     if  (self.Verbose):
        #         print("At the end of ",currentState,",Right Foot z-hat force is",RF_force[2])
        #     if (RF_force[2]>1):
        #         reward += np.clip(100*RF_force[2],0,1000)
        #     if (RF_force[2]>20):
        #         reward -= np.clip(100*(RF_force[2]-20),0,2000)
        #     LH_force=self.sim.gc().EF_force("LeftGripper")
        #     LH_gripper_torque=self.sim.gc().gripper_torque()
        #     if (self.Verbose):
        #         print("LeftGripper force is: ", LH_force)
        #         print("LeftGripper joint torque is: ",LH_gripper_torque)
        #     """add reward for: large gripping force and small LH_force/gripping force ratio so no sliding"""
        #     LH_force_norm=np.linalg.norm(LH_force)
        #     LH_gripper_force=np.abs(LH_gripper_torque)#Gripper joint is prismatic in urdf
        #     reward+=5*np.abs(LH_gripper_force)*np.exp(-50.0*LH_force_norm/LH_gripper_force)

        # elif (currentState=="IngressFSM::AdjustCoM"):
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #         if self.Verbose:
        #             print("ending state because left hand slipped")
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     # """if this state is executed without termination, give some reward"""
        #     # if (not done):
        #     #     reward+=500
        #     """Better have some force on RF in its z direction, but not too much"""
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     if  (self.Verbose):
        #         print("At the end of ",currentState,",Right Foot z-hat force is",RF_force[2])
        #     if (RF_force[2]>10):
        #         reward += np.clip(4*RF_force[2],0,1000)
        #     if (RF_force[2]>300):
        #         reward -= np.clip(10*(RF_force[2]-300),0,700)
        #     """the less force remains on LF, the better"""
        #     LF_force=self.sim.gc().EF_force("LeftFoot")
        #     if (self.Verbose):
        #         print("At the end of ",currentState,",Left foot support force is: ",LF_force)
        #         print("At the end of ", currentState, "CoM is: ",self.sim.gc().real_com())
        #     reward += np.clip(10*(380-LF_force[2]),0,2500)
        #     LH_force=self.sim.gc().EF_force("LeftGripper")
        #     LH_gripper_torque=self.sim.gc().gripper_torque()
        #     if (self.Verbose):
        #         print("LeftGripper force is: ", LH_force)
        #         print("LeftGripper joint torque is: ",LH_gripper_torque)
        #     """Reward CoM to the right"""
        #     CoM_Y=self.sim.gc().real_com()[1]
        #     if CoM_Y < 0.7 and CoM_Y > 0.3:
        #         reward+=np.sqrt(6e7*(0.7-CoM_Y))
        #     """add reward for: large gripping force and small LH_force/gripping force ratio so no sliding"""
        #     LH_force_norm=np.linalg.norm(LH_force)
        #     LH_gripper_force=np.abs(LH_gripper_torque)#Gripper joint is prismatic in urdf
        #     reward+=5*np.abs(LH_gripper_force)*np.exp(-50.0*LH_force_norm/LH_gripper_force)
        # elif (currentState=="IngressFSM::PutLeftFoot::LiftFoot"):
        #     reward+=500
        #     """rewards for the PutLeftFoot meta state should resemble those of the RightFootCloserToCar state"""
        #     """better reduce the couple on rf and lh"""
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     if (self.Verbose):
        #         print("cost for gripper distance is", np.clip(200.0*(np.exp(50.0*minDist)-1),0,200))
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     # """add a reward if this state is executed w/o. termination"""
        #     # if (not done):
        #     #     reward+=1000
        #     """the higher the left foot is lifted, the better"""
        #     LF_trans=self.sim.gc().EF_trans("LeftFoot")
        #     reward+=np.clip(3500.0*(np.exp(20.0*(LF_trans[2]-0.40))-1),0,5000)
        #     reward+=np.clip(1500.0*(np.exp(10.0*(LF_trans[1]-0.95))-1),0,2000)
        #     if (self.Verbose):
        #         print("LeftFoot translation is:", LF_trans)
        # elif (currentState=="IngressFSM::PutLeftFoot::MoveFoot"):
        #     """better reduce the couple on rf and lh"""
        #     #reward += 500#reward for completing a milestone state
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     """LF should be above the car floor(arround 0.4114 in z direction), but not too much"""
        #     LF_trans=self.sim.gc().EF_trans("LeftFoot")
        #     if (LF_trans[2]>0.40):
        #         reward +=5000.0*np.exp(-50.0*abs(LF_trans[2]-0.41))
        #     else:
        #         reward -=8000.0*(0.41-LF_trans[2])
        #     """LF should be more to the right"""
        #     if (LF_trans[1]<0.8):
        #         reward += np.sqrt((0.8-LF_trans[1])*7e7)
        #     if (self.Verbose):
        #         print("LeftFoot translation is:", LF_trans)
        # elif (currentState=="IngressFSM::PutRightFoot" or currentState=="IngressFSM::PutLeftFoot"):
        #     #reward += 500#reward for completing a milestone state
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     if (not done):
        #         reward+=10000
        #         import os
        #         if (not os.path.exists('LFOnCar')):
        #             os.mknod('LFOnCar')
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     """Better have some force on RF in its z direction"""
        #     if (RF_force[2]>0):
        #         reward += np.clip(200*RF_force[2],0,5000)
        #     LF_force=self.sim.gc().EF_force("LeftFoot")
        #     """Better have some force on LF in its z direction as well"""
        #     if (LF_force[2]>0):
        #         reward += np.clip(200*LF_force[2],0,5000)
        # # elif (currentState=="IngressFSM::PutRightFoot"):
        # #     """transition to PutRightFoot is now auto as in the mc_rtc controller this state just changes contacts"""
        # #     stateNumber_=15
        # elif (currentState=="IngressFSM::NudgeUp"):
        #     if (not done):
        #         reward+=42000
        #     else:
        #         reward+=5000
        #     #done=True
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200) 
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     import os
        #     if (not os.path.exists('NudgeUp')):
        #         os.mknod('NudgeUp')
        #     """we also want to minimize the sliding forces"""#-not sure about this though
        #     LF_force=self.sim.gc().EF_force("LeftFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(0.1*abs(LF_force[1])))
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     reward +=50.0*np.exp(-1.0*np.sqrt(0.1*abs(RF_force[1])))
        # elif (currentState=="IngressFSM::NudgeUpPhase2"):
        #     if (not done):
        #         reward+=10000
        #     else:
        #         reward+=5000
        #     #done=True
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     # """Reward CoM to the right"""
        #     # CoM_Y=self.sim.gc().real_com()[1]
        #     # if CoM_Y < 0.6 and CoM_Y > 0.2:
        #     #     reward+=np.sqrt(6e8*(0.6-CoM_Y))
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     # """we also want to minimize the sliding forces"""#-not sure about this though
        #     # LF_force=self.sim.gc().EF_force("LeftFoot")
        #     # reward +=50.0*np.exp(-1.0*np.sqrt(0.1*abs(LF_force[1])))
        #     # RF_force=self.sim.gc().EF_force("RightFoot")
        #     # reward +=50.0*np.exp(-1.0*np.sqrt(0.1*abs(RF_force[1])))
        
        # elif (currentState=="IngressFSM::ScootRight"):            
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200) 
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     if (not done):
        #         reward+=10000
        #     else:
        #         reward-=5000
        #     """encourage to move RightHip to the right"""
        #     RH_trans=self.sim.gc().EF_trans("RightHip")
        #     if (RH_trans[1]<0):
        #         reward +=np.sqrt(-RH_trans[1]*2e8)
        #     # """we also want to minimize the sliding forces"""#-not sure about this though
        #     # LF_force=self.sim.gc().EF_force("LeftFoot")
        #     # reward +=50.0*np.exp(-1.0*np.sqrt(0.1*abs(LF_force[1])))
        #     # RF_force=self.sim.gc().EF_force("RightFoot")
        #     # reward +=50.0*np.exp(-1.0*np.sqrt(0.1*abs(RF_force[1])))
        # elif (currentState=="IngressFSM::SitOnLeft"):
        #     #reward += 10000    #reward for completing a milestone state
        #     """not a good state if lh has slipped"""
        #     p=np.array(self.sim.gc().EF_trans("LeftGripper"))
        #     a=np.array([0.3886,0.6132,1.7415])
        #     b=np.array([0.652,0.628,1.299])
        #     minDist=np.abs(lineseg_dist(p,a,b)-0.022)
        #     reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
        #     """terminate if LH falls off"""
        #     if minDist>0.02:
        #         done=True
        #         self.failure=True
        #     if (not done):
        #         reward+=30000
        #     """better reduce the couple on lf, rf and lh"""
        #     LF_couple=self.sim.gc().EF_couple("LeftFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
        #     RF_couple=self.sim.gc().EF_couple("RightFoot")
        #     #reward +=50.0*np.exp(-1.0*np.sqrt(abs(RF_couple[0])))
        #     LH_couple=self.sim.gc().EF_couple("LeftGripper")
        #     #reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
        #     """we want to lean more weight on hip, thus less weight on both feet"""
        #     LF_force=self.sim.gc().EF_force("LeftFoot")
        #     RF_force=self.sim.gc().EF_force("RightFoot")
        #     reward +=400-LF_force[2]-RF_force[2]
        #     done = True # we call this the terminal state
        #     #reward -=(0.6-self.sim.gc().real_com()[2])*500
        "ADD HERE: use real robot's com, etc, to determine if it has failed; also calculate an extra reward term maybe?"
        "e.g. if (com_actual.z<0.5): reward -= 200 ; done = True"
        if ((not done) and self.sim.gc().real_com()[2]<0.3):
            done = True
            self.failure=True
            #reward -=200
        #reward function. currently for the gripping only;
        # print("current episode is done (finished or fatal failure): %s"%done)
        """when the episode is done, print the terminal state"""
        if done:
            print("episode terminated at: ",currentState)
        if self.Verbose:
            print("Total reward for ",currentState," is: ",reward)
        # if (self.failure):
        #     observation[57]=1.0
        assert not np.any(np.isnan(observation)),"NaN in observation!"
        assert not np.isnan(reward),"NaN in reward!"
        return observation,float(reward),done,{}

    
    def reset(self):
        "for demonstration purpose, no randomization at initial pose for the 1st episode"
        if (self.isFirstEpisode):
            """the gc().reset() shouldn't be here. but for now it is necessary"""
            self.sim.reset()
            self.isFirstEpisode=False
            #self.sim.gc().reset()
        else:
            """the gc().reset_random() shouldn't be here. but for now it is necessary"""
            #self.sim.reset()
            self.sim.reset_random()
        # LHpose=np.concatenate([self.sim.gc().EF_rot("LeftGripper"),self.sim.gc().EF_trans("LeftGripper")])
        # RHpose=np.concatenate([self.sim.gc().EF_rot("RightGripper"),self.sim.gc().EF_trans("RightGripper")])
        # LFpose=np.concatenate([self.sim.gc().EF_rot("LeftFoot"),self.sim.gc().EF_trans("LeftFoot")])
        # RFpose=np.concatenate([self.sim.gc().EF_rot("RightFoot"),self.sim.gc().EF_trans("RightFoot")])
        # com=self.sim.gc().com()
        # observationd=np.concatenate([LHpose,RHpose,LFpose,RFpose,com,[-1.0]])


        com=self.sim.gc().real_com()#3
        stateNumber=np.zeros((6,))#6
        LF_force_z=np.clip(self.sim.gc().EF_force("LeftFoot")[2],0,400)/40.0#1
        RF_force_z=np.clip(self.sim.gc().EF_force("RightFoot")[2],0,400)/40.0#1
        #RF_trans=self.sim.gc().EF_trans("RightFoot")
        RF_pose=np.concatenate([self.sim.gc().EF_rot("RightFoot"),self.sim.gc().EF_trans("RightFoot")])#7
        LF_pose=np.concatenate([self.sim.gc().EF_rot("LeftFoot"),self.sim.gc().EF_trans("LeftFoot")])#7
        # LH_pose=np.concatenate([self.sim.gc().EF_rot("LeftGripper"),self.sim.gc().EF_trans("LeftGripper")])#7
        # RH_pose=np.concatenate([self.sim.gc().EF_rot("RightHipRoot"),self.sim.gc().EF_trans("RightHipRoot")])#7
        posW_trans=np.clip(self.sim.gc().posW_trans(),-10.0,10.0)#3
        posW_rot=np.clip(self.sim.gc().posW_rot(),-10.0,10.0)#4
        velW_trans=np.clip(self.sim.gc().velW_trans(),-10.0,10.0)#3
        velW_rot=np.clip(self.sim.gc().velW_rot(),-10.0,10.0)#3
        # car_height=np.clip(self.sim.gc().Body_trans_car("springCar")[2],-10.0,10.0)
        accW_trans=np.clip(self.sim.gc().accW_trans(),-10.0,10.0)#3
        accW_rot=np.clip(self.sim.gc().accW_rot(),-10.0,10.0)#3
        # LF_gripper_torque=self.sim.gc().gripper_torque()/20.0#1
        observationd=np.concatenate([com,posW_trans,posW_rot,velW_trans,velW_rot,accW_trans,accW_rot,RF_pose,LF_pose,[LF_force_z],[RF_force_z],stateNumber])
        # observationd=np.zeros((44,))
        # with open(self.NLoopsFilename, 'r') as fp:
        #     first_line=fp.readline().strip('\n')
        #     comma=first_line.find(":")
        #     if comma>0:
        #         NLoops=int(first_line[comma+2:])
        #     else:
        #         NLoops=1
        #     fp.close()
        # NLoops+=13000
        # if NLoops>20000:
        #     NLoops=20000
        # #fix NLoops
        # NLoops=20000
        # for element in observationd:
        #     element=element*(1+(np.random.rand()-0.5)*(sqrt(NLoops)*0.003))
        observation = observationd.astype(np.float32)
        #self.sim.gc().init()
        assert not np.any(np.isnan(observation)),"NaN in observation at Init!"
        return observation
    def render (self, moder='human'):
        pass
    def close(self):
        pass


