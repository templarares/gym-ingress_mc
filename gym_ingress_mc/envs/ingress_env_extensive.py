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
def start_callback(action, name, controller):
    # #print("{} starting to run".format(name))
    if (
        name=="IngressFSM::RightFootCloseToCarFSM::LiftFoot" 
     ):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        #com.add("weight",int(2000*(abs(action[0]))))
        right_foot=tasks.add("right_foot")
        target=right_foot.add("target")
        target.add_array("rotation",np.array(action[1:4]*0.2))
        target.add_array("translation",np.array(action[4:7]*0.1+[0.261598,0.845943,0.469149]))
        #right_foot.add("weight",int(2000*(abs(action[8]))))
        #Completion1=right_foot.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        #Completion2=com.add("completion")
        #helper.EditTimeout(Completion2,action[9])
        return config
    elif(name=="IngressFSM::LeftHandToBar"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        #com = tasks.add("com")
        #com.add("weight",int(2000*(abs(action[0]))))
        left_hand = tasks.add("left_hand")
        #left_hand.add("weight",int(2000*(abs(action[0]))))
        target=left_hand.add("target")
        #action=np.array([ 0.44949245, -0.23903632, -0.0928756,  -0.7057361,  -0.29943895, -0.3467496,  0.29004097,  0.0811981 ])
        target.add_array("rotation",np.concatenate([[0],action[1:4]*0.1])+[-0.0768043,0.86447,0.423181,0.260213])
        target.add_array("translation",np.array(action[4:7]*0.02+[0.533733,0.69135,1.62638]))        
        #left_hand.add("weight",int(2000*(abs(action[8]))))
        # Completion1=left_hand.add("completion")
        # helper.EditTimeout(Completion1,action[9])
        return config
    elif(name == "IngressFSM::LeftHandGrip"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        #com.add("weight",int(2000*(abs(action[0]))))
        left_hand = tasks.add("left_hand")
        #left_hand.add("weight",int(2000*(abs(action[0]))))
        target=left_hand.add("target")
        #action=np.array([ 0.44949245, -0.23903632, -0.0928756,  -0.7057361,  -0.29943895, -0.3467496,  0.29004097,  0.0811981 ])
        target.add_array("rotation",np.concatenate([[0],action[1:4]*0.1])+[-0.106661,0.873789,0.416487,0.227276])
        target.add_array("translation",np.array(action[4:7]*0.02+[0.482899,0.623379,1.58913]))        
        #left_hand.add("weight",int(2000*(abs(action[8]))))
        # Completion1=left_hand.add("completion")
        # helper.EditTimeout(Completion1,action[9])
        return config
    elif(
        name=="IngressFSM::RightFootCloseToCarFSM::MoveFoot" or
        name=="IngressFSM::RightFootCloseToCarFSM::PutFoot"
        ):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        #com.add("weight",int(2000*(abs(action[0]))))
        right_foot=tasks.add("right_foot")
        target=right_foot.add("target")
        #target.add_array("rotation",np.array(action[1:4]))
        target.add_array("rotation",np.array([0,0,abs(action[3]*0.6)]))
        target.add_array("translation",np.array(action[4:7]*0.1+[0.380024, 0.20364, 0.421191]))
        #right_foot.add("weight",int(2000*(abs(action[8]))))
        #Completion1=right_foot.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        return config

    elif (name=="IngressFSM::CoMToRightFoot"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com=tasks.add("com")
        com.add_array("move_com",np.array(action[1:4]*0.1+[-0.05,-0.20,0.0]))
        #com.add("weight",int(2000*(abs(action[0]))))
        #Completion1=com.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        return config
    elif (name=="IngressFSM::LandHip" or 
        name=="IngressFSM::LandHipPhase2"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        #com.add("weight",int(2000*(abs(action[0]))))
        right_hip=tasks.add("right_hip")
        right_hip.add_array("position",action[1:4]*0.1+[-0.254778,0.845367,0.849587])
        #target.add_array("rotation",np.array(action[1:4]))
        right_hip_ori=tasks.add("right_hip_ori")
        right_hip_ori.add_array("orientation",np.concatenate([[0],action[4:7]*0.1])+[0.370327,0.671353,0.150604,0.624068])
        #right_hip.add("weight",int(2000*(abs(action[8]))))
        #right_hip_ori.add("weight",int(2000*(abs(action[8]))))
        #Completion1=right_hip.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        #Completion2=right_hip_ori.add("completion")
        #helper.EditTimeout(Completion2,action[9])
        return config
    elif (name == "IngressFSM::AdjustCoM"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com = tasks.add("com")
        com.add_array("com",np.array(action[1:4]*0.1+[0.0642257,-0.336135,0.916262]))
        #com.add("weight",int(2000*(abs(action[8]))))
        #Completion1=com.add("completion")
        #helper.EditTimeout(Completion1,action[9])
    elif (name=="IngressFSM::PutLeftFoot::LiftFoot"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com=tasks.add("com")
        com.add_array("move_com",np.array(action[1:4]*0.1+[-0.1,-0.6,0.0]))
        #com.add("weight",int(1000*(abs(action[0]))))
        left_foot=tasks.add("left_foot")
        target=left_foot.add("target")
        target.add_array("translation",np.array(action[4:7]*0.1+[0.311598, 0.90664, 0.451191]))
        #left_foot.add("weight",int(2000*(abs(action[8]))))
        #Completion1=com.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        #Completion2=left_foot.add("completion")
        #helper.EditTimeout(Completion2,action[9])
        return config
    elif (name=="IngressFSM::PutLeftFoot::MoveFoot"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com=tasks.add("com")
        com.add_array("move_com",np.array(action[1:4]*0.1))
        #com.add("weight",int(1000*(abs(action[0]))))
        left_foot=tasks.add("left_foot")
        target=left_foot.add("target")
        target.add_array("translation",np.array(action[4:7]*0.1+[0.34, 0.703, 0.421191]))
        #left_foot.add("weight",int(2000*(abs(action[8]))))
        #Completion1=com.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        #Completion2=left_foot.add("completion")
        #helper.EditTimeout(Completion2,action[9])
        return config
    elif (name=="IngressFSM::PutLeftFoot::PutFoot"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com=tasks.add("com")
        com.add_array("move_com",np.array(action[1:4]*0.1))
        #com.add("weight",int(1000*(abs(action[0]))))
        left_foot=tasks.add("left_foot")
        target=left_foot.add("target")
        target.add_array("translation",np.array(action[4:7]*0.1+[0.326003,0.63986,0.4065]))
        #left_foot.add("weight",int(2000*(abs(action[8]))))
        #Completion1=com.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        #Completion2=left_foot.add("completion")
        #helper.EditTimeout(Completion2,action[9])
        return config
    elif (name=="IngressFSM::NudgeUp"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        waist=tasks.add("waist_pos")
        waist.add_array("position",np.array(action[1:4]*0.1)+[0.0173735,0.550307,1.16874])
        #waist.add("weight",int(1000*(abs(action[0]))))
        #Completion1=waist.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        return config
    elif (name=="IngressFSM::ScootRight"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        com=tasks.add("com")
        com.add_array("com",np.array(action[1:4]*0.1)+[-0.19, 0.19,0.98])
        #com.add("weight",int(1000*(abs(action[0]))))
        body_pos=tasks.add("body_pos")
        body_pos.add_array("position",np.array(action[4:7]*0.2)+[-0.114,0.13,1.26])
        #Completion1=com.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        #Completion2=body_pos.add("completion")
        #helper.EditTimeout(Completion2,action[9])
        return config
    elif (name=="IngressFSM::SitOnLeft"):
        config = mc_rtc_rl.Configuration()
        tasks = config.add("tasks")
        left_hip=tasks.add("left_hip")
        left_hip.add_array("position",np.array(action[4:7]*0.2)+[-0.109182, 0.406125,0.81])
        #Completion1=left_hip.add("completion")
        #helper.EditTimeout(Completion1,action[9])
        return config
    

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
    def __init__(self,visualization: bool = False):
        #self.low=np.array([-1,-1,-1],dtype=np.float32)
        #self.high=np.array([1,1,1],dtype=np.float32)

        self.action_space=spaces.Box(low=-1.0, high=1.0, shape=(8, ),dtype=np.float32)
        "current fsm state"
        #self.currentFSMState = 
        "observation space--need defination"
        self.observation_space=spaces.Box(low=-10.0, high=10.0, shape=(46, ),dtype=np.float32)

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
        if (self.sim.gc().currentState()=="IngressFSM::RightFootCloseToCar::LiftFoot"):
            pass
        self.sim.gc().set_rlinterface_done_cb(do_nothing)
        self.sim.gc().set_rlinterface_start_cb(partial(start_callback, action))
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
            self.sim.stepSimulation()
            if(self.mjvisual):
                if iter_ % 50 == 0:
                    self.sim.updateScene()
                    render_ = self.sim.render()            
                iter_+=1
            #print(iter_)
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
        com=self.sim.gc().real_com()
        #stateNumber=np.concatenate([[helper.StateNumber(name=currentState)],[]])#1
        stateVec=helper.StateNumber(name=currentState) #=NumOfTotalFSMStates, currently eight
        LF_force_z=np.clip(self.sim.gc().EF_force("LeftFoot")[2],0,400)/40.0#1
        RF_force_z=np.clip(self.sim.gc().EF_force("RightFoot")[2],0,400)/40.0#1
        #RF_trans=self.sim.gc().EF_trans("RightFoot")
        RF_pose=np.concatenate([self.sim.gc().EF_rot("RightFoot"),self.sim.gc().EF_trans("RightFoot")])#7
        LF_pose=np.concatenate([self.sim.gc().EF_rot("LeftFoot"),self.sim.gc().EF_trans("LeftFoot")])#7
        posW_trans=np.clip(self.sim.gc().posW_trans(),-10.0,10.0)#3
        posW_rot=np.clip(self.sim.gc().posW_rot(),-10.0,10.0)#4
        velW_trans=np.clip(self.sim.gc().velW_trans(),-10.0,10.0)#3
        velW_rot=np.clip(self.sim.gc().velW_rot(),-10.0,10.0)#3
        accW_trans=np.clip(self.sim.gc().accW_trans(),-10.0,10.0)#3
        accW_rot=np.clip(self.sim.gc().accW_rot(),-10.0,10.0)#3
        observationd=np.concatenate([com,posW_trans,posW_rot,velW_trans,velW_rot,accW_trans,accW_rot,RF_pose,LF_pose,[LF_force_z],[RF_force_z],stateVec])
        observation = observationd.astype(np.float32)
        #reward: for grasping state, reward = inverse(distance between ef and bar)-time elapsed+stateDone, using the function from minDist.py
        #done: 
        #info: {}
        done = False
        "for completing a state, the reward is 10 by default"
        if (self.sim.gc().running and render_==True):
            reward = 10
        else:
            reward = 0
            done = True
        "negative reward for time elapsed"
        #reward-=self.sim.gc().duration()*1.0

        "if last state is done,done is True and reward+=500;also some states are more rewarding than others"
        if (currentState=="IngressFSM::SitPrep"):
            reward += 500
            done = True
        elif (currentState=="IngressFSM::Grasp"):
            LH_couple=self.sim.gc().EF_couple("LeftGripper")
            # reward is inversely related to the x-coponent of leftgripper's couple 
            # reward +=50.0*np.exp(-1.0*abs(LH_couple[0]))
            #reward is also inversely related to the LH's distance to the bar 
            p=np.array(self.sim.gc().EF_trans("LeftGripper"))
            a=np.array([0.37,0.615,1.77])
            b=np.array([0.706,0.63,1.21])
            minDist=abs(lineseg_dist(p,a,b)-0.0055)
            reward+=200.0*np.exp(-50*minDist)
        elif (currentState=="IngressFSM::RightFootCloseToCarFSM::LiftFoot"):
            """better reduce the couple on lf and lh"""
            LF_couple=self.sim.gc().EF_couple("LeftFoot")
            reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
            LH_couple=self.sim.gc().EF_couple("LeftGripper")
            reward +=50.0*np.exp(-1.0*abs(LH_couple[1]))
            """not a good state if lh has slipped"""
            p=np.array(self.sim.gc().EF_trans("LeftGripper"))
            a=np.array([0.37,0.615,1.77])
            b=np.array([0.706,0.63,1.21])
            minDist=abs(lineseg_dist(p,a,b)-0.0055)
            reward-=np.clip(200.0*(np.exp(50.0*minDist)-1),0,200)
            """the higher the right foot is lifted, the better"""
            RF_trans=self.sim.gc().EF_trans("RightFoot")
            if (RF_trans[2]>0.40):
                reward+=np.clip(150.0*(np.exp(10.0*(RF_trans[2]-0.40))-1),0,300)
        elif (currentState=="IngressFSM::RightFootCloseToCarFSM::MoveFoot"):
            """better reduce the couple on lf and lh"""
            LF_couple=self.sim.gc().EF_couple("LeftFoot")
            reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
            LH_couple=self.sim.gc().EF_couple("LeftGripper")
            reward +=50.0*np.exp(-1.0*abs(LH_couple[0]))
            RF_trans=self.sim.gc().EF_trans("RightFoot")
            """RF should be above the car floor(arround 0.4114 in z direction), but not too much"""
            if (RF_trans[2]>0.40):
                reward +=100.0*np.exp(-50.0*abs(RF_trans[2]-0.41))
            """not a good state if lh has slipped"""
            p=np.array(self.sim.gc().EF_trans("LeftGripper"))
            a=np.array([0.37,0.615,1.77])
            b=np.array([0.706,0.63,1.21])
            minDist=abs(lineseg_dist(p,a,b)-0.0055)
            reward-=np.clip(200.0*(np.exp(50*minDist)-1),0,200)
        elif (currentState=="IngressFSM::RightFootCloseToCar"):
            """better reduce the couple on lf and lh"""
            LF_couple=self.sim.gc().EF_couple("LeftFoot")
            reward +=50.0*np.exp(-1.0*np.sqrt(abs(LF_couple[0])))
            LH_couple=self.sim.gc().EF_couple("LeftGripper")
            reward +=50.0*np.exp(-1.0*abs(LH_couple[0]))
            RF_force=self.sim.gc().EF_force("RightFoot")
            """Better have some force on LF in its z direction"""
            if (RF_force[2]>0):
                reward += np.clip(5*RF_force[2],0,100)
            """not a good state if lh has slipped"""
            p=np.array(self.sim.gc().EF_trans("LeftGripper"))
            a=np.array([0.37,0.615,1.77])
            b=np.array([0.706,0.63,1.21])
            minDist=abs(lineseg_dist(p,a,b)-0.0055)
            reward-=np.clip(200.0*(np.exp(50*minDist)-1),0,200)
            """not a good state if too much torque in the x direction on RF"""
            RF_couple=self.sim.gc().EF_couple("RightFoot")
            reward -=np.clip(10.0*np.exp(np.sqrt(abs(RF_couple[0]))),0,200)
        elif (currentState=="IngressFSM::SitOnLeft"):
            reward +=200
        elif (currentState=="IngressFSM::RightFootStepAdmittance"):
            reward +=200
            """comment out this line when we are ready for later states"""
            done=True
            """not a good state if lh has slipped"""
            p=np.array(self.sim.gc().EF_trans("LeftGripper"))
            a=np.array([0.37,0.615,1.77])
            b=np.array([0.706,0.63,1.21])
            minDist=abs(lineseg_dist(p,a,b)-0.0055)
            reward-=np.clip(200.0*(np.exp(50*minDist)-1),0,200)
        elif (currentState=="IngressFSM::LandHipPhase2"):
            reward += 100    
        elif (currentState=="IngressFSM::PutLeftFoot"):
            reward += 100
            #reward -=(0.6-self.sim.gc().real_com()[2])*500
        "ADD HERE: use real robot's com, etc, to determine if it has failed; also calculate an extra reward term maybe?"
        "e.g. if (com_actual.z<0.5): reward -= 200 ; done = True"
        if ((not done) and self.sim.gc().real_com()[2]<0.6):
            done = True
            #reward -=200
        #reward function. currently for the gripping only;
        # print("current episode is done (finished or fatal failure): %s"%done)
        """when the episode is done, print the terminal state"""
        if done:
            print("episode terminated at: ",currentState)
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
        com=self.sim.gc().real_com()
        stateNumber=np.zeros((8,))
        LF_force_z=np.clip(self.sim.gc().EF_force("LeftFoot")[2],0,400)/40.0#1
        RF_force_z=np.clip(self.sim.gc().EF_force("RightFoot")[2],0,400)/40.0#1
        #RF_trans=self.sim.gc().EF_trans("RightFoot")
        RF_pose=np.concatenate([self.sim.gc().EF_rot("RightFoot"),self.sim.gc().EF_trans("RightFoot")])#7
        LF_pose=np.concatenate([self.sim.gc().EF_rot("LeftFoot"),self.sim.gc().EF_trans("LeftFoot")])#7
        posW_trans=np.clip(self.sim.gc().posW_trans(),-10.0,10.0)#3
        posW_rot=np.clip(self.sim.gc().posW_rot(),-10.0,10.0)#4
        velW_trans=np.clip(self.sim.gc().velW_trans(),-10.0,10.0)#3
        velW_rot=np.clip(self.sim.gc().velW_rot(),-10.0,10.0)#3
        accW_trans=np.clip(self.sim.gc().accW_trans(),-10.0,10.0)#3
        accW_rot=np.clip(self.sim.gc().accW_rot(),-10.0,10.0)#3
        observationd=np.concatenate([com,posW_trans,posW_rot,velW_trans,velW_rot,accW_trans,accW_rot,RF_pose,LF_pose,[LF_force_z],[RF_force_z],stateNumber])
        observation = observationd.astype(np.float32)
        observation = observationd.astype(np.float32)
        #self.sim.gc().init()
        return observation
    def render (self, moder='human'):
        pass
    def close(self):
        pass


