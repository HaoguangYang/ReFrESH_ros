#!/usr/bin/python
"""
ReFrESH module that runs on ROS
v0.2.1
Author: Haoguang Yang
Change Log:
v0.1        09/26/2021      First implementation based on SMACH.
v0.2.0      03/09/2022      Refactoring the code to use without SMACH for better efficiency.
v0.2.1      03/11/2022      Support multi-threaded EX, EV, ES. Support normalization within 
                            reconfig metric class. Support prioritized tasks and preemption.
v0.2.2      03/13/2022      Separated utilities to a separate script. Added handle to access
                            manager and EX/EV/ES processes within the module. Added WiFi and
                            CPU/Memory utilization monitors for modules.
v0.2.3      04/01/2022      Added support for pre- and post- functions for each components.
                            Made TF buffer with registered listener available to managers.

Reconfiguration Framework for distributed Embedded systems on Software and Hardware
Originally designed to run on FPGA, the script brings self-adaptation to robots running Linux
and ROS. Specifically, the triplet EX/EV/ES are enforced in a functional module, with their
states managed by a performance- and resource-aware decider.

Reference for the original work:
Cui, Y., Voyles, R. M., Lane, J. T., Krishnamoorthy, A., & Mahoor, M. H. (2015). A mechanism
for real-time decision making and system maintenance for resource constrained robotic systems
through ReFrESH. Autonomous Robots, 39(4), 487-502.
"""

from typing import Literal
import rospy
import sys
import time
import numpy as np
from ReFRESH_ros_utils import *

"""
Element of a thread in a ReFrESH module
"""
class ModuleComponent:
    def __init__(self, ftype=Ftype.THREAD, args:tuple=(), kwargs:dict={},
                    pre:callable=lambda : None, post:callable=lambda : None):
        self.ftype = ftype
        self.pre = pre
        self.post = post
        self.args = args
        self.kwargs = kwargs

def setComponentProperties(comp, ftype:Literal, ns:str='', exec:callable=None,
                            args:tuple=(), mType:type=None, kwargs:dict={},
                            pre:callable=lambda:None, post:callable=lambda:None):
    comp.ftype = ftype
    comp.pre = pre
    comp.post = post
    if comp.ftype == Ftype.NODE:
        comp.args = ()
        comp.kwargs = {'pkgName': ns, 'execName': exec, 'args': args, **kwargs}
    elif comp.ftype == Ftype.LAUNCH_FILE:
        comp.args = ()
        comp.kwargs = {'pkgName': ns, 'fileName': exec, 'args': args, **kwargs}
    elif comp.ftype == Ftype.THREAD:
        comp.args = ()
        comp.kwargs = {'funcPtr': exec, 'args': args}
    elif comp.ftype == Ftype.TIMER:
        comp.args = ()
        comp.kwargs = {'cb': exec, **kwargs}
    elif comp.ftype == Ftype.SUBSCRIBER:
        comp.args = ()
        comp.kwargs = {'topic': ns, 'msgType': mType, 'cb': exec, 'args': args}
    elif comp.ftype == Ftype.SERVICE:
        comp.args = ()
        comp.kwargs = {'topic': ns, 'srvType': mType, 'cb': exec}
    elif comp.ftype == Ftype.ACTION_SRV:
        comp.args = ()
        comp.kwargs = {'topic': ns, 'actType': mType, 'cb': exec, **kwargs}
    elif comp.ftype == Ftype.ACTION_CLI:
        comp.args = ()
        comp.kwargs = {'topic': ns, 'actType': mType, 'feedback_cb': exec, **kwargs}
    elif comp.ftype == Ftype.CALLABLE:
        comp.args = (exec,)
        comp.kwargs = {}
    else:
        print("ERROR: type not implemented.")
        comp.args = ()
        comp.kwargs = {}

"""
Basic metrics class for deciding reconfiguration
"""
class ReconfigureMetric:
    def __init__(self):
        """
        normalized performance utilization: a continuous value between 0 and 1.
        1: performance degraded to the tolerable bounds.
        0: performance is the same as the ideal condition.
        """
        self.performanceUtil = 0.
        """
        normalized resource utilization: a continuous value between 0 and 1.
        1: the module utilizes all quota of available resources / resource Not Available.
        0: the module utilizes no resource / resource Available.
        """
        self.resourceUtil = 0.
        """
        raw limits, following the performanceDims and resourceDims order
        """
        self.performanceDenominator = None
        self.resourceDenominator = None

    """ Set limits for normalizing the performance metric. """
    def setLimits(self, performanceLim, resourceLim):
        self.performanceDenominator = np.array(performanceLim)
        self.resourceDenominator = np.array(resourceLim)

    """ Normalize performance metric vector and update record. """
    def updateFromRaw(self, performanceDims, resourceDims):
        if self.performanceDenominator and self.resourceDenominator:
            self.performanceUtil = max((np.array(performanceDims)/self.performanceDenominator).tolist())
            self.resourceUtil = max((np.array(resourceDims)/self.resourceDenominator).tolist())
        else:
            print("ERROR: Limits not set. Metrics are not updated.")

    """ Update normalized metrics directly. """
    def update(self, performanceDims=None, resourceDims=None):
        if performanceDims:
            self.performanceUtil = max(performanceDims)
        if resourceDims:
            self.resourceUtil = max(resourceDims)

    """ for the base class, assume the bottleneck is the max among
    normalized performance and resource factors """
    def bottleNeck(self):
        return max([self.performanceUtil, self.resourceUtil])

"""
The base class of a ReFrESH module
"""
class ReFrESH_Module:
    def __init__(self, name:str, priority:int=0, preemptive:bool=False,
                EX_thread:int=1, EV_thread:int=1, ES_thread:int=1):
        self.name = name
        # the higher priority, the more important.
        self.priority = priority
        """preemptive: This module suspends all other modules in the enabled list
        that has lower priority, until finished"""
        self.preemptive = preemptive
        self.managerHandle:Manager = None
        self.lock = threading.Lock()
        self.EX_thread = EX_thread
        self.EV_thread = EV_thread
        self.ES_thread = ES_thread
        """The unified reconfiguration metric is updated by the EV (module ON) or ES (module OFF)"""
        self.reconfigMetric = ReconfigureMetric()
        self.EX = [ModuleComponent() for i in range(self.EX_thread)]
        self.EV = [ModuleComponent() for i in range(self.EV_thread)]
        self.ES = [ModuleComponent() for i in range(self.ES_thread)]

    """
    Helper function to set the property of a thread
    which:  either EX, EV, or ES
    ftype:  enumerated thread function type (Ftype)
    ns:     topic or package name, depending on the context. String
    exec:   function pointer
    args:   arguments of the function pointer. Tuple
    mType:  message/service/action type depending on the context
    kwargs: Keyword arguments. Dict
    ind:    thread index within the list (0..*_thread-1)
    """
    def setComponent(self, which, ftype, ns:str='', exec:callable=None,
                                args:tuple=(), mType:type=None, kwargs:dict={},
                                pre:callable=lambda:None, post:callable=lambda:None, ind:int=0):
        this = None
        if (which in ['EX', 'ex', 'Execute', 'execute', 'Executor', 'executer', self.EX]):
            this = self.EX[ind]
        elif (which in ['EV', 'ev', 'Evaluate', 'evaluate', 'Evaluator', 'evaluator', self.EV]):
            this = self.EV[ind]
        elif (which in ['ES', 'es', 'Estimate', 'estimate', 'Estimator', 'estimator', self.ES]):
            this = self.ES[ind]
        else:
            raise TypeError("Invalid set-property request!")
        setComponentProperties(this, ftype, ns, exec, args, mType, kwargs, pre, post)

    def addComponent(self, which, ftype:Ftype, ns:str='', exec:callable=None,
                        args:tuple=(), mType=None, kwargs:dict={},
                        pre:callable=lambda:None, post:callable=lambda:None):
        this = None
        if (which in ['EX', 'ex', 'Execute', 'execute', 'Executor', 'executer', self.EX]):
            this = self.EX
        elif (which in ['EV', 'ev', 'Evaluate', 'evaluate', 'Evaluator', 'evaluator', self.EV]):
            this = self.EV
        elif (which in ['ES', 'es', 'Estimate', 'estimate', 'Estimator', 'estimator', self.ES]):
            this = self.ES
        else:
            raise TypeError("Invalid set-property request!")
        this.append(ModuleComponent())
        self.setComponent(which, ftype, ns, exec, args, mType, kwargs, pre, post, ind=-1)

    def register(self, handle):
        self.managerHandle = handle

    """Helper function to turn on the module from within"""
    def turnMeOn(self):
        if isinstance(self.managerHandle, Manager):
            self.managerHandle.turnOn(self)
        else:
            print("ERROR: this module is not registered to a manager.")

    """Helper function to turn off the module from within"""
    def turnMeOff(self):
        if isinstance(self.managerHandle, Manager):
            self.managerHandle.turnOff(self)
        else:
            print("ERROR: this module is not registered to a manager.")

    """Helper function to return EX thread handle"""
    def getMyEXhandle(self):
        if isinstance(self.managerHandle, Manager):
            return self.managerHandle.getEXhandle(self)
        else:
            return None

    """Helper function to return EV thread handle"""
    def getMyEVhandle(self):
        if isinstance(self.managerHandle, Manager):
            return self.managerHandle.getEVhandle(self)
        else:
            return None

    """Helper function to return ES thread handle"""
    def getMyEShandle(self):
        if isinstance(self.managerHandle, Manager):
            return self.managerHandle.getEShandle(self)
        else:
            return None

"""
Base class of a module manager and decider.
Manages a given set of modules, documenting their states being either off, ready (preempted), or on.
Turn on or off the modules based on request, or through self adaptation using the basicDecider.
The basicDecider turns off a module as either its performance or resource utilization exceeds 1.
The basicDecider then search within its managed module to start the one with minimum utilization,
taking the larger one of the two aspects.
"""
class Manager:
    def __init__(self, launcher:Launcher, managedModules:dict=[], name:str="Manager",
                    freq:float=5.0, minReconfigInterval:float= 1.0):
        self.name = name
        self.launcher = None
        if isinstance(launcher, Launcher):
            self.launcher = launcher
        else:
            raise TypeError("Not initializing the manager with the correct ROS launcher.")
        rospy.on_shutdown(self.shutdown)
        # make this standard for every manager
        self.tfBuffer = launcher.tfBuffer
        # No memory copy
        self.moduleDict = dict.fromkeys(item for item in managedModules)   # make it a dict of class pointers
        self.lock = threading.Lock()
        self.onDict = dict()      # Dictionary of modules turned ON.    key = module, values = ((ex), (ev))
        self.readyDict = dict()   # Dictionary of preempted modules.    key = module, values = None
        self.offDict = dict()     # Dictionary of modules turned OFF.   key = module, values = (es)
        self.Decider = ModuleComponent(Ftype.TIMER, (freq, self.basicDecider), {})
        self.Decider_proc = None
        self.bottleNeck = 0.0
        self.DeciderCooldownDuration = minReconfigInterval
        for m in managedModules:
            if not isinstance(m, ReFrESH_Module):
                raise TypeError("Not initializing Manager", self.name, \
                                "with the supported module class: ReFrESH_Module.")
            # register handler
            m.register(self)
            # turn on ES
            es = []
            for th in m.ES:
                th.pre()
                es.append(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)))
            es = tuple(es)
            self.offDict.update({m: es})
            print("INFO: Module", m.name, "SPAWNED with Manager", self.name, ".")

    """ Determine if a given module is ON. This function is of O(1) complexity """
    def moduleIsOn(self, module:ReFrESH_Module):
        return module in self.onDict

    """ Determine if a given module is OFF. Has O(1) complexity """
    def moduleIsOff(self, module:ReFrESH_Module):
        return module in self.offDict

    """ Get thread handles / instances of EX components """
    def getEXhandle(self, module:ReFrESH_Module):
        return None if module not in self.onDict else self.onDict[module][0]

    """ Get thread handles / instances of EV components """
    def getEVhandle(self, module:ReFrESH_Module):
        return None if module not in self.onDict else self.onDict[module][1]

    """ Get thread handles / instances of ES components """
    def getEShandle(self, module:ReFrESH_Module):
        return None if module not in self.offDict else self.offDict[module]

    """ Setting attribute of another module by name. Return True if succeeded. """
    def moduleSet(self, moduleName:str, attr:str, value):
        for module in self.moduleDict:
            if module.name != moduleName:
                continue
            module.lock.acquire()
            setattr(module, attr, value)
            module.lock.release()
            return True
        return False

    """ Getting attribute of another module by name. Return None if not found. """
    def moduleGet(self, moduleName:str, attr:str):
        for module in self.moduleDict:
            if module.name != moduleName:
                continue
            if hasattr(module, attr):
                return getattr(module, attr)
        return None

    """
    If this is a preemptive module, when it turns on, it will preempt all other modules with
    prio<=this.prio, unless preempted by a module with prio>this.prio (return true).
    If this is a preemptive module and is already on, it will be preempted by another preemptive
    module with prio >= this.prio (return true).
    If this is a non-preemptive module, it should always give way to preemptive modules with
    prio >= this.prio (return true).
    This function determines if a module should be preempted by other modules. Complexity is O(n)
    """
    def moduleIsPreemptible(self, module:ReFrESH_Module):
        if module.preemptive and self.moduleIsOff(module):
            res = tuple(m for m in self.onDict if (m.priority > module.priority and m.preemptive))
        else:
            res = tuple(m for m in self.onDict if (m.priority >= module.priority and m.preemptive))
        return len(res), res
    
    """ Determine if a module can preempt other modules. Complexity O(n) """
    def moduleCanPreempt(self, module:ReFrESH_Module):
        if not module.preemptive:
            return 0, ()
        if self.moduleIsOff(module):
            res = tuple(m for m in self.onDict if m.priority <= module.priority)
        else:
            res = tuple(m for m in self.onDict if m.priority < module.priority)
        return len(res), res

    """ Turn ON a module that was in OFF state. Preemption of other modules is considered. """
    def turnOn(self, module:ReFrESH_Module):
        if module not in self.moduleDict:
            print("ERROR: Module", module, "(name:", module.name, \
                    ") is not managed by this instance.")
            return
        self.lock.acquire()
        isOn = self.moduleIsOn(module)
        if isOn:
            # module is already on
            self.lock.release()
            print("ERROR: ON request for module", module.name, ", which is already ON.")
            return
        isPe, res_pe = self.moduleIsPreemptible(module)
        if isPe:
            # a preemptive task with higher priority is running. add to ready set instead.
            self.readyDict.update({module: None})
            self.lock.release()
            print("INFO: Module", res_pe[0].name, "preempted", module.name, ".")
            return
        # we are safe to start this module.
        # turn off ES
        isOff = self.moduleIsOff(module)
        if not isOff:
            self.lock.release()
            raise RuntimeError("The managed module,", module.name, \
                                ", is neither ON nor OFF. Something bad happened.")
        esHandles = self.offDict.pop(module)
        for th in esHandles:
            self.launcher.stop(th)
        for th in module.ES:
            th.post()
        # turn on EX
        ex = []
        for th in module.EX:
            th.pre()
            ex.append(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)))
        ex = tuple(ex)
        # turn on EV
        ev = []
        for th in module.EV:
            th.pre()
            ev.append(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)))
        ev = tuple(ev)
        # add (module, proc) to self.onDict
        self.onDict.update({module : (ex, ev)})
        print("INFO: Module", module.name, "ON.")
        # if preemptive, this module preempts all other modules with lower priority
        isPe, peTup = self.moduleCanPreempt(module)
        if not isPe:
            self.lock.release()
            return
        for m in peTup:
            # remove m from self.onDict
            exHandle, evHandle = self.onDict.pop(m)
            #self.turnOff(m)
            # turn off EV
            for th in evHandle:
                self.launcher.stop(th)
            for th in module.EV:
                th.post()
            # turn off EX
            for th in exHandle:
                self.launcher.stop(th)
            for th in module.EX:
                th.post()
            # turn on ES
            es = []
            for th in module.ES:
                th.pre()
                es.append(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)))
            es = tuple(es)
            self.offDict.update({m: es})
            self.readyDict.update({m: None})
            print("INFO: Module", module.name, "preempted", m.name, ".")
        self.lock.release()

    """ Turn OFF a module from ON state. Recovery of other modules from preemption is considered. """
    def turnOff(self, module:ReFrESH_Module):
        if module not in self.moduleDict:
            print("ERROR: Module", module, "(name:", module.name, \
                    ") is not managed by this instance.")
            return
        self.lock.acquire()
        isOn = self.moduleIsOn(module)
        if not isOn:
            if module in self.readyDict:
                self.readyDict.pop(module)
                self.lock.release()
                print("INFO: Module", module.name, "OFF after preemption.")
            else:
                self.lock.release()
                print("ERROR: OFF request for module", module.name, ", which is already OFF.")
            return
        # remove module from self.onSet
        exHandle, evHandle = self.onDict.pop(module)
        # turn off EV
        for th in evHandle:
            self.launcher.stop(th)
        for th in module.EV:
            th.post()
        # turn off EX
        for th in exHandle:
            self.launcher.stop(th)
        for th in module.EX:
            th.post()
        isOff = self.moduleIsOff(module)
        if isOff:
            self.lock.release()
            raise RuntimeError("The managed module,", module.name, \
                                ", is both ON and OFF. Something bad happened.")
        # turn on ES
        es = []
        for th in module.ES:
            th.pre()
            es.append(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)))
        es = tuple(es)
        self.offDict.update({module: es})
        print("INFO: Module", module.name, "OFF.")
        # we just turned off a preemptive module... Is there anything previously preempted?
        if not module.preemptive:
            self.lock.release()
            return
        # O(n^2) complexity in the worst case.
        while len(self.readyDict):
            prio = - sys.maxsize - 1
            nextOn = None
            for m in self.readyDict:
                if m.priority > prio:
                    prio = m.priority
                    nextOn = m
            if nextOn is not None:
                # turn on m
                #self.turnOn(nextOn)
                # turn off ES
                isOff = self.moduleIsOff(nextOn)
                if not isOff:
                    self.readyDict.pop(nextOn)
                    continue
                esHandles = self.offDict.pop(nextOn)
                for th in esHandles:
                    self.launcher.stop(th)
                for th in nextOn.ES:
                    th.post()
                # turn on EX
                ex = []
                for th in nextOn.EX:
                    th.pre()
                    ex.append(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)))
                ex = tuple(ex)
                # turn on EV
                ev = []
                for th in nextOn.EV:
                    th.pre()
                    ev.append(self.launcher.launch(th.ftype, *tuple(th.args), **dict(th.kwargs)))
                ev = tuple(ev)
                # add (module, proc) to self.onSet
                self.onDict.update({nextOn: (ex, ev)})
                self.readyDict.pop(nextOn)
                print("INFO: Module", nextOn.name, "ON from preemption.")
                # we hit another preemptive task with lower priority, but still the highest priority
                # among previously suspended tasks. So we stop here.
                if nextOn.preemptive:
                    break
        self.lock.release()

    """ Turn on a list of modules from external request """
    def requestOn(self, onList:list):
        # hash into Dictionary for O(1) complexity
        onDictL = dict.fromkeys(onList)
        toOn = set()
        # O(n) complexity
        for module in self.offDict:
            if module.name in onDictL:
                toOn.add(module)
            elif module in onDictL:
                toOn.add(module)
        for module in toOn:
            self.turnOn(module)

    """ Turn off a list of modules from external request """
    def requestOff(self, offList:list):
        offDictL = dict.fromkeys(offList)
        toOff = set()
        for module in self.readyDict:
            if module.name in offDictL:
                toOff.add(module)
            elif module in offDictL:
                toOff.add(module)
        for module in toOff:
            self.turnOff(module)
        toOff.clear()
        for module in self.onDict:
            if module.name in offDictL:
                toOff.add(module)
            elif module in offDictL:
                toOff.add(module)
        for module in toOff:
            self.turnOff(module)
    
    def turnOffAll(self):
        toOff = set(self.readyDict.keys())
        for module in toOff:
            self.turnOff(module)
        toOff = set(self.onDict.keys())
        for module in toOff:
            self.turnOff(module)

    def findBestESPerf(self, exclude:tuple=()):
        candidate:ReFrESH_Module = None
        bestPerf = 1.
        for m in set(self.offDict.keys()).difference(set(exclude)):
            # in case the ES is wrapped in a callable function instead of a thread
            EShandle = self.offDict[m]
            if callable(EShandle):
                EShandle()
            tmp = m.reconfigMetric.bottleNeck()
            #print(m.reconfigMetric.performanceUtil, m.reconfigMetric.resourceUtil)
            if tmp < bestPerf:
                candidate = m
                bestPerf = tmp
        return candidate, bestPerf

    def turnOnBestESPerf(self, exclude:tuple=()):
        candidate, bestPerf = self.findBestESPerf(exclude)
        if candidate:
            self.turnOn(candidate)
            return candidate, bestPerf
        else:
            print("ERROR: All non-excluded modules returned infeasible.")
            return None, 1.0

    """
    Implements the Basic functionality of a decider: 
    Go over all turned ON modules and look for performannce bottlenecks that falls bellow bounds (>1.0)
    If any found, search in the list of OFF modules for alternatives.
    If alternative is found, turn off the active module and enable the alternative. and sleep for the minimum interval.
    Assume the EV and ES use a shared object of reconfig metric.
    """
    def basicDecider(self, event):
        toOn = None
        toOff = None
        bottleNeck = 0.0
        #O(n^2) complexity, in the worst case.
        self.lock.acquire()
        for module in self.onDict:
            bottleNeck = max(module.reconfigMetric.bottleNeck(), bottleNeck)
            if bottleNeck < 1.:
                continue
            # performance crisis. do reconfig
            print("INFO: Module", module.name, "falls below (", bottleNeck, "x) desired performance bounds.")
            candidate, bestPerf = self.findBestESPerf(exclude=(module,))
            if not candidate:
                print("WARN: No alternative module satisfies resource and performance bounds. Current value:", \
                    module.reconfigMetric.bottleNeck(), ".")
                continue
            print("INFO: Found alternative module", candidate.name, \
                    "with estimated performance", bestPerf, ".")
            toOff = module
            toOn = candidate
            # if an alternative is found, we do not report the performance crisis.
            bottleNeck = bestPerf
            # we have found a solution.
            break
        self.lock.release()
        self.bottleNeck = bottleNeck
        # pending operation list is not empty. Go ahead and change module on/off status.
        if toOff:
            self.turnOff(toOff)
        if toOn:
            # you may have a bunch of things resumed after a preemptive module is off, so double check here.
            isOn = self.moduleIsOn(toOn)
            if not isOn:
                self.turnOn(toOn)
        if toOff or toOn:
            time.sleep(self.DeciderCooldownDuration)

    """ Starts decider object """
    def run(self, blocking:bool = False):
        if not self.Decider_proc:
            self.Decider_proc = self.launcher.launch(self.Decider.ftype, \
                                                *tuple(self.Decider.args), \
                                                **dict(self.Decider.kwargs))
            print("INFO: Launched decider thread.")
        if blocking:
            self.launcher.spin()

    """ Cleanup """
    def shutdown(self):
        if self.Decider_proc:
            self.launcher.stop(self.Decider_proc)
        while len(self.offDict):
            m, esHandle = self.offDict.popitem()
            # turn off ES
            for th in esHandle:
                self.launcher.stop(th)
            print("INFO: Module", m.name, "SHUTDOWN.")
        self.readyDict.clear()
        while len(self.onDict):
            m, (exHandle, evHandle) = self.onDict.popitem()
            # turn off EV
            for th in evHandle:
                self.launcher.stop(th)
            # turn off EX
            for th in exHandle:
                self.launcher.stop(th)
            print("INFO: Module", m.name, "SHUTDOWN.")
        self.moduleDict.clear()

if __name__ == "__main__":
    # a simple test case with two empty modules.
    taskLauncher = Launcher("RobotTaskRunner")
    testModule1 = ReFrESH_Module("test1", priority=10, preemptive=True)
    testModule2 = ReFrESH_Module("test2")
    taskManager = Manager(taskLauncher, [testModule1, testModule2])
    # non-blocking run
    taskManager.run()
    # turn on by requesting the manager
    taskManager.requestOn(["test1", "test2"])
    rospy.sleep(rospy.Duration(1.0))
    # turn on/off by requesting the module
    testModule1.turnMeOff()
    rospy.sleep(rospy.Duration(1.0))
    testModule2.turnMeOff()
    rospy.sleep(rospy.Duration(1.0))
    # respawnable
    testModule1.turnMeOn()
    rospy.sleep(rospy.Duration(1.0))
    # simulate a situation with degrading module
    testModule1.reconfigMetric.update([1.1], [1.1])
    taskLauncher.spin()
    # shutdown
    #taskManager.shutdown()
    taskLauncher.shutdown()
