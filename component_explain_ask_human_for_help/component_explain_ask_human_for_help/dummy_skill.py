MAX_DISTANCE = 4
# MAX_DISTANCE = 1 # Used only for too far away case

'''
VAR STATE
'''
class VarRange:
    def __init__(self,range_type:str="cat",values:list=None,var_type:type=None,min:float=None,max:float=None):
        self.range_type = range_type
        self.values = values
        self.var_type = var_type
        self.min = min
        self.max = max

    '''
    Constructors
    '''
    @staticmethod
    def normalised_float():
        return VarRange(range_type="cont",var_type=float,min=0,max=1)
    
    @staticmethod
    def categorical(values:list):
        return VarRange(range_type="cat",values=values)
    
    @staticmethod
    def boolean():
        return VarRange(range_type="bool",values=[True,False],var_type=bool)
    
    @staticmethod
    def int_range(min:int,max:int):
        if min>max:
            raise ValueError(f"Minimum {min} must be smaller than or equal to maximum {max}")
        
        return VarRange(range_type="cat",values=list(range(min,max+1)),var_type=int,min=min,max=max)
    
    @staticmethod
    def float_range(min:float,max:float):
        if min>max:
            raise ValueError(f"Minimum {min} must be smaller than or equal to maximum {max}")
        
        return VarRange(range_type="cont",var_type=float,min=min,max=max)
    
    @staticmethod
    def discretised_float_range(min:float,max:float,step:float,dec_places:int=10):
        if min>max:
            raise ValueError(f"Minimum {min} must be smaller than or equal to maximum {max}")

        float_range = [round(min + i * step, dec_places) for i in range(int((max - min) / step) + 1)]
        return VarRange(range_type="disc_cont",values=float_range,var_type=float,min=min,max=max)
    
    @staticmethod
    def any_string():
        return VarRange(range_type="any",var_type=str)
    
    @staticmethod
    def any_int():
        return VarRange(range_type="any",var_type=int)
    
    '''
    Utility
    '''
    def get_max(self):
        if self.max is None:
            raise TypeError(f"Cannot get max value for VarRange type {self.range_type}")
        else:
            return self.max
        
    def get_middle_value(self):
        if self.values is None:
            raise TypeError(f"Cannot get median value for VarRange type {self.range_type}")
        else:
            middleIndex = (len(self.values) - 1)//2
            return self.values[middleIndex] 
    
    '''
    Checks
    '''
    def valid(self,value,ignore_max=True):
        # First, check typing
        if self.var_type is not None:
            # var_type = None allows any type
            if self.var_type == float:
                # Allow floats and ints
                if not isinstance(value,(float,int)):
                    print(f"Value {value} of type {type(value)} is not of type {self.var_type}")
                    return False
            elif not type(value) is self.var_type:
                # For others, must match type exactly
                print(f"Value {value} of type {type(value)} is not of type {self.var_type}")
                return False
            

        if self.range_type in ["cont","disc_cont"]:
            # Continuous between a min and max
            if value < self.min:
                return False
            
            if not ignore_max and value > self.max:
                return False
            
            return True
        elif self.range_type in ["cat","bool"]:
            # Categorical, can check values
            return value in self.values
        elif self.range_type == "any":
            # Allows any variable of the right type
            return True
        else:
            raise ValueError(f"Unrecognised VarRange type {self.range_type}")

'''
STATES
'''

class AskHumanForHelpState:
    ranges = {
        "person_approached":VarRange.boolean(),
        "person_detected":VarRange.boolean(),
        "person_distance":VarRange.float_range(0,5),
        "navigation_failure":VarRange.boolean(),
        "user_accepted":VarRange.boolean(),
        "user_confirmed":VarRange.boolean(),
        "detection_stable":VarRange.boolean(),
        "person_std_rms":VarRange.float_range(0,5),
        "robot_nav_err":VarRange.float_range(0,5),
    }

    default_state = {
        "person_approached":False,
        "person_detected":True,
        "person_distance":2,
        "navigation_failure":False,
        "user_accepted":True,
        "user_confirmed":True,
        "detection_stable":True,
        "person_std_rms":0.01,
        "robot_nav_err":0.01,
    }

    def __init__(self,values=None):
        self.values = values
    
    def get_value(self,var):
        if var in self.values:
            return self.values[var]
        else:
            raise ValueError(f"Var {var} not in state value dictionary")
        
    def set_value(self,var,value):
        if var in self.ranges:
            if self.ranges[var].valid(value):
                self.values[var] = value
            else:
               raise ValueError(f"Value {value} is not a valid value for variable {var}") 
        else:
            raise ValueError(f"Var {var} not specified by the state")


'''
STATE MACHINE NODES
''' 

class AbstractNode:
    name = "AbstractNode"

    def __init__(self):
        pass

    def __eq__(self, other_node):
        return self.name == other_node.name
    
    def execute(self,state:AskHumanForHelpState):
        # Returns a new node
        raise NotImplementedError
    
    @staticmethod
    def input_space():
        return []
    
    @staticmethod
    def output_space():
        return []
    
    @staticmethod
    def transition_space():
        return []
    
    def set_parameters(self,params:dict):
        pass

class InitialNode(AbstractNode):
    name = "Initial"

    def __init__(self):
        pass
    
    def execute(self, state):
        state.set_value("person_approached",False)
        return DetectionNode()
    
    @staticmethod
    def output_space():
        return ["person_approached"]
        
    @staticmethod
    def transition_space():
        return [DetectionNode]
        
class FailureNode(AbstractNode):
    name = "Failure"

    def __init__(self):
        super().__init__()

class SuccessNode(AbstractNode):
    name = "Success"

    def __init__(self):
        super().__init__()
        
class DetectionNode(AbstractNode):
    name = "Detection"
    distance_margin = 0.75
    
    def __init__(self):
        super().__init__()
        self.max_distance = MAX_DISTANCE
        
    def execute(self, state):
        if state.get_value("person_detected") and state.get_value("detection_stable"):
            if state.get_value("person_distance") < self.distance_margin:
                return AskForHelpNode()
            elif state.get_value("person_distance") > self.max_distance:
                return FailureNode()
            else:
                return NavigateNode()
        else:
            return FailureNode()
    
    @staticmethod
    def input_space():
        return ["person_detected","person_distance", "detection_stable","person_std_rms"]
    
    @staticmethod
    def transition_space():
        return [FailureNode,AskForHelpNode,NavigateNode]
    
    def set_parameters(self, params):
        if "max_distance" in params:
            self.max_distance = params["max_distance"]
    
class NavigateNode(AbstractNode):
    name = "Navigate"

    def __init__(self):
        super().__init__()
        
    def execute(self, state):
        if state.get_value("navigation_failure"):
            return FailureNode()
        else:
            state.set_value("person_approached",True)
            return AskForHelpNode()
        
    @staticmethod
    def input_space():
        return ["navigation_failure"]
    
    @staticmethod
    def output_space():
        return ["person_approached","robot_nav_err"]
    
    @staticmethod
    def transition_space():
        return [AskForHelpNode,FailureNode]
    
class AskForHelpNode(AbstractNode):
    name = "AskForHelp"

    def __init__(self):
        super().__init__()
        
    def execute(self, state):
        if state.get_value("user_accepted"):
            return WaitForConfirmationNode()
        else:
            return FailureNode()
        
    @staticmethod
    def input_space():
        return ["user_accepted"]
    
    @staticmethod
    def transition_space():
        return [WaitForConfirmationNode,FailureNode]
    
class WaitForConfirmationNode(AbstractNode):
    name = "WaitForConfirmation"

    def __init__(self):
        super().__init__()
        
    def execute(self, state):
        if state.get_value("user_confirmed"):
            return SuccessNode()
        else:
            return FailureNode()
        
    
    @staticmethod
    def input_space():
        return ["user_confirmed"]
    
    @staticmethod
    def transition_space():
        return [SuccessNode,FailureNode]
    
def get_initial_node():
    return InitialNode()

NAMES_TO_NODES = {
    "Initial":InitialNode,
    "Detection":DetectionNode,
    "Navigate":NavigateNode,
    "AskForHelp":AskForHelpNode,
    "WaitForConfirmation":WaitForConfirmationNode,
    "Success":SuccessNode,
    "Failure":FailureNode,
}






