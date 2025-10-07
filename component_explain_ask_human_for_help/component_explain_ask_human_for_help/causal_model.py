from component_explain_ask_human_for_help.dummy_skill import AskHumanForHelpState,AbstractNode,get_initial_node,NAMES_TO_NODES,VarRange
import networkx as nx
import matplotlib.pyplot as plt
import copy

class FSM_State:
    def __init__(self,causal_graph:nx.DiGraph,correspondence:dict[str,str],nodes:dict[str,AbstractNode],node_types:dict[str,str],values:dict[str]):
        self.causal_graph = causal_graph
        self.correspondence = correspondence
        self.nodes = nodes
        self.node_types = node_types
        self.values = values

        # Ranges
        self.ranges = self.make_range_dict()

    @classmethod
    def copy_state(cls,obj):
        graph_copy = copy.deepcopy(obj.causal_graph)
        correspondence_copy = copy.deepcopy(obj.correspondence)
        nodes_copy = copy.deepcopy(obj.nodes)
        node_types_copy = copy.deepcopy(obj.node_types)
        values_copy = copy.deepcopy(obj.values)

        new_state = cls(graph_copy,correspondence_copy,nodes_copy,node_types_copy,values_copy)

        return new_state
        

    def get_value(self,node):
        return self.values[node]
    
    def get_values(self,nodes:list[str]=None):
        if nodes is None:
            return self.values
        else:
            return {node:self.values[node] for node in nodes}
    
    def set_value(self,node,value):
        # Try casting
        if type(value) == str and self.ranges[node].range_type in ["cont","disc_cont"]:
            if value == "None":
                raise ValueError(f"Cannot set {node} to None, it is not a valid value")
            value = self.ranges[node].var_type(value)
        elif type(value) == str and self.ranges[node].range_type == "bool":
            if value.lower() in ["true","1","yes"]:
                value = True
            elif value.lower() in ["false","0","no"]:
                value = False
            else:
                raise ValueError(f"Cannot cast {value} to boolean")
        if self.ranges[node].valid(value):
            self.values[node] = value
        else:
            #print(type(value),value,self.ranges[node].var_type,self.ranges[node].range_type)
            raise ValueError(f"Value {value} is invalid for node {node}")
        
    def set_values(self,values:dict[str]):
        for node in values:
            if node in self.values:
                self.set_value(node,values[node])
            else:
                raise ValueError(f"Node {node} not found in FSM state")
    
    def can_intervene(self,node):
        # TODO: Maybe restrict some nodes from appearing in the explanation
        # For now, just returns true always
        return True
    
    '''
    RANGES
    '''
    def make_range_dict(self,discretisation_steps:int=10):
        range_dict = {}
        for node in self.causal_graph.nodes:
            if self.node_types[node] == "Executed":
                range_dict[node] = VarRange.boolean()
            elif self.node_types[node] == "Transition":
                fsm_node = self.nodes[self.correspondence[node]]
                transitions = [transition().name for transition in fsm_node.transition_space()]
                range_dict[node] = VarRange.categorical(values=transitions + [None])
            elif self.node_types[node] == "State":
                state_range = AskHumanForHelpState.ranges[self.correspondence[node]]
                if state_range.range_type == "cont" and state_range.var_type == float:
                    # Can create a discretised float range
                    step_size = (state_range.max-state_range.min)/discretisation_steps
                    discrete_range = VarRange.discretised_float_range(state_range.min,state_range.max,step_size)
                    range_dict[node] = discrete_range
                else:
                    range_dict[node] = state_range
            else:
                raise TypeError(f"Unrecognised node type: {self.node_types[node]}")
        return range_dict

    '''
    RUN
    '''

    def make_var_state(self,node):
        var_state_vals = {}
        parents = list(self.causal_graph.predecessors(node))
        for parent in parents:
            if self.node_types[parent] == "State":
                var_name = self.correspondence[parent]
                var_state_vals[var_name] = self.get_value(parent)
        return AskHumanForHelpState(values=var_state_vals)

    def run(self,node):
        # Create var state
        var_state = self.make_var_state(node)

        if self.node_types[node] == "Executed":
            return self.run_executed(node,var_state)
        elif self.node_types[node] == "Transition":
            return self.run_transition(node,var_state)
        elif self.node_types[node] == "State":
            return self.run_state(node,var_state)
        else:
            raise TypeError(f"Unrecognised node type: {self.node_types[node]}")
        
    def run_executed(self,node,var_state):
        parents = list(self.causal_graph.predecessors(node))
        if len(parents) == 0:
            # Case 1: Node is the initial node, it is always executed
            return True
        else:
            # Must have have one or more parents
            execution_parents = [parent for parent in parents if self.node_types[parent]=="Executed"]
            executed_parents = [parent for parent in execution_parents if self.get_value(parent)]
            if len(executed_parents) == 0:
                # Case 2: None of its parents executed
                return False
            else:
                parent_dict = {self.correspondence[parent]:{} for parent in parents}
                for parent in parents:
                    parent_dict[self.correspondence[parent]][self.node_types[parent]] = parent
                
                for parent_node in parent_dict:
                    parent_transition = self.get_value(parent_dict[parent_node]["Transition"])
                    if self.get_value(parent_dict[parent_node]["Executed"]) and parent_transition == self.correspondence[node]:
                        # Case 3: A node executed and transitioned here
                        return True
                # Case 4: Parents executed but did not transition here
                return False
        
    def run_transition(self,node,var_state):
        parents = list(self.causal_graph.predecessors(node))
        execution_parents = [parent for parent in parents if self.node_types[parent]=="Executed"]
        if self.get_value(execution_parents[0]):
            # Case 1: Node executed, return transition function

            fsm_node = self.nodes[self.correspondence[node]]
            return fsm_node.execute(var_state).name
        else:
            # Case 2: Node not executed, return Null
            return None
        
    def run_state(self,node,var_state):
        parents = list(self.causal_graph.predecessors(node))
        if len(parents) == 0:
            # Case 1: Top-level var, no need to do anything
            return self.get_value(node)
        else:
            state_parents = [parent for parent in parents if self.node_types[parent]=="State"]
            if len(state_parents) == len(parents):
                # Case 2: External Var completely determined by other vars
                raise NotImplementedError(f"Have not implemented external vars")
            else:
                # Internal Var
                execution_parent = [parent for parent in parents if self.node_types[parent]=="Executed"][0]
                if not self.get_value(execution_parent):
                    # Case 3: Internal var not updated as parent not executed, keep same value
                    state_parent = [parent for parent in parents if self.node_types[parent]=="State" and self.correspondence[parent] == self.correspondence[node]][0]
                    return var_state.get_value(self.correspondence[state_parent])
                else:
                    # Case 4: Internal Var, calculate by running parent node
                    decision_parent = [parent for parent in parents if self.node_types[parent]=="Decision"][0]
                    fsm_node = self.nodes[self.correspondence[decision_parent]]
                    fsm_node.execute(var_state,decision=self.get_value(decision_parent))
                    return var_state.get_value(self.correspondence[node])
            


class CausalModel:
    def __init__(self):
        # TODO: I guess we assume no loops in the FSM

        # Build state machine representation
        self.build_state_machine()

        # Build the causal model
        self.build_causal_model()

        # Master state
        self.values = self.load_default_state()
        self.fsm_state = FSM_State(self.causal_graph,self.cm_node_corresponding,self.fsm_nodes,self.cm_node_types,self.values)

    '''
    Load State
    '''
    def load_default_state(self):
        return {
            "Executed_Initial":False,
            "Executed_Detection":False,
            "Executed_Navigate":False,
            "Executed_AskForHelp":False,
            "Executed_WaitForConfirmation":False,
            "Executed_Success":False,
            "Executed_Failure":False,
            "Transition_Initial":None,
            "Transition_Detection":None,
            "Transition_Navigate":None,
            "Transition_AskForHelp":None,
            "Transition_WaitForConfirmation":None,
            "Transition_Failure":None,
            "Transition_Success":None,
            "user_accepted_0":False,
            "user_confirmed_0":False,
            "person_approached_0":False,
            "person_detected_0":False,
            "person_distance_0":0,
            "navigation_failure_0":False,
            "person_std_rms_0":0,
            "robot_nav_err_0":0,
        }

    '''
    Building the model
    '''
    def build_state_machine(self):
        self.initial_node = get_initial_node()
        self.fsm_nodes,self.fsm_edges = self.fsm_graph_walk(self.initial_node)
        self.fsm_graph = nx.DiGraph()
        self.fsm_graph.add_nodes_from(list(self.fsm_nodes.keys()))
        self.fsm_graph.add_edges_from(self.fsm_edges)

    def fsm_graph_walk(self,node:AbstractNode,nodes:dict[str:AbstractNode]={},edge_list:list[tuple[str,str]]=[]):
        if node.name not in nodes:
            nodes[node.name] = node

        next_nodes = node.transition_space()
        for next_node in next_nodes:
            next_node_obj = next_node()
            new_edge = (node.name,next_node_obj.name)
            if new_edge not in edge_list:
                edge_list.append(new_edge)

            if next_node_obj.name not in nodes:
                nodes,edge_list = self.fsm_graph_walk(next_node_obj,nodes=nodes,edge_list=edge_list)

        return nodes,edge_list
            
    def set_fsm_parameters(self,params:dict):
        for node in self.fsm_nodes:
            self.fsm_nodes[node].set_parameters(params)

    def build_causal_model(self):
        self.causal_graph = nx.DiGraph()
        self.cm_node_corresponding = {}
        self.cm_node_types = {}

        state_var_counts = {}
        state_vars = {}

        # Start by adding some cm nodes for each fsm node
        for node in self.fsm_nodes:
            # Basic Nodes
            self.causal_graph.add_node(f"Executed_{node}")
            self.causal_graph.add_node(f"Transition_{node}")

            self.cm_node_corresponding[f"Executed_{node}"] = node
            self.cm_node_corresponding[f"Transition_{node}"] = node

            self.cm_node_types[f"Executed_{node}"] = "Executed"
            self.cm_node_types[f"Transition_{node}"] = "Transition"

            self.causal_graph.add_edge(f"Executed_{node}",f"Transition_{node}")

            # Parents
            node_parents = self.fsm_graph.predecessors(node)
            for parent in node_parents:
                self.causal_graph.add_edge(f"Transition_{parent}",f"Executed_{node}")
                self.causal_graph.add_edge(f"Executed_{parent}",f"Executed_{node}")

            # Input Variables
            for var in self.fsm_nodes[node].input_space():
                if var not in state_var_counts:
                    state_var_counts[var] = 0
                    state_vars[var] = []
                self.causal_graph.add_node(f"{var}_{state_var_counts[var]}")
                self.causal_graph.add_edge(f"{var}_{state_var_counts[var]}",f"Transition_{node}")
                self.cm_node_types[f"{var}_{state_var_counts[var]}"] = "State"
                self.cm_node_corresponding[f"{var}_{state_var_counts[var]}"] = var
                state_vars[var].append(f"{var}_{state_var_counts[var]}")
                state_var_counts[var] += 1

            # Output Variables
            for var in self.fsm_nodes[node].output_space():
                if var not in state_var_counts:
                    state_var_counts[var] = 0
                    state_vars[var] = []
                if f"{var}_{state_var_counts[var]}" in self.causal_graph.nodes:
                    state_var_counts[var] += 1 # handles the case where no node uses the output as input before it is again used as output
                self.causal_graph.add_node(f"{var}_{state_var_counts[var]}")
                self.causal_graph.add_edge(f"Executed_{node}",f"{var}_{state_var_counts[var]}")
                self.cm_node_types[f"{var}_{state_var_counts[var]}"] = "State"
                state_vars[var].append(f"{var}_{state_var_counts[var]}")
                self.cm_node_corresponding[f"{var}_{state_var_counts[var]}"] = var

        # Temporal Linking
        for var in state_vars:
            if len(state_vars[var])>1:
                for i in range(len(state_vars[var])-1):
                    self.causal_graph.add_edge(state_vars[var][i],state_vars[var][i+1])

    '''
    INTERVENTION
    '''
    def propagation_order(self,nodes:list[str],graph:nx.DiGraph) -> list[str]:
        # Get order for propagation, ignoring nodes that do not descend from intervened nodes
        order = nx.topological_sort(graph)
        reduced_order = []
        for node in order:
            ancestors = nx.ancestors(graph,node)
            for inode in nodes:
                if inode in ancestors:
                    reduced_order.append(node)
                    break
        return reduced_order
    
    def propagate_interventions(self,order:list[str],state:FSM_State) -> None:
        for node in order:
            new_val = state.run(node)
            state.set_value(node,new_val)

    def intervene(self,interventions:dict,search_graph:nx.DiGraph):    
        # Copy
        new_graph = nx.DiGraph(search_graph)
        new_state = self.fsm_state.copy_state(self.fsm_state)
        
        for node in interventions:
            # Remove parents
            parents = list(self.causal_graph.predecessors(node))
            for parent in parents:
                # Check if parent has already been removed e.g. for variables that are not allowed to be intervened on
                if parent in new_graph.nodes:
                    new_graph.remove_edge(parent,node)

            # Set new values
            new_state.set_value(node,interventions[node])

        # Propagate changes throughout model
        order = self.propagation_order(list(interventions.keys()),new_graph)
        self.propagate_interventions(order,new_state)

        return new_graph,new_state



    '''
    VISUALISATION
    '''
    def visualise_cm(self):
        pos = nx.spring_layout(self.causal_graph)
        nx.draw(self.causal_graph, pos, with_labels=True, node_color='lightblue',
                node_size=1000, edge_color='gray', arrows=True)
        plt.show()



if __name__ == "__main__":
    cm = CausalModel()
    #cm.visualise_cm()

    state = cm.fsm_state