import copy
import networkx as nx
import itertools

from component_explain_ask_human_for_help.causal_model import CausalModel,FSM_State
import component_explain_ask_human_for_help.utils as utils


class CounterfactualExplanation:
    def __init__(self,interventions:dict,counterfactual_foil:dict,state:FSM_State):
        self.reason = {node:state.get_value(node) for node in interventions}
        self.counterfactual_intervention = interventions
        self.counterfactual_foil = counterfactual_foil
        self.state_vals = copy.deepcopy(state.values)
        self.state = state

    def assignment_string(self,names:dict,values:dict=None,node_names:dict[str,str]=None):
        if values is None:
            values = names
        text = ""
        val_list = list(names.keys())
        for i in range(len(val_list)):
            node = val_list[i]
            if node_names is not None:
                node_name = node_names[node]
            else:
                node_name = node
            text += f"{node_name} = {values[node]}"
            if i == len(val_list) - 1:
                continue
            elif i == len(val_list) - 2:
                if len(val_list) > 2:
                    text += ", and "
                else:
                    text += " and "
            else:
                text += ", "
        return text
    
    def fact_text(self,names:dict[str,str]=None):
        return self.assignment_string(self.counterfactual_foil,self.state_vals,node_names=names)
    
    def reason_text(self,names:dict[str,str]=None):
        return self.assignment_string(self.reason,node_names=names)
    
    def intervention_text(self,names:dict[str,str]=None):
        return self.assignment_string(self.counterfactual_intervention,node_names=names)

    def foil_text(self,names:dict[str,str]=None):
        return self.assignment_string(self.counterfactual_foil,node_names=names)

    def text(self,names:dict[str,str]=None):
        return f"The reason that {self.fact_text(names)} is because {self.reason_text(names)}. If instead {self.intervention_text(names)}, then what would have happened is that {self.foil_text(names)}."

class CounterfactualQuery:
    def __init__(self,foils:dict[str,list]):
        '''
        foils is a dict mapping the names of variables in the causal model to lists of acceptable values
        '''
        self.foils = foils

    def __str__(self):
        return str(self.foils)
    
    def foil_vars(self):
        return list(self.foils.keys())

    def satisfies_query(self,state:FSM_State,rounding:bool=True) -> bool:
        '''
        Returns True if the state satisfies the conditions outlined by the foils provided to the query
        '''
        for var in self.foils:
            val = state.get_value(var)
            if rounding and state.ranges[var].range_type == "disc_cont":
                # Round the queried value to the nearest discretisation if applicable
                disc_vals = state.ranges[var].values
                val = utils.take_closest(disc_vals,val)

            if val not in self.foils[var]: 
                return False
        return True

class Explainer:
    def __init__(self,model:CausalModel):
        self.model = model

    def construct_query(self,foils:dict[str,list], remove:list=[], ignore_none:bool=False) -> CounterfactualQuery:
        if ignore_none:
            remove.append(None)

        proper_foil:dict[str,list] = {}
        for var in foils:
            # First, validate var
            if not var in self.model.causal_graph.nodes:
                raise ValueError(f"Unrecognised variable {var} in query")
            else:
                # Obtain foils
                var_range = self.model.fsm_state.ranges[var]

                if foils[var] is not None:
                    # Can just use the provided foils, if they are valid
                    for val in foils[var]:
                        if not var_range.valid(val):
                            raise ValueError(f"Invalid foil {val} for variable {var}")
                        proper_foil[var] = foils[var]
                else:
                    if var_range.values is None:
                        raise TypeError(f"Invalid var range of type {var_range.var_type} for variable {var}")
                    proper_foil[var] = copy.deepcopy(var_range.values)
                
                # Remove real value from possible options
                real_val = self.model.fsm_state.get_value(var)
                if real_val in proper_foil[var]:
                    proper_foil[var].remove(real_val)

                # Remove any specified values
                for remove_val in remove:
                    if remove_val in proper_foil[var]:
                        proper_foil[var].remove(remove_val)
        return CounterfactualQuery(foils=proper_foil)
    
    '''
    SEARCH SPACE
    '''
    def reduce_model(self,query) -> dict[str,list]:
        # First get the list of possible explanation variables
        ancestors = []
        for node in query.foils:
            ancestors += nx.ancestors(self.model.causal_graph,node)
        ancestors = list(set(ancestors))

        # Remove variables that shouldn't be intervened on
        potential_variables = [node for node in ancestors if self.model.fsm_state.can_intervene(node)]

        # Next, get all possible values for each
        search_space:dict[str,list] = {}
        for node in potential_variables:
            values = self.model.fsm_state.ranges[node].values
            if values is None:
                print(self.model.fsm_state.ranges)
                raise TypeError(f"Invalid: var range of type {self.model.fsm_state.ranges[node].range_type} has no values for variable {node}")
            # Remove None from list
            values = [val for val in values if val is not None]
            search_space[node] = copy.deepcopy(values)

        # Remove true values from search space
        for node in potential_variables:
            real_val = self.model.fsm_state.get_value(node)
            # Check if real val is in the space (it may not be due to discretisation)
            if real_val in search_space[node]:
                search_space[node].remove(self.model.fsm_state.get_value(node)) 
        
        return search_space
    
    def search_graph(self,query:CounterfactualQuery,search_space:dict[str,list]):
        allowed_nodes = list(search_space.keys()) + list(query.foils.keys())

        return self.model.causal_graph.subgraph(allowed_nodes)
        
    def generate_combinations(self,search_space, N):
        variable_names = list(search_space.keys())
        combinations = []

        # Generate all combinations of N variable names
        for var_combination in itertools.combinations(variable_names, N):
            # Generate all possible value combinations for the selected variables
            value_combinations = itertools.product(*[search_space[var] for var in var_combination])

            # Create a list of dictionaries representing the changes
            for values in value_combinations:
                change = {var: value for var, value in zip(var_combination, values)}
                combinations.append(change)

        return combinations
    
    '''
    EXPLAIN
    '''
    def explain(self,query:CounterfactualQuery,max_depth:int = None):
        # Validate
        for var in query.foils:
            if query.foils[var] is None:
                continue
            for var_val in query.foils[var]:
                if var_val == self.model.fsm_state.get_value(var):
                    raise ValueError(f"Cannot construct query with {var} = {var_val} as it is the current value of the variable")

        # Start by constructing a new graph only of ancestors to the node in question
        search_space = self.reduce_model(query)
        search_graph = self.search_graph(query, search_space)

        if max_depth is None:
            max_depth = len(search_space.keys())
        else:
            max_depth = min(max_depth,len(search_space.keys()))

        explanations = []
        for i in range(max_depth):
            new_exps = self.explain_to_depth(query=query,search_space=search_space,depth=i+1,search_graph=search_graph)

            explanations += new_exps

            if len(explanations) > 0:
                break

        return explanations
    
    def explain_to_depth(
            self,
            query:CounterfactualQuery,
            search_space:dict[str,list],
            depth:int,
            search_graph:nx.DiGraph,
    ):
        search_combos = self.generate_combinations(search_space=search_space,N=depth)

        explanations = []

        for combo in search_combos:
            
            new_graph,new_state = self.model.intervene(combo,search_graph)

            if query.satisfies_query(new_state):
                explanations.append(CounterfactualExplanation(combo,new_state.get_values(query.foil_vars()),self.model.fsm_state))
        
        return explanations

if __name__ == "__main__":
    model = CausalModel()

    test_vals = {
        "Executed_Initial":True,
        "Executed_Detection":True,
        "Executed_Navigate":True,
        "Executed_AskForHelp":True,
        "Executed_WaitForConfirmation":False,
        "Executed_Success":False,
        "Executed_Failure":True,
        "Transition_Initial":"Detection",
        "Transition_Detection":"Navigate",
        "Transition_Navigate":"AskForHelp",
        "Transition_AskForHelp":"Failure",
        "Transition_WaitForConfirmation":None,
        "user_accepted_0":False,
        "user_confirmed_0":False,
        "person_approached_0":True,
        "person_detected_0":True,
        "person_distance_0":1,
        "navigation_failure_0":False,
    }

    model.fsm_state.set_values(test_vals)
    #model.visualise_cm()

    explainer = Explainer(model=model)
    #query = explainer.construct_query(foils={"Executed_Failure":[False]})
    query = explainer.construct_query(foils={"Transition_AskForHelp":None},ignore_none=True)
    explanations = explainer.explain(query=query)
    
    for exp in explanations:
        print(exp.text())

