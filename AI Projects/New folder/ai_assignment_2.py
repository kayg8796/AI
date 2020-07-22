#added the heuristic,,,now mordifgy greed and insert
# the try functions do not need to return the lower_value,up_value etc
import numpy as np
import time
#here ive added the heuristics
class Node():
    def __init__(self,state,parent,action,depth,diagonal_cost,path_cost,grid_size,allow_diagonals,heuristic_cost):
        self.state = state 
        self.parent = parent # parent node
        self.action = action # move up, left, down, right
        self.depth = depth # depth of the node in the tree
        self.diagonal_cost = diagonal_cost # g(n), the cost to take the step
        self.path_cost = path_cost # accumulated g(n), the cost to reach the current node
        self.h_cost = heuristic_cost # h(n), heuristic cost, cost to reach goal state from the current node
        self.grid_size = grid_size #variable grid size taken from the user
        self.allow_diagonals =allow_diagonals
        self.normal_cost = 1        
        # children node: possible actions that can be taken from a specific node
        self.move_up = None 
        self.move_left = None
        self.move_down = None
        self.move_right = None
        self.diagonal_45 = None
        self.diagonal_135 = None
        self.diagonal_225 = None
        self.diagonal_315 = None
    
    def try_move_up(self):#checking if moving up is a valid move        
        zero_index=[i[0] for i in np.where(self.state=='R')] #checking for the position of the Robot on the grid
        if zero_index[0] == 0 : # ensuring the robot R does not go out of bounds
            return False
        else:
            up_value = self.state[zero_index[0]-1,zero_index[1]] 
            if up_value == 'X': # ensuring the cell above the robot is not occupied by an obstacle
                return False
            new_state = self.state.copy() #if its not, then the robot takes the position of the grid directly above it.
            new_state[zero_index[0],zero_index[1]] = '.'
            new_state[zero_index[0]-1,zero_index[1]] = 'R'
            return new_state.tolist() 
        
    #A similar thing happens for the next seven possible directional movements
    def try_move_left(self):
        zero_index=[i[0] for i in np.where(self.state=='R')]         
        if zero_index[1] == 0:
            return False
        else:
            left_value = self.state[zero_index[0],zero_index[1]-1]
            if left_value == 'X':
                return False
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = '.'
            new_state[zero_index[0],zero_index[1]-1] = 'R'
            return new_state.tolist() 
        
    def try_move_down(self):
        zero_index=[i[0] for i in np.where(self.state=='R')]         
        if zero_index[0] == self.grid_size - 1:
            return False        
        else:
            lower_value = self.state[zero_index[0]+1,zero_index[1]]
            if lower_value == 'X':
                return False
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = '.'
            new_state[zero_index[0]+1,zero_index[1]] = 'R'
            return new_state.tolist()   
        
    def try_move_right(self):
        zero_index=[i[0] for i in np.where(self.state=='R')] 
        
        if zero_index[1] == self.grid_size - 1:
            return False
        else:
            right_value = self.state[zero_index[0],zero_index[1]+1]
            if right_value == 'X':
                return False
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = '.'
            new_state[zero_index[0],zero_index[1]+1] = 'R'
            return new_state.tolist() 
    
    def try_diagonal_45(self):
        zero_index=[i[0] for i in np.where(self.state=='R')] 
        if zero_index[0] == 0 or zero_index[1] == self.grid_size - 1 :
            return False
        else:            
            diag_val_45 = self.state[zero_index[0]-1,zero_index[1]+1]
            if diag_val_45 == 'X':
                return False
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = '.' 
            new_state[zero_index[0]-1,zero_index[1]+1] = 'R'
            return new_state.tolist() 
                
    def try_diagonal_135(self):
        zero_index=[i[0] for i in np.where(self.state=='R')] 
        if zero_index[0] == self.grid_size - 1 or zero_index[1] == self.grid_size - 1 :
            return False
        else:           
            diag_val_135 = self.state[zero_index[0]+1,zero_index[1]+1] 
            if diag_val_135 == 'X':
                return False
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = '.' 
            new_state[zero_index[0]+1,zero_index[1]+1] = 'R'
            return new_state.tolist() 
          
    def try_diagonal_225(self):
        zero_index=[i[0] for i in np.where(self.state=='R')] 
        if zero_index[1] == 0 or zero_index[0] == self.grid_size - 1:
            return False
        else:            
            diag_val_225 = self.state[zero_index[0]+1,zero_index[1]-1] 
            if diag_val_225 == 'X':
                return False  
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = '.' 
            new_state[zero_index[0]+1,zero_index[1]-1] = 'R'
            return new_state.tolist() 
                    
    def try_diagonal_315(self):
        zero_index=[i[0] for i in np.where(self.state=='R')] 
        if zero_index[0] == 0 or zero_index[1] == 0:
            return False
        else:
            diag_val_315 = self.state[zero_index[0]-1,zero_index[1]-1]
            if diag_val_315 == 'X':
                return False
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = '.'
            new_state[zero_index[0]-1,zero_index[1]-1] = 'R'
            return new_state.tolist() 

    #heuristics used for the a_star and greedy search algorithms depending on the values of the diagonal cost
    def h_manhattan(self,current_state,goal_state): #using the half manhattan heuristic
        s1 = np.where(current_state == 'R')
        s2 = np.where(goal_state == 'R')
        hm = (abs(s2[0]-s1[0]) + abs(s2[1]-s1[1]))/2
        return hm
    def h_sld(self,current_state,goal_state): #this is dominant in thecase where diagonal cost is 1.5
        s1 = np.where(current_state == 'R')
        s2 = np.where(goal_state == 'R')
        hm = np.sqrt((s2[0][0]-s1[0][0])**2 + (s2[1][0]-s1[1][0])**2)
        return hm
    
    def node_expansion(self,fringe,node_depth,path_cost_to_node,closed,current_node,current_path_cost,current_depth):
        if current_node.try_move_down():
            new_state  = np.array(current_node.try_move_down())            
            if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_move_down() not in fringe: #reshape method returns list with list reas0n [0]
                #initializing child node with its own attributes                
                current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                      diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost ,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                fringe.append(current_node.move_down) #child nodes are added at the end of the list
                node_depth.append(current_depth+1)   #associated attribute to node
                path_cost_to_node.append(current_path_cost+ self.normal_cost) #associated attribute to node
                                                    
        if current_node.try_move_right():
            new_state  = np.array(current_node.try_move_right())            
            if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_move_right() not in fringe:                
                current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                      diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                fringe.append(current_node.move_right)
                node_depth.append(current_depth+1)
                path_cost_to_node.append(current_path_cost+self.normal_cost)
                 
        if current_node.try_move_up():
            new_state  = np.array(current_node.try_move_up())            
            if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_move_up() not in fringe:                
                current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                      diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                fringe.append(current_node.move_up)
                node_depth.append(current_depth+1)
                path_cost_to_node.append(current_path_cost+self.normal_cost)
        
        if current_node.try_move_left():
            new_state  = np.array(current_node.try_move_left())
            if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_move_left() not in fringe:                
                current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                      diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                fringe.append(current_node.move_left)
                node_depth.append(current_depth+1)
                path_cost_to_node.append(current_path_cost+self.normal_cost)
                                
        if self.allow_diagonals:        
            if current_node.try_diagonal_45():
                new_state  = np.array(current_node.try_diagonal_45())                
                if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_diagonal_45() not in fringe:                    
                    current_node.diagonal_45 = Node(state=new_state,parent=current_node,action='diagonal_45',depth=current_depth+1,\
                                          diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                    fringe.append(current_node.diagonal_45)
                    node_depth.append(current_depth+1)
                    path_cost_to_node.append(current_path_cost+self.diagonal_cost)     
                    
            if current_node.try_diagonal_135():
                new_state  = np.array(current_node.try_diagonal_135())                
                if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_diagonal_135() not in fringe:                    
                    current_node.diagonal_135 = Node(state=new_state,parent=current_node,action='diagonal_135',depth=current_depth+1,\
                                          diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                    fringe.append(current_node.diagonal_135)
                    node_depth.append(current_depth+1)
                    path_cost_to_node.append(current_path_cost+self.diagonal_cost)            
                    
            if current_node.try_diagonal_225():
                new_state  = np.array(current_node.try_diagonal_225())                
                if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_diagonal_225() not in fringe:                    
                    current_node.diagonal_225 = Node(state=new_state,parent=current_node,action='diagonal_225',depth=current_depth+1,\
                                          diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                    fringe.append(current_node.diagonal_225)
                    node_depth.append(current_depth+1)
                    path_cost_to_node.append(current_path_cost+self.diagonal_cost) 
                    
            if current_node.try_diagonal_315():  
                new_state  = np.array(current_node.try_diagonal_315())                
                if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed and current_node.try_diagonal_315() not in fringe:                    
                    current_node.diagonal_315 = Node(state=new_state,parent=current_node,action='diagonal_315',depth=current_depth+1,\
                                          diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                    fringe.append(current_node.diagonal_315)
                    node_depth.append(current_depth+1)
                    path_cost_to_node.append(current_path_cost+self.diagonal_cost)
        return fringe,node_depth,path_cost_to_node
                                    

    def print_path(self): #printing of solution when goal node is found
        state_trace = [self.state] #creating a LIFO stack, starting from the last child to the root parent
        action_trace = [self.action] # variables associated with state to be printed as well
        depth_trace = [self.depth]
        diagonal_cost_trace = [self.diagonal_cost]
        path_cost_trace = [self.path_cost]
        heuristic_cost_trace = [self.h_cost]
        
        # add node information as tracing back up the tree
        while self.parent: #continue as long as a node has a parent,that is till it gets to the first initial state of the robot 
            self = self.parent

            state_trace.append(self.state)
            action_trace.append(self.action)
            depth_trace.append(self.depth)
            diagonal_cost_trace.append(self.diagonal_cost)
            path_cost_trace.append(self.path_cost)
            heuristic_cost_trace.append(self.h_cost) #hcosst function

        # print out the path
        step_counter = 0
        while state_trace:
            print ('step',step_counter)
            print (state_trace.pop())  #taking out the last state to be added
            print ('action=',action_trace.pop(),', depth=',str(depth_trace.pop()),\
            ', step cost=',str(diagonal_cost_trace.pop()),', actual cost(gn) =',\
            str(path_cost_trace.pop()) + ' heuristic cost (hn) = ' + str(heuristic_cost_trace.pop())+'\n')   
            #print('this the actual cost {} and h is : {}'.format(path_cost_trace.pop(),heuristic_cost_trace.pop()))
            step_counter += 1
                            
    def breadth_first_search(self, goal_state):
        start = time.time()        
        fringe = [self] # fringe , list container of type node and fringe initialization with the initial state as was passed by the user
        fringe_num_nodes_popped = 0 # number of nodes popped off the fringe, measuring nodes popped:
        fringe_max_length = 1 # counter to record the max number of nodes to have been present in the fringe        
        node_depth = [0] # list to contain node depths
        path_cost_to_node = [0] # path cost from root node to current node
        closed = set([]) # record of visited states,set used to prevent redundancy
        
        while fringe:     
            
            if len(fringe) > fringe_max_length: 
                fringe_max_length = len(fringe) #updating max number of nodes to have present in the fringe               
            current_node = fringe.pop(0) # select and remove the first node in the fringe that was added from the ecpansion (FIFO)
            fringe_num_nodes_popped += 1 #increments each time a node is popped out of the fringe            
            current_depth = node_depth.pop(0) # select and remove the depth for current node
            current_path_cost = path_cost_to_node.pop(0) # select and remove the path cost for reaching current node
            closed.add(tuple(current_node.state.reshape(1,self.grid_size * self.grid_size)[0])) # checking if node had already been visited
            

            if np.array_equal(current_node.state,goal_state): #goal test
                current_node.print_path()#when goal state is forund execute printpath method that tracks path to root node               
                print ('Time performance :::',str(fringe_num_nodes_popped),'nodes popped off the fringe.')
                print ('Space performance:', str(fringe_max_length),'nodes in the fringe at its max.')
                print ('Time spent: %0.2fs' % (time.time()-start))
                return True
            else:                #expansion to eight children node and filling into fringe
                fringe,node_depth,path_cost_to_node = self.node_expansion(fringe,node_depth,path_cost_to_node,closed,current_node,current_path_cost,current_depth)
                                        
    def depth_first_search(self, goal_state):
            start = time.time()            
            fringe = [self] 
            fringe_num_nodes_popped = 0
            fringe_max_length = 1            
            node_depth = [0]
            path_cost_to_node = [0] 
            closed = set([]) # record closed states
            
            while fringe:
                if len(fringe) > fringe_max_length:
                    fringe_max_length = len(fringe)
                current_node = fringe.pop() # select and remove the last node to be added from the fringe
                fringe_num_nodes_popped += 1                 
                current_depth = node_depth.pop()
                current_path_cost = path_cost_to_node.pop()
                closed.add(tuple(current_node.state.reshape(1,self.grid_size * self.grid_size)[0]))
                if np.array_equal(current_node.state,goal_state):
                    current_node.print_path()
                    print ('Time performance :::',str(fringe_num_nodes_popped),'nodes popped off the fringe.')
                    print ('Space performance:', str(fringe_max_length),'nodes in the fringe at its max.')
                    print ('Time spent: %0.2fs' % (time.time()-start))
                    return True                
                else:                                    
                    fringe,node_depth,path_cost_to_node = self.node_expansion(fringe,node_depth,path_cost_to_node,closed,current_node,current_path_cost,current_depth)                                        
                        
    def iterative_deepening(self, goal_state):
        start = time.time()        
        fringe_num_nodes_popped = 0 
        fringe_max_length = 1        
        for depth_limit in range(60): #iterative depth limit, actually infinite, just used 60 here randomly because i don't expect to have a depth of more than 60
            fringe = [self]
            node_depth = [0]
            path_cost_to_node = [0]
            closed = set([])
            while fringe:                
                if len(fringe) > fringe_max_length:
                    fringe_max_length = len(fringe)
                current_node = fringe.pop() # select and remove the last node to have been added (LIFO)
                fringe_num_nodes_popped += 1 
                current_depth = node_depth.pop() 
                current_path_cost = path_cost_to_node.pop()
                closed.add(tuple(current_node.state.reshape(1,self.grid_size * self.grid_size)[0]))
                if np.array_equal(current_node.state,goal_state):
                    current_node.print_path()
                    print ('Time performance :::',str(fringe_num_nodes_popped),'nodes popped off the fringe.')
                    print ('Space performance:', str(fringe_max_length),'nodes in the fringe at its max.')
                    print ('Time spent: %0.2fs' % (time.time()-start))
                    return True
                else:              
                    if current_depth < depth_limit:
                        fringe,node_depth,path_cost_to_node = self.node_expansion(fringe,node_depth,path_cost_to_node,closed,current_node,current_path_cost,current_depth)
                        
    def uniform_cost_search(self, goal_state):
        start = time.time()        
        fringe = [(self,0)] 
        fringe_num_nodes_popped = 0 
        fringe_max_length = 1         
        node_depth = [(0,0)]
        path_cost_to_node = [0]
        closed = set([])

        while fringe:
            fringe = sorted(fringe, key=lambda x: x[1]) # fringe of (found but unclosed nodes, path cost), ordered by path cost(accumulated step cost)
            node_depth = sorted(node_depth, key=lambda x: x[1])
            path_cost_to_node = sorted(path_cost_to_node, key=lambda x: x)                        
            if len(fringe) > fringe_max_length:
                fringe_max_length = len(fringe)                
            current_node = fringe.pop(0)[0] # select and remove the first node that was added ( prioritized queue)            
            fringe_num_nodes_popped += 1 
            current_depth = node_depth.pop(0)[0]
            current_path_cost = path_cost_to_node.pop(0)
            closed.add(tuple(current_node.state.reshape(1,self.grid_size * self.grid_size)[0]))
            if np.array_equal(current_node.state,goal_state):
                current_node.print_path()               
                print ('Time performance :::',str(fringe_num_nodes_popped),'nodes popped off the fringe.')
                print ('Space performance:', str(fringe_max_length),'nodes in the fringe at its max.')
                print ('Time spent: %0.2fs' % (time.time()-start))
                return True
            else:                                
                if current_node.try_move_down():
                    new_state  = np.array(current_node.try_move_down())                    
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                       
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost ,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                        fringe.append((current_node.move_down,current_path_cost+ self.normal_cost))
                        node_depth.append((current_depth+1,current_path_cost+ self.normal_cost))
                        path_cost_to_node.append(current_path_cost+ self.normal_cost)
                if current_node.try_move_right():
                    new_state  = np.array(current_node.try_move_right())                    
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                        
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                        fringe.append((current_node.move_right,current_path_cost+ self.normal_cost))
                        node_depth.append((current_depth+1,current_path_cost+ self.normal_cost))
                        path_cost_to_node.append(current_path_cost+self.normal_cost)                                 
                if current_node.try_move_up():
                    new_state  = np.array(current_node.try_move_up())                    
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                        
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                        fringe.append((current_node.move_up,current_path_cost+ self.normal_cost))
                        node_depth.append((current_depth+1,current_path_cost+ self.normal_cost))
                        path_cost_to_node.append(current_path_cost+self.normal_cost)                
                if current_node.try_move_left():
                    new_state  = np.array(current_node.try_move_left())                    
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                        
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                        fringe.append((current_node.move_left,current_path_cost+ self.normal_cost))
                        node_depth.append((current_depth+1,current_path_cost+ self.normal_cost))
                        path_cost_to_node.append(current_path_cost+self.normal_cost)                                               
                if self.allow_diagonals:                
                    if current_node.try_diagonal_45():
                        new_state  = np.array(current_node.try_diagonal_45())                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                            
                            current_node.diagonal_45 = Node(state=new_state,parent=current_node,action='diagonal_45',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                            fringe.append((current_node.diagonal_45,current_path_cost+self.diagonal_cost))
                            node_depth.append((current_depth+1,current_path_cost+self.diagonal_cost))
                            path_cost_to_node.append(current_path_cost+self.diagonal_cost)                                     
                    if current_node.try_diagonal_135():
                        new_state  = np.array(current_node.try_diagonal_135())                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:
                            current_node.diagonal_135 = Node(state=new_state,parent=current_node,action='diagonal_135',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                            fringe.append((current_node.diagonal_135,current_path_cost+self.diagonal_cost))
                            node_depth.append((current_depth+1,current_path_cost+self.diagonal_cost))
                            path_cost_to_node.append(current_path_cost+self.diagonal_cost)                                        
                    if current_node.try_diagonal_225():
                        new_state  = np.array(current_node.try_diagonal_225())                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                            
                            current_node.diagonal_225 = Node(state=new_state,parent=current_node,action='diagonal_225',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                            fringe.append((current_node.diagonal_225,current_path_cost+self.diagonal_cost))
                            node_depth.append((current_depth+1,current_path_cost+self.diagonal_cost))
                            path_cost_to_node.append(current_path_cost+self.diagonal_cost)                             
                    if current_node.try_diagonal_315():  
                        new_state  = np.array(current_node.try_diagonal_315())                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                            
                            current_node.diagonal_315 = Node(state=new_state,parent=current_node,action='diagonal_315',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=0)
                            fringe.append((current_node.diagonal_315,current_path_cost+self.diagonal_cost))
                            node_depth.append((current_depth+1,current_path_cost+self.diagonal_cost))
                            path_cost_to_node.append(current_path_cost+self.diagonal_cost)

    def greedy_search(self, goal_state):
        start = time.time()        
        fringe = [(self,0)] # fringe of (found but unclosed nodes, heuristic cost), ordered by heuristic cost
        fringe_num_nodes_popped = 0 
        fringe_max_length = 1         
        node_depth = [(0,0)] # fringe of node depth, (depth, heuristic cost)
        path_cost_to_node = [(0,0)] # fringe for path cost, (path_cost, heuristic cost)
        closed = set([])
        
        while fringe:
            # sort fringe based on heuristic cost, in ascending order
            fringe = sorted(fringe, key=lambda x: x[1])
            node_depth = sorted(node_depth, key=lambda x: x[1])
            path_cost_to_node = sorted(path_cost_to_node, key=lambda x: x[1]) 
            if len(fringe) > fringe_max_length:
                fringe_max_length = len(fringe)
                
            current_node = fringe.pop(0)[0] # prioritized queue
            fringe_num_nodes_popped += 1 
            current_depth = node_depth.pop(0)[0] 
            current_path_cost = path_cost_to_node.pop(0)[0]
            closed.add(tuple(current_node.state.reshape(1,self.grid_size * self.grid_size)[0])) 
            if np.array_equal(current_node.state,goal_state):
                current_node.print_path()   
                print ('Time performance :::',str(fringe_num_nodes_popped),'nodes popped off the fringe.')
                print ('Space performance:', str(fringe_max_length),'nodes in the fringe at its max.')
                print ('Time spent: %0.2fs' % (time.time()-start))
                return True            
            else:                      
                if current_node.try_move_down():
                    new_state  = np.array(current_node.try_move_down())                    
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                        
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False:
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            self.h_cost = self.h_manhattan(new_state,goal_state)          
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost ,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_down,self.h_cost))
                        node_depth.append((current_depth+1,self.h_cost))
                        path_cost_to_node.append((current_path_cost+ self.normal_cost,self.h_cost))
                if current_node.try_move_right():
                    new_state  = np.array(current_node.try_move_right())            
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                   
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False:
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            self.h_cost = self.h_manhattan(new_state,goal_state)
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_right,self.h_cost))
                        node_depth.append((current_depth+1,self.h_cost))
                        path_cost_to_node.append((current_path_cost+self.normal_cost,self.h_cost))  
                if current_node.try_move_up():
                    new_state  = np.array(current_node.try_move_up())              
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                       
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False:
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            self.h_cost = self.h_manhattan(new_state,goal_state)
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_up,self.h_cost))
                        node_depth.append((current_depth+1,self.h_cost))
                        path_cost_to_node.append((current_path_cost+self.normal_cost,self.h_cost))               
                if current_node.try_move_left():
                    new_state  = np.array(current_node.try_move_left())
                    
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False: 
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            
                            self.h_cost = self.h_manhattan(new_state,goal_state)
                        
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_left,self.h_cost))
                        node_depth.append((current_depth+1,self.h_cost))
                        path_cost_to_node.append((current_path_cost+self.normal_cost,self.h_cost))

                if self.allow_diagonals:                
                    if current_node.try_diagonal_45():
                        new_state  = np.array(current_node.try_diagonal_45())         
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                   
                            if self.diagonal_cost==1.5:                                
                                self.h_cost= self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :                                
                                self.h_cost = self.h_manhattan(new_state,goal_state)
                            current_node.diagonal_45 = Node(state=new_state,parent=current_node,action='diagonal_45',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_45,self.h_cost))
                            node_depth.append((current_depth+1,self.h_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,self.h_cost))     
                    if current_node.try_diagonal_135():
                        new_state  = np.array(current_node.try_diagonal_135())         
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                     
                            if self.diagonal_cost==1.5:
                                self.h_cost = self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :
                                self.h_cost = self.h_manhattan(new_state,goal_state)
                            current_node.diagonal_135 = Node(state=new_state,parent=current_node,action='diagonal_135',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_135,self.h_cost))
                            node_depth.append((current_depth+1,self.h_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,self.h_cost))                                        
                    if current_node.try_diagonal_225():
                        new_state  = np.array(current_node.try_diagonal_225())                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:
                            if self.diagonal_cost==1.5:
                                self.h_cost = self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :
                                self.h_cost = self.h_manhattan(new_state,goal_state)                            
                            current_node.diagonal_225 = Node(state=new_state,parent=current_node,action='diagonal_225',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_225,self.h_cost))
                            node_depth.append((current_depth+1,self.h_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,self.h_cost))                            
                    if current_node.try_diagonal_315():  
                        new_state  = np.array(current_node.try_diagonal_315())                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:
                            if self.diagonal_cost==1.5:
                                self.h_cost = self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :
                                self.h_cost = self.h_manhattan(new_state,goal_state)                            
                            current_node.diagonal_315 = Node(state=new_state,parent=current_node,action='diagonal_315',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_315,self.h_cost))
                            node_depth.append((current_depth+1,self.h_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,self.h_cost))    
    
    def a_star_search(self,goal_state):
        start = time.time()
        
        fringe = [(self,0)] # fringe of (found but unclosed nodes, total cost), ordered by the second element
        fringe_num_nodes_popped = 0
        fringe_max_length = 1        
        node_depth = [(0,0)]
        path_cost_to_node = [(0,0)]
        closed = set([])         
        while fringe:
            # sort fringe based on path_cost+heuristic cost, in ascending order
            fringe = sorted(fringe, key=lambda x: x[1])
            node_depth = sorted(node_depth, key=lambda x: x[1])
            path_cost_to_node = sorted(path_cost_to_node, key=lambda x: x[1])  
            if len(fringe) > fringe_max_length:
                fringe_max_length = len(fringe)  
            current_node = fringe.pop(0)[0] 
            fringe_num_nodes_popped += 1 
            current_depth = node_depth.pop(0)[0] 
            current_path_cost = path_cost_to_node.pop(0)[0]
            closed.add(tuple(current_node.state.reshape(1,self.grid_size * self.grid_size)[0])) 
            if np.array_equal(current_node.state,goal_state):
                current_node.print_path()       
                print ('Time performance :::',str(fringe_num_nodes_popped),'nodes popped off the fringe.')
                print ('Space performance:', str(fringe_max_length),'nodes in the fringe at its max.')
                print ('Time spent: %0.2fs' % (time.time()-start))
                return True            
            else:                  
                if current_node.try_move_down():
                    new_state  = np.array(current_node.try_move_down())
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False:
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            self.h_cost = self.h_manhattan(new_state,goal_state)
                        path_cost=current_path_cost + self.normal_cost
                        total_cost = path_cost + self.h_cost
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost ,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_down,total_cost))
                        node_depth.append((current_depth+1,total_cost))
                        path_cost_to_node.append((current_path_cost+ self.normal_cost,total_cost))          
                if current_node.try_move_right():
                    new_state  = np.array(current_node.try_move_right())             
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                   
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False:
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            self.h_cost = self.h_manhattan(new_state,goal_state)
                        path_cost=current_path_cost + self.normal_cost
                        total_cost = path_cost + self.h_cost
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_right,total_cost))
                        node_depth.append((current_depth+1,total_cost))
                        path_cost_to_node.append((current_path_cost+self.normal_cost,total_cost))
                if current_node.try_move_up():
                    new_state  = np.array(current_node.try_move_up())
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:              
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False:
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            self.h_cost = self.h_manhattan(new_state,goal_state)
                        path_cost=current_path_cost + self.normal_cost
                        total_cost = path_cost + self.h_cost
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_up,total_cost))
                        node_depth.append((current_depth+1,total_cost))
                        path_cost_to_node.append((current_path_cost+self.normal_cost,total_cost))     
                if current_node.try_move_left():
                    new_state  = np.array(current_node.try_move_left())   
                    if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:
                        if (self.allow_diagonals == True and self.diagonal_cost==1.5) or self.allow_diagonals == False:                            
                            self.h_cost = self.h_sld(new_state,goal_state)
                        elif self.allow_diagonals == True  and self.diagonal_cost == 1 :
                            self.h_cost = self.h_manhattan(new_state,goal_state)
                        path_cost=current_path_cost + self.normal_cost
                        total_cost = path_cost + self.h_cost              
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              diagonal_cost= self.normal_cost,path_cost=current_path_cost+ self.normal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                        fringe.append((current_node.move_left,total_cost))
                        node_depth.append((current_depth+1,total_cost))
                        path_cost_to_node.append((current_path_cost+self.normal_cost,total_cost))
              
                if self.allow_diagonals:      
                    if current_node.try_diagonal_45():
                        new_state  = np.array(current_node.try_diagonal_45())             
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:            
                            if self.diagonal_cost==1.5:                                
                                self.h_cost = self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :                                
                                self.h_cost = self.h_manhattan(new_state,goal_state)
                            path_cost=current_path_cost + self.diagonal_cost
                            total_cost = path_cost + self.h_cost             
                            current_node.diagonal_45 = Node(state=new_state,parent=current_node,action='diagonal_45',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_45,total_cost))
                            node_depth.append((current_depth+1,total_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,total_cost))     
                    if current_node.try_diagonal_135():
                        new_state  = np.array(current_node.try_diagonal_135())        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:                  
                            if self.diagonal_cost==1.5:
                                self.h_cost = self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :
                                self.h_cost = self.h_manhattan(new_state,goal_state)
                            path_cost=current_path_cost + self.diagonal_cost
                            total_cost = path_cost + self.h_cost
                            current_node.diagonal_135 = Node(state=new_state,parent=current_node,action='diagonal_135',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_135,total_cost))
                            node_depth.append((current_depth+1,total_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,total_cost))                              
                    if current_node.try_diagonal_225():
                        new_state  = np.array(current_node.try_diagonal_225())
                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:
                            if self.diagonal_cost==1.5:
                                self.h_cost = self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :
                                self.h_cost = self.h_manhattan(new_state,goal_state)
                            path_cost=current_path_cost + self.diagonal_cost
                            total_cost = path_cost + self.h_cost # difference with the greedy search method
                            
                            current_node.diagonal_225 = Node(state=new_state,parent=current_node,action='diagonal_225',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_225,total_cost))
                            node_depth.append((current_depth+1,total_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,total_cost)) 
                            
                    if current_node.try_diagonal_315():  
                        new_state  = np.array(current_node.try_diagonal_315())
                        
                        if tuple(new_state.reshape(1,self.grid_size * self.grid_size)[0]) not in closed:
                            if self.diagonal_cost==1.5:
                                self.h_cost = self.h_sld(new_state,goal_state)
                            elif self.diagonal_cost == 1 :
                                self.h_cost = self.h_manhattan(new_state,goal_state)
                            
                            path_cost=current_path_cost + self.diagonal_cost
                            total_cost = path_cost + self.h_cost
                            current_node.diagonal_315 = Node(state=new_state,parent=current_node,action='diagonal_315',depth=current_depth+1,\
                                                  diagonal_cost=self.diagonal_cost,path_cost=current_path_cost+self.diagonal_cost,grid_size=self.grid_size,allow_diagonals=self.allow_diagonals,heuristic_cost=self.h_cost)
                            fringe.append((current_node.diagonal_315,total_cost))
                            node_depth.append((current_depth+1,total_cost))
                            path_cost_to_node.append((current_path_cost+self.diagonal_cost,total_cost))
                
grid = []

def diagonality():    
    k = input( 'are diagonals allowed(yes/no):')
    if k=='yes':
        diagonal = True
        cost = 1 #default value of cost is one
        choice = int( input(' Pleas select 1 or 2 \n 1. diagonal movt cost = 1  \n 2. Diagonal movement cost = 1.5 \n Your choice : '))
        if choice == 1:
            cost =1
        elif choice == 2:
            print('your choice is 2\n')
            cost = 1.5
        else:
            print('wrong selection\n')
    else:
        cost = 0
        diagonal = False
    return (diagonal,cost)


def Grid_creation():
    k = int(input('please enter the size of the Grid:'))
    gridmatrix = []
    for i in range(0,k):
        gridtemp = []
        for j in range(0,k):
            gridtemp.append('.')
        gridmatrix.append(gridtemp)
    obs = int(input('please specify number of obstacles: '))
    for v in range (0,obs): 
        c , r =input('Please, give start position of the robot (column and row, numbering columns starting from 1 left to right and rows starting from 1 down to up) ').split()
        #c= int(input('specify column of obstacle'))
        gridmatrix[k-int(r)][int(c)-1]='X'
    start_column , start_row = input('please input row and column of start position(same convention as above): ').split()
    #start_column = int(input('please input colunm of start position\n'))
    gridmatrix[k-int(start_row)][int(start_column)-1]='R'
    
    goal_col,goal_row = input('please enter row and colunm number of goal position:(same convention as above) ').split()
    #goal_col = int(input('please enter colunm number of goal position:\n'))
    gridmatrix[k-int(goal_row)][int(goal_col)-1]='G'  
    print('Congratulations , grid created!!\n')  
    return np.array(gridmatrix) 

def goal_state(matrix):
    grid1 = matrix
    start= np.where(grid1=='R')
    grid1[int(start[0])][int(start[1])] = '.'
    goal= np.where(grid1=='G')
    grid1[int(goal[0])][int(goal[1])] = 'R'
    return grid1

def algorithms():
    grid_sub = np.copy(grid)
    g_state = goal_state(grid_sub)
    #print('your goal state is :\n {}'.format(g_state))
    diag_val  = diagonality()
    diag_allowed = diag_val[0]
    diag_cost = diag_val[1]
    print('your diag is : {} and {}'.format(diag_val[0],diag_val[1]))  
    n = len(grid[0])
    algorithm = int(input('please select search method:\n 1.Breadth First \n 2.Uniform cost \n 3.Depth first \n 4. iterative deepening \n 5. Greedy \n 6. A* \n Your choice: '))
    

    
    if algorithm == 1:
        print('Here is the bfs search evaluation\n')
        root_node = Node(state=grid,parent=None,action=None,depth=0,diagonal_cost=diag_cost,path_cost=0,grid_size = n,allow_diagonals= diag_allowed, heuristic_cost=0)
        root_node.breadth_first_search(g_state)
    if algorithm == 2:
        print('Here is the ucs search evaluation\n')
        root_node = Node(state=grid,parent=None,action=None,depth=0,diagonal_cost=diag_cost,path_cost=0,grid_size = n,allow_diagonals= diag_allowed, heuristic_cost=0)
        root_node.uniform_cost_search(g_state)
    if algorithm == 3:
        print('Here is the dfs search evaluation\n')
        root_node = Node(state=grid,parent=None,action=None,depth=0,diagonal_cost=diag_cost,path_cost=0,grid_size = n,allow_diagonals= diag_allowed, heuristic_cost=0)
        root_node.depth_first_search(g_state)
    if algorithm == 4:
        print('Here is the idfs search evaluation\n')
        root_node = Node(state=grid,parent=None,action=None,depth=0,diagonal_cost=diag_cost,path_cost=0,grid_size = n,allow_diagonals= diag_allowed, heuristic_cost=0)
        root_node.iterative_deepening(g_state)
    if algorithm == 5:
        print('Here is the greedy search evaluation\n')
        root_node = Node(state=grid,parent=None,action=None,depth=0,diagonal_cost=diag_cost,path_cost=0,grid_size = n,allow_diagonals= diag_allowed, heuristic_cost=0)
        root_node.greedy_search(g_state)
    if algorithm == 6:
        print('Here is the A* evaluation of the search\n')
        root_node = Node(state=grid,parent=None,action=None,depth=0,diagonal_cost=diag_cost,path_cost=0,grid_size = n,allow_diagonals= diag_allowed, heuristic_cost=0)
        root_node.a_star_search(g_state)
        
def last_menu():    
    select =int( input(' Please select 0,1 or 2 \n 1. Give new input \n 2. Search again on current grid \n 0. Exit \n Your choice : '))
    if select == 0:
        exit
    elif select == 2:
        algorithms()
        last_menu()
    elif select == 1:
        new_input()
    else:
        print('wrong input: program terminating!!!\n')
        exit      
def new_input():
    global grid
    grid = Grid_creation()
    print('your grid is :\n {}'.format(grid))
    algorithms()
    last_menu()
n =int( input(' Please select 0 or 1 \n 1. Give new input \n 0. Exit \n Your choice : '))    
if n==0:
    exit    
elif n==1:
    new_input()