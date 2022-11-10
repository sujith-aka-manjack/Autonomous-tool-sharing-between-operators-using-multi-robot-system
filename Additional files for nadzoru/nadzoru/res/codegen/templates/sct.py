import random
import yaml

class SCT:

    ### PUBLIC FUNCTIONS ###

    def __init__(self, filename):

        self._read_supervisor(filename)

        self.callback = {}      # Dict of callback functions
        self.input_buffer = []  # Buffer to record uncontrollable events that occured since the last iteration
        self.last_events = [0] * len(self.EV)   # TODO: Add comment

    # Register a callback function for the given event
    def add_callback(self, event, clbk, ci, sup_data):
        func = {}
        func['callback']    = clbk
        func['check_input'] = ci
        func['sup_data']    = sup_data
        self.callback[self._check_value(event)] = func

    # Run the generator player to execute the next action
    def run_step(self):
        self.input_buffer = []  # Clear buffer
        self._update_input()

        # Get all uncontrollable events
        uce = self.input_buffer

        # Apply all the uncontrollable events
        while uce:
            event = uce.pop(0)
            self._make_transition(event)
            self._exec_callback(event)   # Not used

        # Get the next controllable event
        ce_exists, ce = self._get_next_controllable()

        # Apply the chosen controllable event
        if ce_exists:
            self._make_transition(ce)
            self._exec_callback(ce)

    # Return event info (name of events, controllability)
    def get_events(self):
        return self.EV, self.ev_controllable

    ### PRIVATE FUNCTIONS ###

    # Load supervisor from the yaml file
    def _read_supervisor(self, filename):
        try:
            with open(filename, 'r') as stream:
                f = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print(e) 

        # Supervisor info data structure
        self.num_events = f['num_events']
        self.num_supervisors = f['num_supervisors']
        self.ev_controllable = f['ev_controllable']
        self.EV = {}
        for i, ev in enumerate(f['events']):
            self.EV[ev] = i
        self.sup_events = f['sup_events']
        self.sup_init_state = f['sup_init_state']
        self.sup_current_state = f['sup_current_state']
        self.sup_data_pos = f['sup_data_pos']
        self.sup_data = f['sup_data']

    # Return whether an uncontrollable event has occured
    def _input_read(self, ev):
        if ev < self.num_events and self.callback[ev]:
            return self.callback[ev]['check_input'](self.callback[ev]['sup_data'])
        return False

    # Add uncontrollable events that has occured to the buffer (Without checking for last_events)
    def _update_input(self):
        for i in range(0,self.num_events):
            if not self.ev_controllable[i]: # Check only the UCEs
                if self._input_read(i):
                    self.input_buffer.append(i)
                    self.last_events[i] = 1

    # Given the supervisor and its state, return the position of the current state in the data structure
    def _get_state_position(self, supervisor, state):
        position = self.sup_data_pos[supervisor]    # Jump to the start position of the supervisor
        for s in range(0, state):                   # Keep iterating until the state is reached
            en = self.sup_data[position]            # The number of transitions in the state
            position += en * 3 + 1                  # Next state position (Number transitions * 3 + 1)
        return position

    # When accessing sup_data, if the value is an event name, convert it to its associated event number
    def _check_value(self, index):
        if isinstance(index, str):
            return self.EV[index]    
        return index

    # Apply the transition from current state
    def _make_transition(self, ev):
        num_transitions = None

        # Apply transition to each local supervisor
        for i in range(0, self.num_supervisors):
            if self.sup_events[i][ev]:  # Check if the given event is part of this supervisor
                
                # Current state info of supervisor
                position = self._get_state_position(i, self.sup_current_state[i])
                num_transitions = self.sup_data[position]
                position += 1   # Point to first transition

                # Find the transition for the given event
                while num_transitions:
                    num_transitions -= 1
                    value = self._check_value(self.sup_data[position])
                    if value == ev:
                        self.sup_current_state[i] = (self.sup_data[position + 1] * 256) + (self.sup_data[position + 2])
                        break
                    position += 3

    # Execute callback function
    def _exec_callback(self, ev):
        if ev < self.num_events and self.callback[ev]['callback']:
            self.callback[ev]['callback'](self.callback[ev]['sup_data'])

    # Choose a controllale event from the list of enabled controllable events
    def _get_next_controllable(self):
        
        # Get controllable events that are enabled
        actives = self._get_active_controllable_events()
        
        if not all(v == 0 for v in actives):    # If at least one event is enabled do
            randomPos = random.randrange(actives.count(1))  # Pick a random index (event)
            for i in range(0, self.num_events):
                if not randomPos and actives[i]:
                    return True, i
                elif actives[i]:
                    randomPos -= 1

        return False, None

    # Return all the enabled controllable events
    def _get_active_controllable_events(self):

        events = self.ev_controllable.copy()

        # Check if a controllable event is disabled in any of the supervisors
        for i in range(0, self.num_supervisors):
            ev_disable = [1] * self.num_events  # Init a list where all events are disabled
            for j in range(0, self.num_events): # Enable all events that are not part of this supervisor
                if not self.sup_events[i][j]:
                    ev_disable[j] = 0

            # Get current state
            position = self._get_state_position(i, self.sup_current_state[i])
            num_transitions = self.sup_data[position]
            position += 1

            # Enable all events that have a transition from the current state
            while num_transitions:  
                num_transitions -= 1
                value = self._check_value(self.sup_data[position])
                ev_disable[value] = 0
                position += 3

            # Remove the controllable events to disable, leaving a list of enabled controllable events
            for j in range(0, self.num_events):
                if ev_disable[j] == 1 and events[j]:
                    events[j] = 0

        return events


class SCTProb(SCT):

    ### PUBLIC FUNCTIONS ###

    def __init__(self, filename):
        super().__init__(filename)

    # Set the probability of a transition
    def set_event_prob(self, supervisor, state, event, prob):
        prob_position = self._get_state_position_prob(supervisor, state)
        prob_position += 1  # Point to first probability of the state in the supervisor

        prob_position += self.EV_prob[event]  # Point to the given event's probability

        self.sup_data_prob[prob_position] = prob    # Set new probability

    ### PRIVATE FUNCTIONS ###

    # Load supervisor from the yaml file
    def _read_supervisor(self, filename):
        try:
            with open(filename, 'r') as stream:
                f = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print(e) 

        # Supervisor info data structure
        self.num_events = f['num_events']
        self.num_supervisors = f['num_supervisors'] 
        self.ev_controllable = f['ev_controllable']
        self.EV = {}
        self.EV_prob = {}
        n = 0
        for i, ev in enumerate(f['events']):
            self.EV[ev] = i
            if self.ev_controllable[i]:
                self.EV_prob[ev] = n
                n += 1
        self.sup_events = f['sup_events']
        self.sup_init_state = f['sup_init_state']
        self.sup_current_state = f['sup_current_state']
        self.sup_data_pos = f['sup_data_pos']
        self.sup_data = f['sup_data']
        self.sup_data_prob_pos = f['sup_data_prob_pos'] # Probability info
        self.sup_data_prob = f['sup_data_prob']

    # Given the supervisor and its state, return the position of the current state's probabilities in the data structure
    def _get_state_position_prob(self, supervisor, state):
        prob_position = self.sup_data_prob_pos[supervisor]  # Jump to the start position of the supervisor's probabilities
        for s in range(0, state):                           # Keep iterating until the state is reached
            en = self.sup_data_prob[prob_position]          # The number of probabilities in the state
            prob_position += en + 1                         # Next event's probability position
        return prob_position

    # Choose a controllale event from the list of enabled controllable events using probabilities
    def _get_next_controllable(self):
        # Get controllable events that are enabled
        prob_sum, actives = self._get_active_controllable_events_prob()

        if prob_sum > 0.0001:   # If at least one event is enabled do
            random_value = prob_sum * random.random()
            random_sum = 0
            for i in range(0, self.num_events):
                random_sum += actives[i]    # Add probability of each event until the random value is reached
                if random_value <= random_sum and self.ev_controllable[i]:
                    return True, i

        return False, None

    # Return all the enabled controllable event probabilities and the probability sum
    def _get_active_controllable_events_prob(self):

        prob_sum = 0
        events = self.ev_controllable.copy()

        # Check if a controllable event is disabled in any of the supervisors
        for i in range(0, self.num_supervisors):
            ev_disable = [1] * self.num_events  # Init a list where all events are disabled
            for j in range(0, self.num_events): # Enable all events that are not part of this supervisor
                if not self.sup_events[i][j]:
                    ev_disable[j] = 0

            # Get current state and transition probabilities
            position = self._get_state_position(i, self.sup_current_state[i])
            prob_position = self._get_state_position_prob(i, self.sup_current_state[i])
            num_transitions = self.sup_data[position]
            position += 1
            prob_position += 1

            # Enable all events that have a transition from the current state
            while num_transitions:
                num_transitions -= 1
                value = self._check_value(self.sup_data[position])
                if self.ev_controllable[value] and self.sup_events[i][value]: # Check if the event is controllable and exists in this supervisor
                    ev_disable[value] = 0
                    events[value] *= self.sup_data_prob[prob_position]  # Cumulatively multiply the event's probability from all supervisors
                    prob_position += 1
                position += 3

            # Remove the controllable events to disable, leaving a list of enabled controllable events
            for i in range(0, self.num_events):
                if ev_disable[i] == 1:  # Event multiplied by a probability should be < 1
                    events[i] = 0

        # Sum the probabilities
        for i in range(0, self.num_events):
            prob_sum += events[i]

        return prob_sum, events


class SCTPublic(SCT):

    ### PUBLIC FUNCTIONS ###

    def __init__(self, filename):
        super().__init__(filename)

    # Run the generator player to execute the next action and return public events to broadcast
    def run_step(self):
        self.input_buffer = []  # Clear buffer
        self._update_input()

        # Get public and private uncontrollable events
        uce_pub = []
        uce_priv = []
        for event in self.input_buffer: # Split uncontrollable events
            (uce_pub, uce_priv)[self.ev_shared[event]].append(event)

        # Apply all public uncontrollable events (external)
        while uce_pub:
            event = uce_pub.pop(0)
            self._make_transition(event)
            self._exec_callback(event)   # Not used

        self.event_to_send = [] # Public events to broadcast

        # Apply all private uncontrollable events
        while uce_priv:
            event = uce_priv.pop(0)
            self._make_transition(event)
            self._exec_callback(event)   # Not used
            self._send_public_event(event)

        # Get the next controllable event
        ce_exists, ce = self._get_next_controllable()

        # Apply the chosen controllable event
        if ce_exists:
            self._make_transition(ce)
            self._exec_callback(ce)
            self._send_public_event(ce)

        return self.event_to_send

    ### PRIVATE FUNCTIONS ###

    # Load supervisor from the yaml file
    def _read_supervisor(self, filename):
        try:
            with open(filename, 'r') as stream:
                f = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print(e) 

        # Supervisor info data structure
        self.num_events = f['num_events']
        self.num_supervisors = f['num_supervisors']
        self.EV = {}
        for i, ev in enumerate(f['events']):
            self.EV[ev] = i
        self.ev_shared = f['ev_shared']
        self.ev_controllable = f['ev_controllable']
        self.sup_events = f['sup_events']
        self.sup_init_state = f['sup_init_state']
        self.sup_current_state = f['sup_current_state']
        self.sup_data_pos = f['sup_data_pos']
        self.sup_data = f['sup_data']

    # Store public events to broadcase
    def _send_public_event(self, event):
        if self.ev_shared[event]:
            self.event_to_send.append(event)


class SCTProbPublic(SCTProb, SCTPublic):
    
    ### PUBLIC FUNCTIONS ###

    def __init__(self, filename):
        super().__init__(filename)

    ### PRIVATE FUNCTIONS ###

    # Load supervisor from the yaml file
    def _read_supervisor(self, filename):
        try:
            with open(filename, 'r') as stream:
                f = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print(e) 

        # Supervisor info data structure
        self.num_events = f['num_events']
        self.num_supervisors = f['num_supervisors'] 
        self.ev_controllable = f['ev_controllable']
        self.ev_shared = f['ev_shared']
        self.EV = {}
        self.EV_prob = {}
        n = 0
        for i, ev in enumerate(f['events']):
            self.EV[ev] = i
            if self.ev_controllable[i]:
                self.EV_prob[ev] = n
                n += 1
        self.sup_events = f['sup_events']
        self.sup_init_state = f['sup_init_state']
        self.sup_current_state = f['sup_current_state']
        self.sup_data_pos = f['sup_data_pos']
        self.sup_data = f['sup_data']
        self.sup_data_prob_pos = f['sup_data_prob_pos'] # Probability info
        self.sup_data_prob = f['sup_data_prob']


if __name__ == '__main__':

    # # TEST SCTProb

    # count_a = 0.0
    # count_b = 0.0

    # def ce_a(data):
    #     # print('a')
    #     global count_a
    #     count_a += 1

    # def ce_b(data):
    #     # print('b')
    #     global count_b
    #     count_b += 1

    # def uce_c(data):
    #     # print('c')
    #     return True

    # def uce_d(data):
    #     # print('d')
    #     return False

    # # sct = SCT('/home/genki/GIT/hs-team-py/supervisors/sup_simple.yaml')
    # sct = SCTProb('/home/genki/GIT/hs-team-py/supervisors/sup_simple_prob.yaml')

    # sct.add_callback('EV_a', ce_a, None, None) # EV_a, controllable
    # sct.add_callback('EV_b', ce_b, None, None) # EV_b, controllable
    # sct.add_callback('EV_c', None, uce_c, None) # EV_c, uncontrollable
    # sct.add_callback('EV_d', None, uce_d, None) # EV_d, uncontrollable

    # for i in range(0,1000):
    #     sct.run_step()

    # count_sum = count_a + count_b
    # print('a : {}'.format(count_a/count_sum))
    # print('b : {}'.format(count_b/count_sum))

    # count_a = 0
    # count_b = 0
    # count_sum = 0
    # sct.set_event_prob(0, 1, 'EV_a', 0.9) # prob a to 0.9
    # sct.set_event_prob(0, 1, 'EV_b', 0.1) # prob b to 0.1

    # for i in range(0,1000):
    #     sct.run_step()

    # count_sum = count_a + count_b
    # print('a : {}'.format(count_a/count_sum))
    # print('b : {}'.format(count_b/count_sum))


    # Test SCTPublic

    sct1_b = False
    sct2_b = False

    def ce_a(data):
        print('a')

    def ce_b(data):
        print('b')

    def ce_c(data):
        print('c')

    def uce__b(data):
        global sct1_b
        global sct2_b
        if data == 'sct1' and sct2_b:
            print('received _b from sct2')
            return True
        elif data == 'sct2' and sct1_b:
            print('received _b from sct1')
            return True
        return False

    # sct1 = SCTPublic('/home/genki/GIT/hs-team-py/supervisors/sup_simple_public.yaml')
    sct1 = SCTProbPublic('/home/genki/GIT/hs-team-py/supervisors/sup_simple_prob_public.yaml')
    sct1.add_callback('EV_a', ce_a, None, None) # EV_a, controllable
    sct1.add_callback('EV_b', ce_b, None, None) # EV_b, controllable, public
    sct1.add_callback('EV_c', ce_c, None, None) # EV_c, controllable
    sct1.add_callback('EV__b', None, uce__b, 'sct1') # EV__b, uncontrollable, public

    # sct2 = SCTPublic('/home/genki/GIT/hs-team-py/supervisors/sup_simple_public.yaml')
    sct2 = SCTProbPublic('/home/genki/GIT/hs-team-py/supervisors/sup_simple_prob_public.yaml')
    sct2.add_callback('EV_a', ce_a, None, None) # EV_a, controllable
    sct2.add_callback('EV_b', ce_b, None, None) # EV_b, controllable, public
    sct2.add_callback('EV_c', ce_c, None, None) # EV_c, controllable
    sct2.add_callback('EV__b', None, uce__b, 'sct2') # EV__b, uncontrollable, public

    for i in range(10):
        print('---- {} ----'.format(i))

        msg1 = sct1.run_step()
        msg2 = sct2.run_step()

        sct1_b = 3 in msg1  # Check if other robot has selected b
        sct2_b = 3 in msg2
