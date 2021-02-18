#! /usr/bin/env python

from behaviors import *
from menu import *
import random

class StateBase:
    """
    Base class from which every state inherits from. Each child has
    to provide an implementation for the following methods:
        0. __init__()
        1. _set_id()
        2. _pre_execution()
        3. _execution()
        4. _post_execution()
        5. _get_next_state()
    """
    def __init__(self):
        self.id = self._set_id()
        self.reset()
        
        # TODO: Maybe create objects of each behavior here?

    def reset(self):
        """
        This method gets called by the StateMachine to reset the state
        before transitioning to another one. This allows for a full run
        pass the next time the state gets activated.
        """
        self.run_pre_execution = True
        self.run_execution = True
        self.run_post_execution = True

    def get_id(self):
        """
        Return the state's id.
        """
        return self.id

    def run(self):
        """
        This method gets called by the StateMachine to execute
        actions associated with the state (if active). By default,
        self.run_pre_execution() and self._execution() will only run once
        after the activation while self._post_execution() will be ran in a
        loop and can be used to idle while waiting for a transition.
        """
        if self.run_pre_execution:
            self._pre_execution()
            self.run_pre_execution = False

        elif self.run_execution():
            self._execution()
            self.run_execution = False

        elif self.run_post_execution:
            self._post_execution()

    def _set_id(self):
        """ 
        Set the state's id used by the StateMachine for transitions.
        This method should return a string containing the id.
        """
        raise NotImplementedError()

    def _pre_execution(self):
        """
        Initialization stage of the run method. If self.run_pre_execution
        is not set to true in self._post_execution(), this method will only
        run once after a transition to the state.
        """
        raise NotImplementedError()

    def _execution(self):
        """
        Main stage of the run method. If self.run_execution is not set to
        true in self._post_execution(), this method will only run once after
        a transition to the state.
        """
        raise NotImplementedError()

    def _post_execution(self):
        """
        Post stage of the run method. This method can be used for idling since
        it will be executed in a loop until there's a transition to another
        state. The execution of this method could be blocked by setting
        self.run_post_execution to false.
        """
        raise NotImplementedError()

    def get_next_state(self):
        """
        This method should return the next state's id (string) if an event allows
        a transition. If there is no transition, it should return None.
        """
        raise NotImplementedError()


"""
+-------------------------------------------------+
|                Global Variables                 |
+-------------------------------------------------+
"""
# TODO: Maybe change them for static variables instead?
face_tracking = FaceTracking()
voice_recognition = VoiceRecognition()
voice = Voice()
locomotion = Locomotion()

master_position = {
        "x" : "0",
        "y" : "0",
        "orientation" : "0"
    }

kitchen_position = {
        "x" : "1",
        "y" : "1",
        "orientation" : "180"
    }

"""
+-------------------------------------------------+
|                 States 0 to 5                   |
+-------------------------------------------------+
"""
class State00(StateBase):
    """
    The robot is idling
    """
# /!\ WARNING: This code wasn't tested! /!\
    def __init__(self):
        StateBase.__init__(self)
        
        self.voice_recognition_params = {
            "language": "en-us",
            "skip_keyword": "False",
            "tell_back": "False"
           }

        self.last_face_timestamp = face_tracking.timestamp

    def _set_id(self):
        return 'state 00'

    def _pre_execution(self):
        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.activate()
        voice.deactivate()
        locomotion.deactivate()

    def _execution(self):
        # TODO: Add code here if necessary...
        pass

    def _post_execution(self):
        
        # Checking last time a face was detected
        self.last_face_timestamp = face_tracking.timestamp

        # Listen for an order
        voice_recognition.run(self.voice_recognition_params)

    def get_next_state(self):

        #no face detected, continu idle
        if self.last_face_timestamp == face_tracking.timestamp:
            return None
        #Face detected, move to state 2 of greeting
        if self.last_face_timestamp != face_tracking.timestamp:
            return 'state 02'

        # If the robot is still seeing a face, but the voice recognition failed
        if voice_recognition.speech == "":
            return None

        # if the keyword is detected, move to state 2
        if voice_recognition.speech == keyword:
            return 'state 02'
        


class State01(StateBase):
    """
    The robot says goodbye
    """
# /!\ WARNING: This code wasn't tested! /!\
    def __init__(self):
        StateBase.__init__(self)
        
        self.voice_params = {
            "speech" : "",
            "language" : "en_GB"
            }

    def _set_id(self):
        return 'state 01'

    def _pre_execution(self):
        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.deactivate()
        voice.activate()
        locomotion.deactivate()

    def _execution(self):
        
        self.voice_params["speech"] = "Goodbye, come again Human!"
        
        voice.run(self.voice_params)

    def _post_execution(self):
        # TODO: Add code here if necessary...
        pass

    def get_next_state(self):
        return 'state 00'


class State02(StateBase):
    """
    The robot take the order
    """
    # /!\ WARNING: This code wasn't tested! /!\

    def __init__(self):
        StateBase.__init__(self)

        self.voice_params = {
            "speech" : "",
            "language" : "en_GB"
            }

        self.voice_recognition_params = {
            "language": "en-us",
            "skip_keyword": "False",
            "tell_back": "False"
           }
        #do i need it
        self.last_face_timestamp = face_tracking.timestamp

    def _set_id(self):
        return 'state 02'

    def _pre_execution(self):
        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.activate()
        voice.activate()
        locomotion.deactivate()

    def _execution(self):
       
        # greeting customer and asking for order
        self.voice_params["speech"] = "Good day Human, what kind of food do you want. Please choose from the menu"
        
        voice.run(self.voice_params)

    def _post_execution(self):

        # Listen for an order
        voice_recognition.run(self.voice_recognition_params)

    def get_next_state(self):


        
        if voice_recognition.speech is not "":
            
            if check_if_any_word_in_menu(voice_recognition.speech):
                order = extract_order(voice_recognition.speech)
                return 'state 05'

            else:
                self.voice_params["speech"] = "Sorry, I didn't catch that. Make sure that you're asking for something on the menu."
                voice.run(self.voice_params)
                return None

        return 'state 03'


class State03(StateBase):
    """
    The robot is pushing the customer to order
    """
    # /!\ WARNING: This code wasn't tested! /!\
    def __init__(self):
        StateBase.__init__(self)
        
        self.provocations = [
            "Whenever you're ready, I'm ready human.",
            "I have all day.",
            "May I suggest some delicious apples?",
            "Go on.",
            "Have you made up your mind human?",
            "I am waiting for you order human.", 
            "Please give your slave an order"
            ]
        
        self.voice_params = {
            "speech" : "",
            "language" : "en_GB"
            }

        self.voice_recognition_params = {
            "language": "en-us",
            "skip_keyword": "False",
            "tell_back": "False"
           }

        self.last_face_timestamp = face_tracking.timestamp

    def _set_id(self):
        return 'state 03'

    def _pre_execution(self):
        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.activate()
        voice.activate()
        locomotion.deactivate()

    def _execution(self):
        # This state is more an idling thing, so this method isn't useful in this case
        pass

    def _post_execution(self):

        # Checking last time a face was detected
        self.last_face_timestamp = face_tracking.timestamp
        
        # Push the customer to order something
        random_index = random.randint(0, len(self.provocations)-1)
        self.voice_params["speech"] = self.provocations[random_index]

        voice.run(self.voice_params)

        # Listen for an order
        voice_recognition.run(self.voice_recognition_params)

        # Letting a clue that the robot isn't listening anymore
        self.voice_params["speech"] = "Alright, let me process that..."
        voice.run(self.voice_params)

    def get_next_state(self):
        
        # If the robot didn't recognize a face during the execution,
        # we assume that the customer left
        # TODO: /!\ This might be wrong, I need to think about it! /!\
        if self.last_face_timestamp == face_tracking.timestamp:
            return 'state 01'

        # If the robot is still seeing a face, but the voice recognition failed
        if voice_recognition.speech == "":
            return None

        # If the customer asked for something on the menu
        if check_if_any_word_in_menu(voice_recognition.speech):
            return 'state 05'

        # If the customer said something, but it's not on the menu
        else:
            self.voice_params["speech"] = "Make sure that you're asking for something on the menu human!"
            voice.run(self.voice_params)
            return None


class State04(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        
        self.instructions = [
            "Could you repeat your order please?",
            "Could you tell me what you want one more time?",
            "I'm listening, just repeat your order human."
            ]

        self.voice_params = {
            "speech" : "",
            "language" : "en_GB"
            }

        self.voice_recognition_params = {
            "language": "en-us",
            "skip_keyword": "False",
            "tell_back": "False"
           }

    def _set_id(self):
        return 'state 04'

    def _pre_execution(self):
        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.activate()
        voice.activate()
        locomotion.deactivate()

    def _execution(self):
        self.voice_params["speech"] = "Sorry, what you want doesn't seem to be on the menu."
        voice.run(self.voice_params)

    def _post_execution(self):
        
        random_index = random.randint(0, len(self.instructions)-1)
        self.voice_params["speech"] = self.instructions[random_index]
        
        voice.run(self.voice_params)

        voice_recognition.run(self.voice_recognition_params)

        # Letting a clue that the robot isn't listening anymore
        self.voice_params["speech"] = "Alright, let me process that..."
        voice.run(self.voice_params)

    def get_next_state(self):
        
        if voice_recognition.speech is not "":
            
            if check_if_any_word_in_menu(voice_recognition.speech):
                order = extract_order(voice_recognition.speech)
                return 'state 05'

            else:
                self.voice_params["speech"] = "Sorry, I didn't catch that. Make sure that you're asking for something on the menu."
                voice.run(self.voice_params)
                return None

        return 'state 03'


class State05(StateBase):
    def __init__(self):
        StateBase.__init__(self)
        
        self.run_count = 0

        self.voice_params = {
            "speech" : "",
            "language" : "en_GB"
            }

        self.voice_recognition_params = {
            "language": "en-us",
            "skip_keyword": "False",
            "tell_back": "False"
           }

        self.affirmative_answers = [
            "yes",
            "right",
            "algright",
            "correct"
            ]

    def _set_id(self):
        return 'state 05'

    def _pre_execution(self):
        self.run_count = 0

        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.activate()
        voice.activate()
        locomotion.deactivate()

    def _execution(self):
        speech = "If I understood you right, you want"

        for index in range(len(order)):

            dish = order[index]

            # Adding the dishes to the string
            if dish[-1] == 's':
                speech += " some " + dish

            else:
                if dish[0] == 'b':
                    speech += " a " + dish
                else:
                    speech += " an " + dish
            
            if index < len(order) - 1:
                speech += ","
            
            elif (index >= len(order) - 1) and (len(order) is not 1):
                speech += " and"

        speech += "."

        self.voice_params["speech"] = speech
        voice.run(self.voice_params)

    def _post_execution(self):
        self.voice_params["speech"] = "Did I get the order right? Yes or No."
        voice.run(self.voice_params)

        voice_recognition.run(self.voice_recognition_params)
        
        self.run_count += 1

    def get_next_state(self):

        if voice_recognition.speech == "":
            
            if self.run_count < 3:
                return None
            
            else:
                return 'state 04'
        
        voice_recognition_results = re.sub(r'[^\w\s]','', voice_recognition.speech).lower()
        is_affirmative_answer = bool(set(self.affirmative_answers) & set(voice_recognition_results.split()))

        if is_affirmative_answer:
            return 'state 07'
        
        return 'state 04'


"""
+-------------------------------------------------+
|                 States 6 to 11                  |
+-------------------------------------------------+
"""


class State06(StateBase):
    def __init__(self):
        StateBase.__init__(self)

        self.voice_params = {
            "speech" : "Please help me, I'm stuck. \
            Say OK when I'm good to go",
            "language" : "en_GB"
            }

        self.voice_recognition_params = {
            "language": "en-us",
            "skip_keyword": "False",
            "tell_back": "False"
           }

    def _set_id(self):
        return 'state 06'

    def _pre_execution(self):
        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.activate()
        voice.activate()

    def _execution(self):
        pass

    def _post_execution(self):
        voice.run(self.voice_params)
        voice_recognition.run(self.voice_recognition_params)

    def get_next_state(self):
        if voice_recognition.speech == "OK":
            return 'state 07'
        elif voice_recognition.speech == "reset":
            return 'state 00'
        else:
            return None


class State07(StateBase):
    def __init__(self):
        StateBase.__init__(self)

        self.voice_params = {
            "speech" : "Order coming right on!",
            "language" : "en_GB"
            }

    def _set_id(self):
        return 'state 07'

    def _pre_execution(self):
        voice.activate()
        locomotion.activate()

    def _execution(self):
        voice.run(self.voice_params)
        self.result = locomotion.goto(kitchen_position)

    def _post_execution(self):
        pass

    def get_next_state(self):
        if self.result:
            return 'state 08'
        return 'state 06'

class State08(StateBase):
    def __init__(self):
        StateBase.__init__(self)

        self.voice_params = {
            "speech" : "",
            "language" : "en_GB"
            }

    def _set_id(self):
        return 'state 08'

    def _pre_execution(self):
        face_tracking.activate()
        voice.activate()

    def _execution(self):
        self.voice_params["speech"] = order
        voice.run(self.voice_params)
        rospy.sleep(15)

    def _post_execution(self):
        pass

    def get_next_state(self):
        return "state 10"


class State09(StateBase):
    def __init__(self):
        StateBase.__init__(self)

        self.voice_params = {
            "speech" : "Please help me, I'm stuck. \
            Say OK when I'm good to go",
            "language" : "en_GB"
            }

        self.voice_recognition_params = {
            "language": "en-us",
            "skip_keyword": "False",
            "tell_back": "False"
           }

    def _set_id(self):
        return 'state 09'

    def _pre_execution(self):
        # Turning on and off the behaviors
        face_tracking.activate()
        voice_recognition.activate()
        voice.activate()

    def _execution(self):
        pass

    def _post_execution(self):
        voice.run(self.voice_params)
        voice_recognition.run(self.voice_recognition_params)

    def get_next_state(self):
        if voice_recognition.speech == "OK":
            return 'state 10'
        elif voice_recognition.speech == "reset":
            return 'state 00'
        else:
            return None



class State10(StateBase):
    def __init__(self):
        StateBase.__init__(self)

    def _set_id(self):
        return 'state 10'

    def _pre_execution(self):
        face_tracking.deactivate()
        voice_recognition.deactivate()
        voice.deactivate()
        locomotion.activate()

    def _execution(self):
        self.result = locomotion.goto(master_position)

    def _post_execution(self):
        pass

    def get_next_state(self):
        if self.result:
            return 'state 11'
        return 'state 09'


class State11(StateBase):
    def __init__(self):
        StateBase.__init__(self)

        self.give_order = [
            "Here is your order, have a nice day.",
            "Here's what you asked for, see you again", 
            "Your slave have returned with your bounty"
            ]
        
        self.voice_params = {
            "speech" : "",
            "language" : "en_GB"
            }

    def _set_id(self):
        return 'state 11'

    def _pre_execution(self):
        face_tracking.activate()
        voice_recognition.deactivate()
        voice.activate()
        locomotion.deactivate()

    def _execution(self):
        #Donne la commande et attent 20 secondes avant de retourner au debut de la machine a etat
        random_index = random.randint(0, len(self.give_order)-1)
        self.voice_params["speech"] = self.give_order[random_index]

        voice.run(self.voice_params)
        rospy.sleep(20.)

    def _post_execution(self):
        pass

    def get_next_state(self):
        return 'state 00'
