# This is the main file where you control your bot's strategy

from util.objects import *
from util.routines import *
from util.tools import find_hits
from util.quickchat import *


class Bot(BotCommandAgent):

        #Debug main
    debug_text = ''
    def print_debug(self):
        white = self.renderer.white()
        text = 'my debug text'
        self.renderer.draw_string_2d(10, 150, 3, 3, self.debug_text, white)


    def run(self):
        #Mainly for retreat / defense below
        d1_ball_to_fgoal = abs(self.ball.location.y - self.foe_goal.location.y)
        d2_friend_to_fgoal = abs(self.me.location.y - self.foe_goal.location.y)
        is_infront_of_ball = d1_ball_to_fgoal > d2_friend_to_fgoal
        is_good_retreat = abs(self.ball.location.y - self.me.location.y)
        defense_distance = abs(self.ball.location.y - self.friend_goal.location.y)
        start_defense = defense_distance < 250


        self.print_debug()
        white = self.renderer.white()
        self.renderer.draw_line_3d(self.me.location, self.ball.location, white)
        if self.get_intent() is not None:
            self.debug_intent()
            return
        

        #Kickoff
        if self.kickoff_flag:
            self.set_intent(kickoff_strategy(self))
            self.debug_text = 'kickoff'
            self.add_debug_line('me_to_kickoff', self.me.location, self.ball.location, [0, 0, 255])
            self.add_debug_line('kickoff_to_goal', self.ball.location, self.foe_goal.location, [0, 0, 255])
            return


        #Targets
        targets = {
            'at_foe_goal' : (self.foe_goal.left_post, self.foe_goal.right_post),
            'away_from_friend_net' : (self.friend_goal.right_post, self.friend_goal.left_post) 
        }


        #Hits
        hits = find_hits(self,targets)
        if len(hits['at_foe_goal']) > 0:
            self.set_intent(hits['at_foe_goal'][0])
            print('at their goal')
            self.debug_text = 'Hits'
            return
        

        if len(hits['away_from_friend_net']) > 0:
            print('away from our goal')
            self.set_intent(hits['away_from_friend_net'][0])
            self.debug_text = 'Hits'
            return

        
        #Shooting
        if self.me.airborne and self.get_intent() is None:
            #Calculate distance to the ball
            distance_to_ball = (self.ball.location - self.me.location).magnitude()

            #Calculate intercept time
            intercept_time = self.get_intercept_time(self.ball.location, self.foe_goal.location)

            #Calculate shot vector as the direction from the ball to the foe goal
            shot_vector = (self.foe_goal.location - self.ball.location).normalize()

            #Check if the ball is in the air
            ball_airborne_threshold = 100  # Adjust threshold as needed
            ball_is_airborne = abs(self.ball.velocity.z) > ball_airborne_threshold  # Checking the vertical velocity

            #Ensuring the bot only shoots at the enemy goal
            if distance_to_ball < 200 and intercept_time > 0:
                # Make sure to shoot towards the enemy goal, not my own
                if self.foe_goal.location.y > self.me.location.y:  #Check if the enemy goal is in front of bot
                    if ball_is_airborne:
                        #If ball is in air, aerial shot routine
                        self.set_intent(aerial_shot(self.ball.location, intercept_time, shot_vector, 1))  #1 is default ratio
                        self.debug_text = 'Aerial Shot'
                        self.add_debug_line('aerial_shot', self.me.location, self.foe_goal.location, [255, 0, 255])
                    else:  #Regular shot
                        self.set_intent(shooting(self.ball.location, self.foe_goal.location, shot_vector, intercept_time))
                        self.debug_text = 'Shooting'
                        self.add_debug_line('shooting', self.me.location, self.foe_goal.location, [0, 255, 0])
                return



        #Get the largest boost
        if self.me.boost < 1:  #Threshold for needing boosts
            #Find the closest large boost pad
            closest_boost = self.get_closest_large_boost()
            if closest_boost is not None:
                #If a boost is found, set intent to navigate to that boost
                boost_location = closest_boost.location
                self.set_intent(goto(boost_location))
                self.debug_text = 'Going for closest boost'
                self.add_debug_line('going to closest boost', self.me.location, boost_location, [0, 0, 255])
                return
        
        

        #Retreat
        if is_infront_of_ball or is_good_retreat == 250:
            retreat_location = self.friend_goal.location
            self.set_intent(goto(self.friend_goal.location))
            self.debug_text = 'Retreating'
            self.add_debug_line('retreat', self.me.location, retreat_location, [255, 0, 0])
            return
        

        #Defense
        if abs(self.foes[0].location.x - self.ball.location.x) < 600 and abs(self.foes[0].location.y - self.ball.location.y) < 600 and d1_ball_to_fgoal<1200:
            self.set_intent(goto(self.friend_goal.location))
            if 700 > self.ball.location.x > 300 and d2_friend_to_fgoal > 2000 :
                self.set_intent(atba())
                return
