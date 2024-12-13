# This file holds all of the mechanical tasks, called "routines", that the bot can do
# --- WARNING --- #
# DO NOT change these imports until you've read the note below.

from util.common import *
from util.objects import Routine
import time

# Note that the lines above (imports) have changed from the videos.
# This is to provide a better coding experience for you. Don't change them!
# Unless you know what you're doing :)

# Similarly, in the videos drive does not have "(Routine)" behind it. That's  just telling the program
# that the "drive" class should look like a "Routine" above. It helps with the code suggestions. 

class Game:
    def __init__(self):
        self.previous_time = time.time()  # Initialize the previous time as the current time
        self.delta_seconds = 0.0  # Delta time will be calculated every frame

    def update_delta(self):
        current_time = time.time()  # Get current time
        self.delta_seconds = current_time - self.previous_time  # Calc delta time
        self.previous_time = current_time  # Update previous time for next frame



FIRST_JUMP_DURATION = 0.1
BETWEEN_JUMPS_DELAY = 0.1
SECOND_JUMP_DURATION = 0.05
TIMEOUT = 2.0



class jumper():
    def run(self, agent):
        defaultThrottle(agent,2300)
        if agent.time % 5 == 0:
            agent.controller.jump = False





class SpeedFlip(Routine):
    def __init__(self, right_handed = True, use_boost = True):
        super().__init__()
        self.direction = 1 if right_handed else -1
        self.use_boost = use_boost
        self.timer = 0.0
        self.game = Game()  # Instance of Game class to handle delta time

    def run(self, agent):
        #Update delta time each frame
        self.game.update_delta()
        delta_seconds = self.game.delta_seconds 

        #Always throttle.
        agent.controller.throttle = 1.0

        #Use boost if allowed and speed is less than 2290
        speed = agent.me.velocity.magnitude()
        agent.controller.boost = self.use_boost and speed < 2290

        #Jump squence logic
        if self.timer < FIRST_JUMP_DURATION:
            agent.controller.jump = True
            agent.controller.pitch = 1.0

        elif self.timer < FIRST_JUMP_DURATION + BETWEEN_JUMPS_DELAY:
            agent.controller.jump = False
            agent.controller.pitch = 1.0

        elif self.timer < FIRST_JUMP_DURATION + BETWEEN_JUMPS_DELAY + SECOND_JUMP_DURATION:
            agent.controller.jump = True
            agent.controller.pitch = -1.0
            agent.controller.roll = -0.3 * self.direction

        else:
            agent.controller.jump = False
            agent.controller.pitch = 1.0
            # Use steerPD for roll and yaw
            agent.controller.roll = steerPD(-1.0 * self.direction, 0)
            agent.controller.yaw = steerPD(-1.0 * self.direction, 0)

        # Update the timer based on delta time
        self.timer += delta_seconds

        # Check if the routine is finished
        self.finished = (self.timer > TIMEOUT) or (
                not agent.me.airborne and self.timer > 0.5
        )






class drive(Routine):
    def __init__(self, speed, target=None) -> None:
        self.speed = speed
        self.target = target

    def run(self, agent):
        defaultThrottle(agent, self.speed)
        if self.target is not None:
            relative_target = self.target - agent.me.location
            defaultPD(agent, agent.me.local(relative_target))


class atba(Routine):
    # An example routine that just drives towards the ball at max speed
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)
        

class shooting(Routine):
    def __init__(self, ball_location, target_location, shot_vector, intercept_time, direction=1):
        self.ball_location = ball_location
        self.target_location = target_location
        self.shot_vector = shot_vector
        self.intercept_time = intercept_time
        self.direction = direction
        self.counter = 0
        self.jump_time = 0






    def run(self, agent):
        distance_to_ball = (self.ball_location - agent.me.location).magnitude()
        time_remaining = self.intercept_time - agent.time

        # Ensure the target is the enemy goal
        if self.target_location != agent.foe_goal.location:
            return  # Abort if not targetting the right goal

        if distance_to_ball < 200 and time_remaining > 0.1:
            angles = defaultPD(agent, agent.me.local(self.ball_location - agent.me.location))
            defaultThrottle(agent, 2300, self.direction)

            # Performing the shot
            if self.counter == 0 and time_remaining < 0.5:
                agent.controller.jump = True
                self.jump_time = agent.time
                self.counter += 1

            elif self.counter > 0:
                if agent.time - self.jump_time < 0.1:
                    agent.controller.jump = True
                else:
                    agent.controller.jump = False
                    self.counter += 1

            # Adjust the angles based on shot direction
            if self.counter == 1:
                agent.controller.pitch = steerPD(self.shot_vector.pitch, 0)
                agent.controller.yaw = steerPD(self.shot_vector.yaw, 0)

        elif time_remaining < 0:
            # If we're out of time, stop the routine
            agent.clear_intent()


# Make sure to integrate it into your bot logic, replacing the shooting logic with the new class when appropriate






    def run(self):
        # Shooting logic
        distance_to_ball = (self.ball.location - self.me.location).magnitude()
        intercept_time = self.get_intercept_time(self.ball.location, self.foe_goal.location)

        # Calculate shot vector as the direction from ball to goal
        shot_vector = (self.foe_goal.location - self.ball.location).normalize()
        
        if self.me.airborne and distance_to_ball < 200 and self.get_intent() is None:
            # Create a new shooting routine based on current knowledge
            self.set_intent(shooting(self.ball.location, self.foe_goal.location, shot_vector, intercept_time))
            self.debug_text = 'Shooting'
            self.add_debug_line('shooting', self.me.location, self.foe_goal.location, [0, 255, 0])
            return



    def get_intercept_time(self, ball_location, goal_location):
        # Ball speed can be estimated from the ball's velocity
        ball_velocity = self.ball.velocity.magnitude()  # Get ball's speed
        ball_to_goal_distance = (goal_location - ball_location).magnitude()

        # Intercept time is distance to the goal divided by ball's speed
        # Prevent division by zero if the ball is nearly stationary by returning a default high time.
        if ball_velocity > 1:  # If speed is greater than 1, it's moving; otherwise, it's stuck.
            intercept_time = ball_to_goal_distance / ball_velocity
        else:
            intercept_time = float('inf')  # Set to high value if the ball isn't moving

        return intercept_time






class aerial_shot(Routine):
    def __init__(self, ball_location, intercept_time, shot_vector, ratio):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.shot_vector = shot_vector
        self.intercept = self.ball_location - (self.shot_vector * 110)
        self.jump_threshold = 600
        self.jump_time = 0
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        time_remaining = cap(raw_time_remaining, 0.01, 10.0)

        car_to_ball = self.ball_location - agent.me.location
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))
        car_to_intercept = self.intercept - agent.me.location
        car_to_intercept_perp = car_to_intercept.cross((0, 0, side_of_shot))
        flat_distance_remaining = car_to_intercept.flatten().magnitude()

        # Ensure the target is the enemy goal
        if agent.foe_goal.location != self.ball_location:  # Make sure we're not shooting towards our goal
            # Calculate necessary speeds and control logic as in the original method
            speed_required = flat_distance_remaining / time_remaining
            acceleration_required = backsolve(self.intercept, agent.me, time_remaining, 325)
            local_acceleration_required = agent.me.local(acceleration_required)

            adjustment = car_to_intercept.angle(self.shot_vector) * flat_distance_remaining / 1.57
            adjustment *= (cap(self.jump_threshold - (acceleration_required[2]), 0.0, self.jump_threshold) / self.jump_threshold)
            final_target = self.intercept + ((car_to_intercept_perp.normalize() * adjustment) if self.jump_time == 0 else 0)

            if abs(agent.me.location[1]) > 5150:
                final_target[0] = cap(final_target[0], -750, 750)

            local_final_target = agent.me.local(final_target - agent.me.location)

            # existing debug lines
            agent.line(agent.me.location, self.intercept)
            agent.line(self.intercept - Vector3(0, 0, 100), self.intercept + Vector3(0, 0, 100), [255, 0, 0])
            agent.line(final_target - Vector3(0, 0, 100), final_target + Vector3(0, 0, 100), [0, 255, 0])

            angles = defaultPD(agent, local_final_target)

            if self.jump_time == 0:
                defaultThrottle(agent, speed_required)
                agent.controller.boost = False if abs(angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
                agent.controller.handbrake = True if abs(angles[1]) > 2.3 else agent.controller.handbrake

                velocity_required = car_to_intercept / time_remaining
                good_slope = velocity_required[2] / cap(abs(velocity_required[0]) + abs(velocity_required[1]), 1, 10000) > 0.15
                if good_slope and (local_acceleration_required[2]) > self.jump_threshold and agent.me.velocity.flatten().normalize().dot(acceleration_required.flatten().normalize()) > 0.8:
                    self.jump_time = agent.time
            else:
                time_since_jump = agent.time - self.jump_time

                if agent.me.airborne and local_acceleration_required.magnitude() * time_remaining > 90:
                    angles = defaultPD(agent, local_acceleration_required)
                    if abs(angles[0]) + abs(angles[1]) < 0.45:
                        agent.controller.boost = True
                else:
                    final_target -= Vector3(0, 0, 45)
                    local_final_target = agent.me.local(final_target - agent.me.location)
                    angles = defaultPD(agent, local_final_target)

                if self.counter == 0 and (time_since_jump <= 0.2 and local_acceleration_required[2] > 0):
                    agent.controller.jump = True
                elif time_since_jump > 0.2 and self.counter < 3:
                    agent.controller.jump = False
                    agent.controller.pitch = 0
                    agent.controller.yaw = 0
                    agent.controller.roll = 0
                    self.counter += 1
                elif local_acceleration_required[2] > 300 and self.counter == 3:
                    agent.controller.jump = True
                    agent.controller.pitch = 0
                    agent.controller.yaw = 0
                    agent.controller.roll = 0
                    self.counter += 1

            if raw_time_remaining < -0.25:
                agent.set_intent(recovery())
            if not shot_valid(agent, self, 90):
                agent.clear_intent()





class flip(Routine):
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector: Vector3, cancel=False):
        self.vector = vector.normalize()
        self.pitch = abs(self.vector[0]) * -sign(self.vector[0])
        self.yaw = abs(self.vector[1]) * sign(self.vector[1])
        self.cancel = cancel
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        if elapsed < 0.15:
            agent.controller.jump = True
        elif elapsed >= 0.15 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            agent.set_intent(recovery())





class half_flip:
    def __init__(self):
        self.time = -1
        self.counter = 0
    
    def run(self, agent):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time
        if elapsed <14:
            agent.controller.jump = True
        elif elapsed >= 0.14 and self.counter < 1:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.6:
            agent.controller.jump = True
            agent.controller.pitch = 1
        elif elapsed < 1.4:
#Rotates and lands
            agent.controller.pitch = -1
            agent.controller.roll = 1
            agent.controller.yaw = 1
        elif not agent.me.airborne:
#Stops routine when landed
            agent.pop()





def run(self):
    #Main defense logic
    ball_to_friend_goal = abs(self.ball.location.y - self.friend_goal.location.y)
    ball_to_foe_goal = abs(self.ball.location.y - self.foe_goal.location.y)
    is_opponent_near_goal = ball_to_foe_goal < 250  #Threshold to determine if the opponent can score

    if self.get_intent() is not None:
        self.debug_intent()
        return

    #Handle kickoff situation
    if self.kickoff_flag:
        self.set_intent(kickoff_strategy(self))
        self.debug_text = 'Kickoff'
        return

    #Define defensive positions
    defending_position = Vector3(self.friend_goal.location.x, self.friend_goal.location.y - 500, self.friend_goal.location.z)

    #Check distances to determine whether to defend or retreat
    ball_distance = (self.ball.location - self.me.location).magnitude()
    
    #Defensive strategy: The bot should prioritize defending if the ball is close enough
    if ball_distance < 1000 and is_opponent_near_goal:
        self.set_intent(goto(defending_position))
        self.debug_text = 'Defending'
        self.add_debug_line('defending_position', self.me.location, defending_position, [255, 0, 0])
        return

    #Retreating logic
    is_in_front_of_ball = ball_to_friend_goal < ball_to_foe_goal
    distance_to_friend_goal = (self.friend_goal.location - self.me.location).magnitude()
    
    #If the ball is behind the bot and approaching the friend goal
    if not is_in_front_of_ball and distance_to_friend_goal > 500:
        retreat_location = self.friend_goal.location.copy()  # Retain the goal as a reference point
        self.set_intent(goto(retreat_location))
        self.debug_text = 'Retreating to Goal'
        self.add_debug_line('retreat_to_goal', self.me.location, retreat_location, [0, 0, 255])
        return

    #If the ball is in the opponent's half and has a significant distance to cover
    if ball_distance < 1500 and is_opponent_near_goal:
        # Change intent to intercept or challenge the opponent when close to the ball
        self.set_intent(goto(self.ball.location))
        self.debug_text = 'Challenging the Ball'
        self.add_debug_line('challenging_ball', self.me.location, self.ball.location, [0, 255, 0])
        return

    #Default to wall-fence against the goal if all else fails
    if abs(self.me.location.y) > 4500:  # Adjust threshold based on field dimension
        wall_position = Vector3(self.friend_goal.location.x, self.friend_goal.location.y + (500 * side(self.team)), self.friend_goal.location.z)
        self.set_intent(goto(wall_position))
        self.debug_text = 'Wall Defense'
        self.add_debug_line('wall_defense', self.me.location, wall_position, [255, 255, 0])
        return








class goto(Routine):
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    # TODO - slow down if target is inside our turn radius
    def __init__(self, target: Vector3, vector: Vector3 | None=None, direction=1):
        self.target = target
        self.vector = vector
        self.direction = direction

    def run(self, agent):
        car_to_target = self.target - agent.me.location
        distance_remaining = car_to_target.flatten().magnitude()

        agent.line(self.target - Vector3(0, 0, 500),
                   self.target + Vector3(0, 0, 500), [255, 0, 255])

        if self.vector != None:
            # See commends for adjustment in jump_shot or aerial for explanation
            side_of_vector = sign(self.vector.cross(
                (0, 0, 1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross(
                (0, 0, side_of_vector)).normalize()
            adjustment = car_to_target.angle(
                self.vector) * distance_remaining / 3.14
            final_target = self.target + (car_to_target_perp * adjustment)
        else:
            final_target = self.target

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target, self.direction)
        defaultThrottle(agent, 2300, self.direction)

        agent.controller.boost = False
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 else agent.controller.handbrake

        velocity = 1+agent.me.velocity.magnitude()
        if distance_remaining < 350:
            agent.clear_intent()
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and distance_remaining / velocity > 2.0:
            agent.set_intent(flip(local_target))
        elif abs(angles[1]) > 2.8 and velocity < 200:
            agent.set_intent(flip(local_target, True))
        elif agent.me.airborne:
            agent.set_intent(recovery(self.target))


class goto_boost(Routine):
    # very similar to goto() but designed for grabbing boost
    # if a target is provided the bot will try to be facing the target as it passes over the boost
    def __init__(self, boost, target=None):
        self.boost = boost
        self.target = target

    def run(self, agent):
        car_to_boost = self.boost.location - agent.me.location
        distance_remaining = car_to_boost.flatten().magnitude()

        agent.line(self.boost.location - Vector3(0, 0, 500),
                   self.boost.location + Vector3(0, 0, 500), [0, 255, 0])

        if self.target != None:
            vector = (self.target - self.boost.location).normalize()
            side_of_vector = sign(vector.cross((0, 0, 1)).dot(car_to_boost))
            car_to_boost_perp = car_to_boost.cross(
                (0, 0, side_of_vector)).normalize()
            adjustment = car_to_boost.angle(vector) * distance_remaining / 3.14
            final_target = self.boost.location + \
                (car_to_boost_perp * adjustment)
            car_to_target = (self.target - agent.me.location).magnitude()
        else:
            adjustment = 9999
            car_to_target = 0
            final_target = self.boost.location.copy()

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        agent.controller.boost = self.boost.large if abs(
            angles[1]) < 0.3 else False
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 else agent.controller.handbrake

        velocity = 1+agent.me.velocity.magnitude()
        if self.boost.active == False or agent.me.boost >= 99.0 or distance_remaining < 350:
            agent.clear_intent()
        elif agent.me.airborne:
            agent.set_intent(recovery(self.target))
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and (distance_remaining / velocity > 2.0 or (adjustment < 90 and car_to_target/velocity > 2.0)):
            agent.set_intent(flip(local_target))


class jump_shot(shooting):
    # Hits a target point at a target time towards a target direction
    # Target must be no higher than 300uu unless you're feeling lucky
    #TODO - speed
    def __init__(self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we dodge at
        # 173 is the 93uu ball radius + a bit more to account for the car's hitbox
        self.dodge_point = self.ball_location - (self.shot_vector * 173)
        # Ratio is how aligned the car is. Low ratios (<0.5) aren't likely to be hit properly
        self.ratio = ratio
        # whether the car should attempt this backwards
        self.direction = direction
        # Intercept speed not implemented
        self.speed_desired = speed
        # controls how soon car will jump based on acceleration required. max 584
        # bigger = later, which allows more time to align with shot vector
        #smaller = sooner
        self.jump_threshold = 400
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = car_to_dodge_point.cross(
            (0, 0, side_of_shot))  # perpendicular
        distance_remaining = car_to_dodge_point.magnitude()

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(
            self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650)
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        adjustment = car_to_dodge_point.angle(
            self.shot_vector) * distance_remaining / 2.0  # size of adjustment
        # factoring in how close to jump we are
        adjustment *= (cap(self.jump_threshold -
                       (acceleration_required[2]), 0.0, self.jump_threshold) / self.jump_threshold)
        # we don't adjust the final target if we are already jumping
        final_target = self.dodge_point + \
            ((car_to_dodge_perp.normalize() * adjustment)
             if not self.jumping else 0) + Vector3(0, 0, 50)
        # Ensuring our target isn't too close to the sides of the field, where our car would get messed up by the radius of the curves

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.dodge_point)
        agent.line(self.dodge_point-Vector3(0, 0, 100),
                   self.dodge_point+Vector3(0, 0, 100), [255, 0, 0])
        agent.line(final_target-Vector3(0, 0, 100),
                   final_target+Vector3(0, 0, 100), [0, 255, 0])
        agent.line(agent.ball.location, agent.ball.location +
                   (self.shot_vector * 300))

        # Calling our drive utils to get us going towards the final target
        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.line(agent.me.location, agent.me.location +
                   (self.shot_vector*200), [255, 255, 255])

        agent.controller.boost = False if abs(
            angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 and self.direction == 1 else agent.controller.handbrake

        if not self.jumping:
            if raw_time_remaining <= 0.0 or (speed_required - 2300) * time_remaining > 60 or not shot_valid(agent, self):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we reset
                agent.clear_intent()
                if agent.me.airborne:
                    agent.set_intent(recovery())
            elif local_acceleration_required[2] > self.jump_threshold and local_acceleration_required[2] > local_acceleration_required.flatten().magnitude():
                # Switch into the jump when the upward acceleration required reaches our threshold, and our lateral acceleration is negligible
                self.jumping = True
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 150)) or raw_time_remaining <= -0.9 or (not agent.me.airborne and self.counter > 0):
                agent.set_intent(recovery())
            elif self.counter == 0 and local_acceleration_required[2] > 0.0 and raw_time_remaining > 0.083:
                # Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                # make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                # dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -sign(vector[0])
                    self.y = abs(vector[1]) * sign(vector[1]) * self.direction
                    self.dodging = True
                # simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0



def is_corner_kickoff(agent):
# Get the ball's location relative to the car.
    ball_local = agent.me.local(agent.ball.location)

# Determine if the ball is on the side of the field.
    is_side_kickoff = abs(ball_local[1]) > 5000

# Determine if the ball is in the corner.
    is_corner_kickoff = is_side_kickoff and abs(ball_local[0]) > 3000

    return is_corner_kickoff

#Determines kickoff strat

def kickoff_strategy(agent):
    if is_corner_kickoff(agent):
        return speedflipkickoff()
    else:
        return kickoff()



class kickoff(Routine):
    # A simple 1v1 kickoff that just drives up behind the ball and dodges
    # misses the boost on the slight-offcenter kickoffs haha
    def run(self, agent):
        target = agent.ball.location + Vector3(0, 200*side(agent.team), 0)
        local_target = agent.me.local(target - agent.me.location)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)
        if local_target.magnitude() < 650:
            # flip towards opponent goal
            agent.set_intent(
                flip(agent.me.local(agent.foe_goal.location - agent.me.location)))



    #Perfect against slow kickoffs / only works on corner
class speedflipkickoff(Routine):
    def run(self, agent):
        # Check if the bot has collected the first boost pad
        if agent.me.boost >= 25:
            # Determine right_handed based on the ball's position relative to the car
            right_handed = agent.me.local(agent.ball.location)[1] < 0
            agent.set_intent(SpeedFlip(right_handed=right_handed))
        else:
            super().run(agent) 








class recovery(Routine):
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target

    def run(self, agent):
        if self.target != None:
            local_target = agent.me.local(
                (self.target-agent.me.location).flatten())
        else:
            local_target = agent.me.local(agent.me.velocity.flatten())

        defaultPD(agent, local_target)
        agent.controller.throttle = 1
        if not agent.me.airborne:
            agent.clear_intent()



class short_shot(Routine):
    # This routine drives towards the ball and attempts to hit it towards a given target
    # It does not require ball prediction and kinda guesses at where the ball will be on its own
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball = (agent.ball.location - agent.me.location).normalize()
        distance = (agent.ball.location - agent.me.location).magnitude()
        ball_to_target = (self.target - agent.ball.location).normalize()

        relative_velocity = car_to_ball.dot(
            agent.me.velocity-agent.ball.velocity)
        if relative_velocity != 0.0:
            eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
        else:
            eta = 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross((0, 0, 1))
        right_vector = car_to_ball.cross((0, 0, -1))
        target_vector = -ball_to_target.clamp(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector*(distance/2))

        # Some adjustment to the final target to ensure we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        agent.line(final_target-Vector3(0, 0, 100), final_target +
                   Vector3(0, 0, 100), [255, 255, 255])

        angles = defaultPD(agent, agent.me.local(
            final_target-agent.me.location))
        defaultThrottle(agent, 2300 if distance >
                        1600 else 2300-cap(1600*abs(angles[1]), 0, 2050))
        agent.controller.boost = False if agent.me.airborne or abs(
            angles[1]) > 0.3 else agent.controller.boost
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 else agent.controller.handbrake

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.set_intent(flip(agent.me.local(car_to_ball)))

