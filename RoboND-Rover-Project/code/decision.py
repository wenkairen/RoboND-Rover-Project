import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def forward(Rover):
    # Check the extent of navigable terrain
    if len(Rover.nav_angles) >= Rover.stop_forward:

        ## if the vel not reach the max
        if 0 < Rover.vel < Rover.max_vel:
            Rover.throttle = Rover.throttle_set
            Rover.steer = np.clip(np.mean(Rover.nav_angles )* 180 / np.pi, -10, 10) + Rover.offset

        ## the vel is not moving 
        elif Rover.vel <= 0:
            if Rover.throttle == 0:
                Rover.throttle = 2
                Rover.steer = np.clip(np.mean(Rover.nav_angles )* 180 / np.pi, -10, 10)+ Rover.offset
            else:
                ## stuck posion switch into stop mode
                Rover.throttle = 0
                Rover.steer = -15
                Rover.mode = 'stop'

        ## too fast , coasting
        else:
            Rover.throttle = 0
            Rover.steer = np.clip(np.mean(Rover.nav_angles)* 180 / np.pi, -10, 10) + Rover.offset
        Rover.brake = 0


    elif len(Rover.nav_angles) < Rover.stop_forward:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
        Rover.mode = 'stop'

def stop(Rover):
    ## if the rover still has vel, then keep braking
    if Rover.vel > 0.2:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0

    ## in the stop mode to decide the next move 
    elif Rover.vel <= 0.2:
        if len(Rover.nav_angles) < Rover.go_forward:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -15  

        elif len(Rover.nav_angles) >= Rover.go_forward:
            Rover.throttle = Rover.throttle_set
            Rover.brake = 0
            Rover.steer = np.clip(np.mean(Rover.nav_angles)* 180 / np.pi, -10, 10) + Rover.offset
            Rover.mode = 'forward'

def next_state(Rover):
    ## the offset is mean to smooth the wall huging 
    ## if the sample is detected     
    if Rover.sample_detected:
        ## close enough to pick
        if Rover.near_sample:
            Rover.brake = Rover.brake_set
            Rover.throttle = 0
            Rover.steer = 0
        ## or based on the rover speed to get close to the rock
        else:
            ## below are three different vel conditions to adjust
            if  0 < Rover.vel < Rover.max_vel:
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0

            ## the rover is not moving 
            elif Rover.vel == 0:
                ## check if we aplly throttle
                if Rover.throttle == 0: 
                    Rover.throttle = 2
                    Rover.steer = np.clip(np.mean(Rover.nav_angles)* 180 / np.pi, -15, 15) + Rover.offset
                    Rover.brake = 0
                else:
                    ## there is throttle but the car is not moving, so switch into stop mode
                    Rover.throttle = 0
                    Rover.steer = -15
                    Rover.mode = 'stop'
            else:
                ## too fast, slow down with 1/3 of brake
                Rover.throttle = 0
                Rover.brake = 10

            ## avoid returning to same position
            Rover.steer = np.clip(np.mean(Rover.nav_angles) * 180 / np.pi, -5, 15)

        Rover.sample_detected = False

    elif Rover.mode == 'forward':
        forward(Rover)
    
    elif Rover.mode == 'stop':
        stop(Rover)

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0


def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    #record start position
    if Rover.is_start:
        Rover.start_pos = Rover.pos
        ## only record the start pos once
        Rover.is_start = False

    if Rover.nav_angles is not None:

        #if all samples are collected, path planning to the origin
        if Rover.samples_found == 6:
            dist = np.sqrt((Rover.pos[0] - Rover.start_pos[0])**2 + (Rover.pos[1] - Rover.start_pos[1])**2)
            # stop if come backto the origin
            if dist <= 1:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.steer = 0
            
            else:
                ## continue moving until close enough.
                next_state(Rover)

        ## mapping and find samples
        else:
            next_state(Rover)

    ## if the state is satisfied, then pick up the rock
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover


