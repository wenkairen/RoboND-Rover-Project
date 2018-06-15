
## Project: Search and Sample Return

---
[//]: # (Image References)


[image1]: ./misc/rover_image.jpg
[image2]: ./output/original.jpg
[image3]: ./output/ground.jpg
[image4]: ./output/rock_threshed.jpg
[image5]: ./output/nav_angles.jpg
---
### Writeup / README

### Notebook Analysis

![alt text][image1]

#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
The orignial image from camera:
![alt text][image2]
The color selection is used to identify the ground with thresh (160 , 160, 160):
![alt text][image3]
Find the rock thresh (110, 110, 50):
![alt text][image4]
The navigate angles is used to guide the rover in directions:

        ## Calculate pixel values in rover-centric coords and distance/angle to all pixels
        xpix, ypix = rover_coords(threshed)
        dist, angles = to_polar_coords(xpix, ypix)
        mean_dir = np.mean(angles)
        
![alt text][image5]
#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

The pipeline steps is shown below

    1) Define source and destination points for perspective transform
    2) Apply perspective transform
    3) Apply color threshold to identify navigable terrain/obstacles/rock samples
        obs_map = np.absolute(np.float32(threshed) - 1) * mask
    4) Convert thresholded image pixel values to rover-centric coords
    5) Convert rover-centric pixel values to world coords
    6) Update worldmap (to be displayed on right side of screen)
    7) Make a mosaic image
    
the result is shown in the output folder
[link to my video result](./output/test_mapping.mp4)

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  




### For the perception_step(), the modification is based process image() .

* Define source and destination points for perspective transform

this method is based on the calibration of the grid image, by manually choose the points to perform warp transform

        dst_size = 5 
        bottom_offset = 6
        source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
        destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                          [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                          [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                          [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                          ])
                      
* The perspective view is shown below, the mask returned is intended for obstcales later

        def perspect_transform(img, src, dst):
            M= cv2.getPerspectiveTransform(src, dst)
            warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
            mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
            return warped, mask
        
* Apply color threshold to identify navigable terrain/obstacles/rock samples

The color_thresh() is modifided from the previous code, it combines both ground and rock identification, since the rock is yellow with lighter color, we decide to use HSV channnel to make the thresh more stable.

        ## obstacle map is based on the ground
        obs_map = np.absolute(np.float32(ground) - 1) * mask
        ground, rock = color_thresh(warped)
        
        def color_thresh(img, rgb_thresh=(160, 160, 160),rock_thresh = (20,100,100)):
            # Create an array of zeros same xy size as img, but single channel
            ground = np.zeros_like(img[:,:,0])
            rock = np.zeros_like(img[:,:,0])
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

            ground_thresh = (img[:,:,0] > rgb_thresh[0]) \
                        & (img[:,:,1] > rgb_thresh[1]) \
                        & (img[:,:,2] > rgb_thresh[2])

            rock_thresh = (hsv[:, :, 0] > rock_thresh[0]) \
                   & (hsv[:, :, 1] > rock_thresh[1]) \
                   & (hsv[:, :, 2] > rock_thresh[2])

            ground[ground_thresh] = 1
            rock[rock_thresh] = 1

            return ground,rock
            
* Update Rover.vision_image (this will be displayed on left side of screen)

        Rover.vision_image[:,:,2] = ground * 255
        Rover.vision_image[:,:,0] = obs_map * 255
        Rover.vision_image[:,:,1] = rock * 255
        
* Convert thresholded image pixel values to rover-centric coords and Convert rover-centric pixel values to world coords, this coordinate conversion step is mainly used for the worldmap later, now all the ground, bostacles and rock coordinates are in the same system as the worldmap. so we can perform the mapp steps.

        xpix, ypix = rover_coords(ground)
        obsxpix,obsypix = rover_coords(obs_map)
        rock_x, rock_y = rover_coords(rock)
        x_world, y_world = pix_to_world(xpix, ypix, 
                                            Rover.pos[0], Rover.pos[1],Rover.yaw, 
                                            Rover.worldmap.shape[0],scale)
        obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, 
                                                    Rover.pos[0], Rover.pos[1],Rover.yaw, 
                                                    Rover.worldmap.shape[0], scale)

        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, 
                                                        Rover.pos[0], Rover.pos[1], Rover.yaw, 
                                                        Rover.worldmap.shape[0], scale)
                                                        
* Update the map when roll and pitch angle are near zero , becasue the perspective transform is assumed the Rover parallel to the ground, this will largely increase the accuracy of feildty of the worldmap.

        if ((Rover.pitch > 359.5 or Rover.pitch < 0.5) and (Rover.roll > 359.5 or Rover.roll < 0.5)):   
            Rover.worldmap[y_world, x_world, 2] += 1
            Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
            Rover.worldmap[rock_y_world, rock_x_world, 1] += 1 
* Update the Rover angles to navigate the terrain based rover_coords(). If there is a rock in the view, then follow the rock directions, else follow the ground.

        ##  if there is a rock in the map, then follow the rock angle
        if rock.any():
            dist, angles = to_polar_coords(rock_x, rock_y)
            Rover.nav_dists = dist
            Rover.nav_angles = angles
            Rover.sample_detected = True
        else:    
            dist, angles = to_polar_coords(xpix, ypix)
            Rover.nav_dists = dist
            Rover.nav_angles = angles

### For the decision_step()
The oervall deicision tree is shown below:
Logic shown below: 

        check and record the start position
        check is if there is engough pix to move
            if we have found all 6 samples
                find the distance to the start point and use path_planning alogirthme to co back
            else:
                come to next_state() and conitinue searching
        check if the pickup condition is satified

Inside the next_state() function:
there are three large parts of the mode to decide, the sample is in our sight, how the decision will be made based on the current state, and how to switch state in each mode

        def next_state(Rover):
            if Rover.sample_detected:
                    sample_dectected mode()
            elif Rover.mode == 'forward':
                forward(Rover)
            elif Rover.mode == 'stop':
                stop(Rover)
                
If the sample detected:
if the rock is in the sight, first, we evalute whether if it is close enough to pickup, else, we need to adjust the velocity to make the rover drive towards the rock, the velocity condiction are divided into three parts, too slow, stop condition, and too fast, in each section, the Rover throttle,steer, and brake value are applied accordingly. The offset value here is manully adjusted to force the Rover drive towards the left wall, so that it can map the majority of the worldmap.
     
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
            
If in the forward mode:
if the Rover is in the foward mode,first ckeck whether to continue moving with enough pix, if continue moving, then check if Rover the max velocity , and fully accelate, if the Rover is not moving, also two conditions to consider, and if the Rover is too fast, just coasting for a while. But if there is not enough pix to move, the Rover stops and switch into stop mode.


          ## Check the extent of navigable terrain
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

If in the stop mode:
if the Rover is in the stop mode, first check whether the velocity is small enough ,if not, the Rover is set to fully brake, else, check the current navigate angles in front of the car to make decision to move or not.


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


### Result Analysis and future work
The final resuls is shown below, the Rove can successfully find all 6 rock. 

[image6]: ./output/result.jpg

![alt text][image6]



which mapped 97.1% of the worldmap,the fidelity is 76.1%. But there are still lots of parts need to be improved, I haven't finish the path planning part by using a* to come back to the start location, also, the time required to finshied the mapping is unstable, which depends on the rocks location, especially in some hanging over conner which sometimes takes several rounds to find. Also the offset value for the hugging wall is manually turned, which sometimes makes the driving not stable and smooth, which in the future we need to controller to based the velocity to make the control command more stable.
