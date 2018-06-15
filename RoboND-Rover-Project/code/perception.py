import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
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


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:,:,0]), M, (img.shape[1], img.shape[0]))
    
    return warped, mask

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # Define source and destination points for perspective transform
    dst_size = 5 

    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    # Apply perspective transform
    warped, mask = perspect_transform(Rover.img, source, destination)

    # Apply color threshold to identify navigable terrain/obstacles/rock samples
    ground, rock = color_thresh(warped)
    obs_map = np.absolute(np.float32(ground) - 1) * mask

    # Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,2] = ground * 255
    Rover.vision_image[:,:,0] = obs_map * 255
    Rover.vision_image[:,:,1] = rock * 255

    xpix, ypix = rover_coords(ground)
    obsxpix,obsypix = rover_coords(obs_map)
    rock_x, rock_y = rover_coords(rock)

        
    # Convert rover-centric pixel values to world coordinates

    scale = 2 * dst_size

    x_world, y_world = pix_to_world(xpix, ypix, 
                                        Rover.pos[0], Rover.pos[1],Rover.yaw, 
                                        Rover.worldmap.shape[0],scale)

    obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, 
                                                Rover.pos[0], Rover.pos[1],Rover.yaw, 
                                                Rover.worldmap.shape[0], scale)


    rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, 
                                                    Rover.pos[0], Rover.pos[1], Rover.yaw, 
                                                    Rover.worldmap.shape[0], scale)


    ## update the map when roll and pitch angle are near zero
    ## becasue the perspective transform is assumed the Rover parallel to the ground
    if ((Rover.pitch > 359.5 or Rover.pitch < 0.5) and (Rover.roll > 359.5 or Rover.roll < 0.5)):

        ## Update Rover worldmap (to be displayed on right side of screen)
        Rover.worldmap[y_world, x_world, 2] += 1
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
        # pos = Rover.worldmap[:,:,2] > 0
        # Rover.worldmap[pos,0] = 0
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1 

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

    return Rover