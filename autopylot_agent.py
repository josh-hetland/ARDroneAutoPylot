'''
Python green-ball-tracking agent for AR.Drone Autopylot program.  
    Copyright (C) 2013 Simon D. Levy
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 You should have received a copy of the GNU Lesser General Public License 
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 You should also have received a copy of the Parrot Parrot AR.Drone 
 Development License and Parrot AR.Drone copyright notice and disclaimer 
 and If not, see 
   <https://projects.ardrone.org/attachments/277/ParrotLicense.txt> 
 and
   <https://projects.ardrone.org/attachments/278/ParrotCopyrightAndDisclaimer.txt>.
'''

# PID parameters
Kpx = 0.25
Kpy = 0.25
Kdx = 0.25
Kdy = 0.25
Kix = 0
Kiy = 0

import cv2
import numpy as np
import object_tracker

# Routine called by C program.
def action(img_bytes, img_width, img_height, is_belly, pass_button, ctrl_state, vbat_flying_percentage, theta, phi, psi, altitude, vx, vy):

    # Set up command defaults
    zap = 0
    phi = 0     
    theta = 0 
    gaz = 0
    yaw = 0
    
    if pass_button != 0:
        print("pass button detected [ %i ]" % pass_button )

    # Set up state variables first time around
    if not hasattr(action, 'count'):
        action.count = 0
        action.errx_1 = 0
        action.erry_1 = 0
        action.phi_1 = 0
        action.gaz_1 = 0
        action.yaw_1 = 0
        action.reference_size = -1
        
        
    # Create full-color image from bytes
    # Create full-color image from bytes
    image = np.frombuffer(img_bytes, np.uint8)
    image = np.ndarray.reshape(image, (img_height,img_width,3))
    
    # Grab centroid of object
    ctr = object_tracker.track(image, action.reference_size)
    
    # Use centroid if it exists
    if ctr:
        # Compute proportional distance (error) of centroid from image center
        # @todo i was getting non-zero values from this, i think i need to check that first
        errx =  _dst(ctr, 0, img_width)
        erry = -_dst(ctr, 1, img_height)
        
        
        # Compute vertical, horizontal velocity commands based on PID control after first iteration
        if action.count > 0:
            if ctr[0] < (img_width / 3) or ctr[0] > ((img_width / 3) * 2):
                #The object is too far outside of center, turn instead of drift
                yaw = _pid(action.yaw_1, errx, action.errx_1, Kpx, Kix, Kdx)
            else:
                phi = _pid(action.phi_1, errx, action.errx_1, Kpx, Kix, Kdx)
            gaz = _pid(action.gaz_1, erry, action.erry_1, Kpy, Kiy, Kdy)
            
        if pass_button == 12:
            # the reference button was pressed, store this size as the baseline
            action.reference_size = ctr[2]
            print("pass_button: storing reference size [ %i ]" % action.reference_size)
            
        if action.reference_size != -1:
            #set theta based on reference if present (no proportional change, keep it slow for now)
            # threshold for pitch adjustment is when it changes by more than 1/4th of its reference size
            if ctr[2] < ( action.reference_size - (action.reference_size / 4) ):
                theta = -.1
            elif ctr[2] > ( action.reference_size + (action.reference_size / 4) ):
                theta = .1
                
        # Remember PID variables for next iteration
        action.errx_1 = errx
        action.erry_1 = erry
        action.phi_1 = phi
        action.gaz_1 = gaz
        action.yaw_1 = yaw
        action.count += 1
        
    print('battery=%2d%% tilt=%+f roll=%+f vert=%+f yaw=%+f' % \
                          (vbat_flying_percentage, theta, phi, gaz, yaw))
    # Send control parameters back to drone
    return (zap, phi, theta, gaz, yaw)



# Simple PID controller from http://www.control.com/thread/1026159301
def _pid(out_1, err, err_1, Kp, Ki, Kd):
    return Kp*err + Ki*(err+err_1) + Kd*(err-err_1) 



# Returns proportional distance to image center along specified dimension.
# Above center = -; Below = +
# Right of center = +; Left = -
def _dst(ctr, dim, siz):
    siz = siz/2
    return (ctr[dim] - siz) / float(siz)    
