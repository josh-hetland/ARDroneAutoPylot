#!/usr/bin/env python

'''
Track a green ball using OpenCV.
    Copyright (C) 2015 Conan Zhao and Simon D. Levy
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
'''

import cv2
import numpy as np

# For OpenCV2 image display
WINDOW_NAME = 'ColorTracker'
WINDOW2_NAME = 'Mask'

def track(image,reference_size):

    '''Accepts BGR image as Numpy array
       Returns: (x,y,a) coordinates of centroid if found and the area of the object
                (-1,-1,-1) if no centroid was found
                None if user hit ESC
    '''

    if not hasattr(track,'size'):
        track.size = -1

    # Blur the image to reduce noise
    blur = cv2.GaussianBlur(image, (5,5),0)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    #hl = cv2.getTrackbarPos('h low','Sliders')
    #hu = cv2.getTrackbarPos('h high','Sliders')
    #sl = cv2.getTrackbarPos('s low','Sliders')
    #su = cv2.getTrackbarPos('s high','Sliders')
    #vl = cv2.getTrackbarPos('v low','Sliders')
    #vu = cv2.getTrackbarPos('v high','Sliders')

    # Threshold the HSV image for a specific color
    #lower_variable = np.array([hl,sl,vl])
    #upper_variable = np.array([hu,su,vu])
    lower_orange = np.array([6,168,175])
    upper_orange = np.array([24,255,255])
    lower_variable = np.array([40,70,70])
    upper_variable = np.array([80,200,200])

    # Threshold the HSV image
    mask = cv2.inRange(hsv, lower_variable, upper_variable)
    
    # Blur the mask
    bmask = cv2.GaussianBlur(mask, (5,5),0)

    # Take the moments to get the centroid
    moments = cv2.moments(bmask)
    m00 = moments['m00']
    centroid_x, centroid_y = None, None
    if m00 != 0 and m00 > 5000:
        centroid_x = int(moments['m10']/m00)
        centroid_y = int(moments['m01']/m00)

    # Assume no centroid
    ctr = ( -1, -1 )
    object_values = ( 0, 0, 0 )

    # Use centroid if it exists
    if centroid_x != None and centroid_y != None:

        ctr = ( centroid_x, centroid_y )
        object_values = ( centroid_x, centroid_y, m00 )

        if cv2.waitKey(1) & 0xFF == 32:
            track.size = m00

        # Put black circle in at centroid in image
        cv2.circle(image, ctr, 4, (0,0,0))
        # Put a rectable around the detected mass in image
        #cv2.rectable(image, (),(),(255,0,0),2)
        text_color = (0,255,0)
        if reference_size != -1:
            if m00 < ( reference_size - (reference_size / 4) ):
                text_color = (0,255,255)
            elif m00 > ( reference_size + (reference_size / 4) ):
                text_color = (0,0,255)
                    
        cv2.putText( image, "%.2f" %  ( m00 / 1000 ),
                     (image.shape[1] - 300, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                     2.0,text_color, 3)

        


    # Display full-color image
    cv2.imshow(WINDOW2_NAME, bmask)
    cv2.imshow(WINDOW_NAME, image)

    

    # Force image display, setting centroid to None on ESC key input
    if cv2.waitKey(1) & 0xFF == 27:
        object_values = None
    
    # Return coordinates of centroid
    return object_values

def Callback( value ):
    return




# Test with input from camera
if __name__ == '__main__':
    # Creating track bar
    cv2.namedWindow('Sliders')
    cv2.createTrackbar('h low', 'Sliders',6,179,Callback)
    cv2.createTrackbar('h high', 'Sliders',24,179,Callback)
    cv2.createTrackbar('s low', 'Sliders',168,255,Callback)
    cv2.createTrackbar('s high', 'Sliders',255,255,Callback)
    cv2.createTrackbar('v low', 'Sliders',175,255,Callback)
    cv2.createTrackbar('v high', 'Sliders',255,255,Callback)

    # get the video source
    capture = cv2.VideoCapture(0)

    while True:

        okay, image = capture.read()

        if okay:

            if not track(image,-1):
                break
          
            if cv2.waitKey(1) & 0xFF == 27:
                break

        else:
            print('Capture failed')
            break
