#!/usr/bin/env python
import math
import cv2
import numpy as np
from cv_bridge import CvBridge      # to convert between ros2 image and numpy array (for opencv)

# Imports from Michael:
from detection.detection import Detection 

def approx_to_polygon(contour):
    # Approximate contour to polygon
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
    return approx

def is_square(approx):
    # Check a bunch of conditions whether a contour is square-like
    # Approximation must have 4 corners
    if len(approx) != 4:
        return False

    # Must be convex
    if not cv2.isContourConvex(approx):
        return False

    # Area check
    area = cv2.contourArea(approx)                      
    min_area = 500                          # might need to finetune
    if area < min_area:
        return False

    # Check angles ~ 90 degrees using cosine
    pts = approx.reshape(4, 2)
    for i in range(4):
        p0 = pts[i]
        p1 = pts[(i + 1) % 4]
        p2 = pts[(i + 2) % 4]

        v1 = p0 - p1
        v2 = p2 - p1

        cos_angle = abs(
            np.dot(v1, v2) /
            (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-10)
        )

        if cos_angle > 0.3:  # ~72–108 degrees                      # might need to finetune
            return False
        
    # Side length check (Is it a square or a rectangle)
    side_lengths = []
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        side_length = np.linalg.norm(p1 - p2)
        side_lengths.append(side_length)
    min_side = min(side_lengths)
    max_side = max(side_lengths)

    aspect_tolerance = 0.2   # 20% tolerance, tunable parameter
    if (max_side - min_side) / max_side > aspect_tolerance:
        return False

    return True

def draw_cs_on_image(image,centerpoint,angle):
    # Draw cube center and orientation onto image
    arrow_length = 50
    theta = angle/180*math.pi
    cx,cy = centerpoint
    # X axis:
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(cx+arrow_length*math.cos(theta)),int(cy+arrow_length*math.sin(theta))),(0,0,255),2)
    # Y axis:
    cv2.arrowedLine(image, (int(cx), int(cy)), (int(cx-arrow_length*math.sin(theta)),int(cy+arrow_length*math.cos(theta))),(0,255,0),2)

def draw_target_on_image(image,x,y):
    cv2.drawMarker(image,(int(x),int(y)),(0,0,255),cv2.MARKER_CROSS,15,2)

def saturate_difference(current,previous,limit):
    if abs(current - previous) > limit:
        if (current - previous) > 0:
            return previous + limit
        else:
            return previous - limit
    else:
        return current
    
def is_the_target_cube_colored(msg, width_target, height_target, publisher):
    # Takes a square area around the target pixel in the input image, 
    # and checks whether its average color matches one of the possible cube colors

    # Convert ros2 Image to numpy array
    bridge = CvBridge()
    raw_image = bridge.imgmsg_to_cv2(       
        msg,
        desired_encoding='passthrough'
    )
    bgr_image = cv2.cvtColor(raw_image,cv2.COLOR_YUV2BGR_YUY2)
    ksl = 7      # kernel side length, how big of a square to analyze around the target pixel
    crop = bgr_image[height_target-ksl:height_target+ksl+1, width_target-ksl:width_target+ksl+1]
    average = np.mean(crop, axis=(0, 1))
    b,g,r = average/255

    # Debug
    out_msg = bridge.cv2_to_imgmsg(
            crop,
            encoding='bgr8'
        )
    out_msg.header = msg.header
    publisher.publish(out_msg)
    
    h,s,v = Detection.rgb_to_hsv(None, r, g, b)     # TODO replace this method w a regular function eventually
    print(r,g,b)
    print([h,s,v])
    if is_red(h,s,v): 
        print("Red cube grabbed")
        return True
    elif is_blue(h,s,v): 
        print("Blue cube grabbed")
        return True
    elif is_green(h,s,v): 
        print("Green cube grabbed")
        return True
    elif is_wood(h,s,v): 
        print("Wood cube grabbed")
        return True
    else:
        return False
              
    
def find_cube_in_image_msg(msg, publisher1, publisher2, publisher3, publisher4, width_target, height_target, publish_debug_images):
    # Looks for cube top face position and orientation in image
    # Debug: publishes substep images of the detection process using the given publishers

    # Convert ros2 Image to numpy array
    bridge = CvBridge()
    raw_image = bridge.imgmsg_to_cv2(       
        msg,
        desired_encoding='passthrough'
    )

    # Check whether camera settings are as intended
    image_shape = raw_image.shape
    image_half_width = image_shape[1]/2
    image_half_height = image_shape[0]/2
    assert image_half_width == 320      
    assert image_half_height == 240

    # Image preprocess:
    # HSL filtering - non-aggressive, just takes out really dark and really gray pixels
    bgr_image = cv2.cvtColor(raw_image,cv2.COLOR_YUV2BGR_YUY2)
    hls = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HLS)
    lower = np.array([0, 25, 25])     # H, L, S
    upper = np.array([179, 255, 255])
    mask = cv2.inRange(hls, lower, upper)
    filtered = cv2.bitwise_and(bgr_image, bgr_image, mask=mask)

    # Grayscale, blur so texture wont get detected as edges, Canny edge detection
    gray = cv2.cvtColor(filtered, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.5)
    canny = cv2.Canny(gray, 50,150)
        
    # Thicken edges so cube faces become distinctly separate
    kernel = np.ones((5,5), np.uint8)
    canny = cv2.dilate(canny, kernel)
    canny = cv2.bitwise_not(canny)

    cube_position_in_frame = False
    cube_orientation_in_frame = False
    cube_position_available = False

    # Pass 1: If it can clearly see the cube top face, mark it
    contours, hierarchy = cv2.findContours(
        canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    for i in range(len(contours)):
        #if hierarchy[0][i][2] == -1:            # look for only the innermost square, sometimes the cube shadow seems like an enveloping larger cube face
        poly = approx_to_polygon(contours[i])
        if is_square(poly):
            rect = cv2.minAreaRect(poly)  # returns ((cx, cy), (width, height), angle)
            centerpoint = rect[0]
            angle = rect[2]         # in degrees
            cube_position_in_frame = centerpoint
            cube_orientation_in_frame = angle
            cube_position_available = True
            if publish_debug_images:    # mark cube pose in debug image
                cv2.drawContours(bgr_image, contours, i, (255,0,0), 4)
                draw_cs_on_image(bgr_image,centerpoint,angle)

    # Pass 2: If it cannot see a clear cube top face, try to mark a large smudge distinct from the background
    if not cube_position_available:
        _, bw = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        kernel = np.ones((9,9), np.uint8)
        bw_opened = cv2.morphologyEx(bw, cv2.MORPH_OPEN, kernel, iterations=2)
        
        contours, hierarchy = cv2.findContours(
            bw_opened, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])     # if the smudge is large enough, treat it as the cube       
            min_area = 1000                          # might need to finetune
            if area > min_area:
                rect = cv2.minAreaRect(contours[i])  # returns ((cx, cy), (width, height), angle)
                centerpoint = rect[0]
                angle = rect[2]         # in degrees
                cube_position_in_frame = centerpoint
                cube_orientation_in_frame = angle
                cube_position_available = True
                if publish_debug_images:    # mark cube pose in debug image
                    cv2.drawContours(bgr_image, contours, i, (255,0,0), 4)
                    draw_cs_on_image(bgr_image,centerpoint,angle)

    if publish_debug_images:
        out_msg = bridge.cv2_to_imgmsg(         # convert the np array back to ros2 Image msg
            gray,
            encoding='mono8'
        )
        out_msg.header = msg.header
        publisher1.publish(out_msg)     

        out_msg = bridge.cv2_to_imgmsg(
            canny,
            encoding='mono8'
        )
        out_msg.header = msg.header
        publisher3.publish(out_msg)   

        draw_target_on_image(bgr_image,width_target,height_target)
        out_msg2 = bridge.cv2_to_imgmsg(
            bgr_image,
            encoding='bgr8'
        )
        out_msg2.header = msg.header
        publisher2.publish(out_msg2)

        try:
            out_msg = bridge.cv2_to_imgmsg( 
                bw_opened,
                encoding='mono8'
            )
            out_msg.header = msg.header
            publisher4.publish(out_msg)
        except:
            pass
    
    # Return pose if something was detected, otherwise raise an error
    if cube_position_available:    
        return cube_position_in_frame, cube_orientation_in_frame
    else:
        raise Exception("Cube not found in frame")

def is_red(h,s,v):          # Tuned for arm camera, not the same as the values used in detection
    return True if (h <= 20 or h >= 340) and s > 0.5 and v > 0.5 else False

def is_blue(h,s,v):
    return True if (h >= 180 and h <= 200) and s > 0.5 and v > 0.4 else False

def is_green(h,s,v):
    return True if 140 <= h <= 180 and s > 0.5 and v > 0.25 else False

def is_wood(h,s,v):
    return True if 20 <= h <= 60 and 0.3 < s < 0.6 and 0.3 < v < 0.5 else False



def main():
    raise Exception("old, do not use, keeping this here just for safety")


if __name__ == '__main__':
    main()
