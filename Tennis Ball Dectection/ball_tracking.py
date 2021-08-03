#!/usr/bin/env python
import numpy as np
import cv2
import time
from collections import deque
pts = deque(maxlen=64)
def nothing(x):
    pass

cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LS", "Tracking", 0, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 0, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)

# import imutils

def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    #if show: 
        #cv2.imshow("RGB Image",rgb_image)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    # rgb_image = imutils.resize(rgb_image, width=600)
    rgb_image = cv2.GaussianBlur(rgb_image, (11, 11), 0)
    #rgb_image = cv2.bilateralFilter(rgb_image, 15, 75, 75) 
    #rgb_image = cv2.medianBlur(rgb_image,5)
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    #cv2.imshow("hsv image",hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
    

    return mask


    
def houghcircles(rgb_image,b_image):
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
    rgb_image = cv2.bilateralFilter(rgb_image, 15, 75, 75)
    b_image = cv2.erode(b_image, None, iterations=2)
    b_image = cv2.dilate(b_image, None, iterations=2)
    #b_image = cv2.bilateralFilter(b_image, 15, 75, 75) 
    cv2.imshow("Mask",b_image)

    circles = cv2.HoughCircles(b_image,cv2.HOUGH_GRADIENT, 1.7, 30,param1 = 50, param2 =35, minRadius = 10, maxRadius = 200)
    if circles is not None:
    # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
    # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(rgb_image, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(rgb_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        # cv2.namedWindow("Image",cv2.WINDOW_NORMAL)
        
        cv2.imshow("hough output",rgb_image)


def getContours(binary_image):  
    #_, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #           hierarchy                                cv2.CHAIN_APPROX_SIMPLE)
    _,contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')    


    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        center = None
        if (area>1500):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            center = (cx,cy)
            font = cv2.FONT_HERSHEY_SIMPLEX
            #cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(rgb_image, (int(x),int(y)),(int)(radius),(0,0,255),1)
            #cv2.circle(rgb_image, (cx,cy), 5, (0, 0, 255), -1)
            cv2.circle(rgb_image, (int(x),int(y)), 5, (0, 0, 255), -1)
            #cv2.putText(rgb_image,'BALL',(cx,cy), font, 1,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(rgb_image,'BALL',(int(x),int(y)), font, 1,(255,255,255),2,cv2.LINE_AA)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            print ("Area: {}, Perimeter: {}".format(area, perimeter))
        #pts.appendleft(center)
    #print ("number of contours: {}".format(len(contours)))
    # for i in range(1, len(pts)):
    #     if pts[i - 1] is None or pts[i] is None:
    #         continue
    #     thickness = int(np.sqrt(64 / float(i + 1)) * 2.5)
    #     cv2.line(rgb_image, pts[i - 1], pts[i], (0, 0, 255), thickness)
    cv2.imshow("RGB Image Contours Output",rgb_image)

    #cv2.imshow("Black Image Contours",black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame):

    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")

    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")

    #yellowLower =(l_h, l_s, l_v)
    #yellowUpper = (u_h, u_s, u_v)
    #20,40,120
    #yellowLower =(20, 38, 147)
    yellowLower =(20, 100, 50)
    yellowUpper = (44, 255, 255)
    rgb_image = image_frame
    rgb_image1 = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    binary_image_mask1 = filter_color(rgb_image1, yellowLower, yellowUpper)
    houghcircles(rgb_image1,binary_image_mask1)
    draw_ball_contour(binary_image_mask, rgb_image,contours)



def main():
    video_capture = cv2.VideoCapture(0)
    video_capture.set(cv2.CAP_PROP_BUFFERSIZE,1)
    # fps = int(video_capture.get(5))
    # print(fps)
    #video_capture = cv2.VideoCapture('ros_essentials_cpp/src/topic03_perception/video/tennis-ball-video.mp4')
    
    
    while(True):
        ret, frame = video_capture.read()
        detect_ball_in_a_frame(frame)
        time.sleep(0.033)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()



cv2.waitKey(0)
cv2.destroyAllWindows()
