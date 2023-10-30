import cv2
import numpy as np
from imutils import contours
from skimage import measure
import imutils

class LEDDetector:
    def __init__(self, image_path):
        self.image = image_path
        self.contour_list = []
        self.static_list=[]
        self.alien_list=[]
        self.area_list = []
        self.process_image()

    def process_image(self):
        image_gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        image_blur = cv2.GaussianBlur(image_gray, (11, 11), 0)
        # print("size:",self.image.shape)
        threshold_value = 200  # Adjust this threshold value as needed
        _, thresholded = cv2.threshold(image_blur, threshold_value, 255, cv2.THRESH_BINARY)

        cnts = cv2.findContours(thresholded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = contours.sort_contours(cnts)[0]

        for (i, c) in enumerate(cnts):
            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            
            self.static_list.append((cX, cY))
            area = cv2.contourArea(cnts[0])
            self.area_list.append(area)
            cv2.circle(self.image, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
            cv2.putText(self.image, f"#{i + 1}", (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        
        if len(self.static_list)>1:
            for (X,Y) in self.static_list:
                for (x,y) in self.static_list:
                    if abs(x-X)<=40 and abs(y-Y)<=40:
                        self.alien_list.append((x,y))
                self.contour_list.append(self.alien_list)
                self.alien_list=[]
        return True   
    def save_results(self, image_output_path, text_output_path):
        cv2.imwrite(image_output_path, self.image)

        with open(text_output_path, "w") as file:
            file.write(f"No. of LEDs detected: {len(self.contour_list)}\n")
            for i, centroid in enumerate(self.contour_list):
                file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {self.area_list[i]}\n")

if __name__ == "__main__":
    # Initialize the LEDDetector with the image path
    led_detector = LEDDetector('led.jpg')

    # Process the image and detect LEDs
    led_detector.process_image()

    # Save the results to image and text files
    led_detector.save_results("LD_2000_led_detection_results.png", "LD_2000_led_detection_results.txt")
