import cv2
import numpy as np
from imutils import contours
from skimage import measure
import imutils

class LEDDetector:
    def __init__(self, image_path):
        self.image = image_path
        cv2.imshow("1",self.image)
        self.contour_list = []
        self.area_list = []
        self.process_image()

    def process_image(self):
        image_gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("2",image_gray)
        image_blur = cv2.GaussianBlur(image_gray, (11, 11), 0)
        thresh = cv2.threshold(image_blur, 200, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)
        labels = measure.label(thresh, background=0)
        mask = np.zeros(thresh.shape, dtype="uint8")
        cv2.imshow("blur",image_blur)
        cv2.imshow("thresh",thresh)
        for label in np.unique(labels):
            if label == 0:
                continue
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
            if numPixels > 300:
                mask = cv2.add(mask, labelMask)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = contours.sort_contours(cnts)[0]

        for (i, c) in enumerate(cnts):
            (x, y, w, h) = cv2.boundingRect(c)
            ((cX, cY), radius) = cv2.minEnclosingCircle(c)
            self.contour_list.append((cX, cY))
            area = cv2.contourArea(cnts[0])
            self.area_list.append(area)
            cv2.circle(self.image, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
            cv2.putText(self.image, f"#{i + 1}", (x, y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)

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
