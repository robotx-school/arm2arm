import cv2
import numpy as np
import math

class Warper:
    def __init__(self, aruco_dict=cv2.aruco.DICT_4X4_250, cache = False, field_size = [515, 315, 0, 0]):
        self.detector = self.get_detector(aruco_dict)
        self.cache = cache
        self.field_size = field_size
        self.h = []
        self.height, self.weight = 0, 0

        
    def get_detector(self, aruco_dict=cv2.aruco.DICT_4X4_250):
        dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector

    def warp_field(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        height, weight, _ = img.shape
        self.height, self.weight = height, weight
        if self.cache and list(self.h) != []:
            return cv2.warpPerspective(img, self.h, (weight, height))

        res = self.detector.detectMarkers(gray)
        coords = [0, 0, 0, 0]

        if res[1] is not None and (0 in res[1] and 1 in res[1] and 2 in res[1] and 3 in res[1]):
            for i in range(4):
                marker = i
                index = np.where(res[1] == marker)[0][0]
                pt0 = res[0][index][0][marker].astype(np.int16)
                coords[i] = list(pt0)
            with open("123.txt", "w", encoding="utf-8") as file:
                file.write(str(coords))
        else:
            return img
        input_pt = np.array(coords)
        output_pt = np.array([[0, 0], [weight, 0], [weight, height], [0, height]])
        self.h, _ = cv2.findHomography(input_pt, output_pt)
            
        res_img = cv2.warpPerspective(img, self.h, (weight, height))
        return res_img

    def get_pos(self, x, y):
        return round((x+self.field_size[2])*(self.field_size[0]/self.weight)), round((y+self.field_size[3])*(self.field_size[1]/self.height))
