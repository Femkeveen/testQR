import cv2
import numpy as np
import cv2.aruco as aruco
import time

class GetQRCode():
    def __init__(self):
        #camera calibration parameters
        self.mtx = np.array([[320, 0., 220],[0., 320, 120],[0., 0., 1.]])
        self.distCoeffs = np.array([0.03, -0.1, -0.002, 0, 0.1])
        self.h, self.w = 320, 480

        self.mtx, roi = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.distCoeffs, (self.h, self.w), 0.0, (self.h, self.w)
            )
        self.x, self.y, self.z = 0, 0, 0

        #camera initialization
        self.cap = cv2.VideoCapture(0) #setup camera object ' cap' 
        self.cap.set(3, self.w)
        self.cap.set(4, self.h)

    def QRCodeDisplay(self):
        ret, frame = self.cap.read()  #get a image of the QR code

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100) 
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #convert to grayscale
        parameters = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, parameters)
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)
   
        cv2.imshow('frame', frame) #show the image frame

        markerLength = 0.07; #Length of QR code, necessary for scaling the translation vector
        if markerIds is not None: #in case QR code is detected
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, markerLength, self.mtx, self.distCoeffs) #estimating rotation and translation of detected markers
            (rvec - tvec).any()  # get rid of that nasty numpy value array error, check if nonzeo element are present
            # print(tvec)
            self.x,self.y,self.z = tvec[0][0][0], tvec[0][0][1], tvec[0][0][2] #extracting x, y, z coordinates

            for i in range(rvec.shape[0]):
                frame = cv2.drawFrameAxes(frame, self.mtx, self.distCoeffs, rvec[i, :, :], tvec[i, :, :], 0.03) #0.03 is the length of the axes that will be drawn
                aruco.drawDetectedMarkers(frame, markerCorners)

                

            aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            
            #center coordinates
            center_x, center_y = frame.shape[1] //2, frame.shape[0]//2
            #draw small circle at the center of the screen
            cv2.circle(frame, (center_x, center_y), 3, (0,255,0), -1) #image frame, coordinates, radius of the circle (in pixels), color, -1: filled

            cv2.imshow('frame', frame)
        else:
            self.x = None;

        return self.x
        
    def EndQRCode(self):
        self.cap.release()
        cv2.destroyAllWindows()


#calculate the percentage of detected qr codes
#x involves the offset 
    def QR_detection_rate(x, x_total, x_detection):
        x_total.append(x)

        if x is not None:   
            x_det = 1 #return 1 if QR code is detected
        else:
            x_det = 0 #else return 0

        x_detection.append(x_det)
        n_detected = x_detection.count(1) #total number of detected QR code frames
        detection_rate = n_detected/len(x_total)
        
        return x_total, x_detection, detection_rate


if __name__ == '__main__':
    code =  GetQRCode()
    
    while True:
        x = code.QRCodeDisplay() #calculate the 
        print(x)
        c = cv2.waitKey(10) #delay between calculating the qr code positions in ms
        if c == 27:
            code.EndQRCode() 

        # import numpy as np
        # import matplotlib.pyplot as plt
        # offset = np.append(error_saved, x)
        # plt.plot(error_values)






