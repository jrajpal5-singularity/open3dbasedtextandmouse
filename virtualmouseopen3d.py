# this file is now not associated with main project (main.py file)
import autopy
import cv2
import pyrealsense2 as rs
import numpy as np
import mediapipe as mp
import time
import open3d as o3d
import math


class handfinder():
    def __init__( self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,1,
                                        self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]

    def findHands(self, img, draw=True):
        # imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        imgRGB = img
        self.results = self.hands.process(imgRGB)
        # print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms,
                                               self.mpHands.HAND_CONNECTIONS)

        return img

    def findPosition(self, cimg, handNo=0, draw=True):
        xList = []
        yList = []

        bbox = []
        self.lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                # print(id, lm)
                h, w, c = cimg.shape
                # h2, w2, c2 = dimg.shape
                # print(h2, w2, c2, "h2 w2 c2")
                # print(h,w,c)
                cx, cy = int(lm.x * w), int(lm.y * h)
                xList.append(cx)
                yList.append(cy)
                # print(id, cx, cy)
                self.lmList.append([id, cx, cy])
                if draw:
                    cv2.circle(cimg, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
                    # cv2.circle(dimg, (cx, cy), 5, (255, 0, 255), cv2.FILLED)

            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            bbox = xmin, ymin, xmax, ymax

            if draw:
                cv2.rectangle(cimg, (xmin - 20, ymin - 20), (xmax + 20, ymax + 20),
                              (0, 255, 0), 2)

        return self.lmList, bbox

    def fingersUp(self):
        fingers = []
        # Thumb
        if self.lmList[self.tipIds[0]][1] > self.lmList[self.tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)

        # Fingers
        for id in range(1, 5):

            if self.lmList[self.tipIds[id]][2] < self.lmList[self.tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)

        # totalFingers = fingers.count(1)

        return fingers

    # def findDistance(self, p1, p2, img, draw=True,r=15, t=3):
    #     x1, y1 = self.lmList[p1][1:]
    #     x2, y2 = self.lmList[p2][1:]
    #     cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    #
    #     if draw:
    #         cv2.line(img, (x1, y1), (x2, y2), (255, 0, 255), t)
    #         cv2.circle(img, (x1, y1), r, (255, 0, 255), cv2.FILLED)
    #         cv2.circle(img, (x2, y2), r, (255, 0, 255), cv2.FILLED)
    #         cv2.circle(img, (cx, cy), r, (0, 0, 255), cv2.FILLED)
    #     length = math.hypot(x2 - x1, y2 - y1)
    #
    #     return length, img, [x1, y1, x2, y2, cx, cy]

    def camerastart(self):
        print("Loading Intel Realsense Camera")
        self.pipeline = rs.pipeline()

        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        pTime = 0
        cTime = 0
        prev_z = 0
        click_count = 0

        frameR = 100  # Frame Reduction
        smoothening = 7
        plocX, plocY = 0, 0
        clocX, clocY = 0, 0

        # detector = self.handDetector(maxHands=1)
        wScr, hScr = autopy.screen.size()





        try:


            while True:

                self.frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(self.frames)
                self.depth_frame = aligned_frames.get_depth_frame()
                self.color_frame = aligned_frames.get_color_frame()

                if not self.depth_frame or not self.color_frame:
                    # If there is no frame, probably camera not connected, return False
                    print(
                        "Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
                    exit(0)



                # Convert images to numpy arrays
                self.depth_image = np.asanyarray(self.depth_frame.get_data())
                self.color_image = np.asanyarray(self.color_frame.get_data())

                #numpy based camera portrait and landscape didn't worked correctly
                # self.color_image = np.rot90(self.color_image, k=1, axes=(0, 1))
                # self.depth_image = np.rot90(self.depth_image, k=1, axes=(0, 1))
                #
                # self.color_image = np.fliplr(self.color_image)
                # self.depth_image = np.fliplr(self.depth_image)





                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)

                depth_colormap_dim = self.depth_image.shape
                color_colormap_dim = self.color_image.shape

                # If depth and color resolutions are different, resize color image to match depth image for display
                # if depth_colormap_dim != color_colormap_dim:
                #     resized_color_image = cv2.resize(self.color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
                #     self.images = np.hstack((resized_color_image, self.depth_colormap))
                #     self.color_image = resized_color_image
                # else:
                #     self.images = np.hstack((self.color_image, self.depth_colormap))

                cimg = self.findHands(self.color_image)
                # dimg = self.findHands(self.depth_image)
                lmList, bbox = self.findPosition(cimg)
                # lmList1, bbox1 = self.findPosition(dimg)
                forefinger = 8
                if len(lmList) != 0:
                    # # print(lmList[forefinger])
                    # # print("dimg location ",(self.depth_image[lmList[forefinger][1],lmList[forefinger][2]]))
                    # cv2.circle(cimg, (lmList[forefinger][1],lmList[forefinger][2]), 2, color=(255, 255, 255), thickness=-1)
                    # # cv2.rectangle(cimg, (5,5), (635,475), color=(0,0,0), thickness=1)
                    curr_z = self.depth_image[lmList[forefinger][1],lmList[forefinger][2]]
                    diff_z = (curr_z-prev_z)
                    # if diff_z in range(30,500):
                    #     clicked = True
                    #     click_count +=1
                    #     print("difference is ",diff_z)
                    #     print("click was done ",click_count)
                    # else:
                    #     pass
                    # prev_z = curr_z
                    x1, y1 = lmList[8][1:]
                    # x2, y2 = lmList[12][1:]

                    fingers = self.fingersUp()

                    cv2.rectangle(cimg, (frameR, frameR), (640 - frameR, 480 - frameR),
                                  (255, 0, 255), 2)

                    # 4. Only Index Finger : Moving Mode
                    # if fingers[1] == 1 and fingers[2] == 0:
                    #     # 5. Convert Coordinates
                    #     x3 = np.interp(x1, (frameR, 640 - frameR), (0, wScr))
                    #     y3 = np.interp(y1, (frameR, 480 - frameR), (0, hScr))
                    #     # 6. Smoothen Values
                    #     clocX = plocX + (x3 - plocX) / smoothening
                    #     clocY = plocY + (y3 - plocY) / smoothening
                    #
                    #     # 7. Move Mouse
                    #     autopy.mouse.move(wScr - clocX, clocY)
                    #     cv2.circle(cimg, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
                    #     plocX, plocY = clocX, clocY

                    # # 8. Both Index and middle fingers are up : Clicking Mode
                    # 4. Only Index Finger : Moving Mode
                    if fingers[1] == 1 and fingers[2] == 0:
                        # 9. Find distance between fingers
                        # length, img, lineInfo = self.findDistance(8, 12, img)

                            # 5. Convert Coordinates
                        x3 = np.interp(x1, (frameR, 640 - frameR), (0, wScr))
                        y3 = np.interp(y1, (frameR, 480 - frameR), (0, hScr))
                        # 6. Smoothen Values
                        clocX = plocX + (x3 - plocX) / smoothening
                        clocY = plocY + (y3 - plocY) / smoothening

                        # 7. Move Mouse
                        autopy.mouse.move(wScr - clocX, clocY)
                        cv2.circle(cimg, (x1, y1), 15, (255, 0, 255), cv2.FILLED)
                        plocX, plocY = clocX, clocY


                        print(diff_z)
                        # 10. Click mouse if distance short

                        if diff_z in range(50,500):

                            print("difference is ", diff_z)
                            print("click was done ",click_count)
                            click_count += 1
                            autopy.mouse.click()
                            time.sleep(0.001)
                        elif diff_z <0:
                            print("negative difference is ", diff_z)
                            print("click was released ", click_count)

                    prev_z = curr_z



                cTime = time.time()
                fps = 1 / (cTime - pTime)
                pTime = cTime

                cv2.putText(cimg, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
                            (255, 0, 255), 3)
                # cv2.putText(dimg, str(int(fps)), (10, 70), cv2.FONT_HERSHEY_PLAIN, 3,
                #             (255, 0, 255), 3)


                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', cimg)
                # cv2.imshow('RealSense', dimg)
                # images = np.hstack((cimg, dimg))
                # cv2.imshow('RealSense', images)

                key = cv2.waitKey(1)
                if key == 27:
                    cv2.destroyAllWindows()
                    break




            pass #exit(0) was here


        # except Exception as e:
        #     print(e)
        #     pass

        finally:

            # center = (self.point_x/1000, self.point_y/1000, (int(distance_mm + diskradius))/1000)
            self.pipeline.stop()
            cv2.destroyAllWindows()
            cv2.waitKey(1000)
            # print("center is ")
            # print(center)
            # return center,self.diskalignment

if __name__ == "__main__":
    diskalign= handfinder()
    diskalign.camerastart()


