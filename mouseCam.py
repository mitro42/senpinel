from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2

# initialize the camera and grab a reference to the raw camera capture
pictureSize = (640, 480)
fps = 10
def createEmptyImage():
	return np.zeros((pictureSize[1], pictureSize[0], 3), np.uint8)

camera = PiCamera()
camera.resolution = pictureSize
camera.framerate = fps
rawCapture = PiRGBArray(camera, size=pictureSize)

# allow the camera to warmup
time.sleep(0.1)
lastImage = createEmptyImage()
liveWindowName = "Live"
diffWindowName = "Diff"

cv2.namedWindow(liveWindowName)
cv2.namedWindow(diffWindowName)

cv2.moveWindow(liveWindowName, 0, 10)
cv2.moveWindow(diffWindowName, pictureSize[0], 10)
imageCount = 0
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
recording = False
# capture frames from the camera
lastChangeTime = time.time() 
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	
	
	d = createEmptyImage()
	cv2.absdiff(lastImage, image, d)
	
	#d = cv2.adaptiveThreshold(d, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
	ret, d = cv2.threshold(d, 30, 255, cv2.THRESH_BINARY)
	d = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
	changeSum = np.sum(d)
	#cv2.imshow(liveWindowName, image)	
	#cv2.imshow(diffWindowName, d)	
	
	if changeSum != 0:
		print(changeSum, changeSum / (pictureSize[0]*pictureSize[1]))
	if changeSum > 5000:
		if not recording:
			frameCount = 0
			fileName = "vid_%s_%06d.avi"%(time.strftime("%Y_%m_%d_%H_%M_%S"), imageCount)
			recording = True
			outFile = cv2.VideoWriter(fileName, fourcc, fps, pictureSize)
			imageCount += 1
		lastChangeTime = time.time()
	else:
		print(time.time() - lastChangeTime)
		if recording and time.time()- lastChangeTime > 3:
			recording = False
			outFile.release()

	if recording:	
		outFile.write(image)
		print("recording... frame %d"%frameCount)
		frameCount += 1

	lastImage = image
	rawCapture.truncate(0)
	
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break

outFile.release()
