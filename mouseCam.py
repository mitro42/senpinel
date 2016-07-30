from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2

# initialize the camera and grab a reference to the raw camera capture
pictureSize = (1024, 768)

def createEmptyImage():
	return np.zeros((pictureSize[1], pictureSize[0], 3), np.uint8)

camera = PiCamera()
camera.resolution = pictureSize
camera.framerate = 5
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
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
	
	#cv2.imshow(liveWindowName, image)	
	d = createEmptyImage()
	cv2.absdiff(lastImage, image, d)
	
	#d = cv2.adaptiveThreshold(d, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
	ret, d = cv2.threshold(d, 30, 255, cv2.THRESH_BINARY)
	d = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
	#cv2.imshow(diffWindowName, d)	
	if np.sum(d) > 100000:
		fileName = "pic_%s_%06d.jpg"%(time.strftime("%Y_%m_%d_%H:%M:%S"), imageCount)
		cv2.imwrite(fileName, image)
		print("image saved: ", fileName)
		imageCount += 1
	
	key = cv2.waitKey(1) & 0xFF
	
	lastImage = image
	rawCapture.truncate(0)
	
	if key == ord("q"):
		break
