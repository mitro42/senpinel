from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import os

# settings
pictureSize = (640, 480)
fps = 10
onscreenDisplay = True
saveVideo = True

class VideoOutput(object):
	def __init__(self, enabled, fps, resolution):
		self.fps = fps
		self.resolution = resolution

		self.videoCount = 0;
		self.fileName = ""
		self.saveEnabled = enabled
		self.frameCount = 0
		self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')

	def __del__(self):
		self.outFile.release()

	def startRecording(self):
		print("startRecording")
		while self.fileName =="" or os.path.exists(self.fileName):
			self.videoCount += 1
			self.fileName = "vid_%06d.avi"%(self.videoCount)

		self.outFile = cv2.VideoWriter(self.fileName,
			self.fourcc,
			self.fps,
			self.resolution)
		print("Start recording to file ", self.fileName)

	def saveImage(self, image):
		if self.saveEnabled:
			self.outFile.write(image)
			print("recording frame %d"%self.frameCount)
		self.frameCount += 1


	def stopRecording(self):
		self.fileName = ""
		if self.saveEnabled:
			self.outFile.release()


def createEmptyImage():
	return np.zeros((pictureSize[1], pictureSize[0], 3), np.uint8)

def addTimeStamp(image):
	size = (255, 22)
	topLeft = (int((pictureSize[0] - size[0])/2), 20)
	bottomRight = (topLeft[0] + size[0], topLeft[1] + size[1])
	bottomLeft = (topLeft[0], topLeft[1] + size[1]-2)
	cv2.rectangle(image, topLeft, bottomRight, (0,0,0), -1)
	font = cv2.FONT_HERSHEY_PLAIN
	text = time.strftime("%Y.%m.%d %H:%M:%S")
	cv2.putText(image, text, bottomLeft, font, 1.5, (255, 255, 255), 1, cv2.LINE_AA)

camera = PiCamera()
camera.resolution = pictureSize
camera.framerate = fps
rawCapture = PiRGBArray(camera, size=pictureSize)

# allow the camera to warmup
time.sleep(0.1)
lastImage = createEmptyImage()

if onscreenDisplay:
	liveWindowName = "Live"
	diffWindowName = "Diff"

	cv2.namedWindow(liveWindowName)
	cv2.namedWindow(diffWindowName)

	cv2.moveWindow(liveWindowName, 0, 10)
	cv2.moveWindow(diffWindowName, pictureSize[0], 10)


recording = False
# capture frames from the camera
lastChangeTime = time.time()
videoOuput = VideoOutput(saveVideo, fps, pictureSize)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	rawImage = frame.array

	d = createEmptyImage()
	cv2.absdiff(lastImage, rawImage, d)

	ret, d = cv2.threshold(d, 30, 255, cv2.THRESH_BINARY)
	d = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
	changeSum = np.sum(d)

	image = rawImage.copy()
	addTimeStamp(image)
	if onscreenDisplay:
		cv2.imshow(liveWindowName, image)
		cv2.imshow(diffWindowName, d)

	if changeSum != 0:
		print(changeSum, changeSum / (pictureSize[0]*pictureSize[1]))
	if changeSum > 5000:
		if not recording:
			videoOuput.startRecording()
			recording = True
		lastChangeTime = time.time()
	else:
		print(time.time() - lastChangeTime)
		if recording and time.time()- lastChangeTime > 3:
			recording = False
			videoOuput.stopRecording()

	if recording:
		videoOuput.saveImage(image)

	lastImage = rawImage
	rawCapture.truncate(0)

	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break

