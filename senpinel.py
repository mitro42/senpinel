from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
import os
import configparser

class VideoOutput(object):
	def __init__(self, enabled, fps, resolution):
		self.fps = fps
		self.resolution = resolution

		self.videoCount = 0;
		self.fileName = ""
		self.saveEnabled = enabled
		self.frameCount = 0
		self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')
		self.outFile = None

	def __del__(self):
		self.stopRecording()

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
		if self.saveEnabled and self.outFile != None:
			self.outFile.release()
		self.fileName = ""


def createEmptyImage(imageSize):
	return np.zeros((imageSize[1], imageSize[0], 3), np.uint8)

def addTimeStamp(image):
	size = (255, 22)
	topLeft = (int((image.shape[1] - size[0])/2), 20)
	bottomRight = (topLeft[0] + size[0], topLeft[1] + size[1])
	bottomLeft = (topLeft[0], topLeft[1] + size[1]-2)
	cv2.rectangle(image, topLeft, bottomRight, (0,0,0), -1)
	font = cv2.FONT_HERSHEY_PLAIN
	text = time.strftime("%Y.%m.%d %H:%M:%S")
	cv2.putText(image, text, bottomLeft, font, 1.5, (255, 255, 255), 1, cv2.LINE_AA)


def getSettings(fileName):
	config = configparser.ConfigParser()
	config['general'] = {}
	config['general']['onscreen_display'] = 'False'
	config['general']['save_video'] = 'True'
	config['general']['stop_after'] = '5'
	config['video_input'] = {}
	config['video_input']['width'] = '800'
	config['video_input']['height'] = '600'
	config['video_input']['fps'] = '15'
	config['video_input']['iso'] = '1600'
	config['detection'] = {}
	config['detection']['threshold'] = '20000'

	if os.path.exists(fileName):
		config.read(fileName)
	else:
		with open(fileName, 'w') as configfile:
			config.write(configfile)

	settings = {}
	settings['onscreen_display'] = config['general'].getboolean('onscreen_display')
	settings['save_video'] = config['general'].getboolean('save_video')
	settings['input_resolution'] = (int(config['video_input']['width']), int(config['video_input']['height']))
	settings['fps'] = int(config['video_input']['fps'])
	settings['iso'] = int(config['video_input']['iso'])
	settings['threshold'] = int(config['detection']['threshold'])
	settings['stop_after'] = float(config['general']['stop_after'])
	return settings

def main():
	settings = getSettings('.senpinel')

	camera = PiCamera()
	camera.resolution = settings['input_resolution']
	camera.framerate = settings['fps']
	camera.iso = settings['iso']
	rawCapture = PiRGBArray(camera, size=settings['input_resolution'])

	# allow the camera to warmup
	time.sleep(0.1)
	firstImageCaptured = False

	if settings['onscreen_display']:
		liveWindowName = "Live"
		#diffWindowName = "Diff"

		cv2.namedWindow(liveWindowName)
		#cv2.namedWindow(diffWindowName)

		cv2.moveWindow(liveWindowName, 0, 10)
		#cv2.moveWindow(diffWindowName, settings['input_resolution'][0], 10)


	recording = False
	# capture frames from the camera
	lastChangeTime = time.time()
	videoOuput = VideoOutput(settings['save_video'], settings['fps'], settings['input_resolution'])
	print("Waiting for the camera")
	time.sleep(3)
	print("Start watching")
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		rawImage = frame.array
		if not firstImageCaptured:
			lastImage = rawImage
			firstImageCaptured = True

		d = createEmptyImage(settings['input_resolution'])
		cv2.absdiff(lastImage, rawImage, d)

		ret, d = cv2.threshold(d, 30, 255, cv2.THRESH_BINARY)
		d = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
		changeSum = np.sum(d)

		image = rawImage.copy()
		addTimeStamp(image)
		if settings['onscreen_display']:
			cv2.imshow(liveWindowName, image)
			#cv2.imshow(diffWindowName, d)

		if changeSum != 0:
			print(changeSum, changeSum / (settings['input_resolution'][0]*settings['input_resolution'][1]))
		if changeSum > settings['threshold']:
			if not recording:
				videoOuput.startRecording()
				recording = True
			lastChangeTime = time.time()
		else:
			print(time.time() - lastChangeTime)
			if recording and time.time()- lastChangeTime > settings['stop_after']:
				recording = False
				videoOuput.stopRecording()

		if recording:
			videoOuput.saveImage(image)

		lastImage = rawImage
		rawCapture.truncate(0)

		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):
			break

if __name__ == "__main__":
	main()