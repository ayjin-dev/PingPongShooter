from numpy import array, argsort
from pickle import load

RESOLUTIONS = array([1280, 720])# array([1920, 1080])
SCALE = 0.25

NUM_OF_MAX_AREAS = 10
CAMERA = 0

from cv2 import CAP_PROP_FRAME_WIDTH,CAP_PROP_FRAME_HEIGHT,VideoCapture,MORPH_OPEN,COLOR_BGR2GRAY,RETR_TREE, CHAIN_APPROX_SIMPLE, resize, cvtColor, COLOR_BGR2YCrCb,inRange,bitwise_and,morphologyEx,findContours,contourArea,boundingRect,rectangle,circle,line, threshold, THRESH_BINARY, CAP_PROP_AUTO_EXPOSURE, CAP_PROP_EXPOSURE
class BaseProc(object):
    def __init__(self):
        self.cap = VideoCapture(CAMERA)
        self.__scale = tuple((RESOLUTIONS * SCALE).astype(int))
        self.cap.set(CAP_PROP_FRAME_WIDTH, RESOLUTIONS[0])
        self.cap.set(CAP_PROP_FRAME_HEIGHT, RESOLUTIONS[1])
        self.cap.set(CAP_PROP_AUTO_EXPOSURE, 1)
        self.cap.set(CAP_PROP_EXPOSURE, 1000)
        _, self.frame = self.cap.read()
        self.img_resize()
        self.morph_img = self.frame
        self.iters = 1
        self.k_size = 1

        self.colors = ('red', 'blue', 'green','pink')
        self.store = {}

        self.min_ycb = [0,0,0]
        self.max_ycb = [255, 255, 255]
        self.iters = 1
        self.k_size = 1
        self.morph_elem = None

    def img_resize(self):
        self.frame = resize(self.frame,  self.__scale)

    def cvt_ycb(self, orig_img):
        image_ycb = cvtColor(orig_img, COLOR_BGR2YCrCb)
        mask = inRange(image_ycb, array(self.min_ycb), array(self.max_ycb))
        return bitwise_and(orig_img, orig_img, mask=mask)

    def morph_transform(self):
        self.morph_img = morphologyEx(cvtColor(self.cvt_ycb(self.frame), COLOR_BGR2GRAY), MORPH_OPEN, self.morph_elem, iterations=self.iters)
        _,self.morph_img = threshold(self.morph_img,0, 255,THRESH_BINARY)

    def select_area(self):
        contours, _ = findContours(self.morph_img,RETR_TREE,CHAIN_APPROX_SIMPLE)
        if len(contours)<=0:
            return None

        # return array([boundingRect(contours[c]) for c in argsort(array([contourArea(c) for _, c in enumerate(contours)]))[-NUM_OF_MAX_AREAS:]])
        return array([boundingRect(contours[c]) for c in argsort([contourArea(c) for _, c in enumerate(contours)])[-NUM_OF_MAX_AREAS:]])


    def draw_ctr(self, coordinate, color=(0,255,0),lw=2):
        x,y,w,h = coordinate
        rectangle(self.frame, (x,y), (x+w,y+h), color, lw)
        target_center = x+w//2, y+h//2
        circle(self.frame, target_center, 2, color,-1)

    def change_color(self, color):
        print('change color:', color)
        (self.min_ycb, self.max_ycb,self.k_size, self.iters) = self.store[color]

    def restore_config(self, cfg="config.pkl"):
        try:
            self.store = load(open(cfg, 'rb'))
            print('loaded config')
        except FileNotFoundError:
            print('config not found!')
            return False
        return True
