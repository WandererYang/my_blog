# coding=UTF-8
import sys,cv2,time,threading,signal
import numpy as np
import PCA9685

isFaceExisted = False 
x_,y_,w_,h_ = 0,0,0,0
width,height = 640,480
min_plus,max_plus = 500,2500
mutex0 = threading.Lock()
dx,dy = 0,0

def camera_detect():
    global isFaceExisted,width,height,x_,y_,w_,h_,dx,dy
    faceCascade = cv2.CascadeClassifier('Cascades/haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)
    cap.set(3,width)
    cap.set(4,height)
    while True:
        ret,img = cap.read()
        img = cv2.flip(img, 1)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
        gray,
        scaleFactor=1.2,
        minNeighbors=5,     
        minSize=(20, 20))
        for (x,y,w,h) in faces:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            x_,y_,w_,h_ = x,y,w,h
        if len(faces) == 0:
            mutex0.acquire()
            isFaceExisted = False
            mutex0.release()
        else:
            mutex0.acquire()
            isFaceExisted = True
            mutex0.release()
        dx = x_ + 0.5 * w_ - 0.5 * width
        dy = y_ + 0.5 * h_ - 0.5 * height
        cv2.imshow('video',img)
        cv2.waitKey(30)

    cap.release()
    cv2.destroyAllWindows()
        
def servo_control():
    global isFaceExisted,width,height,min_plus,max_plus,x_,y_,w_,h_,dx,dy
    no_face_counter,x_current,y_current,step,detect_angle = 0,1500,1500,0.005,25.0
    pwm = PCA9685.PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)
    pwm.setServoPulse(0,x_current) #500-2500
    pwm.setServoPulse(1,y_current)
    time.sleep(2) 
    while True:
        if isFaceExisted:
            if no_face_counter > 0:
                no_face_counter -= 1
        else:
            if no_face_counter < 5:
                no_face_counter += 1
        if no_face_counter != 5:
            if (dx >= detect_angle) | (dx <= -detect_angle):
                x_current += dx * step
                pwm.setServoPulse(0,x_current)
            if (dy >= detect_angle) | (dy <= -detect_angle):
                y_current -= dy * step
                pwm.setServoPulse(1,y_current)
        time.sleep(0.08)     
    
def quit(signum,frame):
    print("exit face tracking!")
    sys.exit()

def main():
    try:
        signal.signal(signal.SIGINT,quit)
        signal.signal(signal.SIGTERM,quit)
        
        t1 = threading.Thread(target=camera_detect)
        t2 = threading.Thread(target=servo_control)
        t1.setDaemon(True)
        t2.setDaemon(True)
        t1.start()
        t2.start()
        
        while True:
            pass
    except Exception,exc:
        print (exc)
    
if __name__ == "__main__":
    main()
