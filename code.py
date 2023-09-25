
import cv2
import RPi.GPIO as GPIO
import time
import imutils

GPIO.setwarnings(False) # tat canh bao
GPIO.setmode(GPIO.BCM)

#Khai bao chan L298
GPIO.setup(5,GPIO.OUT)
GPIO.setup(6,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)

GPIO.setup(16,GPIO.OUT)
GPIO.setup(26,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

pwm_left=GPIO.PWM(13,1000)
pwm_right=GPIO.PWM(12,1000)

pwm_left.start(0)
pwm_right.start(0)
#Khai bao chan cam bien hong ngoai
GPIO.setup(17,GPIO.IN)

#khai bao chan servo
servoPIN = 22
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 22 for PWM with 50Hz
p.start(5.5)
# time.sleep(1)
# p.stop()

greenLower = (11, 70, 40)  # gia tri mau HSV
greenUpper = (60, 230, 255)

# dieu khien dong co
def forward(speed):
    pwm_left.ChangeDutyCycle(speed)  #0-100
    GPIO.output(5,GPIO.HIGH)
    GPIO.output(6,GPIO.LOW)
    
    pwm_right.ChangeDutyCycle(speed)
    GPIO.output(26,GPIO.HIGH)
    GPIO.output(16,GPIO.LOW)
    print("forward")

def left(speed):
    pwm_left.ChangeDutyCycle(speed)
    GPIO.output(5,GPIO.HIGH)
    GPIO.output(6,GPIO.LOW)
    
    pwm_right.ChangeDutyCycle(speed)
    GPIO.output(26,GPIO.LOW)
    GPIO.output(16,GPIO.HIGH)
    print("left")
def right(speed):
    pwm_left.ChangeDutyCycle(speed)
    GPIO.output(5,GPIO.LOW)
    GPIO.output(6,GPIO.HIGH)
    
    pwm_right.ChangeDutyCycle(speed)
    GPIO.output(26,GPIO.HIGH)
    GPIO.output(16,GPIO.LOW)
    print("right")
def stop():
    pwm_left.ChangeDutyCycle(0)
    GPIO.output(5,GPIO.LOW)
    GPIO.output(6,GPIO.LOW)
    
    pwm_right.ChangeDutyCycle(0)
    GPIO.output(16,GPIO.LOW)
    GPIO.output(26,GPIO.LOW)
    #print("stop")
  

    
vs = cv2.VideoCapture(0)# khoi dong camera

while vs.isOpened():
    
    ret, frame = vs.read() # doc anh tu camera
    
    frame = imutils.resize(frame, width=300)   #resize anh
    
    blurred = cv2.GaussianBlur(frame, (11, 11), 0) # loc anh
    
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # chueyn sang hsv
    
    mask = cv2.inRange(hsv, greenLower, greenUpper) # loc nguong 
    
    mask = cv2.erode(mask, None, iterations=2)  # xoi mon
    mask = cv2.dilate(mask, None, iterations=2)
    # tim cac coutours 
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts) 
    
    for c in cnts: 
        if cv2.contourArea(c) > 200: 
            print(cv2.contourArea(c))
            c = max(cnts, key=cv2.contourArea)  # tim contour lon nhat
            ((x, y), radius) = cv2.minEnclosingCircle(c) # xac dinh duong tron
            
            
            if y>200:
                forward(20)
                break
                
            if x <= 200 and x >= 100:
                forward(20)
            elif x > 200:
                left(2)
            else:
                right(2)
            
            cv2.putText(frame,"x:{}  y:{} ".format(int(x),int(y)), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
            cv2.putText(frame,"R:{} ".format(int(radius)), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
        else:
            stop()
    if GPIO.input(17) == 0:
        stop()
        print("ball")
        p.ChangeDutyCycle(12.5)
        time.sleep(0.5)
        p.ChangeDutyCycle(5.5)
        time.sleep(0.5)
        
    
    cv2.imshow("Frame", frame)
    # cv2.imshow("HSV", hsv)
    # cv2.imshow("mask",mask)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
   
vs.release()
cv2.destroyAllWindows()
p.stop()
GPIO.cleanup()