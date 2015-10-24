import cv2

def main():
  cap = cv2.VideoCapture(0)
  while True:
    ret, frame = cap.read()
    cv2.imshow("camera", frame)
    c = cv2.waitKey(1)

if __name__ == '__main__':
  main()