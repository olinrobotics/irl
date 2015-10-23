#!/usr/bin/python
import roslib; roslib.load_manifest('edwin')
import rospy
from std_msgs.msg import String


def publisher():
    rospy.init_node("edwin_brain",anonymous = True)

    pub = rospy.Publisher('face_location', String, queue_size = 10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        faces = face_cascade.detectMultiScale(frame, scaleFactor=1.2, minSize=(20,20))
        if len(faces) > 0:
            faceObjects = []
            for (x, y, w, h) in faces:
                faceObjects.append(Face(x,y,w,h))

            for (x,y,w,h) in faces:
                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255))

            face = faceObjects[largestFaceIndex(faceObjects)]
            print "largest face at:"+str(face.midx)+", "+str(face.midy)
            pub.publish(str(face.midx)+","+str(face.midy))
        else:
            print "no face detected: returning (x,y) = (300,250)"
            pub.publish(str(300)+","+str(250))
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    publisher()
