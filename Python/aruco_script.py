# %%
import cv2
import numpy as np
import cv2.aruco as aruco
import os
from time import sleep

# Print video capture devices
#def print_video_devices():
#    for i in range(10):
#        cap = cv2.VideoCapture(i)
#        if cap.read()[0]:
#            print(f"Device {i} is a camera")
#        else:
#            print(f"Device {i} is not a camera")
#        cap.release()
#
#print_video_devices()


# %%
# Generate aruco markers
def generate_aruco_markers():
    # Dictionary of 250 markers
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    # Create an image from the dictionary
    for i in range(250):
        img = aruco.drawMarker(aruco_dict, i, 700)

        # Get current dir path
        path = os.getcwd()

        cv2.imwrite(path + "/markers/" + str(i) + ".jpg", img)


# Detect aruco markers
def detect_aruco_markers(aruco_dict, frame):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect aruco markers
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # Draw detected markers
    frame = aruco.drawDetectedMarkers(frame, corners, ids)

    return frame, corners, ids


# %%
# Take a picture each second

#cam = cv2.VideoCapture(1)
#for i in range(20):
#
#    ret, frame = cam.read()
#
#    print(f'Taking picture {i}')
#        
#    cv2.imwrite("C:\\Users\\Marcus\\Documents\\GitHub\\Zephyr-drone\\Python\\calibration\\" + str(i) + ".jpg", frame)
#    sleep(1)
    

# %%
# Calibrate camera
def calibrate_camera():
    # Get current dir path
    path = os.getcwd()

    # Termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 7, 3), np.float32)

    # Create an array with coordinates of the corners
    objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane

    # Get images from the calibration folder
    images = [f for f in os.listdir(path + "/callibImg") if f.endswith('.jpg')]

    for fname in images:
        # Read image
        img = cv2.imread(path + "/callibImg/" + fname)

        # Resize image (480x640)
        img = cv2.resize(img, (640, 480))

        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 7), None)

        # If found, add object points, image points (after refining them)
        if ret:
            objpoints.append(objp)

            # Refine the corners
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Add image points
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)

    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return ret, mtx, dist, rvecs, tvecs

# Undistort image
def undistort_image(img, mtx, dist):
    # Undistort image
    dst = cv2.undistort(img, mtx, dist, None, mtx)

    return dst

ret, mtx, dist, rvecs, tvecs = calibrate_camera()

#dst = cv2.imread("C:\\Users\\Marcus\\Documents\\GitHub\\Zephyr-drone\\Python\\calibration\\19.jpg")

#img = undistort_image(dst, mtx, dist)

# Set img side by side with undistorted image
#img = np.hstack((img, dst))
#cv2.imshow('img', img)
#cv2.waitKey(0)

# %%
# Estimate pose from aruco marker
def estimate_pose_from_aruco_marker(frame, mtx, dist):

    # Get marker length
    markerLength = 0.05
    mtx = np.array(mtx)

    # Get aruco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    # Get camera parameters
    parameters = aruco.DetectorParameters_create()

    # Detect aruco markers
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # Estimate pose
    print()
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, mtx, dist)

    # draw axis for the aruco markers
    if ids is not None:
        for i in range(len(ids)):
            frame = cv2.drawFrameAxes(frame, mtx, dist, rvec[i], tvec[i], 0.01)
            #frame = aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.01)

    # Get x, y, z
    if tvec is not None:
        x = tvec[0][0][0]
        y = tvec[0][0][1]
        z = tvec[0][0][2]
        return frame, rvec, tvec, x, y, z
    else:
        return frame, rvec, tvec, None, None, None

# %%
cam = cv2.VideoCapture(0)

# Show camera feed
while True:
    ret, frame = cam.read()
#    img,corners,ids = detect_aruco_markers(aruco.Dictionary_get(aruco.DICT_6X6_250), frame)
    img,rvec,tvec,x,y,z = estimate_pose_from_aruco_marker(frame, mtx, dist)
    # Add x y z coords to bottom left corner with a small font

    if(x is not None):
        x = round(x, 2)

    if(y is not None):
        y = round(y, 2)

    if(z is not None):
        z = round(z, 2)

    # Undistort image
    img2 = undistort_image(img, mtx, dist)

    cv2.putText(img, "x: {0}".format(x), (10, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(img, "y: {0}".format(y), (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(img, "z: {0}".format(z), (10, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # Stitch images together
    img = np.hstack((img, img2))
    cv2.imshow("Camera", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


