#!/usr/bin/env python3

import cv2 as cv # OpenCV library
import numpy as np
import glob
# checkerboard contants

def make_shots():
    cap = cv.VideoCapture(0)

    while(True):
        ret, frame = cap.read()
        cv.imshow("video feed", frame)
        if cv.waitKey(1) & 0xFF == ord('y'):
            cv.imwrite("video_shot.png", frame)
            cv.destroyAllWindows()
            break

"""
chessboard_dim = (9, 6)
frame_size = (480, 640)

term_criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

obj_pts = np.zeros((chessboard_dim[0] * chessboard_dim[1], 3), np.float32)
#obj_pts = np.mgrid[0:chessboard_dim[0], 0:chessboard_dim[1].T.reshape(-1, 2)]
obj_pts *= 20

print(obj_pts)

obj_3d_pts = []
left_2d_pts = []
right_2d_pts = []

imgs_left = glob.glob("calib_samples/left/*")
imgs_right = glob.glob("calib_samples/right/*")




for left, right in zip(imgs_left, imgs_right):

    img_left = cv.imread(left)
    img_right = cv.imread(right)
    gr_img_left = cv.cvtColor(img_left, cv.COLOR_BGR2GRAY)
    gr_img_right = cv.cvtColor(img_right, cv.COLOR_BGR2GRAY)

    ret_left, cor_left = cv.findChessboardCorners(gr_img_left, chessboard_dim, None)
    ret_right, cor_right = cv.findChessboardCorners(gr_img_left, chessboard_dim, None)

    if ret_left and ret_right:
        obj_3d_pts.append(obj_pts)

        cor_left = cv.cornerSubPix(gr_img_left, cor_left, (11, 11), (-1, -1), term_criteria)
        left_2d_pts.append(cor_left)

        cor_right = cv.cornerSubPix(gr_img_right, cor_right, (11, 11), (-1, -1), term_criteria)
        right_2d_pts.append(cor_right)

        cv.drawChessboardCorners(img_left, chessboard_dim, cor_left, ret_left)
        cv.imshow("image left", img_left)
        cv.drawChessboardCorners(img_right, chessboard_dim, cor_right, ret_right)
        cv.imshow("image left", img_right)
        cv.waitKey(1000)

cv.destroyAllWindows()

ret_left, matrix_left, dist_left, rvecs_left, tvecs_left = cv.calibrateCamera(obj_3d_pts, left_2d_pts, frame_size, None, None)
height_left, width_left, chan_left = img_left.shape
new_matrix_left, roi_left = cv.getOptimalNewCameraMatrix(matrix_left, dist_left, (width_left, height_left), 1, (width_left, height_left))

ret_right, matrix_right, dist_right, rvecs_rightright, tvecs_right = cv.calibrateCamera(obj_3d_pts, right_2d_pts, frame_size, None, None)
height_right, width_right, chan_right = img_right.shape
new_matrix_right, roi_right = cv.getOptimalNewCameraMatrix(matrix_right, dist_right, (width_right, height_right), 1, (width_right, height_right))

flags = 0
flags |= cv.CALIB_FIX_INTRINSIC

term_criteria_stereo = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
ret_stereo, new_matrix_left, dist_left, new_matrix_right, dist_right, rot, trans, ess_matrix, fund_matrix = cv.stereoCalibrate(obj_3d_pts, left_2d_pts, right_2d_pts, new_matrix_left, dist_left, new_matrix_right, dist_right, gr_img_left.shape[::-1], term_criteria_stereo, flags)

rect_scale = 1.0

rect_left, rect_right, proj_matrix_left, proj_matrix_right, matrix_Q, roi_left, roi_right = cv.stereoRectify(new_matrix_left, dist_left, new_matrix_right, dist_right, gr_img_left.shape[::1], rot, trans, rect_scale, (0,0))#

stereo_map_left = cv.initUndistortRectifyMap(new_matrix_left, dist_left, rect_left, proj_matrix_left, gr_img_left.shape[::-1], cv.CV_16SC2)
stereo_map_right = cv.initUndistortRectifyMap(new_matrix_right, dist_right, rect_right, proj_matrix_right, gr_img_right.shape[::-1], cv.CV_16SC2)

cv_calib_file = cv.FileStorage("calib/stereo_map.xml", cv.FILE_STORAGE_WRITE)

cv_calib_file.write("stereoMapLeftX", stereo_map_left[0])
cv_calib_file.write("stereoMapLeftY", stereo_map_left[0])
cv_calib_file.write("stereoMapRightX", stereo_map_right[0])
cv_calib_file.write("stereoMapRightY", stereo_map_right[1])
"""
if __name__ == '__main__':
    make_shots()
