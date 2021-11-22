import time

import cv2
import numpy as np
from matplotlib import pyplot as plt


class BinaryToFit:
    def __init__(self, show_results=False, camera_type='left', callback=None):
        # was the line detected in the last iteration?
        self.detected = False
        # x values of the last n fits of the line
        self.recent_xfit = [0, 0, 0]
        # whether oor not to draw results to cv2 (use for debugging)
        self.show_results = show_results
        # the most resent binary_image
        self.binary_image = None
        self.outlied = True
        self.camera_type = camera_type

        self.visualize_callback = callback

        # Images will be rotated according to the type of camera.
        # It can only be left oor right.
        if camera_type == 'left':
            self.image_rotation_matrix = cv2.ROTATE_90_COUNTERCLOCKWISE
        else:
            self.image_rotation_matrix = cv2.ROTATE_90_CLOCKWISE

    def getFitParams(self, binary_image):
        self.binary_image = cv2.rotate(binary_image, self.image_rotation_matrix)

        # If last image detected a line don't calculate the histogram again
        if self.detected:
            x, y = self.searchAroundPoly()
        else:
            x, y = self.applyHistorgram()
        # Fit new polynomials
        self.fitPoly(x, y, self.binary_image.shape)

        return self.recent_xfit

    def applyHistorgram(self):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(self.binary_image, axis=0)

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        x_base = np.argmax(histogram)

        # HYPERPARAMETERS
        # Choose the number of sliding windows
        nwindows = 10
        # Set the width of the windows +/- margin
        margin = 50
        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Set height of windows - based on nwindows above and image shape
        window_height = np.int32(self.binary_image.shape[0] // nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = self.binary_image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated later for each window in nwindows
        x_current = x_base

        # Create empty lists to receive left and right lane pixel indices
        lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = self.binary_image.shape[0] - (window + 1) * window_height
            win_y_high = self.binary_image.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            # Identify the nonzero pixels in x and y within the window #
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_inds) > minpix:
                x_current = np.int32(np.mean(nonzerox[good_inds]))

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            lane_inds = np.concatenate(lane_inds)
            self.detected = True
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass

        # Extract left and right line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        return y, x

    def searchAroundPoly(self):
        # HYPERPARAMETER
        # Choose the width of the margin around the previous polynomial to search
        # The quiz grader expects 100 here, but feel free to tune on your own!
        margin = 20

        # Grab activated pixels
        nonzero = self.binary_image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Set the area of search based on activated x-values
        # within the +/- margin of our polynomial function
        # Hint: consider the window areas for the similarly named variables
        # in the previous quiz, but change the windows to our new search area
        lane_inds = ((nonzerox < (self.recent_xfit[0] * (nonzeroy**2) + self.recent_xfit[1] * nonzeroy + self.recent_xfit[2] + margin))
                     & (nonzerox > (self.recent_xfit[0] * (nonzeroy**2) + self.recent_xfit[1] * nonzeroy + self.recent_xfit[2] - margin)))

        # Again, extract left and right line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        return y, x

    def fitPoly(self, x, y, binary_image_shape):
        try:
            # Fit a first order polynomial to each with np.polyfit()
            self.recent_xfit = np.polyfit(x, y, 2)
            '''if abs(self.recent_xfit[0] - fit[0]) < 0.3 or self.outlied:
                self.recent_xfit = fit
                self.outlied = False
            else:
                self.outlied = True'''

        except:
            self.detected = False

        # Visualization
        if self.show_results:
            tmp_img = self.binary_image*255
            draw_img = np.dstack((tmp_img, tmp_img, tmp_img))

            # Generate x and y values for plotting
            ploty = np.linspace(0, binary_image_shape[0] - 1, binary_image_shape[0], dtype=np.int32)
            try:
                plotx = self.recent_xfit[0] * (ploty**2) + self.recent_xfit[1] * ploty + self.recent_xfit[2]
            except TypeError:
                # Avoids an error if `left` and `right_fit` are still none or incorrect
                print('The function failed to fit a line!')
                self.detected = False
                plotx = ploty

            plotx = np.int32(plotx)
            cv2.polylines(draw_img, [np.asarray([plotx, ploty]).T], False, (0, 0, 255), thickness=3)
            cv2.circle(draw_img, (103, 500), radius=5, color=(255, 255, 0))
            cv2.imshow(self.camera_type, draw_img)
            cv2.waitKey(1)


if __name__ == '__main__':
    fitter = BinaryToFit(show_results=True, camera_type='left')

    for i in range(0, 1000):
        start = time.time()
        image = cv2.imread(f"development/predictions/camera{i}.png")
        image = image[:, :, 0] / 255
        fitter.getFitParams(image)
        print(f"FPS: {1/(time.time() - start)}")