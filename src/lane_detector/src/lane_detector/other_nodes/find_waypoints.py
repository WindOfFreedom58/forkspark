import numpy as np
import cv2
import os
import matplotlib.pyplot as plt


class Waypoints:

    def __init__(self, src, dst):
        self.src = src
        self.dst = dst
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)
        self.sample_threshold = 4000

    # return arrays: (AXIS 0, AXIS1)
    def get_lanes(self, cluster):
        lane_indexes = np.unique(cluster)[1:]
        lanes = list()
        for i in lane_indexes:
            indexes = np.where(cluster == i)
            lane_x = indexes[0]
            lane_y = indexes[1]
            lane = np.stack([lane_x, lane_y], axis=1)
            lanes.append(lane)
        return lanes

    # array is a 2d array
    def appropriate_values(self, array, min_val1, max_val1, min_val2, max_val2):
        x = array[:, 0]
        y = array[:, 1]

        ind1 = np.where((x >= min_val1) & (x <= max_val1))
        ind2 = np.where((y >= min_val2) & (y <= max_val2))

        common_indexes = np.intersect1d(ind1, ind2)

        new_arr = np.stack([x[common_indexes], y[common_indexes]], axis=1)
        return new_arr

    def if_valid(self, waypoints, threshold):
        pass

    def does_intersect(self,lane1, lane2):
        pass
    def inverse_2d_arr(self, arr):
        return np.stack([arr[:, 1], arr[:, 0]], axis=1)

    def draw_lanes(self, img, lanes, color):
        for lane in lanes:
            img[lane[:, 0], lane[:, 1]] = color

        return img

    # Input lane arrays: (Axis 0, Axis 1)
    def draw_lanes_cv2(self, img, lanes, color, radius=4, thickness=4):
        for lane in lanes:
            for x, y in zip(lane[:, 0], lane[:, 1]):
                cv2.circle(img, center=(int(y), int(x)), radius=radius, thickness=thickness, color=color)

        return img

    # Input coords: (Axis 0, Axis1)
    def draw_coords_cv2(self, img, coords, color, radius=4, thickness=4):
        for x, y in coords:
            cv2.circle(img, center=(int(y), int(x)), radius=radius, color=color, thickness=thickness)

        return img

    # Input arrays: (Axis 0, Axis1), return arrays: (Axis0, Axis1)
    def convert_birdseye_coords(self, M, lane, shape):
        y_lane = lane[:, 0]
        x_lane = lane[:, 1]

        x = (M[0][0] * x_lane + M[0][1] * y_lane + M[0][2]) / (M[2][0] * x_lane + M[2][1] * y_lane + M[2][2])
        y = (M[1][0] * x_lane + M[1][1] * y_lane + M[1][2]) / (M[2][0] * x_lane + M[2][1] * y_lane + M[2][2])

        x_indexes = np.where((x >= 0) & (x < shape[1]))[0]
        y_indexes = np.where((y >= 0) & (y < shape[0]))[0]
        common_indexes = np.intersect1d(x_indexes, y_indexes)

        return np.stack([y[common_indexes].astype("int32"), x[common_indexes].astype("int32")], axis=1)

    def convert_birdseye_img(self, image, M):
        return cv2.warpPerspective(image, M, (image.shape[1], image.shape[0]))

    # Takes input lanes as (Axis 0, Axis 1)
    # Returns middle coords as (Axis 0, Axis1)
    # Returns fitted lanes as (Axis 0, Axis 1)
    def find_middle(self, lanes):
        fits = list()

        x_axis = 0
        y_axis = 1

        # Fit a second degree poly to each lane
        for lane in lanes:
            if lane.shape[0] != 0:
                fits.append(np.polyfit(lane[:, x_axis], lane[:, y_axis], 2))

        # Make them polynomial objects
        polynomials = [np.poly1d(fit) for fit in fits]
        predictions = [poly(lanes[i][:, x_axis]) for i, poly in enumerate(polynomials)]

        predicted_lanes = list()

        # Stack the predictions of each poly
        for i, prediction in enumerate(predictions):
            arr = np.stack([lanes[i][:, x_axis], prediction], axis=1)
            predicted_lanes.append(arr.astype("int32"))

        # Smaller lane will be used to draw tangents
        n_samples = [lane.shape[0] for lane in lanes]
        tangent_poly = n_samples.index(min(n_samples))
        intersect_poly = n_samples.index(max(n_samples))

        # Compute derivatives and find normal lines
        derivative1 = np.polyder(polynomials[tangent_poly])
        derivative_vals = derivative1(predicted_lanes[tangent_poly][:, x_axis])
        a = -1 / derivative_vals
        b = (predicted_lanes[tangent_poly][:, x_axis] / derivative_vals) + predicted_lanes[tangent_poly][:, y_axis]
        coeffs = np.stack([a, b], axis=1)
        normal_lines = [np.poly1d(coeff) for coeff in coeffs]
        y_pred_normals = np.stack([line(predicted_lanes[intersect_poly][:, x_axis]) for line in normal_lines], axis=0)

        # Find the best intersection of each normal with the interserct_poly
        differences = np.abs(y_pred_normals - predicted_lanes[intersect_poly][:, y_axis])
        mins = np.amin(differences, axis=1)

        # Find middle points
        all_coords = list()
        for i in range(mins.shape[0]):
            index = np.where(differences[i] == mins[i])
            xs = predicted_lanes[intersect_poly][:, x_axis][index][0]
            ys = predicted_lanes[intersect_poly][:, y_axis][index][0]
            x1 = predicted_lanes[tangent_poly][:, x_axis][i]
            y1 = predicted_lanes[tangent_poly][:, y_axis][i]
            x = (xs + x1) / 2
            y = (ys + y1) / 2
            all_coords.append([x, y])

        return np.array(all_coords), predicted_lanes

    def find_middle_curves(self, lane_coeffs, vertical, horizontal=None):
        points = list()

        for coeff in lane_coeffs:
            a = coeff[0]
            b = coeff[1]
            c = coeff[2]

            x = -b / (2 * a)
            y = c - (b ** 2) / (4 * a)
            points.append([x, y])

        average = np.mean(np.array(points), axis=0).astype("int32")
        x0 = average[0]
        y0 = average[1]

        x1 = vertical

        n_samples = x1 - x0 + 1
        y_space = np.array([y0] * n_samples)
        x_space = np.linspace(x0, x1, abs(x1 - x0 + 1))
        try:

            points = np.stack([x_space, y_space], axis=1)

        except:
            points = None

        return points

    def find_middle_line(self, straight_line_coeffs, lines, diff_threshold=0.3, recursive=True):

        average_coeffs = np.mean(straight_line_coeffs, axis=0)
        av_slope = abs(average_coeffs[0])

        if abs(av_slope) <= diff_threshold or not recursive:
            mid_line = np.poly1d(average_coeffs)

            samples = [line.shape[0] for line in lines]
            dominant_index = samples.index(max(samples))

            lane_x = lines[dominant_index][:, 0]
            y = mid_line(lane_x)
            mid_coords = np.stack([lane_x, y], axis=1)

        else:
            slopes = [abs(coeff[0]) for coeff in straight_line_coeffs]
            min_slope = slopes.index(min(slopes))
            mid_coords = self.obtain_forward_direction(lines[min_slope], straight_line_coeffs[min_slope])


        return mid_coords

    def obtain_forward_direction(self, line, coeffs, D=800, recursive = False):
        line2, coeffs2 = self.translate_straight_line(line, coeffs, D)
        forward_coords = self.find_middle_line(np.stack([coeffs2, coeffs], axis=0), [line, line2], recursive)
        return forward_coords

    # Takes lane coords as (Axis 0, Axis 1)
    def translate_quadratic_lane(self, lane, coeff, L, mult=1):
        a = coeff[0]
        b = coeff[1]

        x_0 = lane[:, 0]
        y_0 = lane[:, 1]
        m = 2 * a * x_0 + b

        n = ((m ** 2) + 1) ** 0.5
        sign = 1 if a > 0 else -1
        sign *= mult

        x = x_0 + sign * m * L / n
        y = y_0 - sign * L / n

        return np.stack([x, y], axis=1)

    # Takes line coords as (Axis 0, Axis 1)
    def translate_straight_line(self, lane, coeff, L):
        m = coeff[0]
        b = coeff[1]
        sign = 1 if m < 0 else -1

        x_arr = lane[:, 0]

        coeff2 = np.array([-m, b + sign * L * (1 + m ** 2) ** 0.5])
        line2 = np.poly1d(coeff2)
        y_arr = line2(x_arr)
        lane2 = np.stack([x_arr, y_arr], axis=1)

        return lane2, coeff2

    def predict_1_lane(self, lane, threshold=0.4, D1=500, D2=800, mult=-1):
        lane_x = lane[:, 0]
        lane_y = lane[:, 1]

        # Find out whether that is a straight line or quadratic curve
        line_fit = np.polyfit(lane_x, lane_y, 1)
        line = np.poly1d(line_fit)
        line_preds = line(lane_x)
        line_mistakes = np.sum(np.abs(lane_y - line_preds)) / lane_y.shape[0]

        quad_fit = np.polyfit(lane_x, lane_y, 2)
        poly = np.poly1d(quad_fit)
        poly_preds = poly(lane_x)
        poly_mistakes = np.sum(np.abs(lane_y - poly_preds)) / lane_y.shape[0]

        difference = poly_mistakes - line_mistakes

        if abs(difference) <= threshold or difference > 0:
            waypoints = self.obtain_forward_direction(lane, line_fit, D2)

            return waypoints, None, None, np.stack([lane_x,line_preds],axis=1)

        else:
            waypoints = self.translate_quadratic_lane(lane, quad_fit, D1, mult=mult)

            if quad_fit[0] > 0:
                return None, waypoints, None,[np.stack([lane_x,poly_preds], axis=1)]

            else:
                return None, None, waypoints,[np.stack([lane_x,poly_preds], axis=1)]

    # Make predictions on 2 lanes.
    # Threshold: Threshold for deciding whether the lane is a straight line or a quadratic curve
    # D1: Translation distance of quadratic curves
    # D2: Translation distance of straight lines
    # vertical: Starting point of straight line in two diverging curve situation
    def predict_2_lanes(self, birdseye_lanes, threshold=0.4, D1=500, D2=800, vertical=1079):

        fits = list()
        predicted_lanes = list()
        line_count = 0
        quadratic_count = 0

        middle_coords = None
        right_coords = None
        left_coords = None

        for lane in birdseye_lanes:
            lane_x = lane[:, 0]
            lane_y = lane[:, 1]

            line_fit = np.polyfit(lane_x, lane_y, 1)
            line = np.poly1d(line_fit)
            line_preds = line(lane_x)
            line_mistakes = np.sum(np.abs(lane_y - line_preds)) / lane_y.shape[0]

            quad_fit = np.polyfit(lane_x, lane_y, 2)
            poly = np.poly1d(quad_fit)
            poly_preds = poly(lane_x)
            poly_mistakes = np.sum(np.abs(lane_y - poly_preds)) / lane_y.shape[0]

            difference = poly_mistakes - line_mistakes

            if abs(difference) <= threshold or difference > 0:
                fits.append(line_fit)
                predicted_lanes.append(np.stack([lane_x, line_preds], axis=1))
                line_count += 1
            else:
                fits.append(quad_fit)
                predicted_lanes.append(np.stack([lane_x, poly_preds], axis=1))
                quadratic_count += 1

        # One straight line and one polynomial.
        if line_count == quadratic_count == 1:
            for fit, lane in zip(fits, predicted_lanes):
                # Translate the straight line to obtain forward direction.
                if fit.shape[0] == 2:
                    line_coords, line_coeffs = self.translate_straight_line(lane, fit, D2)
                    middle_coords = self.find_middle_line(np.stack([fit, line_coeffs], axis=0), [line_coords, lane])


                # Translate the quadratic curve to obtain left or right turning direction.
                else:
                    turning_coords = self.translate_quadratic_lane(lane, fit, D1)

                    if fit[0] > 0:
                        right_coords = turning_coords

                    else:
                        left_coords = turning_coords

        # 2 straight lines. Just find the middle of them.
        elif line_count == 2:
            middle_coords = self.find_middle_line(fits, predicted_lanes)

        # 2 quadratic curves. Find whether they diverge from each other or they form a single road.
        else:
            sign = fits[0][0] * fits[1][0]

            # If their primary coefficients have the same sign, they form a single road.
            if sign > 0:
                net_fit = (fits[0] + fits[1]) / 2
                n_p = np.poly1d(net_fit)
                preds = n_p(predicted_lanes[0][:, 0])
                middle_coords = np.stack([predicted_lanes[0][:, 0], preds], axis=1)

            # If not, they are diverging from each other so we need to find three different roads: Left, right, straight.
            else:
                for fit, lane in zip(fits, predicted_lanes):
                    coords = self.translate_quadratic_lane(lane, fit, D1)

                    if fit[0] > 0:
                        right_coords = coords

                    else:
                        left_coords = coords

                middle_coords = self.find_middle_curves(fits, vertical)

        return middle_coords, right_coords, left_coords, predicted_lanes
