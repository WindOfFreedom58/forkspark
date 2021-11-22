import cv2
import numpy as np

def apply_mask(image, color):

    hsv_frame = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

    mask = cv2.inRange(
        hsv_frame, COLOR_VALUES[color][0], COLOR_VALUES[color][1])

    if color == 'red' or color == 'green':
        mask += cv2.inRange(hsv_frame,
                            COLOR_VALUES[color][2], COLOR_VALUES[color][3])

    colored_mask = cv2.bitwise_and(image, image, mask=mask)

    return mask, colored_mask

def find_max_brightness(red_cmask, green_cmask, yellow_cmask):

    h, s, v_red = cv2.split(red_cmask)
    h, s, v_green = cv2.split(green_cmask)
    h, s, v_yellow = cv2.split(yellow_cmask)

    redSum = v_red.sum()
    greenSum = v_green.sum()
    yellowSum = v_yellow.sum()

    sums = [redSum, greenSum, yellowSum]
    maxSumIndex = 0
    maxSum = redSum

    for i, sum in enumerate(sums[1:]):
        if sum > maxSum:
            maxSum = sum
            maxSumIndex = i+1

    if maxSum < BRIGHTNESS_THRESHOLD:
        maxSumIndex = -1

    switcher = {
       -1: 'isik yok',
        0: 'kirmizi isik',
        1: 'yesil isik',
        2: 'yesil isik'
    }

    return switcher[maxSumIndex]

def determine_light_color(img, xmin, xmax, ymin, ymax):

    cropped_img = img[ymin:ymax, xmin:xmax]

    big_img = cv2.resize(cropped_img, (640, 640))
    cv2.imshow("w3", cv2.cvtColor(big_img, cv2.COLOR_RGB2BGR))

    red_mask, red_cmask = apply_mask(cropped_img, 'red')
    green_mask, green_cmask = apply_mask(cropped_img, 'green')
    yellow_mask, yellow_cmask = apply_mask(cropped_img, 'yellow')


    return find_max_brightness(red_cmask, green_cmask, yellow_cmask)

def draw(img, xmin, ymin, xmax, ymax, name):
    img = cv2.line(img, (xmin, ymin), (xmin, ymax), color=(255, 0, 0), thickness=3)
    img = cv2.line(img, (xmin, ymin), (xmax, ymin), color=(255, 0, 0), thickness=3)
    img = cv2.line(img, (xmin, ymax), (xmax, ymax), color=(255, 0, 0), thickness=3)
    img = cv2.line(img, (xmax, ymin), (xmax, ymax), color=(255, 0, 0), thickness=3)

    img = cv2.putText(img, name, (xmin, ymax), cv2.FONT_HERSHEY_SIMPLEX, 1,
                      (0, 0, 255), thickness=2)

    return img


BRIGHTNESS_THRESHOLD = 700

LOW_RED_1 = np.array([0, 50, 70])
HIGH_RED_1 = np.array([20, 255, 255])

LOW_RED_2 = np.array([165, 140, 70])
HIGH_RED_2 = np.array([180, 255, 255])

LOW_GREEN_1 = np.array([32, 50, 70])
HIGH_GREEN_1 = np.array([72, 255, 255])

LOW_GREEN_2 = np.array([72, 100, 70])
HIGH_GREEN_2 = np.array([83, 255, 255])

LOW_YELLOW = np.array([28,  65, 25])
HIGH_YELLOW = np.array([32, 255, 255])

COLOR_VALUES = {"red": [LOW_RED_1, HIGH_RED_1, LOW_RED_2, HIGH_RED_2],
                "green": [LOW_GREEN_1, HIGH_GREEN_1, LOW_GREEN_2, HIGH_GREEN_2],
                "yellow": [LOW_YELLOW, HIGH_YELLOW]}


def light_color_from_position(bbox):
    """
    :param bbox = [xmin ymin xmax ymax img]

    """
    xmin = bbox[0]
    ymin = bbox[1]
    xmax = bbox[2]
    ymax = bbox[3]
    img = bbox[4]

    cropped_img = img[ymin:ymax, xmin:xmax]

    hsl_cropped_img = cv2.cvtColor(cropped_img, cv2.COLOR_RGB2HLS)
    lightness = hsl_cropped_img[:,:,1]

    max_val = np.amax(lightness)
    max_pos = np.where(max_val)
