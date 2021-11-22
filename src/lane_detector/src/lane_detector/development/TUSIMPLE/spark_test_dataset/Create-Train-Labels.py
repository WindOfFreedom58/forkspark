import glob
import sys
import json


# Export path is the path to the export file from scalabel
# Binary path is where the binary images will be stored
# gt path is where the instance images will be stored
def main(export_path, train_label_path, source_path):
    # Loading the data
    f = open(export_path, "r")
    train_label_file = open(train_label_path, "w")

    data = json.load(f)
    ts = [t / 500.0 for t in range(501)]  # Something that will be used in bezier curves
    # Iterating through label data for each image
    h_samples = [15 * i for i in range(70)]

    for src in data:

        n_lines = len(src["labels"])  # Number of lines
        # pass unlabeled frames
        if n_lines == 0:
            continue

        lanes_arr = []
        for lane_number, lane in enumerate(src["labels"]):
            x_arr = [-2 for _ in range(70)]
            types = lane["poly2d"][0]["types"]
            points = lane["poly2d"][0]["vertices"]
            start = None
            nodes = list()

            for index, point in enumerate(points):
                point = tuple([int(i) for i in point])
                p_type = types[index]

                if p_type == "L":
                    if start is None:
                        start = point
                        nodes.append(point)

                    else:
                        nodes.append(point)
                        # Create bezier curves and take the points on it
                        bezier_points = [(int(i), int(j)) for i, j in make_bezier(nodes)(ts)]
                        for center in bezier_points:
                            x = int(center[0])
                            y = int(center[1])
                            if y in h_samples:
                                x_arr[h_samples.index(y)] = x
                        start = point
                        nodes.clear()
                        nodes.append(start)
                else:
                    nodes.append(point)
            lanes_arr.append(x_arr)

        file_name = source_path
        file_name += src["name"].split("/")[-1]
        json_line = json.dumps({"h_samples": h_samples, "lanes": lanes_arr, "raw_file": file_name})
        train_label_file.write(json_line + "\n")
    f.close()
    train_label_file.close()



def make_bezier(xys):
    # xys should be a sequence of 2-tuples (Bezier control points)
    n = len(xys)
    combinations = pascal_row(n - 1)

    def bezier(ts):
        # This uses the generalized formula for bezier curves
        # http://en.wikipedia.org/wiki/B%C3%A9zier_curve#Generalization
        result = []
        for t in ts:
            tpowers = (t ** i for i in range(n))
            upowers = reversed([(1 - t) ** i for i in range(n)])
            coefs = [c * a * b for c, a, b in zip(combinations, tpowers, upowers)]
            result.append(
                tuple(sum([coef * p for coef, p in zip(coefs, ps)]) for ps in zip(*xys)))
        return result

    return bezier


def pascal_row(n, memo={}):
    # This returns the nth row of Pascal's Triangle
    if n in memo:
        return memo[n]
    result = [1]
    x, numerator = 1, n
    for denominator in range(1, n // 2 + 1):
        x *= numerator
        x /= denominator
        result.append(x)
        numerator -= 1
    if n & 1 == 0:
        # n is even
        result.extend(reversed(result[:-1]))
    else:
        result.extend(reversed(result))
    memo[n] = result
    return result


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("usage: [SCALABEL_OUTPUTS_PATH] [TRAIN_LABELS_PATH] [SOURCE_IMAGES_PATH]")
        sys.exit()
    for index, json_label_path in enumerate(glob.glob('{:s}/*.json'.format(sys.argv[1]))):
        main(json_label_path, sys.argv[2]+str(index)+".json", sys.argv[3])