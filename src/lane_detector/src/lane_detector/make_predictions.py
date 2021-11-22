import os
import cv2
import torch
import sys

from model2 import Lanenet
import numpy as np
import ast

"""
Given an array of images, make predictions on them.
img_array: An array of images (image_number,height,width,channels).
model_object: return value of load_model function.
shape: The shape that the model is trained with.
returns:
Predicted embedding
"""


def predict_from_array(img_array, model_object, shape, device_name="gpu"):
    if len(img_array.shape) != 4 or np.all(np.isnan(img_array)):
        raise ValueError("Image array has either a bad shape or contains null images!")

    only_predictions = list()

    for image in img_array:
        gt_image = np.copy(image)
        org_shape = image.shape
        gt_image = cv2.resize(gt_image, dsize=shape, interpolation=cv2.INTER_LINEAR)
        gt_image = gt_image / 127.5 - 1.0
        gt_image = torch.tensor(gt_image, dtype=torch.float)
        gt_image = np.transpose(gt_image, (2, 0, 1))

        if device_name == "gpu":
            binary_final_logits, instance_embedding = model_object(gt_image.unsqueeze(0).cuda())
            instance_embedding = instance_embedding[0].detach().cpu().numpy().transpose(1, 2, 0)
            binary_img = torch.argmax(binary_final_logits, dim=1).squeeze().cpu().numpy()
            binary_img[0:50, :] = 0

        else:
            binary_final_logits, instance_embedding = model_object(gt_image.unsqueeze(0).cpu())
            instance_embedding = instance_embedding[0].detach().cpu().numpy().transpose(1, 2, 0)
            binary_img = torch.argmax(binary_final_logits, dim=1).squeeze().cpu().numpy()
            binary_img[0:50, :] = 0


        instance_embedding[binary_img < 0.5] = 0
        rbg_emb = cv2.resize(instance_embedding, dsize=(org_shape[1], org_shape[0]), interpolation=cv2.INTER_LINEAR)
        only_predictions.append(rbg_emb)

    return np.array(only_predictions)


"""
Given the path to the json file, it draws the lanes to the raw_file.
If absolute path is true, it uses the exact raw_file path.
If raw_file contains a relative path, set absoulte_path= False and specify the root path. 
display_time: If none, it displays the images without any time limit and user can pass the images via pressing a button.
Otherwise it shows the image for display_time amound of mili seconds
"""


def show_predictions(path_to_json_file, absolute_path=True, root=None, display_time=None):
    if absolute_path is False and root is None:
        raise ValueError("You need to specify a root!")

    with open(path_to_json_file, "r") as f:
        dictionaries = f.readlines()

        for dictionary in dictionaries:
            d = ast.literal_eval(dictionary)

            if absolute_path:
                source_path = d["raw_file"]

            else:
                source_path = os.path.join(root, d["raw_file"])

            img = cv2.imread(source_path)

            n_lanes = len(d["lanes"])

            colors = list()

            for i in range(n_lanes):
                if i % 3 == 0:
                    color = (255 - 40 * i, 0, 0)

                elif i % 3 == 1:
                    color = (0, 255 - 40 * i, 0)

                else:
                    color = (0, 0, 255 - 40 * i)

                colors.append(color)

            for index, lane in enumerate(d["lanes"]):
                for x, y in zip(lane, d["h_samples"]):
                    if x > 0:
                        cv2.circle(img, center=(x, y), radius=2, thickness=2, color=colors[index])

            cv2.imshow("window", img)
            cv2.waitKey([0 if display_time is None else display_time][0])
            cv2.destroyAllWindows()


"""
Load the model to make real time predictions.
model_path: Path to the .model file
returns:
LaneNet_model: A model object
device: PyTorch device for evaluating. cpu or gpu are the options
"""


def load_model(model_path, device_name="gpu"):
    device = torch.device('cuda')
    LaneNet_model = Lanenet(2, 4)
    LaneNet_model.load_state_dict(torch.load(model_path))
    LaneNet_model.to(device)
    free_embeding_variables(LaneNet_model)

    return LaneNet_model, device

"""We don't use clustering branch of lanenet but the model file does contin weights for clustering.
 After the weights are inited this func frees clustering related variables to save memory from GPU"""
def free_embeding_variables(model):
    # Free embedding branch
    del model.upsample_embedding_4_0
    del model.regular_embedding_4_1
    del model.regular_embedding_4_2
    del model.upsample_embedding_5_0
    del model.regular_embedding_5_1
    del model.embedding_transposed_conv
    # Free stage 3 embedding
    del model.regular_embedding_3_0
    del model.dilated_embedding_3_1
    del model.asymmetric_embedding_3_2
    del model.dilated_embedding_3_3
    del model.regular_embedding_3_4
    del model.dilated_embedding_3_5
    del model.asymmetric_bembedding_3_6
    del model.dilated_embedding_3_7
"""
Returns every coordinate of each lane.
clusters: Third output of predict_from_array
returns:
For each image in clusters, it returns a list of lanes.
"""


def get_lane_coordinates(clusters):
    coordinates_in_images = list()
    for cluster in clusters:
        lanes = list()
        elements = np.unique(cluster)[1:]
        for line_index in elements:
            indexes = np.where(cluster == line_index)
            coordinates = list(zip(indexes[0], indexes[1]))
            lanes.append(coordinates)

        coordinates_in_images.append(lanes)

    return coordinates_in_images