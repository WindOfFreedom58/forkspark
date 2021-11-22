import os.path as ops
import numpy as np
import torch
import cv2
import os
from sklearn.cluster import MeanShift

import time

from lane_detector.lane_detector.k_means_pytorch import KMeans


def gray_to_rgb_emb(gray_img):
    """
    :param gray_img: torch tensor 256 x 512
    :return: numpy array 256 x 512
    """
    H, W = gray_img.shape
    element = torch.unique(gray_img).numpy()
    rbg_emb = np.zeros((H, W, 3))
    color = [[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 215, 0], [0, 255, 255]]
    for i in range(len(element)):
        rbg_emb[gray_img == element[i]] = color[i]
    return rbg_emb / 255


def process_instance_embedding(instance_embedding, binary_img, distance=1, lane_num=5):
    start = time.time()

    embedding = instance_embedding[0].detach().numpy().transpose(1, 2, 0)

    cluster_result = np.zeros(binary_img.shape, dtype=np.int32)
    cluster_list = embedding[binary_img > 0]
    print(cluster_list.shape)
    print(cluster_list[0, 0])

    mean_shift = MeanShift(bandwidth=distance, bin_seeding=True, n_jobs=-1)
    mean_shift.fit(cluster_list)
    labels = mean_shift.labels_

    print("MeanShift time: %.4f" % (time.time() - start))
    start = time.time()

    tensor_cl = torch.tensor(cluster_list)
    cl, c = KMeans(tensor_cl, 4, verbose=False)
    print(f"labels: {labels.shape}")
    print(f"cl: {cl.shape}")
    print(f"c: {c.shape}")
    print(f"NEW-k: {time.time() - start}")

    cluster_result[binary_img > 0] = labels + 1
    cluster_result[cluster_result > lane_num] = 0
    for idx in np.unique(cluster_result):
        if len(cluster_result[cluster_result == idx]) < 15:
            cluster_result[cluster_result == idx] = 0

    H, W = binary_img.shape
    rbg_emb = np.zeros((H, W, 3))
    color = [[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 215, 0], [0, 255, 255]]
    element = np.unique(cluster_result)
    for i in range(len(element)):
        rbg_emb[cluster_result == element[i]] = color[i]

    return rbg_emb / 255, cluster_result


def video_to_clips(video_name):
    test_video_dir = ops.split(video_name)[0]
    outimg_dir = ops.join(test_video_dir, 'clips')
    if ops.exists(outimg_dir):
        print('Data already exist in {}'.format(outimg_dir))
        return
    if not ops.exists(outimg_dir):
        os.makedirs(outimg_dir)
    video_cap = cv2.VideoCapture(video_name)
    frame_count = 0
    all_frames = []

    while (True):
        ret, frame = video_cap.read()
        if ret is False:
            break
        all_frames.append(frame)
        frame_count = frame_count + 1

    for i, frame in enumerate(all_frames):
        out_frame_name = '{:s}.png'.format('{:d}'.format(i + 1).zfill(6))
        out_frame_path = ops.join(outimg_dir, out_frame_name)
        cv2.imwrite(out_frame_path, frame)
    print('finish process and save in {}'.format(outimg_dir))


def process_instance_embedding_cuda(instance_embedding, binary_img, distance=1, lane_num=5, device=torch.device("cuda")):
    embedding = instance_embedding[0]
    embedding = embedding.permute(1, 2, 0)

    cluster_result = torch.zeros_like(binary_img, dtype=torch.int64, device=device)
    cluster_list = embedding[binary_img > 0]

    labels, centers = KMeans(cluster_list, lane_num, verbose=False)
    print(f"labels: {labels.shape}")
    print(f"cl: {labels.shape}")
    print(f"c: {centers.shape}")

    cluster_result[binary_img > 0] = labels + 1
    cluster_result[cluster_result > lane_num] = 0

    cluster_unique = torch.unique(cluster_result)
    for idx in cluster_unique:
        if len(cluster_result[cluster_result == idx]) < 15:
            cluster_result[cluster_result == idx] = 0

    H, W = binary_img.shape
    rbg_emb = torch.zeros(H, W, 3, dtype=torch.int64, device=device)
    color = torch.tensor([[0, 0, 0], [255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 215, 0], [0, 255, 255]]).cuda()
    element = cluster_unique
    for i in range(len(element)):
        rbg_emb[cluster_result == element[i]] = color[i]
    rbg_emb = rbg_emb.cpu().numpy()
    cluster_result = cluster_result.cpu().numpy()
    return rbg_emb / 255, cluster_result
