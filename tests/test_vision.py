import numpy as np
import math
import vision
import unittest
from vision import find_target
import cv2
import csv

def test_black():
    res = vision.find_target(np.zeros(shape=(320, 240, 3), dtype=np.uint8))
    assert res[2] is 0

def test_sample_images():
    variables = ['x', 'nt']
    with open('sample_img/tests.csv', 'r') as csvfile:
        # filename, x, y, w, h
        testreader = csv.reader(csvfile, delimiter=',')
        for sample in testreader:
            image = cv2.imread('sample_img/' + sample[0])
            # Rescale if necessary
            height, width, channels = image.shape
            x, img, num_targets = find_target(image)
            assert num_targets == int(sample[2])
            if num_targets == 0:
                x = None
                continue
            if x != None and sample[1] != None:
                sample[1] = 2 * float(sample[1]) / width - 1
                assert math.isclose(x, sample[1], abs_tol=0.1)
