import cv2
import numpy as np

def load_gray(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    return img.astype(np.float32)

img1 = load_gray("map_fast_edited.pgm")  # original (shifted)
img2 = load_gray("map_fast_edited_2.pgm")  # cropped version

# ensure same size for correlation (pad smaller one if needed)
h = max(img1.shape[0], img2.shape[0])
w = max(img1.shape[1], img2.shape[1])

def pad(img, h, w):
    out = np.zeros((h, w), dtype=np.float32)
    out[:img.shape[0], :img.shape[1]] = img
    return out

img1 = pad(img1, h, w)
img2 = pad(img2, h, w)

# phase correlation
shift = cv2.phaseCorrelate(img1, img2)

(dx, dy) = shift[0]

print("Estimated shift (pixels):")
print("dx:", dx)
print("dy:", dy)
