import cv2
import numpy as np

# Lee la imagen en escala de grises
image = cv2.imread('control\modeloVision\path.jpeg',0)

# Binariza la imagen utilizando un umbral
_, threshold = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

# Calcula los momentos de la imagen binarizada
moments = cv2.moments(threshold)

# Calcula el centroide de la imagen
cx = int(moments['m10'] / moments['m00'])
cy = int(moments['m01'] / moments['m00'])

# Muestra el centroide en la imagen
cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)

# Muestra la imagen con el centroide
cv2.imshow('Image with Centroid', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
