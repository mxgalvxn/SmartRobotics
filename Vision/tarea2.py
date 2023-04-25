import cv2
import numpy as np

# Cargar la imagen
img = cv2.imread("road152.png", 0)

# Agregar ruido gaussiano
noise = np.zeros(img.shape, np.int8)
cv2.randn(noise, 0, 20)

noisy_img = cv2.add(img, noise)

# Aplicar umbralización
ret, thresh = cv2.threshold(noisy_img, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

# Eliminar ruido
kernel = np.ones((3, 3), np.uint8)
opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

# Encontrar marcadores
sure_bg = cv2.dilate(opening, kernel, iterations=3)
dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
ret, sure_fg = cv2.threshold(dist_transform, 0.7*dist_transform.max(), 255, 0)
sure_fg = np.uint8(sure_fg)
unknown = cv2.subtract(sure_bg, sure_fg)

ret, markers = cv2.connectedComponents(sure_fg)

# Aplicar Watershed
markers = markers + 1
markers[unknown==255] = 0
markers = cv2.watershed(cv2.cvtColor(np.uint8(noisy_img), cv2.COLOR_GRAY2BGR), markers)

# Dibujar contornos en la imagen original
img[markers == -1] = [255, 0, 0]

# Mostrar resultados
cv2.imshow("Original Image", img)
cv2.imshow("Noisy Image", noisy_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
