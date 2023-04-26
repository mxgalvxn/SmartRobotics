import cv2
import numpy as np

# Cargar y procesar cada una de las imágenes
for i in range(1, 11):
    # Cargar la imagen en escala de grises
    img = cv2.imread(f'road{i}.png', 0)

    # Agregar ruido gaussiano
    noise = np.zeros(img.shape, np.float32)
    cv2.randn(noise, 0, 20)

    noisy_img = cv2.addWeighted(img, 1, noise, 1, 0, dtype=cv2.CV_8U)

    # Aplicar umbralización
    _, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
    _, thresh2 = cv2.threshold(noisy_img, 127, 255, cv2.THRESH_BINARY_INV)

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
    markers = cv2.watershed(cv2.cvtColor(noisy_img, cv2.COLOR_GRAY2BGR), markers)

    # Dibujar contornos en la imagen original
    img[markers == -1] = 255

    # Mostrar resultados
    cv2.imshow(f"Original Image {i}", img)
    cv2.imshow(f"Noisy Image {i}", noisy_img)
    cv2.imshow(f'Threshold {i}', thresh)
    cv2.imshow(f'Noisy threshold {i}' , thresh2)
    cv2.imshow(f'Opening {i}', opening)
    #cv2.imshow('Markers', markers)
    cv2.waitKey(0)

    # Calculamos el umbral óptimo utilizando el método de Otsu
    ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    ret, thresh2 = cv2.threshold(noisy_img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    # Mostramos el resultado
    cv2.imshow(f'Imagen original {i}', img)
    cv2.imshow(f"Noisy Image {i}", noisy_img)

