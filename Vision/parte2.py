import cv2
import numpy as np

# Inicializar el objeto de captura de video de la cámara web
cap = cv2.VideoCapture(0)

while True:
    # Leer un fotograma de la cámara web
    ret, img = cap.read()

    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Agregar ruido gaussiano
    noise = np.zeros(gray.shape, np.float32)
    cv2.randn(noise, 0, 20)

    noisy_img = cv2.addWeighted(gray, 1, noise, 1, 0, dtype=cv2.CV_8U)

    # Aplicar umbralización
    _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
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
    cv2.imshow("Original Image", img)
    cv2.imshow("Noisy Image", noisy_img)
    cv2.imshow('Threshold', thresh)
    cv2.imshow('Noisy threshold' , thresh2)
    cv2.imshow('Opening', opening)
    #cv2.imshow('Markers', markers)

    # Esperar a que se presione la tecla 'q' para salir
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los recursos de la cámara y cerrar las ventanas de visualización
cap.release()
cv2.destroyAllWindows()



# Crear objeto VideoCapture
cap = cv2.VideoCapture(0)

while True:
    # Capturar frame de la cámara
    ret, frame = cap.read()

    # Convertir a escala de grises
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Aplicar la transformada de Fourier discreta
    dft = cv2.dft(np.float32(img), flags=cv2.DFT_COMPLEX_OUTPUT)

    # Desplazar el espectro al centro de la imagen
    dft_shift = np.fft.fftshift(dft)

    # Crear una máscara de alta frecuencia para filtrar la imagen
    rows, cols = img.shape
    crow, ccol = rows // 2, cols // 2
    mask = np.zeros((rows, cols, 2), np.float32)
    mask[crow - 30:crow + 30, ccol - 30:ccol + 30] = 1

    # Aplicar la máscara al espectro
    dft_shift_filtered = dft_shift * mask

    # Desplazar el espectro filtrado al centro de la imagen
    dft_filtered = np.fft.ifftshift(dft_shift_filtered)

    # Aplicar la transformada inversa de Fourier
    img_filtered = cv2.idft(dft_filtered)
    img_filtered = cv2.magnitude(img_filtered[:, :, 0], img_filtered[:, :, 1])

    # Mostrar la imagen filtrada
    cv2.imshow('Imagen filtrada', img_filtered)

    # Esperar por tecla 'q' para salir
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar objeto VideoCapture y destruir ventanas
cap.release()
cv2.destroyAllWindows()

# Inicializar la cámara web
cap = cv2.VideoCapture(0)

import cv2
import numpy as np

# Inicializar la cámara web
cap = cv2.VideoCapture(0)

while True:
    # Capturar el cuadro de la cámara
    ret, frame = cap.read()

    # Convertir a escala de grises
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img2 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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

    # Calculamos el umbral óptimo utilizando el método de Otsu
    ret, thresh = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    ret, thresh2 = cv2.threshold(noisy_img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

    # Mostramos el resultado
    cv2.imshow('Imagen original', img2)
    cv2.imshow("Noisy Image", noisy_img)
    cv2.imshow('Imagen segmentada', thresh)
    cv2.imshow('Imagen segmentada con ruido', thresh2)

    # Esperar por tecla 'q' para salir
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar objeto VideoCapture y destruir ventanas
cap.release()
cv2.destroyAllWindows()