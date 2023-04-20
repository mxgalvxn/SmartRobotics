import cv2
import numpy as np

# Cargar las imágenes
img1 = cv2.imread('edificio1.jpg', 0)
blur1 = cv2.GaussianBlur(img1, (5, 5), 0)
edges1 = cv2.Canny(blur1, 100, 200)
cv2.imshow('Bordes Canny 1', edges1)


img2 = cv2.imread('edificio2.jpg', 0)
blur2 = cv2.GaussianBlur(img2, (5, 5), 0)
edges2 = cv2.Canny(blur2, 100, 200)
cv2.imshow('Bordes Canny 2', edges2)


img3 = cv2.imread('edificio3.jpg', 0)
blur3 = cv2.GaussianBlur(img3, (5, 5), 0)
edges3 = cv2.Canny(blur3, 100, 200)
cv2.imshow('Bordes Canny 3', edges3)


img4 = cv2.imread('c:/Users/Admin/Desktop/Semestre 6/GitHub/SmartRobotics-main/SmartRobotics-main/Vision/edificio4.jpg', 0)
blur4 = cv2.GaussianBlur(img4, (5, 5), 0)
edges4 = cv2.Canny(blur4, 100, 200)
cv2.imshow('Bordes Canny 4', edges4)


img5 = cv2.imread('edificio5.jpg', 0)
blur5 = cv2.GaussianBlur(img5, (5, 5), 0)
edges5 = cv2.Canny(blur5, 100, 200)
cv2.imshow('Bordes Canny 5', edges5)
cv2.waitKey(0)

# Aplicar el proceso a cada imagen
'''for img in [img1, img2, img3, img4, img5]:
    # Suavizado de la imagen con un filtro Gaussiano
    blur = cv2.GaussianBlur(img, (5, 5), 0)

    # Detección de bordes con el algoritmo de Canny
    edges = cv2.Canny(blur, 100, 200)

    # Mostrar la imagen de bordes
    cv2.imshow('Bordes Canny', edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()'''

def hysteresis_thresholding(image, low_threshold, high_threshold):
    # Convertir la imagen a escala de grises si no lo está
    if len(image.shape) > 2:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Aplicar un filtro de mediana para reducir el ruido
    image = cv2.medianBlur(image, 5)

    # Aplicar el umbral alto y bajo
    image_low = cv2.threshold(image, low_threshold, 255, cv2.THRESH_BINARY)[1]
    image_high = cv2.threshold(image, high_threshold, 255, cv2.THRESH_BINARY)[1]

    # Crear una imagen en blanco para guardar los bordes finales
    edges = np.zeros_like(image)

    # Definir los píxeles que están entre los umbrales como bordes potenciales
    edges[(image >= low_threshold) & (image <= high_threshold)] = 255

    # Utilizar la función cv2.findContours para encontrar los contornos de los bordes potenciales
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filtrar los contornos por área y agregar los bordes finales a la imagen de salida
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 50:
            cv2.drawContours(image, [contour], 0, 255, -1)

    return image

# Lista de nombres de archivos de imagen
image_files = ["edificio1.jpg","edificio2.jpg","edificio3.jpg","edificio4.jpg","edificio5.jpg"]

# Umbral bajo y alto
low_threshold = 50
high_threshold = 150

# Procesar cada imagen
for image_file in image_files:
    # Leer la imagen
    image = cv2.imread(image_file)

    # Aplicar el algoritmo de Hysteresis Thresholding
    edges = hysteresis_thresholding(image, low_threshold, high_threshold)

    # Mostrar la imagen de salida
    cv2.imshow("Edges", edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#Operaciones morfológicas

# Cargando la imagen
img = cv2.imread('edificio1.jpg', 0)

# Definiendo el kernel (estructura) para la operación de erosión
kernel = np.ones((5,5), np.uint8)

# Aplicando la operación de erosión
erosion = cv2.erode(img, kernel, iterations=1)

# Mostrando la imagen original y la imagen erosionada
cv2.imshow('Imagen original', img)
cv2.imshow('Imagen erosionada', erosion)

# Esperando a que se presione una tecla para cerrar las ventanas
cv2.waitKey(0)
cv2.destroyAllWindows()

#Operación de dilatación
# Cargando la imagen
img = cv2.imread('edificio2.jpg', 0)

# Definiendo el kernel (estructura) para la operación de dilatación
kernel = np.ones((5,5), np.uint8)

# Aplicando la operación de dilatación
dilatacion = cv2.dilate(img, kernel, iterations=1)

# Mostrando la imagen original y la imagen dilatada
cv2.imshow('Imagen original', img)
cv2.imshow('Imagen dilatada', dilatacion)

# Esperando a que se presione una tecla para cerrar las ventanas
cv2.waitKey(0)
cv2.destroyAllWindows()

#Operación de apertura
# Cargando la imagen
img = cv2.imread('edificio3.jpg', 0)

# Definiendo el kernel (estructura) para la operación de apertura
kernel = np.ones((5,5), np.uint8)

# Aplicando la operación de apertura
opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

# Mostrando la imagen original y la imagen con apertura
cv2.imshow('Imagen original', img)
cv2.imshow('Imagen con apertura', opening)

# Esperando a que se presione una tecla para cerrar las ventanas
cv2.waitKey(0)
cv2.destroyAllWindows()

#Operación de cerradura 
# Cargando la imagen
img = cv2.imread('edificio4.jpg', 0)

# Definiendo el kernel (estructura) para la operación de cerradura
kernel = np.ones((5,5), np.uint8)

# Aplicando la operación de cerradura
closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

# Mostrando la imagen original y la imagen con cerradura
cv2.imshow('Imagen original', img)
cv2.imshow('Imagen con cerradura', closing)

# Esperando a que se presione una tecla para cerrar las ventanas
cv2.waitKey(0)
cv2.destroyAllWindows()