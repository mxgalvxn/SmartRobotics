import cv2
import numpy as np
import os   
from PIL import Image


# Función para resaltar bordes en una imagen en escala de grises
def resaltar_bordes(img_gris, color_bordes):
    # Aplicar un filtro de Canny para detectar los bordes
    bordes = cv2.Canny(img_gris, 100, 200)
    # Crear una imagen en color con el mismo tamaño que la imagen original
    img_color = cv2.cvtColor(img_gris, cv2.COLOR_GRAY2BGR)
    # Superponer los bordes detectados en la imagen en color
    img_color[bordes != 0] = color_bordes
    return img_color

# Función para detectar líneas, círculos y polígonos en una imagen en escala de grises
def detectar_figuras(img_gris):
    # Aplicar un filtro de Canny para detectar los bordes
    bordes = cv2.Canny(img_gris, 100, 200)
    # Aplicar un filtro de Hough para detectar líneas
    lineas = cv2.HoughLinesP(bordes, 1, cv2.cv2.PI/180, 100, minLineLength=100, maxLineGap=10)
    # Dibujar las líneas detectadas en una imagen en blanco
    lineas_img = np.zeros(img_gris.shape, np.uint8)
    for linea in lineas:
        x1, y1, x2, y2 = linea[0]
        cv2.line(lineas_img, (x1, y1), (x2, y2), 255, 2)
    # Aplicar un filtro de Hough para detectar círculos
    circulos = cv2.HoughCircles(img_gris, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)
    # Dibujar los círculos detectados en una imagen en blanco
    circulos_img = np.zeros(img_gris.shape, np.uint8)
    if circulos is not None:
        circulos = np.round(circulos[0, :]).astype("int")
        for (x, y, r) in circulos:
            cv2.circle(circulos_img, (x, y), r, 255, 2)
    # Aplicar un filtro de contornos para detectar polígonos
    _, contornos, _ = cv2.findContours(bordes, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # Dibujar los contornos detectados en una imagen en blanco
    contornos_img = np.zeros(img_gris.shape, np.uint8)
    cv2.drawContours(contornos_img, contornos, -1, 255, 2)
    # Combinar las imágenes de líneas, círculos y contornos en una sola imagen
    figuras_img = cv2.bitwise_or(lineas_img, circulos_img)
    figuras_img = cv2.bitwise_or(figuras_img, contornos_img)
    return figuras_img

# Configurar la captura de video desde la cámara
cap = cv2.VideoCapture(0)

# Ciclo principal para capturar y procesar cada frame del video
while True:
    # Leer el frame actual
    ret, frame = cap.read()

    # Convertir el frame a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Aplicar el detector de bordes Canny al frame en escala de grises
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)

    # Convertir los bordes a color rojo y superponerlos en la imagen original
    edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    edges_color[:,:,0] = 0
    edges_color[:,:,1] = 0
    edges_color[:,:,2] = 255
    frame_edges = cv2.addWeighted(frame, 0.8, edges_color, 1, 0)

    # Detectar y resaltar líneas, círculos y otros polígonos en el frame en escala de grises
    canny = cv2.Canny(gray, 50, 150)
    contours, hierarchy = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(gray, contours, -1, (0,255,0), 2)

    # Mostrar el video en pantalla
    cv2.imshow('Video original', frame)
    cv2.imshow('Video con bordes', frame_edges)
    cv2.imshow('Video con polígonos resaltados', gray)

    # Salir si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar la cámara y cerrar las ventanas
cap.release()
cv2.destroyAllWindows()
