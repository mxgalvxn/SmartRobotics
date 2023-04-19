import cv2
import numpy as np

# Crea el objeto de captura de video
cap = cv2.VideoCapture(0)

# Define el tamaño del marco de video
frame_size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

# Crea las ventanas para mostrar los videos
cv2.namedWindow('Video', cv2.WINDOW_NORMAL)
cv2.namedWindow('Gray', cv2.WINDOW_NORMAL)
cv2.namedWindow('Edges', cv2.WINDOW_NORMAL)
cv2.namedWindow('Shapes', cv2.WINDOW_NORMAL)

# Define el umbral para la detección de bordes
threshold1 = 30
threshold2 = 100

# Crea el kernel para la detección de bordes
kernel = np.ones((5,5),np.uint8)

while True:
    # Captura el marco de video
    ret, frame = cap.read()

    # Convierte el marco a niveles de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detecta los bordes del marco
    edges = cv2.Canny(gray, threshold1, threshold2)

    # Aplica un filtro de cierre a los bordes para mejorar la detección
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    # Encuentra los contornos de los objetos en el marco
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Dibuja los contornos en el marco de video
    shapes = frame.copy()
    cv2.drawContours(shapes, contours, -1, (0, 0, 255), 2)

    # Muestra los videos en las ventanas correspondientes
    cv2.imshow('Video', frame)
    cv2.imshow('Gray', gray)
    cv2.imshow('Edges', cv2.bitwise_or(frame, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)))
    cv2.imshow('Shapes', shapes)

    # Sale del ciclo cuando se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera la memoria y cierra todas las ventanas
cap.release()
cv2.destroyAllWindows()
