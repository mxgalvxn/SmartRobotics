import cv2
import numpy as np
import time

# Configurar la cámara y definir las resoluciones de la pantalla
cap = cv2.VideoCapture(0)
width, height = 640, 480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

# Crear una ventana para mostrar el video capturado y otra para mostrar la segmentación de la imagen
cv2.namedWindow('Captura de video')
cv2.namedWindow('Segmentación')

while True:
    # Capturar el fotograma de video actual de la cámara
    ret, frame = cap.read()
    
    # Aplicar la umbralización a la imagen capturada
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    
    # Mostrar el video capturado y el video segmentado en sus ventanas correspondientes
    cv2.imshow('Captura de video', frame)
    cv2.imshow('Segmentación', thresh)
    
    # Esperar un tiempo para que el usuario pueda ver el video antes de capturar el siguiente fotograma
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
# Liberar la cámara y destruir todas las ventanas abiertas
cap.release()
cv2.destroyAllWindows()
