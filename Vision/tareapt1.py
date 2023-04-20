import cv2
import numpy as np
import os   
from PIL import Image

# Cargando la imagen
imagen = Image.open("road152.png")

#imagen = Image.open('chango.jpeg')

# Mostrando la imagen en color
#imagen.show()
#cv2.imshow('Imagen original', np.array(imagen))

# Convertir la imagen a escala de grises
escala_grises = imagen.convert('L')

# Mostrando la imagen en escala de grises   
#escala_grises.show()

# Aplicar un umbral de 128 a la imagen
umbral = 128
imagen_binaria = escala_grises.point(lambda x: 0 if x < umbral else 255, '1')

# Mostrar la imagen binaria
#imagen_binaria.show()

# Separar los canales de color
canales = imagen.split()

# Aplicar un umbral de 128 al canal rojo
umbral = 180
r = canales[0].point(lambda x: 0 if x < umbral else 255, 'L')

# Recombinar los canales de color para formar la imagen binaria
imagen_binR = Image.merge('RGB', (r, canales[1], canales[2]))

# Mostrar la imagen binaria
#imagen_binR.show()

img = np.array(escala_grises)
# Aplicar ruido gaussiano
media = 0
desviacion = 30
ruido = np.zeros(img.shape, np.uint8)
cv2.randn(ruido, media, desviacion)
img_ruidosa = cv2.add(img, ruido)

# Aplicar filtro de media
img_filtrada_media = cv2.blur(img_ruidosa, (5,5))

# Aplicar filtro Gaussiano
img_filtrada_gaussiano = cv2.GaussianBlur(img_ruidosa, (5,5), 0)

# Mostrar las imágenes
cv2.imshow('Imagen en blanco y negro', img)
cv2.imshow('Imagen con ruido gaussiano', img_ruidosa)
cv2.imshow('Imagen filtrada con filtro de media', img_filtrada_media)
cv2.imshow('Imagen filtrada con filtro Gaussiano', img_filtrada_gaussiano)
cv2.waitKey(0)
cv2.destroyAllWindows()