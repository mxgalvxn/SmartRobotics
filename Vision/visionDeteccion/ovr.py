import cv2
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC

# Lista de nombres de archivos de imágenes en formato PNG
imagenes = ['c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road2.png', 'c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road3.png', 'c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road9.png', 'c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road152.png', 'c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road153.png']

# Crear el detector y el descriptor BRIEF
detector = cv2.ORB_create()
descriptor = cv2.xfeatures2d.BriefDescriptorExtractor_create()

# Listas para almacenar los descriptores y etiquetas de todas las imágenes
descriptores_totales = []
etiquetas = []

# Procesar cada imagen por separado
for etiqueta, imagen_nombre in enumerate(imagenes):
    # Leer la imagen en escala de grises
    imagen = cv2.imread(imagen_nombre, 0)
    
    # Encontrar los puntos clave y los descriptores
    puntos_clave = detector.detect(imagen, None)
    puntos_clave, descriptores = descriptor.compute(imagen, puntos_clave)
    
    # Agregar los descriptores a la lista total
    descriptores_totales.extend(descriptores)
    
    # Agregar etiquetas correspondientes a las imágenes
    etiquetas.extend([etiqueta] * len(descriptores))

    # Dibujar los puntos clave en la imagen
    imagen_con_puntos = cv2.drawKeypoints(imagen, puntos_clave, None, color=(0, 255, 0), flags=0)
    
    # Mostrar la imagen con los puntos clave
    cv2.imshow(imagen_nombre, imagen_con_puntos)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Convertir las listas en matrices numpy
descriptores_totales = np.array(descriptores_totales)
etiquetas = np.array(etiquetas)

# Dividir el conjunto de datos en entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(descriptores_totales, etiquetas, test_size=0.2, random_state=42)

# Crear un clasificador SVM para OvR
clasificador = SVC(kernel='linear', decision_function_shape='ovr')

# Entrenar el clasificador
clasificador.fit(X_train, y_train)

# Evaluar el clasificador en el conjunto de prueba
accuracy = clasificador.score(X_test, y_test)
print("Exactitud del clasificador:", accuracy)
