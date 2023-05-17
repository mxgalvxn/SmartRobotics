import cv2
import numpy as np
from sklearn.svm import SVC
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score

# Lista de nombres de archivos de imágenes en formato PNG
imagenes = ['c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road2.png','c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road3.png','c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road9.png','c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road152.png','c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/road153.png']

# Crear el detector y el descriptor BRIEF
detector = cv2.ORB_create()
descriptor = cv2.xfeatures2d.BriefDescriptorExtractor_create()

# Listas para almacenar los descriptores y etiquetas de todas las imágenes
descriptores_totales = []
etiquetas = []

# Procesar cada imagen por separado
for imagen_nombre in imagenes:
    # Leer la imagen en escala de grises
    imagen = cv2.imread(imagen_nombre, 0)
    
    # Encontrar los puntos clave y los descriptores
    puntos_clave = detector.detect(imagen, None)
    puntos_clave, descriptores = descriptor.compute(imagen, puntos_clave)
    
    # Agregar los descriptores a la lista total
    descriptores_totales.extend(descriptores)
    
    # Agregar etiquetas correspondientes a las imágenes
    etiquetas.extend([imagen_nombre] * len(descriptores))

# Convertir las listas en matrices numpy
descriptores_totales = np.array(descriptores_totales)
etiquetas = np.array(etiquetas)

# Dividir los datos en conjuntos de entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(descriptores_totales, etiquetas, test_size=0.2, random_state=42)

# Crear y entrenar el clasificador SVM
classifier = SVC()
classifier.fit(X_train, y_train)

# Predecir las etiquetas de prueba
y_pred = classifier.predict(X_test)

# Calcular la precisión
accuracy = accuracy_score(y_test, y_pred)
print("Precisión:", accuracy)
