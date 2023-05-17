import cv2
import numpy as np
from sklearn.svm import SVC
from sklearn.multiclass import OneVsRestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score

# Leer las imágenes de entrenamiento y las etiquetas
imagenes = ['1.jpg', '2.jpg']
etiquetas = ['claseA', 'claseB']

# Crear una lista para almacenar las características y las etiquetas
caracteristicas = []
etiquetas_bin = []

# Extraer las características de las imágenes
for imagen in imagenes:
    # Leer la imagen utilizando OpenCV
    img = cv2.imread(imagen)
    
    # Extraer las características de la imagen (por ejemplo, usando el histograma de color)
    caracteristica = extract_features(img)
    
    # Agregar la característica a la lista de características
    caracteristicas.append(caracteristica)

# Binarizar las etiquetas
for etiqueta in etiquetas:
    if etiqueta == 'claseA':
        etiquetas_bin.append(1)
    else:
        etiquetas_bin.append(0)

# Convertir las listas en matrices numpy
caracteristicas = np.array(caracteristicas)
etiquetas_bin = np.array(etiquetas_bin)

# Dividir los datos en conjuntos de entrenamiento y prueba
X_train, X_test, y_train, y_test = train_test_split(caracteristicas, etiquetas_bin, test_size=0.2, random_state=42)

# Crear el clasificador OvR utilizando SVM como clasificador base
classifier = OneVsRestClassifier(SVC())

# Entrenar el clasificador
classifier.fit(X_train, y_train)

# Predecir las etiquetas de prueba
y_pred = classifier.predict(X_test)

# Calcular la precisión
accuracy = accuracy_score(y_test, y_pred)
print("Precisión:", accuracy)
