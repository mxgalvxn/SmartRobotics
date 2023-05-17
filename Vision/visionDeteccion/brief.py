import cv2
import numpy as np

def brief_descriptor(image, keypoints, patch_size=48, descriptor_size=256):
    # Definir los pares de puntos para BRIEF
    # Aquí se utilizan las coordenadas del punto como semilla para generar números aleatorios
    np.random.seed(0)
    pairs = np.random.randint(-patch_size // 2, patch_size // 2, size=(descriptor_size, 2))
    
    # Convertir la imagen a escala de grises
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Calcular los descriptores BRIEF
    descriptors = np.zeros((len(keypoints), descriptor_size), dtype=np.uint8)
    for i, keypoint in enumerate(keypoints):
        x, y = map(int, keypoint.pt)
        patch = gray_image[y - patch_size // 2:y + patch_size // 2, x - patch_size // 2:x + patch_size // 2]
        values = patch[pairs[:, 0] + patch_size // 2, pairs[:, 1] + patch_size // 2]
        descriptors[i] = (values > patch.mean()).astype(np.uint8)
    
    return descriptors

# Cargar una imagen
image = cv2.imread('c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/1.jpg')

# Detectar puntos clave utilizando el detector de esquinas FAST
fast = cv2.FastFeatureDetector_create()
keypoints = fast.detect(image, None)

# Calcular los descriptores BRIEF para los puntos clave detectados
descriptors = brief_descriptor(image, keypoints)

# Dibujar los puntos clave en la imagen y mostrar la imagen resultante
image_with_keypoints = cv2.drawKeypoints(image, keypoints, None, color=(0, 255, 0))
cv2.imshow('Image with keypoints', image_with_keypoints)
cv2.waitKey(0)

