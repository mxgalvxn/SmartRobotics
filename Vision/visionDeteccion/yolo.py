import cv2
import numpy as np

# Cargar los archivos de configuración y pesos del modelo YOLO
net = cv2.dnn.readNetFromDarknet('c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/yolov3.cfg', 'c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/yolov3.weights')

# Obtener los nombres de las clases de los objetos detectables
with open('c:/Users/Admin/Desktop/Semestre 6/visionDeteccion/coco.names', 'r') as f:
    classes = [line.strip() for line in f.readlines()]

# Configurar los parámetros de la detección
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]

# Cargar la imagen de entrada
image =  cv2.VideoCapture(0)

# Obtener la altura y anchura de la imagen
height, width, _ = image.shape

# Realizar la detección de objetos
blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
net.setInput(blob)
outs = net.forward(output_layers)

# Procesar las detecciones
class_ids = []
confidences = []
boxes = []
for out in outs:
    for detection in out:
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]
        if confidence > 0.5:
            center_x = int(detection[0] * width)
            center_y = int(detection[1] * height)
            w = int(detection[2] * width)
            h = int(detection[3] * height)
            x = int(center_x - w / 2)
            y = int(center_y - h / 2)
            class_ids.append(class_id)
            confidences.append(float(confidence))
            boxes.append([x, y, w, h])

# Aplicar la supresión no máxima para eliminar detecciones superpuestas
indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

# Mostrar los resultados de la detección en la imagen
font = cv2.FONT_HERSHEY_SIMPLEX
for i in indices:
    i = i[0]
    x, y, w, h = boxes[i]
    label = classes[class_ids[i]]
    confidence = confidences[i]
    color = (0, 255, 0)
    cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
    cv2.putText(image, f'{label}: {confidence:.2f}', (x, y - 10), font, 0.5, color, 2)

# Mostrar la imagen con las detecciones
cv2.imshow('YOLO Object Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
