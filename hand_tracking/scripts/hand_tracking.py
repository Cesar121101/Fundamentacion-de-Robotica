import cv2
import numpy as np

# Crear un objeto de captura de vídeo
cap = cv2.VideoCapture(0)

# Definir rango de color de la piel en el espacio de color HSV
skin_lower = np.array([0, 20, 70], dtype=np.uint8)
skin_upper = np.array([20, 255, 255], dtype=np.uint8)

while True:
    # Leer cada fotograma del vídeo
    ret, frame = cap.read()

    # Cambiar el espacio de color del fotograma de BGR a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Crear una máscara utilizando la técnica de detección de la piel
    skin_mask = cv2.inRange(hsv, skin_lower, skin_upper)

    # Aplicar operaciones de morfología para eliminar el ruido
    kernel = np.ones((3,3), np.uint8)
    skin_mask = cv2.erode(skin_mask, kernel, iterations=1)
    skin_mask = cv2.dilate(skin_mask, kernel, iterations=1)

    # Encontrar los contornos de la mano en la máscara de piel
    contours = cv2.findContours(skin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Dibujar un cuadro delimitador alrededor de la mano
    if len(contours) > 0:
        max_contour = max(contours, key=lambda x: cv2.contourArea(x, False))
        if max_contour.size >= 3:
            x,y,w,h = cv2.boundingRect(max_contour)
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)

    # Mostrar el fotograma en una ventana
    cv2.imshow('Hand Tracking', frame)

    # Esperar a que se presione la tecla 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los recursos
cap.release()
cv2.destroyAllWindows()
