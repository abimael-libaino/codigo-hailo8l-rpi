import cv2

# Defina o endereço RTSP
rtsp_url = 'rtsp://192.168.144.25:8554/main.264'

# Configurar OpenCV para usar GStreamer
gstreamer_pipeline = f'rtspsrc location={rtsp_url} latency=0 buffer-mode=auto ! rtph265depay ! h265parse ! decodebin ! videoconvert ! autovideosink'

# Captura do vídeo do endereço RTSP com GStreamer
cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Erro ao abrir o stream RTSP")
    exit()

# Ajustar o buffer do OpenCV
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Loop para ler e exibir o feed da câmera
while True:
    ret, frame = cap.read()
    if not ret:
        print("Falha ao capturar frame")
        break

    # Exibir o frame capturado
    cv2.imshow('RTSP Feed', frame)

    # Pressione 'q' para sair
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
