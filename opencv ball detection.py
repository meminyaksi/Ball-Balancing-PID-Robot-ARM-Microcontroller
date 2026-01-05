import cv2
import numpy as np
import serial
import time;


ser = serial.Serial('/dev/cu.usbmodemSDA2D034E001', 115200, timeout=1)
time.sleep(2)  # Bağlantının oturması için 2 saniye bekle

# Video kaynağını seç (kameradan veya bir video dosyasından).
# Kameradan: 0, Video dosyasından: "video_path.mp4"
video_source = 1  # Kamerayı kullanmak için

# Turuncu rengin HSV renk aralığı
lower_orange = np.array([5, 100, 100])  # Turuncunun alt sınırı
upper_orange = np.array([25, 255, 255])  # Turuncunun üst sınırı

# Video yakalama nesnesini başlat
cap = cv2.VideoCapture(video_source)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Video akışı sona erdi veya kamera açılamadı.")
        break

    # Görüntüyü yeniden boyutlandır (isteğe bağlı)
    frame = cv2.resize(frame, (640, 480))

    # Görüntüyü HSV renk uzayına çevir
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Turuncu renk için maske oluştur
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Gürültüyü temizlemek için maske üzerinde morfolojik işlemler
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Turuncu alanı görüntüde tespit etmek
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    def send_coordinates(x, y):
        coord = f"{x},{y}\n"  # X ve Y koordinatlarını string olarak hazırla
        ser.write(coord.encode())  # Byte formatında Arduino'ya gönder
        print(f"Gönderilen: {coord}")

    for contour in contours:
        # Kontur alanını filtrele (çok küçük alanları hariç tut)
        if cv2.contourArea(contour) > 500:
            # Tespit edilen konturların etrafına bir dikdörtgen çiz
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Merkez noktasını belirle
            center = (x + w // 2, y + h // 2)
            cv2.circle(frame, center, 5, (255, 0, 0), -1)
            cv2.putText(frame, "Turuncu Top", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(center)
            send_coordinates(center[0], center[1])
            time.sleep(0.05)  # 50ms bekleyerek daha stabil iletişim sağla

    # Orijinal görüntü ve maske çıktısı
    cv2.imshow("Orijinal Görüntü", frame)
    cv2.imshow("Turuncu Maske", mask)

    # Çıkış için 'q' tuşuna basın
    if cv2.waitKey(1) & 0xFF == ord('q'):

        break

# Tüm pencereleri kapat ve kamerayı serbest bırak
cap.release()
cv2.destroyAllWindows()
ser.close()  # İşlem bitince seri portu kapat