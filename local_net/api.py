import socket
import time

# --- Настройки ---
ESP_IP = '192.168.4.1'  # IP-адрес точки доступа ESP32 (посмотри в Serial Monitor при запуске ESP32)
TCP_PORT = 8080         # Порт TCP-сервера на ESP32
MESSAGE_PREFIX = "Hello from Python!" # Префикс сообщения

def send_message(message):
    """Функция для подключения и отправки одного сообщения"""
    try:
        # Создаем TCP-сокет
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Connecting to {ESP_IP}:{TCP_PORT}...")
        
        # Подключаемся к серверу ESP32
        sock.connect((ESP_IP, TCP_PORT))
        print("Connected!")

        # Отправляем сообщение (добавляем \n, т.к. ESP32 читает до \n)
        full_message = message + "\n"
        print(f"Sending: {full_message.strip()}")
        sock.send(full_message.encode('utf-8'))

        # (Опционально) Прочитаем ответ от ESP32 (если он его отправляет)
        # response = sock.recv(1024).decode('utf-8').strip()
        # print(f"ESP32 replied: {response}")

        # Закрываем соединение
        sock.close()
        print("Message sent and connection closed.")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    counter = 0
    try:
        print("Press Ctrl+C to stop.")
        while True:
            message = f"{MESSAGE_PREFIX} #{counter}"
            send_message(message)
            counter += 1
            time.sleep(3)  # Пауза 3 секунды между отправками
    except KeyboardInterrupt:
        print("\nStopped by user.")
