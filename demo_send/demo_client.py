import socket
import threading
import time
import select # Для неблокирующего чтения TCP

# --- Настройки ---
# Замените на IP-адрес вашего ESP32 в вашей локальной сети
ESP32_TCP_IP = "192.168.1.100"  # <-- Укажите реальный IP ESP32
ESP32_TCP_PORT = 8888            # Порт TCP-сервера на ESP32
UDP_BIND_IP = "0.0.0.0"        # Слушаем на всех интерфейсах
UDP_BIND_PORT = 12345           # Порт, на котором ESP32 отправляет UDP

tcp_socket = None
tcp_connected = False

def connect_tcp():
    global tcp_socket, tcp_connected
    try:
        tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Устанавливаем таймаут для соединения
        tcp_socket.settimeout(5)
        print(f"Попытка подключения к {ESP32_TCP_IP}:{ESP32_TCP_PORT}...")
        tcp_socket.connect((ESP32_TCP_IP, ESP32_TCP_PORT))
        tcp_socket.settimeout(None) # Убираем таймаут после подключения, если нужно постоянное соединение
        tcp_connected = True
        print(f"Подключено к ESP32 по TCP!")
        return tcp_socket
    except Exception as e:
        print(f"Ошибка подключения по TCP: {e}")
        tcp_connected = False
        if tcp_socket:
            tcp_socket.close()
        return None

def send_command(command):
    """Отправить команду по TCP."""
    global tcp_socket, tcp_connected
    if tcp_connected and tcp_socket:
        try:
            # Добавляем символ новой строки, если ESP32 ожидает его как разделитель
            message = command.strip() + '\n'
            tcp_socket.sendall(message.encode('utf-8'))
            print(f"Отправлена команда: {command}")
        except BrokenPipeError:
            print("Соединение с ESP32 по TCP разорвано (Broken Pipe).")
            tcp_connected = False
        except ConnectionResetError:
            print("Соединение с ESP32 по TCP сброшено.")
            tcp_connected = False
        except Exception as e:
            print(f"Ошибка отправки команды: {e}")
            tcp_connected = False
            tcp_socket.close()

def listen_tcp():
    """Поток для прослушивания входящих данных по TCP (если ESP32 будет их отправлять)."""
    global tcp_socket, tcp_connected
    while tcp_connected:
        try:
            # Используем select для неблокирующего ожидания данных
            ready = select.select([tcp_socket], [], [], 0.1) # Таймаут 0.1с
            if ready[0]:
                data = tcp_socket.recv(1024) # Получаем до 1024 байт
                if 
                    response = data.decode('utf-8').strip()
                    print(f"[TCP from ESP32]: {response}")
                else: # Если recv возвращает пустую строку, значит соединение закрыто
                    print("Соединение по TCP закрыто удалённым хостом.")
                    tcp_connected = False
                    tcp_socket.close()
                    break
        except ConnectionResetError:
            print("Соединение по TCP сброшено удалённым хостом.")
            tcp_connected = False
            tcp_socket.close()
            break
        except Exception as e:
            if tcp_connected: # Печатаем ошибку только если соединение ещё считается активным
                print(f"Ошибка приёма данных по TCP: {e}")
            tcp_connected = False
            tcp_socket.close()
            break

def listen_udp():
    """Поток для прослушивания входящих UDP-пакетов."""
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((UDP_BIND_IP, UDP_BIND_PORT))
    print(f"Слушаю UDP на {UDP_BIND_IP}:{UDP_BIND_PORT}")

    try:
        while True:
            data, addr = udp_socket.recvfrom(1024) # Буфер 1024 байта
            message = data.decode('utf-8')
            print(f"[UDP from {addr}]: {message}")
            # Тут можно обработать полученные телеметрические данные
    except KeyboardInterrupt:
        print("\nОстановка прослушивания UDP.")
    finally:
        udp_socket.close()

if __name__ == "__main__":
    # Подключаемся к ESP32 по TCP
    tcp_sock = connect_tcp()
    if not tcp_sock:
        print("Не удалось подключиться по TCP. Запуск только UDP-приемника невозможен в этом примере, так как TCP-канал управления основной.")
        exit()

    # Запускаем поток для прослушивания TCP (если ESP32 будет отправлять что-то обратно)
    tcp_thread = threading.Thread(target=listen_tcp)
    tcp_thread.daemon = True # Поток завершится, когда основной процесс завершится
    tcp_thread.start()

    # Запускаем поток для прослушивания UDP
    udp_thread = threading.Thread(target=listen_udp)
    udp_thread.daemon = True
    udp_thread.start()

    # Основной поток - обработка ввода пользователя для отправки команд
    print("\nВведите команды ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'STOP', 'quit'):")
    try:
        while True:
            command = input("> ").strip().upper()
            if command == 'QUIT':
                break
            elif command in ['FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'STOP']:
                send_command(command)
            else:
                print(f"Неизвестная команда: {command}. Используйте FORWARD, BACKWARD, LEFT, RIGHT, STOP, или quit.")

    except KeyboardInterrupt:
        print("\nВыход...")

    # Закрываем TCP-соединение
    if tcp_socket:
        tcp_socket.close()
    print("Соединения закрыты.")
