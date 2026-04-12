#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>

// Настройки Wi-Fi
const char* ssid = "realme C61";
const char* password = "2022y2022y";

// TCP Server
WiFiServer server_tcp(8888); // TCP сервер на порту 8888

// UDP Client
WiFiUDP udp_client;
IPAddress remote_UDP_IP(10, 111, 81, 61); // IP-адрес получателя UDP-пакетов (например, твой комп)
const unsigned int remote_UDP_Port = 12345; // Порт получателя UDP-пакетов

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("ESP32 IP address (for TCP): ");
  Serial.println(WiFi.localIP()); // Этот IP используется для подключения TCP-клиента

  // Запуск TCP-сервера
  server_tcp.begin();
  server_tcp.setNoDelay(true); // Опционально: уменьшить задержку TCP

  // UDP клиент не требует начального begin() для отправки, только для приёма.
  // Но если ты будешь *ещё и принимать* UDP, тогда udp.begin(port);
}

void loop() {
  // --- Обработка ВХОДЯЩИХ TCP-пакетов ---

  static WiFiClient client_tcp; // Проверяем, есть ли TCP-клиент
  if ( server_tcp.hasClient() ) {
    client_tcp = server_tcp.available();
  }

  if (client_tcp.connected() ) {
    if (client_tcp.connected()) {
      while (client_tcp.available()) {
        String command = client_tcp.readStringUntil('\n'); // Или другой разделитель
        Serial.println("TCP Command received: " + command);
        // ... Обработать команду управления ...
        // Пример: если command == "FORWARD", то включить моторы вперёд
      }
    }
    // Обычно TCP-соединение держится, но его можно закрыть, если нужно.
    // client_tcp.stop();
  }

  // --- Отправка ИСХОДЯЩИХ UDP-пакетов ---
  // Пример: отправляем какие-то данные каждые N миллисекунд
  static unsigned long last_udp_send_time = 0;
  const unsigned long udp_interval = 100; // Интервал в миллисекундах (например, 10 раз в секунду)

  if (millis() - last_udp_send_time > udp_interval) {
    // Подготовить данные для отправки (например, данные одометрии)
    String telemetry_data = "ODO_X=" + String(1.23) + ",ODO_Y=" + String(4.56) + ",ODO_THETA=" + String(1.57);
    // Или данные "поляны", например, облако точек в каком-то формате...

    // Отправить UDP-пакет
    udp_client.beginPacket(remote_UDP_IP, remote_UDP_Port);
    udp_client.print(telemetry_data.c_str()); // Отправляем строку как есть
    // udp_client.write(data_buffer, data_length); // Или отправляем бинарные данные
    udp_client.endPacket();

    Serial.println("UDP Telemetry sent: " + telemetry_data);
    last_udp_send_time = millis(); // Обновляем время
  }

  // --- Остальная логика ---
  // Обновление датчиков, моторов, вычисление одометрии и т.д.
}
