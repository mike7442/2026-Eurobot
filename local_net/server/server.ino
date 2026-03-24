#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// Настройки точки доступа
const char* ssid = "ESP32_TCP_Server"; // Имя Wi-Fi сети ESP32
const char* password = "123456789";    // Пароль (минимум 8 символов)

// Настройки TCP-сервера
WiFiServer server(8080); // Открываем TCP-сервер на порту 8080
WiFiClient client;       // Объект для подключенного клиента

void setup() {
  // Инициализация Serial для вывода в Arduino IDE
  Serial.begin(115200);
  delay(10);

  // Создаем точку доступа
  Serial.print("Creating Access Point named: ");
  Serial.println(ssid);
  WiFi.softAP(ssid, password); // Создаем точку доступа

  IPAddress IP = WiFi.softAPIP(); // Получаем IP-адрес точки доступа ESP32
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.begin(); // Запускаем TCP-сервер
  Serial.println("TCP Server started on port 8080");
  Serial.println("Connect your other device to Wi-Fi: " + String(ssid));
  Serial.println("Then connect TCP client to IP: " + IP.toString() + " : 8080");
  Serial.println("--- Ready to receive data ---");
}

void loop() {
  // Проверяем, есть ли новый клиент, пытающийся подключиться
  if (server.hasClient()) {
    if (!client || !client.connected()) { // Если старый клиент отключен
      if (client) { // Закрываем предыдущее соединение, если осталось
        client.stop();
      }
      client = server.available(); // Принимаем нового клиента
      Serial.println("[INFO] New client connected!");
      Serial.print("[INFO] Client IP: ");
      Serial.println(client.remoteIP()); // Показываем IP клиента (другого устройства)
    } else {
      // Если есть еще один клиент, и текущий занят, закрываем лишнего
      WiFiClient tempClient = server.available();
      tempClient.stop();
    }
  }

  // Если клиент подключен и есть данные
  if (client && client.connected() && client.available()) {
    String receivedData = client.readStringUntil('\n'); // Читаем строку до символа новой строки
    Serial.print("[RECEIVED] From client: ");
    Serial.println(receivedData); // Печатаем полученные данные в Serial Monitor

    // (Опционально) Отправляем ответ обратно клиенту (можно удалить, если не нужно)
    client.println("Echo: " + receivedData);
    Serial.print("[SENT] Echoed back to client: ");
    Serial.println(receivedData);
  }

  // Проверяем, не потерял ли клиент соединение
  if (client && !client.connected()) {
     Serial.println("[INFO] Client disconnected.");
     client.stop(); // Очищаем соединение
  }

  delay(50); // Небольшая задержка для стабильности
}
