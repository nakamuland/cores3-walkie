#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_camera.h>
#include <img_converters.h>
#include <esp_heap_caps.h>
#include <atomic>
#include <string.h>

#include "walkie_config.h"

// CoreS3 GC0308 camera pin map (M5Stack CoreS3).
static camera_config_t g_cameraConfig = {};

static constexpr uint16_t VOICE_PORT = 4210;
static constexpr uint16_t IMAGE_PORT = 4211;
static constexpr uint16_t VOICE_MAGIC = 0xC35A;
static constexpr uint32_t AUDIO_SAMPLE_RATE = 16000;
static constexpr size_t AUDIO_SAMPLES = 320;       // 20 ms of mono PCM
static constexpr size_t AUDIO_BYTES = AUDIO_SAMPLES * sizeof(int16_t);
static constexpr size_t TX_BUFFER_COUNT = 2;
static constexpr uint8_t AUDIO_CHANNEL = 0;
static constexpr size_t RX_BUFFER_COUNT = 3;
static constexpr uint8_t JPEG_QUALITY = 72;
static constexpr uint32_t MAX_IMAGE_BYTES = 512 * 1024;
static constexpr uint32_t BATTERY_POLL_INTERVAL_MS = 30000;

struct __attribute__((packed)) VoiceHeader {
  uint16_t magic;
  uint16_t sequence;
};

enum TxBufferState : uint8_t {
  TX_BUFFER_FREE = 0,
  TX_BUFFER_QUEUED = 1,
  TX_BUFFER_READY = 2,
  TX_BUFFER_SENDING = 3,
};

static WiFiUDP g_voiceUdp;
static WiFiServer g_imageServer(IMAGE_PORT);
static IPAddress g_peerIp;
static int16_t g_txAudio[TX_BUFFER_COUNT][AUDIO_SAMPLES];
static std::atomic<TxBufferState> g_txBufferState[TX_BUFFER_COUNT] = {
    TX_BUFFER_FREE, TX_BUFFER_FREE};
static int16_t g_rxAudio[RX_BUFFER_COUNT][AUDIO_SAMPLES];
static volatile bool g_rxBufferFree[RX_BUFFER_COUNT] = {true, true, true};
static uint16_t g_txSequence = 0;
static bool g_transmitting = false;
static bool g_photoRequested = false;
static bool g_pttWaitingForPeer = false;
static uint32_t g_lastReconnectMs = 0;
static size_t g_nextTxBuffer = 0;
static int32_t g_batteryLevel = -1;
static uint32_t g_lastBatteryPollMs = 0;

static void onMicBufferReleased(void *args, void *data, size_t length) {
  (void)args;
  for (size_t i = 0; i < TX_BUFFER_COUNT; ++i) {
    if (data == static_cast<void *>(g_txAudio[i])) {
      // Mic_Class guarantees exactly array_len samples on completion.
      g_txBufferState[i].store(length == AUDIO_SAMPLES ? TX_BUFFER_READY : TX_BUFFER_FREE,
                               std::memory_order_release);
      return;
    }
  }
}

static void markAllTxBuffersFree() {
  for (size_t i = 0; i < TX_BUFFER_COUNT; ++i) {
    g_txBufferState[i].store(TX_BUFFER_FREE, std::memory_order_release);
  }
  g_nextTxBuffer = 0;
}

static void onSpeakerBufferReleased(void *args, const void *data, uint8_t channel) {
  (void)args;
  if (channel != AUDIO_CHANNEL) return;
  for (size_t i = 0; i < RX_BUFFER_COUNT; ++i) {
    if (data == static_cast<const void *>(g_rxAudio[i])) {
      g_rxBufferFree[i] = true;
      return;
    }
  }
}

static void markAllRxBuffersFree() {
  for (size_t i = 0; i < RX_BUFFER_COUNT; ++i) g_rxBufferFree[i] = true;
}

static bool startSpeaker() {
  // isEnabled() only means the board has a configured speaker pin. end() does
  // not clear that configuration, so use isRunning() to detect the live task.
  if (!M5.Speaker.isRunning() && !M5.Speaker.begin()) return false;
  M5.Speaker.setVolume(160);
  M5.Speaker.setBufferReleaseCallback(nullptr, onSpeakerBufferReleased);
  markAllRxBuffersFree();
  return true;
}

static void drawBatteryIndicator() {
  if (g_batteryLevel < 0) return;
  const int32_t width = M5.Display.width();
  const int32_t x = width > 58 ? width - 58 : 0;
  M5.Display.fillRect(x, 0, width - x, 16, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setCursor(x + 3, 3);
  M5.Display.printf("BAT %d%%", static_cast<int>(g_batteryLevel));
  M5.Display.setTextSize(2);
  M5.Display.setTextDatum(middle_center);
}

static void updateBatteryIndicator(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - g_lastBatteryPollMs < BATTERY_POLL_INTERVAL_MS) return;
  g_lastBatteryPollMs = now;

  int32_t level = M5.Power.getBatteryLevel();
  if (level < 0) return;
  if (level > 100) level = 100;
  if (level == g_batteryLevel) return;

  g_batteryLevel = level;
  drawBatteryIndicator();
}

static void renderControls(const char *status) {
  const int32_t width = M5.Display.width();
  const int32_t height = M5.Display.height();
  const int32_t mid = width / 2;
  const int32_t buttonY = height - 78;

  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextSize(2);
  M5.Display.drawString("CoreS3 Walkie", mid, 20);
  M5.Display.setTextSize(1);
  M5.Display.drawString(status, mid, 50);
  M5.Display.drawString(DEVICE_IS_AP ? "Role: ACCESS POINT" : "Role: CLIENT", mid, 70);

  const uint16_t pttColor = g_transmitting ? TFT_RED : TFT_DARKGREY;
  M5.Display.fillRoundRect(6, buttonY, mid - 12, 70, 8, pttColor);
  M5.Display.drawRoundRect(6, buttonY, mid - 12, 70, 8, TFT_WHITE);
  M5.Display.fillRoundRect(mid + 6, buttonY, mid - 12, 70, 8, TFT_BLUE);
  M5.Display.drawRoundRect(mid + 6, buttonY, mid - 12, 70, 8, TFT_WHITE);
  M5.Display.setTextSize(2);
  M5.Display.drawString("HOLD TO", mid / 2, buttonY + 25);
  M5.Display.drawString("TALK", mid / 2, buttonY + 49);
  M5.Display.drawString("SEND", mid + mid / 2, buttonY + 25);
  M5.Display.drawString("PHOTO", mid + mid / 2, buttonY + 49);
  drawBatteryIndicator();
}

static bool initCamera() {
  M5.In_I2C.release();

  g_cameraConfig.pin_pwdn = -1;
  g_cameraConfig.pin_reset = -1;
  g_cameraConfig.pin_xclk = -1;
  g_cameraConfig.pin_sccb_sda = 12;
  g_cameraConfig.pin_sccb_scl = 11;
  g_cameraConfig.pin_d7 = 47;
  g_cameraConfig.pin_d6 = 48;
  g_cameraConfig.pin_d5 = 16;
  g_cameraConfig.pin_d4 = 15;
  g_cameraConfig.pin_d3 = 42;
  g_cameraConfig.pin_d2 = 41;
  g_cameraConfig.pin_d1 = 40;
  g_cameraConfig.pin_d0 = 39;
  g_cameraConfig.pin_vsync = 46;
  g_cameraConfig.pin_href = 38;
  g_cameraConfig.pin_pclk = 45;
  g_cameraConfig.xclk_freq_hz = 20000000;
  g_cameraConfig.ledc_timer = LEDC_TIMER_0;
  g_cameraConfig.ledc_channel = LEDC_CHANNEL_0;
  // GC0308 supplies RGB565; the sketch converts each still frame to JPEG.
  g_cameraConfig.pixel_format = PIXFORMAT_RGB565;
  g_cameraConfig.frame_size = FRAMESIZE_QVGA;
  g_cameraConfig.jpeg_quality = 0;
  g_cameraConfig.fb_count = 1;
  g_cameraConfig.fb_location = CAMERA_FB_IN_PSRAM;
  g_cameraConfig.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  g_cameraConfig.sccb_i2c_port = -1;

  const esp_err_t err = esp_camera_init(&g_cameraConfig);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%X\n", static_cast<unsigned int>(err));
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) sensor->set_framesize(sensor, FRAMESIZE_QVGA);
  return true;
}

static void startNetwork() {
  WiFi.mode(DEVICE_IS_AP ? WIFI_AP : WIFI_STA);
  WiFi.setSleep(false);  // Keep short UDP voice packets responsive.

  const IPAddress apIp(192, 168, 4, 1);
  const IPAddress clientIp(192, 168, 4, 2);
  const IPAddress subnet(255, 255, 255, 0);

  if (DEVICE_IS_AP) {
    g_peerIp = clientIp;
    WiFi.softAPConfig(apIp, apIp, subnet);
    if (!WiFi.softAP(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL, 0, 1)) {
      Serial.println("SoftAP start failed.");
    }
  } else {
    g_peerIp = apIp;
    WiFi.config(clientIp, apIp, subnet, apIp);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < 20000) {
      M5.update();
      delay(250);
    }
  }

  if (DEVICE_IS_AP || WiFi.status() == WL_CONNECTED) {
    const IPAddress localIp = DEVICE_IS_AP ? WiFi.softAPIP() : WiFi.localIP();
    Serial.printf("Wi-Fi IP: %s\n", localIp.toString().c_str());
  } else {
    Serial.println("Wi-Fi not connected yet; will retry in the background.");
  }

  if (!g_voiceUdp.begin(VOICE_PORT)) Serial.println("UDP voice port start failed.");
  g_imageServer.begin();
}

static bool peerConnected() {
  return DEVICE_IS_AP ? (WiFi.softAPgetStationNum() > 0)
                      : (WiFi.status() == WL_CONNECTED);
}

static bool isPttTouchDown() {
  if (M5.Touch.getCount() == 0) return false;
  const auto &touch = M5.Touch.getDetail(0);
  const int32_t mid = M5.Display.width() / 2;
  const int32_t buttonY = M5.Display.height() - 78;
  return touch.isPressed() && touch.x < mid && touch.y >= buttonY;
}

static void checkPhotoTouch() {
  if (M5.Touch.getCount() == 0) return;
  const auto &touch = M5.Touch.getDetail(0);
  const int32_t mid = M5.Display.width() / 2;
  const int32_t buttonY = M5.Display.height() - 78;
  if (touch.wasPressed() && touch.x >= mid && touch.y >= buttonY) {
    g_photoRequested = true;
  }
}

static bool beginTransmit() {
  M5.Speaker.stop();
  M5.Speaker.end();
  markAllRxBuffersFree();
  if (M5.Mic.begin()) {
    markAllTxBuffersFree();
    g_transmitting = true;
    renderControls("TRANSMITTING - release to listen");
    Serial.println("PTT: transmit");
    return true;
  }

  Serial.println("Microphone start failed; returning to receive mode.");
  g_transmitting = false;
  startSpeaker();
  renderControls("MIC ERROR - listening");
  return false;
}

static void endTransmit() {
  // end() joins the capture task and clears requests it drops; once it returns,
  // neither a pending capture nor its callback can still access these buffers.
  M5.Mic.end();
  markAllTxBuffersFree();  // Discard any completed-but-unsent tail at PTT release.
  g_transmitting = false;
  if (!startSpeaker()) Serial.println("Speaker start failed.");
  renderControls("READY - hold left button to talk");
  Serial.println("PTT: receive");
}

static void sendVoicePacket(const int16_t *samples) {
  const VoiceHeader header = {VOICE_MAGIC, g_txSequence++};
  if (g_voiceUdp.beginPacket(g_peerIp, VOICE_PORT)) {
    g_voiceUdp.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    g_voiceUdp.write(reinterpret_cast<const uint8_t *>(samples), AUDIO_BYTES);
    g_voiceUdp.endPacket();
  }
}

static void queueMicBuffers() {
  for (size_t i = 0; i < TX_BUFFER_COUNT; ++i) {
    TxBufferState expected = TX_BUFFER_FREE;
    if (!g_txBufferState[i].compare_exchange_strong(
            expected, TX_BUFFER_QUEUED, std::memory_order_acq_rel)) {
      continue;
    }

    // record() only enqueues this destination. The release callback marks it
    // READY after all 320 samples have actually been written.
    if (!M5.Mic.record(g_txAudio[i], AUDIO_SAMPLES, AUDIO_SAMPLE_RATE, false)) {
      g_txBufferState[i].store(TX_BUFFER_FREE, std::memory_order_release);
    }
  }
}

static void sendCompletedVoicePackets() {
  while (true) {
    bool sentOne = false;
    for (size_t offset = 0; offset < TX_BUFFER_COUNT; ++offset) {
      const size_t i = (g_nextTxBuffer + offset) % TX_BUFFER_COUNT;
      TxBufferState expected = TX_BUFFER_READY;
      if (!g_txBufferState[i].compare_exchange_strong(
              expected, TX_BUFFER_SENDING, std::memory_order_acq_rel)) {
        continue;
      }

      // The callback's release-store makes the completed samples visible here.
      sendVoicePacket(g_txAudio[i]);
      g_txBufferState[i].store(TX_BUFFER_FREE, std::memory_order_release);
      g_nextTxBuffer = (i + 1) % TX_BUFFER_COUNT;
      sentOne = true;
      break;
    }
    if (!sentOne) return;
  }
}

static void drainCurrentUdpPacket() {
  uint8_t discard[64];
  while (g_voiceUdp.available() > 0) {
    const int amount = min(static_cast<int>(sizeof(discard)), g_voiceUdp.available());
    g_voiceUdp.read(discard, amount);
  }
}

static int findFreeRxBuffer() {
  for (size_t i = 0; i < RX_BUFFER_COUNT; ++i) {
    if (g_rxBufferFree[i]) return static_cast<int>(i);
  }
  return -1;
}

static void receiveVoicePackets() {
  while (!g_transmitting) {
    const int packetSize = g_voiceUdp.parsePacket();
    if (packetSize <= 0) return;

    if (g_voiceUdp.remoteIP() != g_peerIp ||
        packetSize != static_cast<int>(sizeof(VoiceHeader) + AUDIO_BYTES)) {
      drainCurrentUdpPacket();
      continue;
    }

    VoiceHeader header = {};
    if (g_voiceUdp.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) != sizeof(header) ||
        header.magic != VOICE_MAGIC) {
      drainCurrentUdpPacket();
      continue;
    }

    const int bufferIndex = findFreeRxBuffer();
    if (bufferIndex < 0) {
      drainCurrentUdpPacket();  // Drop rather than delay live voice.
      continue;
    }

    int16_t *buffer = g_rxAudio[bufferIndex];
    const int bytesRead = g_voiceUdp.read(reinterpret_cast<uint8_t *>(buffer), AUDIO_BYTES);
    if (bytesRead != static_cast<int>(AUDIO_BYTES)) {
      g_rxBufferFree[bufferIndex] = true;
      drainCurrentUdpPacket();
      continue;
    }

    g_rxBufferFree[bufferIndex] = false;
    const bool queued = M5.Speaker.playRaw(buffer, AUDIO_SAMPLES, AUDIO_SAMPLE_RATE,
                                           false, 1, AUDIO_CHANNEL, false);
    if (!queued) g_rxBufferFree[bufferIndex] = true;
  }
}

static bool writeAll(WiFiClient &client, const uint8_t *data, size_t length) {
  while (length > 0) {
    const size_t chunk = min(length, static_cast<size_t>(1024));
    const size_t written = client.write(data, chunk);
    if (written == 0) {
      if (!client.connected()) return false;
      delay(1);
      continue;
    }
    data += written;
    length -= written;
  }
  return true;
}

static void encodeLength(uint32_t length, uint8_t header[4]) {
  header[0] = static_cast<uint8_t>(length >> 24);
  header[1] = static_cast<uint8_t>(length >> 16);
  header[2] = static_cast<uint8_t>(length >> 8);
  header[3] = static_cast<uint8_t>(length);
}

static uint32_t decodeLength(const uint8_t header[4]) {
  return (static_cast<uint32_t>(header[0]) << 24) |
         (static_cast<uint32_t>(header[1]) << 16) |
         (static_cast<uint32_t>(header[2]) << 8) |
         static_cast<uint32_t>(header[3]);
}

static void rotateCameraFrame180(camera_fb_t *frame) {
  if (frame == nullptr || frame->format != PIXFORMAT_RGB565 || frame->buf == nullptr) return;
  const size_t pixelCount = static_cast<size_t>(frame->width) * frame->height;
  if (frame->len < pixelCount * sizeof(uint16_t)) return;

  auto *pixels = reinterpret_cast<uint16_t *>(frame->buf);
  for (size_t i = 0; i < pixelCount / 2; ++i) {
    const uint16_t opposite = pixels[pixelCount - 1 - i];
    pixels[pixelCount - 1 - i] = pixels[i];
    pixels[i] = opposite;
  }
}

static bool captureAndSendPhoto() {
  camera_fb_t *frame = esp_camera_fb_get();
  if (frame == nullptr) {
    Serial.println("Camera capture failed.");
    return false;
  }

  rotateCameraFrame180(frame);
  uint8_t *jpeg = nullptr;
  size_t jpegLength = 0;
  const bool encoded = frame2jpg(frame, JPEG_QUALITY, &jpeg, &jpegLength);
  esp_camera_fb_return(frame);
  if (!encoded || jpeg == nullptr || jpegLength == 0 || jpegLength > MAX_IMAGE_BYTES) {
    if (jpeg != nullptr) free(jpeg);
    Serial.println("JPEG conversion failed or image exceeded the size limit.");
    return false;
  }

  WiFiClient client;
  client.setTimeout(5000);
  if (!client.connect(g_peerIp, IMAGE_PORT)) {
    free(jpeg);
    Serial.println("Photo connection failed.");
    return false;
  }

  uint8_t lengthHeader[4];
  encodeLength(static_cast<uint32_t>(jpegLength), lengthHeader);
  const bool sent = writeAll(client, lengthHeader, sizeof(lengthHeader)) &&
                    writeAll(client, jpeg, jpegLength);
  client.flush();
  client.stop();
  Serial.printf("Photo %s: %u bytes\n", sent ? "sent" : "send failed",
                static_cast<unsigned int>(jpegLength));
  free(jpeg);
  return sent;
}

static bool readExact(WiFiClient &client, uint8_t *destination, size_t length,
                      uint32_t timeoutMs) {
  const uint32_t started = millis();
  size_t received = 0;
  while (received < length && millis() - started < timeoutMs) {
    const int available = client.available();
    if (available > 0) {
      const size_t amount = min(static_cast<size_t>(available), length - received);
      const int count = client.read(destination + received, amount);
      if (count > 0) received += static_cast<size_t>(count);
    } else if (!client.connected()) {
      break;
    } else {
      delay(1);
    }
  }
  return received == length;
}

static void overlayControlsOnPhoto() {
  const int32_t width = M5.Display.width();
  const int32_t height = M5.Display.height();
  const int32_t mid = width / 2;
  const int32_t buttonY = height - 78;
  M5.Display.fillRoundRect(6, buttonY, mid - 12, 70, 8, TFT_DARKGREY);
  M5.Display.drawRoundRect(6, buttonY, mid - 12, 70, 8, TFT_WHITE);
  M5.Display.fillRoundRect(mid + 6, buttonY, mid - 12, 70, 8, TFT_BLUE);
  M5.Display.drawRoundRect(mid + 6, buttonY, mid - 12, 70, 8, TFT_WHITE);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextSize(2);
  M5.Display.drawString("HOLD TO", mid / 2, buttonY + 25);
  M5.Display.drawString("TALK", mid / 2, buttonY + 49);
  M5.Display.drawString("SEND", mid + mid / 2, buttonY + 25);
  M5.Display.drawString("PHOTO", mid + mid / 2, buttonY + 49);
  drawBatteryIndicator();
}

static void handleImageServer() {
  WiFiClient client = g_imageServer.available();
  if (!client) return;

  client.setTimeout(5000);
  uint8_t lengthHeader[4];
  if (!readExact(client, lengthHeader, sizeof(lengthHeader), 5000)) {
    client.stop();
    return;
  }

  const uint32_t jpegLength = decodeLength(lengthHeader);
  if (jpegLength == 0 || jpegLength > MAX_IMAGE_BYTES) {
    client.stop();
    Serial.println("Rejected invalid photo size.");
    return;
  }

  uint8_t *jpeg = static_cast<uint8_t *>(heap_caps_malloc(
      jpegLength, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (jpeg == nullptr) {
    client.stop();
    Serial.println("Not enough PSRAM for received photo.");
    return;
  }

  const bool received = readExact(client, jpeg, jpegLength, 20000);
  client.stop();
  if (!received) {
    heap_caps_free(jpeg);
    Serial.println("Photo transfer was incomplete.");
    return;
  }

  M5.Display.drawJpg(jpeg, jpegLength, 0, 0);
  overlayControlsOnPhoto();
  heap_caps_free(jpeg);
  Serial.printf("Photo received: %u bytes\n", static_cast<unsigned int>(jpegLength));
}

void setup() {
  Serial.begin(115200);
  delay(250);

  auto config = M5.config();
  M5.begin(config);
  // Register before the first record(); callback runs on Mic's capture task.
  M5.Mic.setBufferReleaseCallback(nullptr, onMicBufferReleased);
  M5.Display.setRotation(3);
  updateBatteryIndicator(true);
  renderControls("Starting camera and Wi-Fi...");

  if (!initCamera()) {
    renderControls("CAMERA ERROR - check Serial Monitor");
    while (true) delay(1000);
  }

  startNetwork();
  startSpeaker();
  renderControls(peerConnected() ? "READY - hold left button to talk"
                                 : (DEVICE_IS_AP ? "Waiting for client..."
                                                 : "Waiting for access point..."));
  Serial.println("CoreS3 Walkie ready.");
  Serial.printf("AP SSID: %s | Peer: %s\n", WIFI_SSID, g_peerIp.toString().c_str());
}

void loop() {
  M5.update();
  updateBatteryIndicator();
  checkPhotoTouch();
  const bool pttDown = isPttTouchDown();

  if (pttDown && !g_transmitting) {
    if (peerConnected()) {
      g_pttWaitingForPeer = false;
      beginTransmit();
    } else if (!g_pttWaitingForPeer) {
      g_pttWaitingForPeer = true;
      renderControls("Peer not connected yet");
    }
  } else if (!pttDown && g_transmitting) {
    endTransmit();
  }
  if (!pttDown) g_pttWaitingForPeer = false;

  if (WiFi.status() != WL_CONNECTED && !DEVICE_IS_AP) {
    if (millis() - g_lastReconnectMs >= 5000) {
      g_lastReconnectMs = millis();
      WiFi.reconnect();
      renderControls("Reconnecting to access point...");
    }
  }

  if (g_transmitting) {
    sendCompletedVoicePackets();
    queueMicBuffers();
    delay(1);
    return;
  }

  if (g_photoRequested) {
    g_photoRequested = false;
    renderControls("Capturing and sending photo...");
    const bool sent = captureAndSendPhoto();
    renderControls(sent ? "PHOTO SENT" : "PHOTO FAILED - see Serial");
  }

  receiveVoicePackets();
  handleImageServer();
  delay(1);
}
