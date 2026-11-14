
#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_idf_version.h"
#include "mqtt_client.h"
#include "cJSON.h"
// =====================================================================
// THƯ VIỆN NHẬN DIỆN HOA QUẢ EDGE IMPULSE
#include <fruit_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
// =====================================================================
// ======================= THÔNG TIN WIFI ==============================
#define WIFI_SSID       "ROBOTIC"       // <--- ĐIỀN TÊN WIFI CỦA BẠN
#define WIFI_PASSWORD   "KTRB&HTTM"     // <--- ĐIỀN MẬT KHẨU WIFI
// ======================= THÔNG TIN EMQX CLOUD ========================
#define MQTT_URI        "mqtts://s319e126.ala.asia-southeast1.emqxsl.com:8883"
#define MQTT_USERNAME   "Esp32s3No1"
#define MQTT_PASSWORD   "Phuongdz001" // <--- ĐIỀN MẬT KHẨU EMQX CỦA BẠN
#define MQTT_CLIENT_ID  "ESP32S3-MQTT-PUBLISHER"
#define MQTT_TOPIC      "factory/machine01/ack/position"
#define MQTT_QOS        1
#define MQTT_RETAIN     0
// Tần suất gửi dữ liệu lên Cloud (ms)
#define DETECTION_CLEAR_FRAMES 3
// Ngưỡng diện tích (pixel^2 trên khung 96x96) để phân loại sz: "1"=nhỏ, "2"=vừa, "3"=lớn
#define SIZE_SMALL_MAX_AREA   100.0f
#define SIZE_MEDIUM_MAX_AREA  300.0f
// Chứng chỉ CA của EMQX Cloud (DigiCert Global Root G2 - từ emqxsl-ca.crt)
static const char *EMQX_CA_CERT =
  "-----BEGIN CERTIFICATE-----\n"
  "MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh\n"
  "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
  "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
  "MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n"
  "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
  "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n"
  "9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI\n"
  "2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx\n"
  "1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ\n"
  "q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz\n"
  "tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ\n"
  "vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP\n"
  "BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV\n"
  "5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY\n"
  "1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4\n"
  "NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG\n"
  "Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91\n"
  "8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe\n"
  "pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl\n"
  "MrY=\n"
  "-----END CERTIFICATE-----\n";
// =====================================================================
// PINOUT CHUẨN ESP32-S3 WROOM CAM
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5
#define Y9_GPIO_NUM     16
#define Y8_GPIO_NUM     17
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     12
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM     8
#define Y3_GPIO_NUM     9
#define Y2_GPIO_NUM     11
#define VSYNC_GPIO_NUM  6
#define HREF_GPIO_NUM   7
#define PCLK_GPIO_NUM   13
// Bộ đệm ảnh JPEG trong PSRAM
#define JPEG_BUF_MAX (40 * 1024)
static uint8_t *stream_jpeg_buf = NULL;
static size_t   stream_jpeg_len = 0;
static SemaphoreHandle_t stream_mutex;
static uint8_t *ai_jpeg_buf = NULL;
static size_t   ai_jpeg_len = 0;
static bool     ai_frame_ready = false;
static SemaphoreHandle_t ai_mutex;
static uint8_t *local_jpeg   = NULL; // Buffer tạm cho Task AI trong PSRAM
static uint8_t *snapshot_rgb = NULL; // 320x240 RGB888 trong PSRAM
httpd_handle_t camera_httpd = NULL;
static httpd_handle_t camera_stream_httpd = NULL;
char ai_result_text[64] = "📷 Đang quét...";
// Biến toàn cục MQTT
static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;
static uint32_t event_counter = 0;
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
// ===================== ÁNH XẠ CLASS ID & SIZE ID (CHUỖI) =============
static const char* get_class_id_str(const char *label) {
  if (strcmp(label, "orange") == 0 || strcmp(label, "cam") == 0)    return "1";
  if (strcmp(label, "lemon") == 0  || strcmp(label, "chanh") == 0)  return "2";
  if (strcmp(label, "tomato") == 0 || strcmp(label, "cachua") == 0) return "3";
  return "1";
}
static const char* get_size_id_str(float area) {
  if (area <= SIZE_SMALL_MAX_AREA)  return "1";
  if (area <= SIZE_MEDIUM_MAX_AREA) return "2";
  return "3";
}
// ===================== MQTT PUBLISH & HANDLER ========================
static void mqtt_publish_data(const char *class_id, const char *size_id) {
  if (!mqtt_connected || mqtt_client == NULL) {
      return;
  }
  cJSON *root = cJSON_CreateObject();
  if (root == NULL) {
      Serial.println("[MQTT] JSON allocation failed");
      return;
  }
  event_counter++;

  // Đúng format: {"event":"new_object","track_id":7,"class_id":"1","sz":"1"}
  cJSON_AddStringToObject(root, "event", "new_object");
  cJSON_AddNumberToObject(root, "track_id", event_counter);
  cJSON_AddStringToObject(root, "class_id", class_id);
  cJSON_AddStringToObject(root, "sz", size_id);
  char *json_data = cJSON_PrintUnformatted(root);
  if (json_data == NULL) {
      Serial.println("[MQTT] JSON serialization failed");
      cJSON_Delete(root);
      return;
  }
  int message_id = esp_mqtt_client_publish(
      mqtt_client,
      MQTT_TOPIC,
      json_data,
      0,
      MQTT_QOS,
      MQTT_RETAIN
  );
  if (message_id >= 0) {
      Serial.printf("[MQTT] PUBLISH SUCCESS: ID=%d | JSON: %s\n", message_id, json_data);
  } else {
      Serial.println("[MQTT] PUBLISH FAILED");
  }
  cJSON_free(json_data);
  cJSON_Delete(root);
}
static void mqtt_event_handler(
  void *handler_args,
  esp_event_base_t base,
  int32_t event_id,
  void *event_data
) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  switch ((esp_mqtt_event_id_t)event_id) {
      case MQTT_EVENT_CONNECTED:
          mqtt_connected = true;
          Serial.printf("[MQTT] EMQX CONNECTED | Publish topic: %s\n", MQTT_TOPIC);
          break;
      case MQTT_EVENT_DISCONNECTED:
          mqtt_connected = false;
          Serial.println("[MQTT] EMQX DISCONNECTED");
          break;
      case MQTT_EVENT_PUBLISHED:
          Serial.printf("[MQTT] BROKER ACK: message_id=%d\n", event->msg_id);
          break;
      case MQTT_EVENT_ERROR:
          Serial.println("[MQTT] ERROR");
          if (event->error_handle != NULL) {
              Serial.printf("[MQTT] Error type: %d\n", event->error_handle->error_type);
          }
          break;
      default:
          break;
  }
}
static void mqtt_init(void) {
  if (mqtt_client != NULL) {
      return;
  }
  Serial.println("[MQTT] Initializing MQTT...");
  esp_mqtt_client_config_t config = {};
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  config.broker.address.uri = MQTT_URI;
  config.broker.verification.certificate = EMQX_CA_CERT;
  config.credentials.username = MQTT_USERNAME;
  config.credentials.client_id = MQTT_CLIENT_ID;
  config.credentials.authentication.password = MQTT_PASSWORD;
  config.session.keepalive = 60;
  config.network.reconnect_timeout_ms = 2000;
  config.network.timeout_ms = 10000;
  config.buffer.size = 4096;
#else
  config.uri = MQTT_URI;
  config.cert_pem = EMQX_CA_CERT;
  config.username = MQTT_USERNAME;
  config.client_id = MQTT_CLIENT_ID;
  config.password = MQTT_PASSWORD;
  config.keepalive = 60;
  config.reconnect_timeout_ms = 2000;
  config.network_timeout_ms = 10000;
  config.buffer_size = 4096;
#endif
  mqtt_client = esp_mqtt_client_init(&config);
  if (mqtt_client == NULL) {
      Serial.println("[MQTT] Initialization failed");
      return;
  }
  esp_mqtt_client_register_event(
      mqtt_client,
      MQTT_EVENT_ANY,
      mqtt_event_handler,
      NULL
  );
  esp_err_t result = esp_mqtt_client_start(mqtt_client);
  if (result != ESP_OK) {
      Serial.printf("[MQTT] Start failed: %s\n", esp_err_to_name(result));
      esp_mqtt_client_destroy(mqtt_client);
      mqtt_client = NULL;
      return;
  }
  Serial.println("[MQTT] Client started");
}
// ===================== GIAO DIỆN WEB HIỂN THỊ CAMERA NÉT =========================
static esp_err_t index_handler(httpd_req_t *req) {
  char html[1200];
  String stream_url = "http://" + WiFi.localIP().toString() + ":81/stream";
  int html_len = snprintf(html, sizeof(html),
    "<html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>"
    "<title>AI Fruit Classifier</title>"
    "<style>"
    "body { text-align:center; background:#0f141c; color:#eee; font-family:sans-serif; padding:15px; }"
    "h1 { color:#00e5ff; font-size:24px; margin-bottom:12px; }"
    "#cam-box { display:inline-block; border:3px solid #00e5ff; border-radius:14px; overflow:hidden; box-shadow:0 8px 30px rgba(0,0,0,0.7); }"
    "#stream { width:320px; height:240px; display:block; }"
    "#result { font-size:22px; font-weight:bold; color:#00ff66; background:#18261e; padding:12px 28px; border-radius:30px; display:inline-block; min-width:280px; margin-top:16px; border:2px solid #00ff66; }"
    "</style>"
    "</head><body>"
    "<h1>AI Phân Loại Hoa Quả (Camera Nét HD)</h1>"
          "<div id='cam-box'><img id='stream' src='%s'></div>"
    "<div><div id='result'>Đang quét...</div></div>"
    "<script>"
    "setInterval(async () => {"
    "  try {"
    "    let r = await fetch('/result');"
    "    let txt = await r.text();"
    "    document.getElementById('result').innerText = txt;"
    "  } catch(e) {}"
    "}, 300);"
    "</script>"
          "</body></html>", stream_url.c_str());
      if (html_len < 0 || html_len >= sizeof(html)) {
              return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                      "Could not generate camera page");
      }
      httpd_resp_send(req, html, html_len);
  return ESP_OK;
}
static esp_err_t result_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_send(req, ai_result_text, strlen(ai_result_text));
  return ESP_OK;
}
// ===================== LUỒNG STREAM VIDEO MƯỢT (~25 FPS) ====================
static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = ESP_OK;
  char part_buf[64];
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  while (true) {
      size_t len = 0;
      if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          len = stream_jpeg_len;
          if (len > 0 && stream_jpeg_buf != NULL) {
              size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, len);
              res = httpd_resp_send_chunk(req, part_buf, hlen);
              if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)stream_jpeg_buf, len);
              if (res == ESP_OK) res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
          }
          xSemaphoreGive(stream_mutex);
      }
      if (res != ESP_OK) break;
      vTaskDelay(35 / portTICK_PERIOD_MS);
  }
  return res;
}
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_open_sockets = 5;
  config.stack_size = 8192;
  httpd_uri_t index_uri  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = NULL };
  httpd_uri_t result_uri = { .uri = "/result", .method = HTTP_GET, .handler = result_handler, .user_ctx = NULL };
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(camera_httpd, &index_uri);
      httpd_register_uri_handler(camera_httpd, &result_uri);
  } else {
      Serial.println("[HTTP] Failed to start page/result server");
      return;
  }
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port = config.ctrl_port + 1;
  stream_config.max_open_sockets = 2;
  stream_config.stack_size = 8192;
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  if (httpd_start(&camera_stream_httpd, &stream_config) == ESP_OK) {
      httpd_register_uri_handler(camera_stream_httpd, &stream_uri);
  } else {
      Serial.println("[HTTP] Failed to start camera stream server");
  }
}
// ===================== TÁC VỤ 1: CHỤP ẢNH TỪ CAMERA (CORE 0) =====================
void camera_capture_task(void *pvParameters) {
  while (1) {
      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
          vTaskDelay(15 / portTICK_PERIOD_MS);
          continue;
      }
      if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          if (fb->len < JPEG_BUF_MAX) {
              memcpy(stream_jpeg_buf, fb->buf, fb->len);
              stream_jpeg_len = fb->len;
          }
          xSemaphoreGive(stream_mutex);
      }
      if (xSemaphoreTake(ai_mutex, 0) == pdTRUE) {
          if (!ai_frame_ready && fb->len < JPEG_BUF_MAX) {
              memcpy(ai_jpeg_buf, fb->buf, fb->len);
              ai_jpeg_len = fb->len;
              ai_frame_ready = true;
          }
          xSemaphoreGive(ai_mutex);
      }
      esp_camera_fb_return(fb);
      vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

// ===================== TÁC VỤ 2: XỬ LÝ AI & ĐẨY MQTT (CORE 1) =====================
void camera_ai_task(void *pvParameters) {
  Serial.println("[AI] Task AI đã khởi động trên Core 1!");
  size_t local_len = 0;
  bool detection_latched = false;
  uint8_t no_detection_frames = 0;
  while (1) {
      bool has_frame = false;
      if (xSemaphoreTake(ai_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          if (ai_frame_ready) {
              memcpy(local_jpeg, ai_jpeg_buf, ai_jpeg_len);
              local_len = ai_jpeg_len;
              ai_frame_ready = false;
              has_frame = true;
          }
          xSemaphoreGive(ai_mutex);
      }
      if (!has_frame) {
          vTaskDelay(30 / portTICK_PERIOD_MS);
          continue;
      }

      bool ok = fmt2rgb888(local_jpeg, local_len, PIXFORMAT_JPEG, snapshot_rgb);
      if (!ok) {
          vTaskDelay(30 / portTICK_PERIOD_MS);
          continue;
      }
      ei::image::processing::crop_and_interpolate_rgb888(
          snapshot_rgb,
          320,
          240,
          snapshot_rgb,
          EI_CLASSIFIER_INPUT_WIDTH,
          EI_CLASSIFIER_INPUT_HEIGHT
      );

      signal_t signal;
      signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
      signal.get_data = [](size_t offset, size_t length, float *out_ptr) -> int {
          size_t pixel_ix = offset * 3;
          for (size_t i = 0; i < length; i++) {
              out_ptr[i] = (float)((snapshot_rgb[pixel_ix + 2] << 16) |
                                   (snapshot_rgb[pixel_ix + 1] << 8)  |
                                    snapshot_rgb[pixel_ix]);
              pixel_ix += 3;
          }
          return 0;
      };

      ei_impulse_result_t result = {0};
      EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

      if (res == EI_IMPULSE_OK) {
          bool found = false;
          #if EI_CLASSIFIER_OBJECT_DETECTION == 1
              for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                  auto bb = result.bounding_boxes[ix];
                  if (bb.value >= 0.15f) {
                      Serial.printf(">>> [PHÁT HIỆN]: %s | Độ tin cậy: %.1f%% | Tọa độ: (x: %u, y: %u)\n",
                                    bb.label, bb.value * 100.0f, bb.x, bb.y);

                      if (strcmp(bb.label, "orange") == 0 || strcmp(bb.label, "cam") == 0) {
                          snprintf(ai_result_text, sizeof(ai_result_text), "QUẢ CAM (%.0f%%)", bb.value * 100);
                      } else if (strcmp(bb.label, "lemon") == 0 || strcmp(bb.label, "chanh") == 0) {
                          snprintf(ai_result_text, sizeof(ai_result_text), "QUẢ CHANH (%.0f%%)", bb.value * 100);
                      } else if (strcmp(bb.label, "tomato") == 0 || strcmp(bb.label, "cachua") == 0) {
                          snprintf(ai_result_text, sizeof(ai_result_text), "CÀ CHUA (%.0f%%)", bb.value * 100);
                      } else {
                          snprintf(ai_result_text, sizeof(ai_result_text), "%s (%.0f%%)", bb.label, bb.value * 100);
                      }

                      if (!detection_latched && mqtt_connected) {
                          float area = (float)bb.width * (float)bb.height;
                          mqtt_publish_data(
                              get_class_id_str(bb.label),
                              get_size_id_str(area)
                          );
                          detection_latched = true;
                      }
                      found = true;
                      break;
                  }
              }
          #else
              for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                  if (result.classification[i].value > 0.25f) {
                      const char *label = result.classification[i].label;
                      float val = result.classification[i].value;
                      Serial.printf(">>> [PHÁT HIỆN]: %s (%.1f%%)\n", label, val * 100.0f);
                      snprintf(ai_result_text, sizeof(ai_result_text), "%s (%.0f%%)", label, val * 100);

                      if (!detection_latched && mqtt_connected) {
                          mqtt_publish_data(get_class_id_str(label), "1");
                          detection_latched = true;
                      }

                      found = true;
                      break;
                  }
              }
          #endif
          if (!found) {
              if (no_detection_frames < DETECTION_CLEAR_FRAMES) {
                  no_detection_frames++;
              }
              if (no_detection_frames >= DETECTION_CLEAR_FRAMES) {
                  detection_latched = false;
              }
              Serial.println("... Đang quét: Chưa thấy quả ...");
              snprintf(ai_result_text, sizeof(ai_result_text), "📷 Không phát hiện quả");
          } else {
              no_detection_frames = 0;
          }
      }
      vTaskDelay(150 / portTICK_PERIOD_MS);
  }
}

// ===================== KHỞI TẠO HỆ THỐNG =========================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== HỆ THỐNG PHÂN LOẠI HOA QUẢ ESP32-S3 + MQTT ===");
  if (ESP.getPsramSize() == 0) {
      Serial.println("LỖI: Chưa bật PSRAM! Hãy bật OPI PSRAM.");
      return;
  }
  stream_jpeg_buf = (uint8_t *)ps_malloc(JPEG_BUF_MAX);
  ai_jpeg_buf     = (uint8_t *)ps_malloc(JPEG_BUF_MAX);
  local_jpeg      = (uint8_t *)ps_malloc(JPEG_BUF_MAX);
  snapshot_rgb    = (uint8_t *)ps_malloc(256 * 1024);

  if (!stream_jpeg_buf || !ai_jpeg_buf || !local_jpeg || !snapshot_rgb) {
      Serial.println("LỖI: Cấp phát PSRAM thất bại!");
      return;
  }

  stream_mutex = xSemaphoreCreateMutex();
  ai_mutex     = xSemaphoreCreateMutex();
  if (stream_mutex == NULL || ai_mutex == NULL) {
      Serial.println("LỖI: Tạo mutex thất bại!");
      return;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.printf("\nWiFi OK! IP Web: http://%s\n", WiFi.localIP().toString().c_str());

  mqtt_init();

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM; config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count     = 2;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
      Serial.printf("LỖI KHỞI TẠO CAMERA: 0x%x\n", err);
      return;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 0);
      s->set_contrast(s, 1);
      s->set_saturation(s, 2);
      s->set_whitebal(s, 1);
      s->set_awb_gain(s, 1);
      s->set_exposure_ctrl(s, 1);
  }
  startCameraServer();

  BaseType_t capture_task_result = xTaskCreatePinnedToCore(
      camera_capture_task, "CamTask", 4096, NULL, 2, NULL, 0);
  BaseType_t ai_task_result = xTaskCreatePinnedToCore(
      camera_ai_task, "AI_Task", 32 * 1024, NULL, 1, NULL, 1);
  if (capture_task_result != pdPASS || ai_task_result != pdPASS) {
      Serial.println("LỖI: Không tạo được camera hoặc AI task!");
      return;
  }

  Serial.println(">>> HỆ THỐNG ĐÃ SẴN SÀNG!");
}
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
      mqtt_connected = false;
      Serial.println("[WiFi] Mất kết nối, đang thử kết nối lại...");
      WiFi.reconnect();
  }
  vTaskDelay(pdMS_TO_TICKS(5000));
}



