#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// =====================================================================
// THƯ VIỆN NHẬN DIỆN HOA QUẢ EDGE IMPULSE
#include <fruit_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
// =====================================================================

// ======================= THÔNG TIN WIFI ==============================
const char* ssid = "ROBOTIC";         // <--- ĐIỀN TÊN WIFI CỦA BẠN
const char* password = "KTRB&HTTM";    // <--- ĐIỀN MẬT KHẨU WIFI
// =====================================================================

// Pinout chuẩn ESP32-S3 WROOM CAM
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

// Bộ đệm ảnh JPEG chất lượng cao trong PSRAM
#define JPEG_BUF_MAX (40 * 1024)
static uint8_t *stream_jpeg_buf = NULL;
static size_t   stream_jpeg_len = 0;
static SemaphoreHandle_t stream_mutex;

static uint8_t *ai_jpeg_buf = NULL;
static size_t   ai_jpeg_len = 0;
static bool     ai_frame_ready = false;
static SemaphoreHandle_t ai_mutex;

static uint8_t *snapshot_rgb = NULL; // 320x240 RGB888 trong PSRAM

httpd_handle_t camera_httpd = NULL;
char ai_result_text[64] = "📷 Đang quét...";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ===================== GIAO DIỆN WEB HIỂN THỊ CAMERA NÉT =========================
static esp_err_t index_handler(httpd_req_t *req) {
    const char* html = 
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
      "<h1>🍊 AI Phân Loại Hoa Quả (Camera Nét HD)</h1>"
      "<div id='cam-box'><img id='stream' src='/stream'></div>"
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
      "</body></html>";
    httpd_resp_send(req, html, strlen(html));
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
        vTaskDelay(35 / portTICK_PERIOD_MS); // Giữ tốc độ stream mượt mà
    }
    return res;
}

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_open_sockets = 5;
    config.stack_size = 8192;
    
    httpd_uri_t index_uri  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = NULL };
    httpd_uri_t stream_uri = { .uri = "/stream",  .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
    httpd_uri_t result_uri = { .uri = "/result",  .method = HTTP_GET, .handler = result_handler, .user_ctx = NULL };
    
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &stream_uri);
        httpd_register_uri_handler(camera_httpd, &result_uri);
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

        // 1. Sao chép khung hình JPEG chất lượng cao cho Web Stream
        if (xSemaphoreTake(stream_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (fb->len < JPEG_BUF_MAX) {
                memcpy(stream_jpeg_buf, fb->buf, fb->len);
                stream_jpeg_len = fb->len;
            }
            xSemaphoreGive(stream_mutex);
        }

        // 2. Chuyển frame cho Task AI nếu AI đang rảnh
        if (xSemaphoreTake(ai_mutex, 0) == pdTRUE) {
            if (!ai_frame_ready && fb->len < JPEG_BUF_MAX) {
                memcpy(ai_jpeg_buf, fb->buf, fb->len);
                ai_jpeg_len = fb->len;
                ai_frame_ready = true;
            }
            xSemaphoreGive(ai_mutex);
        }

        // Trả buffer ngay lập tức để không bao giờ bị nghẽn camera
        esp_camera_fb_return(fb);

        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

// ===================== TÁC VỤ 2: XỬ LÝ AI & IN LOG VS CODE (CORE 1) =====================
void camera_ai_task(void *pvParameters) {
    Serial.println("[AI] Task AI đã khởi động trên Core 1!");

    static uint8_t local_jpeg[JPEG_BUF_MAX];
    size_t local_len = 0;

    while (1) {
        // Chờ nhận frame mới từ Camera
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

        // 1. Giải mã JPEG 320x240 sang RGB888 trong PSRAM
        bool ok = fmt2rgb888(local_jpeg, local_len, PIXFORMAT_JPEG, snapshot_rgb);
        if (!ok) {
            vTaskDelay(30 / portTICK_PERIOD_MS);
            continue;
        }

        // 2. Thu nhỏ ảnh từ 320x240 về 96x96 (vùng trung tâm) cho mô hình
        ei::image::processing::crop_and_interpolate_rgb888(
            snapshot_rgb,
            320,
            240,
            snapshot_rgb,
            EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT
        );

        // 3. Chuẩn bị tín hiệu ảnh và chuyển hệ màu BGR -> RGB chuẩn
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

        // 4. Chạy mô hình Edge Impulse
        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res == EI_IMPULSE_OK) {
            bool found = false;

            #if EI_CLASSIFIER_OBJECT_DETECTION == 1
                for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                    auto bb = result.bounding_boxes[ix];
                    if (bb.value >= 0.15) { // Ngưỡng nhận diện nhạy 15%
                        // IN TRỰC TIẾP LÊN TERMINAL / SERIAL CỦA VISUAL STUDIO CODE
                        Serial.printf(">>> [PHÁT HIỆN]: %s | Độ tin cậy: %.1f%% | Tọa độ: (x: %u, y: %u)\n",
                                      bb.label, bb.value * 100.0, bb.x, bb.y);
                        
                        if (strcmp(bb.label, "orange") == 0 || strcmp(bb.label, "cam") == 0) {
                            snprintf(ai_result_text, sizeof(ai_result_text), "🍊 QUẢ CAM (%.0f%%)", bb.value * 100);
                        } else if (strcmp(bb.label, "lemon") == 0 || strcmp(bb.label, "chanh") == 0) {
                            snprintf(ai_result_text, sizeof(ai_result_text), "🍋 QUẢ CHANH (%.0f%%)", bb.value * 100);
                        } else if (strcmp(bb.label, "tomato") == 0 || strcmp(bb.label, "cachua") == 0) {
                            snprintf(ai_result_text, sizeof(ai_result_text), "🍅 CÀ CHUA (%.0f%%)", bb.value * 100);
                        } else {
                            snprintf(ai_result_text, sizeof(ai_result_text), "📦 %s (%.0f%%)", bb.label, bb.value * 100);
                        }
                        found = true;
                        break;
                    }
                }
            #else
                for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                    if (result.classification[i].value > 0.25) {
                        Serial.printf(">>> [PHÁT HIỆN]: %s (%.1f%%)\n", 
                                      result.classification[i].label, result.classification[i].value * 100.0);
                        found = true;
                    }
                }
            #endif

            if (!found) {
                Serial.println("... Đang quét: Chưa thấy quả ...");
                snprintf(ai_result_text, sizeof(ai_result_text), "📷 Không phát hiện quả");
            }
        }

        vTaskDelay(200 / portTICK_PERIOD_MS); // Quét 4-5 lần mỗi giây
    }
}

// ===================== KHỞI TẠO HỆ THỐNG =========================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== HỆ THỐNG PHÂN LOẠI HOA QUẢ ESP32-S3 ===");

    if (ESP.getPsramSize() == 0) { 
        Serial.println("LỖI: Chưa bật PSRAM! Hãy bật OPI PSRAM."); 
        return; 
    }

    // Cấp phát bộ nhớ đệm trong PSRAM
    stream_jpeg_buf = (uint8_t *)ps_malloc(JPEG_BUF_MAX);
    ai_jpeg_buf     = (uint8_t *)ps_malloc(JPEG_BUF_MAX);
    snapshot_rgb    = (uint8_t *)ps_malloc(256 * 1024); // Đủ chứa 320x240 RGB888

    stream_mutex = xSemaphoreCreateMutex();
    ai_mutex     = xSemaphoreCreateMutex();

    // Kết nối WiFi
    WiFi.begin(ssid, password);
    Serial.print("Đang kết nối WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.printf("\nWiFi OK! IP Web: http://%s\n", WiFi.localIP().toString().c_str());

    // CẤU HÌNH CAMERA: CHUẨN QVGA (320x240) JPEG NÉT CĂNG
    camera_config_t config;
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
    config.pixel_format = PIXFORMAT_JPEG;   // Nén JPEG phần cứng trực tiếp
    config.frame_size   = FRAMESIZE_QVGA;    // 320x240 - Nét căng chi tiết, không vỡ hạt!
    config.jpeg_quality = 10;               // Chất lượng ảnh cao
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("LỖI KHỞI TẠO CAMERA: 0x%x\n", err);
        return;
    }

    // Tinh chỉnh màu sắc cảm biến OV3660 để quả lên màu rực rỡ như thật
    sensor_t * s = esp_camera_sensor_get();
    if (s != NULL) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 0);     
        s->set_contrast(s, 1);       
        s->set_saturation(s, 2);     // Đẩy màu cam/vàng lên đậm đà
        s->set_whitebal(s, 1);       
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);  
    }

    startCameraServer();

    // Task 1: Chụp ảnh liên tục đưa vào luồng (Core 0)
    xTaskCreatePinnedToCore(camera_capture_task, "CamTask", 4096, NULL, 2, NULL, 0);

    // Task 2: Chạy nhận diện AI và in kết quả ra VS Code (Core 1)
    xTaskCreatePinnedToCore(camera_ai_task, "AI_Task", 32 * 1024, NULL, 1, NULL, 1);

    Serial.println(">>> HỆ THỐNG ĐÃ SẴN SÀNG! KẾT QUẢ SẼ IN LIÊN TỤC Ở DƯỚI:");
}

void loop() { 
    delay(10000); 
}