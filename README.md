# Đồ án tốt nghiệp — hộp đen xe điện

Firmware ESP-IDF cho sa bàn giám sát và cảnh báo xe điện, dùng ESP32 DOIT
DevKit V1. Project firmware nằm trong [`do_an_tn/`](./do_an_tn/).

## Bắt đầu nhanh

1. Cài ESP-IDF 5.3.x và extension ESP-IDF cho VS Code.
2. Mở riêng thư mục `do_an_tn` làm workspace ESP-IDF.
3. Chọn target ESP32, build; xem pin mặc định và cảnh báo an toàn trong
   [`do_an_tn/README.md`](./do_an_tn/README.md).
4. Tạo nhánh riêng cho mỗi tính năng, mở pull request để cộng sự review trước
   khi gộp vào `main`.

## Trạng thái

Hiện là **khung prototype theo component**: demo ADC/PWM, MPU6050 và khóa
relay đã có. Cấu trúc firmware nằm trong `do_an_tn/components/`:

- `crash_detector`: đo góc MPU6050 và phát hiện nghiêng kéo dài.
- `motor_controller`: đọc tay ga, xuất PWM motor và điều khiển relay.
- `headlight_controller`: xuất PWM đèn.
- `power_monitor`, `gps_tracker`, `data_logger`, `telemetry_link`: có API khung;
  phần cứng INA226/GPS/EEPROM/MQTT chưa được triển khai.
- `project_types`: kiểu dữ liệu telemetry và sự kiện dùng chung.

GPS, circular logging và IoT chưa được triển khai. Cần xác nhận build trong
môi trường ESP-IDF của từng cộng sự trước khi thử phần cứng.

## Phân chia phát triển gợi ý

- **Điều khiển phần cứng:** `do_an_tn/components/motor_controller`
- **Nguồn/năng lượng:** hoàn thiện `power_monitor` với INA226 và tính toán pin
- **Định vị:** hoàn thiện `gps_tracker` với UART/NEO-6M
- **Hộp đen:** hoàn thiện `data_logger` với EEPROM, định dạng bản ghi và vòng ghi
- **Kết nối:** hoàn thiện `telemetry_link` với MQTT/dashboard; lỗi mạng không
  được vô hiệu hóa an toàn cục bộ

Khi thêm component, đặt dưới `do_an_tn/components/`, cập nhật `CMakeLists.txt`
và README của module; không commit `build/`, `sdkconfig` cá nhân, binary hay
thông tin Wi-Fi/token. Thay đổi pin phải đi cùng cập nhật bảng chân và kiểm tra
an toàn.

README trong `do_an_tn/` có bảng phân công file theo component và các việc
cần hoàn thiện cho từng module.
