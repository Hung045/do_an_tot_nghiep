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
- `power_monitor`, `gps_tracker`, `data_logger`: có driver khởi đầu cho
  INA226, NEO-6M và AT24C256, được lấy mẫu/ghi vòng 1 Hz.
- `telemetry_link`: vẫn là API khung; Wi-Fi/MQTT/dashboard chưa triển khai.
- `project_types`: kiểu dữ liệu telemetry và sự kiện dùng chung.

Build đã được kiểm tra với ESP-IDF 5.3.3. Các driver cần được thử trên module
thực; INA226 mặc định giả định shunt R002/20 A và giá trị này phải khớp phần
cứng đã mua. Cần kiểm tra lại sơ đồ chân, điện áp logic và cực tính relay trước
khi cấp nguồn tải.

## Phân chia phát triển gợi ý

- **Điều khiển phần cứng:** `do_an_tn/components/motor_controller`
- **Nguồn/năng lượng:** xác nhận shunt INA226 và hiệu chuẩn đo; xây dựng mô hình
  ước lượng pin 3S đã hiệu chuẩn
- **Định vị:** kiểm thử UART/NMEA ngoài trời, xác nhận tọa độ và dữ liệu stale
- **Hộp đen:** kiểm thử EEPROM sau mất nguồn và kiểm tra vòng ghi trên module thật
- **Kết nối:** hoàn thiện `telemetry_link` với MQTT/dashboard; lỗi mạng không
  được vô hiệu hóa an toàn cục bộ

Khi thêm component, đặt dưới `do_an_tn/components/`, cập nhật `CMakeLists.txt`
và README của module; không commit `build/`, `sdkconfig` cá nhân, binary hay
thông tin Wi-Fi/token. Thay đổi pin phải đi cùng cập nhật bảng chân và kiểm tra
an toàn.

README trong `do_an_tn/` có bảng phân công file theo component và các việc
cần hoàn thiện cho từng module.
