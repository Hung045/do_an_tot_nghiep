## 1. Hệ thống cần làm gì?

| Phần | Hành vi hiện có trong code |
|---|---|
| Động cơ | Điều khiển PWM 0–100% theo biến trở, qua relay và LR7843 |
| Khởi động | Giữ đầu ra tắt đến khi đọc được ga ≤2%; không cần giữ 1 giây |
| Chạy/dừng | Ga và ga sau lọc ≥5% thì đóng relay, chờ 100 ms rồi chạy; ga ≤2% thì ra lệnh PWM 0 và relay OFF trong vòng xử lý đó |
| Phát hiện ngã | MPU nghiêng ≥60° liên tục 200 ms → khóa động cơ và cảnh báo |
| MPU lỗi | Dừng ngay; cần 3 mẫu tốt trải trên ≥40 ms trong hạn 200 ms để phục hồi; yêu cầu ga về 0 trước khi chạy lại |
| Khóa lỗi | Lỗi MPU kéo dài, thiếu MPU bắt buộc, lỗi ADC/đầu ra hoặc ngã → `FAULT_LOCKED`; sửa nguyên nhân rồi reset |
| Đèn | Dưới 30 lux bật, trên 60 lux tắt, ở giữa giữ trạng thái; mất BH1750 thì bật dự phòng; khóa lỗi thì nhấp nháy |
| Servo | Theo roll MPU; tâm 90°, giới hạn 60–120°; mất dữ liệu hoặc khóa lỗi thì hướng về giữa |
| GPS | Đọc vị trí, hiển thị bản đồ; phân biệt không có UART, chưa có fix và dữ liệu cũ |
| INA226 | Đo điện áp/dòng với shunt R020; chưa tính sức khỏe pin hoặc ngắt quá dòng |
| EEPROM | Lưu vòng 512 bản ghi có CRC, đọc lại sau reset |
| MQTT/web | Theo dõi trạng thái và cảnh báo; web không điều khiển xe; mất mạng không chặn task điều khiển |

Trạng thái xe: `LOCKED → READY → RELAY_WAIT → RUNNING`.
Trả ga về 0 → `READY`; MPU đang phục hồi → `SENSOR_WAIT`; lỗi chốt → `FAULT_LOCKED`.
Ga về 0 không tự xóa `FAULT_LOCKED`. Relay/PWM/góc servo trên web là **lệnh**, không phải phản hồi phần cứng.

## 2. Thông số cần giữ thống nhất

| Hạng mục | Giá trị hiện tại |
|---|---|
| Biến trở / motor / relay | GPIO34 / GPIO25 / GPIO27; relay kích mức thấp |
| Đèn / SG90 | GPIO26 / GPIO23 |
| MPU6050 | SDA21, SCL22; địa chỉ 0x68 |
| BH1750 / INA226 / EEPROM | Chung SDA18, SCL19; địa chỉ 0x23 hoặc 0x5C / 0x40 / 0x50 |
| GPS | TX GPS → GPIO16; RX GPS ← GPIO17; 9600 baud |
| Điều khiển / dữ liệu trực tiếp | Khoảng 20 ms / 200 ms (5 Hz), thêm bản tin khi trạng thái đổi |
| INA226 + EEPROM / Monitor | Task lưu trữ tối đa khoảng 1 Hz / log khoảng 1 Hz |
| MQTT | `vehicle/motor`, `vehicle/telemetry`: QoS 0; `vehicle/event`: QoS 1; không retained |
| Dữ liệu cũ | Web cảnh báo sau 5 giây không nhận; số đo nguồn quá 2,5 giây không còn hợp lệ |
| Build | ESP32, ESP-IDF 5.5.5; kit hiện flash 4 MB, phân vùng ứng dụng 1 MB |

200 ms là chu kỳ tạo mẫu, **không bảo đảm web nhận trong 200 ms**.
Đơn vị chung: góc °, điện áp V, dòng mA, tốc độ m/s, thời gian ms.
Dùng cờ hợp lệ và `null` cho số đo không hợp lệ theo schema hiện có; lux -1 là chưa biết.
Trạng thái ghi EEPROM là kết quả hoàn tất gần nhất, không xác nhận từng bản tin web đã lưu.

Chi tiết: [đấu dây và build](README.md), [chạy web](web/README.md),
[GPIO và hiệu chỉnh](components/board_config/include/BoardConfig.h),
[JSON MQTT](components/telemetry_link/TelemetryLink.cpp).
Không đưa mật khẩu hoặc IP laptop cố định vào SPEC.

## 3. Kiểm thử trước khi coi là hoàn thành

Các ô dưới đây **chưa xác nhận đạt toàn bộ trên phiên bản hiện tại**.

- [ ] Reset khi ga cao: động cơ không tự chạy; về 0 rồi tăng ga mới chạy.
- [ ] Trả ga ≤2%: PWM về 0, relay thực tế ngắt được tải.
- [ ] MPU lỗi/ngã: dừng động cơ, đèn cảnh báo, servo hướng giữa, sự kiện được gửi khi có mạng.
- [ ] Thay đổi ánh sáng và nghiêng MPU: đèn/servo hoạt động đúng, không kẹt cơ khí.
- [ ] GPS có fix, mất fix, mất UART: Monitor và web phân biệt đúng.
- [ ] INA226 được đối chiếu với đồng hồ; EEPROM ghi và đọc lại đúng sau reset.
- [ ] Mất mạng/broker: điều khiển cục bộ tiếp tục, web báo cũ và cập nhật lại khi kết nối phục hồi.
- [ ] Chạy cùng động cơ/servo: kiểm tra reset, timeout và độ trễ web; lưu log để đối chiếu.

Mỗi lần thử ghi ngày, phiên bản firmware, phần cứng và kết quả.
Tắt nguồn trước khi đổi dây; không tạo lỗi bằng cách chập SDA/SCL.

## 4. Tình trạng và việc còn lại

**Đã từng quan sát:** motor chạy theo ga; MPU/BH1750 đọc được; GPS có FIX;
web nhận dữ liệu. Đây chưa phải nghiệm thu độ ổn định toàn hệ thống.

**Chưa giải quyết/kiểm chứng đầy đủ:** timeout MQTT, I2C lỗi khi tích hợp tải,
INA226 gây lỗi bus và EEPROM báo MISSING trong các lần thử gần đây.
Chưa có phản hồi relay/servo, đo RPM, bảo vệ quá dòng, SOC/SOH hoặc OTA.

**Đề xuất, chưa coi là đã triển khai:**

1. Thêm lỗi MQTT chi tiết, RSSI, reconnect và thời gian gửi.
2. Tối ưu chu kỳ gửi dữ liệu nhanh/chậm sau khi đo độ trễ.
3. Công bố INA226 trước khi chờ EEPROM; phân biệt ghi đang chờ/thành công/lỗi/cũ.
4. Nhấp nháy đèn theo thời gian thực thay vì đếm lần gọi.
5. Tự thử lại cảm biến bus phụ với khoảng nghỉ tăng dần.
6. Thêm test tích hợp rồi tách nhỏ `main.cpp`.

## 5. Phân công và cập nhật

- Thành viên 2: MPU, GPS, EEPROM.
- Bạn: tích hợp main, động cơ, đèn/servo, MQTT/web. Phụ trách INA226 cần nhóm xác nhận.
- Đổi hành vi, GPIO hoặc dữ liệu chung: thống nhất → ghi đề xuất trong SPEC → sửa code → kiểm thử → cập nhật kết quả.
- Sửa bug để đúng SPEC: sửa code và kiểm thử, không cần đổi yêu cầu.
- Không ghi “đã kiểm thử” chỉ vì build thành công. Commit code và tài liệu liên quan cùng nhau.
