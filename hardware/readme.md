# Phần cứng hệ thống xe điện

Thiết kế mạch nguyên lý và PCB bằng Altium Designer, sử dụng ESP32 DOIT DevKit V1 để điều khiển động cơ, đèn, servo MG90S và đọc các cảm biến.

## File thiết kế

- [Project Altium](ic_xe_dien/ic_xe_dien.PrjPcb)
- [Schematic](ic_xe_dien/ic_xe_dien.SchDoc)
- [PCB](ic_xe_dien/PCB2.PcbDoc)

## Thành phần chính

ESP32, biến trở mô phỏng tay ga, module LR7843, relay 5 V, đèn LED, servo MG90S, MPU6050, BH1750, INA219, EEPROM AT24C256 và GPS NEO-6M.

## Sơ đồ chân theo firmware

| Chức năng | GPIO ESP32 |
|---|---|
| Tay ga | 32 — ADC1 channel 4 |
| PWM động cơ | 33 |
| PWM đèn | 25 |
| Servo MG90S | 26 |
| Relay | 27 |
| MPU6050 SDA / SCL | 21 / 22 |
| BH1750, EEPROM, INA219 SDA / SCL | 18 / 19 |
| GPS: ESP32 TX → GPS RX | 17 |
| GPS: ESP32 RX ← GPS TX | 16 |

Đối chiếu bảng này với schematic và PCB trước khi lắp. Cấu hình nằm trong [BoardConfig.h](../software/components/board_config/include/BoardConfig.h).

## Nguồn và hoạt động

- Dùng nguồn hạ áp 5 V từ pin 12 V danh định; các khối nối chung GND. Cấp nguồn từng module theo đúng định mức.
- Servo dùng 5 V, không cấp từ chân 3V3. Biến trở lấy nguồn 3V3.
- Động cơ điều chỉnh theo tay ga; relay đóng/ngắt nguồn tải. Kiểm tra cực tính điều khiển relay qua transistor trước khi nối động cơ.
- Đèn tự bật dưới 30 lux, tắt trên 60 lux. LED 1 W cần tầng cấp dòng phù hợp, không lấy công suất trực tiếp từ GPIO.
- Servo bù nghiêng ngang theo tỷ lệ 1:1, tâm 90°, giới hạn lệnh 50–130°. Dựng xe thẳng thì trở về tâm; góc thực tế cần hiệu chỉnh khi lắp.

## Tiến độ

- Đã hoàn thành schematic và bố trí/đi dây PCB.
- MG90S đã chạy đúng chuỗi lệnh trong bài thử riêng.
- Mạch thử tích hợp đã đọc được MPU6050, BH1750, EEPROM, INA219 và điều khiển các đầu ra.
- GPS đang báo `NO_FIX`, chưa xác nhận định vị thành công.
- Chưa ghi nhận kết quả ERC/DRC và kiểm thử PCB sau gia công.

Trước khi đặt mạch: chạy ERC/DRC, kiểm tra footprint, chân module và đường nguồn. Khi nhận bo: kiểm tra chập nguồn, đo 5 V/3V3 rồi thử từng khối trước khi chạy toàn hệ thống.

Firmware và hướng dẫn nạp: [software/README.md](../software/README.md).
