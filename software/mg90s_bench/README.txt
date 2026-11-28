THỬ MG90S VỚI ESP32 — PROJECT ĐỘC LẬP

Chỉ cần ESP32 thường và servo MG90S. Không cần MPU6050 hoặc project xe.
Mở chính thư mục mg90s_bench bằng VS Code (File > Open Folder).
Code thử: main/main.cpp trong thư mục này.

1. Đấu dây khi tắt nguồn
- Dây tín hiệu servo (cam/vàng) -> GPIO14 (D14).
- Dây đỏ -> nguồn 5V ổn áp riêng.
- Dây nâu/đen -> GND nguồn 5V và GND ESP32 (nối chung).
- ESP32 cấp nguồn qua USB. Không cấp servo bằng chân 3V3.
- Thử không gắn đèn/tải; để tay servo quay tự do, đánh dấu vị trí tâm.

2. Nạp
Chọn ESP-IDF và COM của ESP32 thử, chạy Build -> Flash -> Monitor.
Hoặc trong ESP-IDF terminal, tại thư mục mg90s_bench:
  idf.py set-target esp32
  idf.py -p COMxx flash monitor
Thay COMxx bằng cổng thật. Project được viết cho ESP-IDF 5.5.5.

3. Quan sát
Chạy một lần: 90 -> 80 -> 90 -> 100 -> 90 -> 70 -> 110 -> 90.
Bước đầu giữ 5 giây, các bước sau giữ 3 giây sau khi di chuyển.
Lệnh thay đổi 1 độ mỗi 20 ms (50 độ/giây danh định).
Servo phải dịch một góc nhỏ rồi đứng yên ở mỗi HOLD.
Các lần cmd=90 phải đưa tay servo về cùng dấu tâm ban đầu.
Sau DONE vẫn phát xung 1500 us để giữ tâm. Nhấn EN/RESET để lặp lại.
cmd là góc danh định, không phải góc trục được đo. Góc thực cần hiệu chuẩn.
Mapping danh định 0..180 độ -> 1000..2000 us; bài thử chỉ dùng
70..110 độ, tương ứng 1389..1611 us; PWM 50 Hz, độ phân giải 16 bit.
Không tự mở rộng tới 0/180 độ khi chưa xác định giới hạn cơ khí.

4. Kết quả cần gửi
Gửi log STEP/HOLD/DONE và video tay servo: có giữ yên không, các lần 90
có về cùng vị trí không, hai phía có quay ngược chiều nhau không.
Nếu trục quay nhiều vòng, rung mạnh hoặc kẹt: ngắt nguồn servo.
Bài thử này kiểm tra servo/xung/nguồn, chưa kiểm tra bù nghiêng theo xe.
