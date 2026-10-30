# Web theo dõi ESP32

Giao diện chỉ đọc dữ liệu thật từ `vehicle/motor`, `vehicle/telemetry` và
`vehicle/event`; không gửi lệnh điều
khiển. Laptop chạy Mosquitto và web server; laptop và điện thoại có thể cùng
mở dashboard khi kết nối chung một mạng Wi-Fi/LAN.

1. Cấu hình Mosquitto để nhận kết nối trong mạng nội bộ. Ví dụ trong file
   cấu hình Mosquitto:

   ```text
   listener 1883 0.0.0.0
   protocol mqtt
   listener 9001 0.0.0.0
   protocol websockets
   allow_anonymous true
   ```

   `allow_anonymous true` chỉ phù hợp cho mạng thử nghiệm tin cậy. Khởi động
   lại Mosquitto sau khi sửa cấu hình.
2. Tại thư mục gốc `software`, chạy:

   ```powershell
   python -m http.server 8000 --bind 0.0.0.0 --directory web
   ```

3. Tìm địa chỉ IPv4 của laptop bằng `ipconfig`. Nếu Windows Firewall hỏi,
   chỉ cho phép Python/Mosquitto trên mạng Private tin cậy.
4. Trên laptop, mở `http://127.0.0.1:8000`. Trên điện thoại cùng Wi-Fi, mở
   `http://<IPv4-của-laptop>:8000`, ví dụ `http://192.168.1.20:8000`.
   Dashboard tự kết nối WebSocket tới cùng địa chỉ laptop ở cổng 9001.
5. Đặt MQTT broker URI của firmware thành `mqtt://<IPv4-của-laptop>:1883`
   trong menuconfig, rồi build và flash lại để ESP32 gửi dữ liệu qua broker
   trong mạng LAN.
6. ESP32 gửi dữ liệu mỗi giây. Xoay biến trở và so sánh với Monitor. Dashboard
   hiển thị ga/PWM, trạng thái relay, pha điều khiển, thời gian hoạt động và
   biểu đồ 60 mẫu telemetry gần nhất.
7. Ngắt ESP32 để kiểm tra cảnh báo dữ liệu cũ sau 5 giây; kết nối lại để
   xác nhận dữ liệu tiếp tục cập nhật. Cảnh báo cũng nêu khi lệnh relay/PWM
   không nhất quán với pha điều khiển. Đây chỉ là kiểm tra telemetry, không
   thay thế bảo vệ phần cứng hay xác nhận trạng thái thực tế của relay.
   Ctrl+C dừng HTTP server.

MQTT.js 5.10.4 được tải từ CDN unpkg; laptop cần Internet khi tải trang.
Nếu không tải được thư viện, trang hiển thị thông báo lỗi. Cổng 8000 phục
vụ HTML, cổng 1883 nhận MQTT từ ESP32, cổng 9001 nhận MQTT qua WebSocket từ
các trình duyệt. Nếu điện thoại không kết nối được, kiểm tra cả hai thiết bị
cùng mạng, router không bật client/AP isolation, Mosquitto đang lắng nghe
trên LAN và firewall cho phép các cổng trên mạng Private.

Chỉ dùng cấu hình anonymous trên mạng nội bộ tin cậy. Không cấu hình
port-forwarding hoặc mở các cổng này ra Internet; để truy cập từ ngoài nhà
cần một giải pháp có xác thực và mã hóa riêng.

Giá trị relay và PWM là lệnh phần mềm, không phải phản hồi phần cứng hay RPM.
Cảnh báo dữ liệu cũ dựa trên thời gian trình duyệt nhận bản tin hợp lệ gần
nhất; `uptime_ms` là thời gian chạy của ESP32, không phải giờ trên laptop.
