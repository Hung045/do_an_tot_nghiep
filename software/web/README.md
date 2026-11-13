# Web theo dõi ESP32

Giao diện chính gồm trạng thái xe, mức điều khiển PWM/tay ga, góc nghiêng,
GPS, bản đồ và sự kiện gần nhất. Mở **Chi tiết kỹ thuật** để xem ADC,
relay, bộ lọc, trạng thái module, biểu đồ và JSON MQTT. PWM là duty lệnh,
không phải công suất điện đo được. Trạng thái đèn chưa được gửi qua MQTT.

Bản đồ dùng Leaflet/OpenStreetMap và cần Internet để tải ảnh nền. Nút
**Bám theo xe** cho phép dừng tự dịch chuyển bản đồ để xem vùng khác.
Khi mất định vị hoặc không nhận telemetry cảm biến trong 5 giây, điểm GPS
cũ được làm mờ và có thông báo. Đường nối giữ tối đa 100 vị trí nhận gần
nhất trong phiên mở trang; dao động GPS khi đứng yên cũng có thể tạo đường.
Sau khi cập nhật các file web, nhấn Ctrl+F5 trong trình duyệt; không cần
nạp lại firmware cho thay đổi giao diện này.

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
6. ESP32 tạo dữ liệu trực tiếp mỗi 200 ms (5 lần/giây); Monitor vẫn in mỗi giây.
   INA219 và EEPROM chạy ở task riêng, tối đa khoảng 1 lần/giây. Trạng thái ghi
   EEPROM là kết quả lần ghi gần nhất, không phải xác nhận từng bản tin web.
   Xoay biến trở và so sánh với Monitor. Dashboard
   hiển thị ga/PWM, trạng thái relay, pha điều khiển, thời gian hoạt động và
   biểu đồ 60 mẫu telemetry gần nhất (khoảng 12 giây khi mạng ổn định).
7. Ngắt ESP32 để kiểm tra cảnh báo dữ liệu cũ sau 5 giây; kết nối lại để
   xác nhận dữ liệu tiếp tục cập nhật. Cảnh báo cũng nêu khi lệnh relay/PWM
   không nhất quán với pha điều khiển. Đây chỉ là kiểm tra telemetry, không
   thay thế bảo vệ phần cứng hay xác nhận trạng thái thực tế của relay.
   Ctrl+C dừng HTTP server.

Khi chưa có MPU6050, có thể kiểm tra riêng giao diện sự kiện bằng cách gửi một
bản tin thủ công từ PowerShell:

```powershell
& "<Mosquitto-install-folder>\mosquitto_pub.exe" -h localhost -p 1883 -t "vehicle/event" -m '{"event":"crash_detected","gps_valid":false,"latitude_deg":null,"longitude_deg":null}'
```

Dashboard phải hiển thị "Phát hiện xe ngã", thời điểm nhận và thông báo chưa
có vị trí GPS hợp lệ.

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

## Chẩn đoán GPS

Firmware gửi thêm `gps_status`, `gps_rx_bytes`, `gps_nmea_ok`, `gps_rejected`,
`gps_rx_age_ms` và `gps_nmea_age_ms` trong `vehicle/telemetry`. Xem JSON trong
phần chi tiết để theo dõi bộ đếm. Số byte/bản tin là tổng từ lúc khởi tạo;
tuổi dữ liệu `-1` nghĩa là chưa nhận lần nào.

| Trạng thái | Ý nghĩa và bước kiểm tra |
| --- | --- |
| `INIT_ERROR` | Không khởi tạo được UART |
| `NO_DATA` | Chưa nhận byte: kiểm tra nguồn, GND, TX GPS → D16 |
| `INVALID_NMEA` | Có byte mới nhưng chưa có NMEA đúng checksum trong 5 giây; kiểm tra baud 9600, dây và định dạng đầu ra GPS |
| `NO_NAV` | Có NMEA mới nhưng chưa có GGA/RMC được nhận dạng gần đây |
| `NO_FIX` | Có GGA/RMC mới nhưng chưa có vị trí hợp lệ; thử anten ở nơi thoáng trời |
| `FIX_OK` | Có vị trí hợp lệ chưa quá 5 giây |
| `STALE` | Từng nhận UART nhưng không có byte mới ít nhất 5 giây |
| `UART_ERROR` | Lệnh đọc UART bị lỗi |

Thông báo dữ liệu cũ của toàn dashboard vẫn độc lập với trạng thái GPS:
MQTT đang cập nhật không có nghĩa GPS đã định vị. Firmware cũ không có các
trường chẩn đoán vẫn hiển thị được, nhưng không phân biệt được mất UART và
chưa có fix. Sau khi cập nhật web, nhấn Ctrl+F5.

## Đèn pha và SG90

### Phục hồi MPU và khóa an toàn

`vehicle/motor` có trạng thái `SENSOR_WAIT`: MPU đang lỗi/phục hồi và lệnh động cơ
phải là relay OFF, PWM 0%. Nếu phục hồi ổn định trong 200 ms, hệ thống về `LOCKED`
và yêu cầu đọc được tay ga <=2%, không cần giữ 1 giây. Nếu quá hạn, hệ thống chốt `FAULT_LOCKED`,
ngừng đọc MPU đến khi reset và gửi `fault_reason` để web hiện nguyên nhân.
Trong lúc khóa, `mpu_valid:false` không đồng nghĩa cảm biến đã hỏng; việc đọc
đã tạm dừng có chủ đích. Không bỏ khóa để thử động cơ khi chưa xử lý nguyên nhân.

Khối **Ánh sáng và hướng chiếu** nhận trường `lighting` trong `vehicle/telemetry`
mỗi khoảng 200 ms. Servo vẫn được điều khiển cục bộ theo chu kỳ 20 ms; web chỉ
hiển thị nên không dùng chuyển động trên web để đánh giá độ trễ cơ khí.

- `output_valid`: lần cập nhật PWM đèn thành công và mẫu còn mới.
- `ambient_lux`: độ sáng BH1750; `-1` khi chưa có số đo hợp lệ.
- `duty_pct`: lệnh độ sáng 0–100%; `fault_blink` báo chế độ nhấp nháy cảnh báo.
- `servo_ready`, `servo_tracking`: bộ phát PWM sẵn sàng và đang bám góc MPU.
- `roll_valid`, `roll_deg`: góc nghiêng ngang có dấu và tình trạng mẫu MPU.
- `servo_cmd_deg`, `servo_target_deg`: góc lệnh hiện tại và mục tiêu; không phải
  góc đo được từ servo. Không có phản hồi xác nhận servo đã được cắm hoặc quay.

Sau 5 giây không nhận telemetry, khối này mờ đi và báo dữ liệu cũ. Firmware cũ
không có `lighting` sẽ hiện chưa có dữ liệu, không suy đoán trạng thái đèn.
Sau khi flash và cập nhật web, nhấn **Ctrl+F5**.

### Kiểm tra trên bàn

1. Ngắt nguồn tải động cơ. SG90: tín hiệu GPIO23, nguồn ổn áp riêng khoảng 5 V,
   GND nguồn servo nối chung GND ESP32. Không cấp SG90 bằng 3V3 hoặc pin 3S trực tiếp.
2. Khởi động khi MPU nằm phẳng, chưa gắn tay servo/đèn. Xác nhận lệnh gần 90°,
   sau đó tắt nguồn và lắp tay servo để đèn hướng thẳng. Trục servo đặt đứng
   để đèn quay trái/phải; dùng đèn nhẹ và để dây đủ chùng.
3. Nghiêng MPU lần lượt trái/phải khoảng 15–30°. Góc mục tiêu và góc lệnh phải
   đổi mượt, trong giới hạn mặc định 60–120°. Về phẳng thì trở về giữa.
4. Nếu quay ngược hướng mong muốn, đổi `DEMO_SERVO_DIRECTION` từ `1` thành `-1`
   trong `BoardConfig.h`, build/flash lại. Nếu mặt phẳng lắp bị lệch, chỉnh
   `DEMO_SERVO_ROLL_ZERO_DEG` theo góc roll đo khi xe đứng thẳng.
5. Che BH1750 xuống dưới 30 lux: đèn tăng sáng; chiếu sáng lên trên 60 lux:
   đèn giảm sáng. Khoảng 30–60 lux giữ trạng thái trước để tránh chớp liên tục.
6. Nghiêng trên 60° lâu hơn 200 ms: khóa an toàn, đèn nhấp nháy, servo về giữa.
   Dựng lại, trả ga về 0 và reset để chạy tiếp. Kiểm tra đèn/servo thực tế,
   không chỉ nhìn lệnh trên web. Đèn 3 V vẫn cần nguồn/driver phù hợp riêng;
   GPIO26 chỉ là tín hiệu điều khiển driver đèn.

## Trạng thái INA219

`power_available` cho biết INA219 khởi tạo thành công; `power_valid` cho biết
lần đọc hiện tại thành công và các giá trị hữu hạn. Khi đọc lỗi, firmware gửi
`power_valid:false`, `bus_voltage_v:null`, `current_ma:null`; web hiện
"Lỗi đọc INA219" và không hiển thị số 0 giả. Giá trị 0 V/0 mA đọc thành công
vẫn được hiển thị bình thường, nhưng không xác nhận đường đo tải đã nối đúng.
Số đo dòng còn phụ thuộc cấu hình shunt đúng với module.
Firmware cũ thiếu `power_valid` được ghi rõ là chưa có xác nhận số đo.
