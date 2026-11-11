"use strict";

const MAX_SAMPLES = 60;
const STALE_AFTER_SECONDS = 5;
const VALID_PHASES = ["LOCKED", "READY", "RELAY_WAIT", "RUNNING", "SENSOR_WAIT", "FAULT_LOCKED"];
const VALID_EVENTS = ["crash_detected", "power_warning", "sensor_fault"];
const PHASE_LABELS = {
  LOCKED: "Chờ về ga 0", READY: "Sẵn sàng", RELAY_WAIT: "Chuẩn bị chạy",
  RUNNING: "Đang chạy", SENSOR_WAIT: "Chờ MPU phục hồi", FAULT_LOCKED: "Khóa an toàn"
};
const samples = [];
const brokerHost = location.hostname || "127.0.0.1";
const brokerUrl = `ws://${brokerHost}:9001`;

const show = (id, value) => {
  document.getElementById(id).textContent = value;
};

let lastReceived = null;
let lastUptime = null;
let lastEvent = null;
let lastSystemReceived = null;
let lastSystemData = null;
let followVehicle = true;
let gpsMap = null;
let gpsMarker = null;
let gpsTrail = null;
let hasCenteredMap = false;
const gpsTrailPoints = [];

function initializeMap() {
  if (typeof L === "undefined") {
    show("map-status", "Không tải được thư viện bản đồ. Kiểm tra Internet rồi tải lại trang.");
    return;
  }

  gpsMap = L.map("gps-map").setView([21.0285, 105.8542], 12);
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: "&copy; OpenStreetMap contributors"
  }).addTo(gpsMap);
  gpsTrail = L.polyline([], {
    color: "#42d6bd",
    weight: 3,
    opacity: 0.75
  }).addTo(gpsMap);
}

function updateGpsFreshness() {
  if (!lastSystemData) return;
  const stale = (performance.now() - lastSystemReceived) / 1000 >= STALE_AFTER_SECONDS;
  document.body.classList.toggle("sensors-stale", stale);
  if (gpsMarker) gpsMarker.setOpacity(stale || !lastSystemData.gps_valid ? 0.45 : 1);
  if (stale) {
    show("lighting-freshness", "Dữ liệu cũ — chờ ESP32");
    show("gps-summary", "Dữ liệu cũ");
    show("gps-status", "Chờ cập nhật từ ESP32");
    if (gpsMap) show("map-status", "Mất cập nhật GPS — điểm trên bản đồ là vị trí nhận trước đó.");
  }
}

function updateMap(data) {
  if (!gpsMap) return;
  if (!data.gps_available || !data.gps_valid) {
    if (gpsMarker) gpsMarker.setOpacity(0.45);
    show("map-status", gpsMarker
      ? "GPS mất định vị — đang giữ vị trí nhận gần nhất."
      : "Đang chờ GPS xác định vị trí xe.");
    return;
  }

  const point = [data.latitude_deg, data.longitude_deg];
  if (!gpsMarker) {
    gpsMarker = L.marker(point).addTo(gpsMap);
  } else {
    gpsMarker.setLatLng(point);
  }
  gpsMarker.setOpacity(1);
  gpsMarker.bindPopup(
    `<strong>Vị trí xe</strong><br>${data.latitude_deg.toFixed(6)}, ${data.longitude_deg.toFixed(6)}` +
    `<br>${data.satellites ?? 0} vệ tinh · ${(data.speed_mps ?? 0).toFixed(2)} m/s`
  );

  const previous = gpsTrailPoints[gpsTrailPoints.length - 1];
  const positionChanged = !previous || previous[0] !== point[0] || previous[1] !== point[1];
  if (positionChanged) {
    gpsTrailPoints.push(point);
    if (gpsTrailPoints.length > 100) gpsTrailPoints.shift();
    gpsTrail.setLatLngs(gpsTrailPoints);
  }

  if (!hasCenteredMap) {
    gpsMap.setView(point, 17);
    hasCenteredMap = true;
  } else if (followVehicle && positionChanged) {
    gpsMap.panTo(point, { animate: true, duration: 0.5 });
  }

  show("map-status",
    `Đã định vị · ${data.satellites ?? 0} vệ tinh`);
  const externalLink = document.getElementById("map-external-link");
  externalLink.href = `https://www.openstreetmap.org/?mlat=${data.latitude_deg}&mlon=${data.longitude_deg}#map=18/${data.latitude_deg}/${data.longitude_deg}`;
  externalLink.hidden = false;
}

function setConnection(message, state) {
  const badge = document.getElementById("connection");
  badge.className = `connection-badge is-${state}`;
  show("connection-text", message);
}

function setAlert(message, level) {
  const panel = document.getElementById("alert-panel");
  if (panel.className === `alert-panel alert-${level}` &&
      document.getElementById("alert-message").textContent === message) {
    return;
  }
  panel.className = `alert-panel alert-${level}`;
  show("alert-message", message);
}

function updateFreshness() {
  updateGpsFreshness();
  const age = lastReceived === null ? null : (performance.now() - lastReceived) / 1000;
  const stale = age === null || age >= STALE_AFTER_SECONDS;
  document.body.classList.toggle("stale", stale);
  const freshness = document.getElementById("freshness");
  freshness.className = `freshness-value ${stale ? "warning" : "good"}`;
  show("freshness", age === null ? "Chưa nhận dữ liệu" : stale
    ? `Dữ liệu cũ · ${Math.floor(age)} giây`
    : `Đang cập nhật · ${Math.floor(age)} giây trước`);

  if (stale && age !== null) {
    const fault = document.body.dataset.phase === "FAULT_LOCKED";
    setAlert(fault ? "Xe đã báo khóa an toàn. Hiện mất cập nhật; kiểm tra ESP32 và kết nối."
      : "Mất dữ liệu động cơ quá 5 giây. Kiểm tra ESP32 và kết nối.", fault ? "danger" : "warning");
  }
}

function updateChart() {
  const plotLeft = 44;
  const plotTop = 20;
  const plotWidth = 666;
  const plotHeight = 180;
  const pointsFor = key => samples.map((sample, index) => {
    const x = plotLeft + (samples.length < 2
      ? plotWidth
      : index * plotWidth / (MAX_SAMPLES - 1));
    const y = plotTop + (100 - sample[key]) * plotHeight / 100;
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(" ");

  document.getElementById("throttle-line").setAttribute("points", pointsFor("throttle_pct"));
  document.getElementById("pwm-line").setAttribute("points", pointsFor("pwm_pct"));
  show("sample-count", `${samples.length} / ${MAX_SAMPLES} mẫu`);
}

function validateTelemetry(data) {
  if (!data || typeof data !== "object" || Array.isArray(data)) return false;
  const percents = [data.throttle_pct, data.filtered_pct, data.pwm_pct];
  return percents.every(value => Number.isFinite(value) && value >= 0 && value <= 100) &&
    Number.isInteger(data.raw) && data.raw >= 0 && data.raw <= 4095 &&
    Number.isInteger(data.uptime_ms) && data.uptime_ms >= 0 &&
    typeof data.relay_cmd === "boolean" &&
    (data.fault_reason === undefined || (typeof data.fault_reason === "string" && data.fault_reason.length <= 160)) &&
    VALID_PHASES.includes(data.phase);
}

function checkCommandState(data) {
  const isOffPhase = data.phase === "LOCKED" || data.phase === "READY" ||
    data.phase === "FAULT_LOCKED" || data.phase === "SENSOR_WAIT";
  const inconsistent =
    (isOffPhase && (data.relay_cmd || data.pwm_pct > 0)) ||
    (data.phase === "RELAY_WAIT" && (!data.relay_cmd || data.pwm_pct > 0)) ||
    (data.phase === "RUNNING" && !data.relay_cmd);

  if (inconsistent) {
    setAlert(
      `Lệnh điều khiển không khớp trạng thái ${data.phase}. Kiểm tra Monitor và phần cứng; dashboard không xác nhận trạng thái thực tế.`,
      "danger"
    );
    return;
  }

  if (lastEvent?.event === "crash_detected" && data.phase === "FAULT_LOCKED") {
    const location = lastEvent.gps_valid
      ? ` tại ${lastEvent.latitude_deg.toFixed(6)}, ${lastEvent.longitude_deg.toFixed(6)}`
      : "; chưa có vị trí GPS hợp lệ";
    setAlert(`Phát hiện xe ngã${location}. Hệ thống đã khóa an toàn.`, "danger");
    return;
  }

  if (data.phase === "FAULT_LOCKED") {
    setAlert(`Khóa an toàn${data.fault_reason ? ": " + data.fault_reason : ""}. Đọc MPU đã tạm dừng; xử lý nguyên nhân rồi reset.`, "danger");
    return;
  }

  if (data.phase === "SENSOR_WAIT") {
    setAlert("MPU đang phục hồi; động cơ đã dừng. Sau phục hồi cần trả ga về 0 để mở khóa lại.", "warning");
    return;
  }

  if (data.phase === "RUNNING") {
    setAlert("Động cơ đang nhận lệnh chạy. Đảm bảo khu vực thử nghiệm an toàn.", "warning");
    return;
  }

  if (data.relay_cmd) {
    setAlert("Relay đang được lệnh bật; đây không phải phản hồi tiếp điểm.", "warning");
    return;
  }

  if (data.phase === "LOCKED") {
    setAlert("Đưa tay ga về 0–2% trong một giây để mở khóa khởi động.", "warning");
    return;
  }

  setAlert("Không phát hiện trạng thái lệnh bất thường.", "success");
}

function formatUptime(milliseconds) {
  const totalSeconds = Math.floor(milliseconds / 1000);
  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;
  if (hours > 0) return `${hours} giờ ${minutes} phút`;
  if (minutes > 0) return `${minutes} phút ${seconds} giây`;
  return `${seconds} giây`;
}

function renderTelemetry(data) {
  if (lastUptime !== null && data.uptime_ms < lastUptime) {
    samples.length = 0;
    lastEvent = null;
    document.getElementById("event-panel").classList.remove("has-event");
    gpsTrailPoints.length = 0;
    if (gpsTrail) gpsTrail.setLatLngs([]);
    show("event-status", "Chưa có sự kiện sau khi khởi động lại");
    show("event-location", "—");
    show("event-time", "—");
  }
  lastUptime = data.uptime_ms;

  show("throttle", `${data.throttle_pct}%`);
  show("filtered", `${data.filtered_pct.toFixed(1)}%`);
  show("pwm", `${data.pwm_pct.toFixed(1)}%`);
  show("relay", data.relay_cmd ? "BẬT" : "TẮT");
  show("phase", PHASE_LABELS[data.phase]);
  document.body.dataset.phase = data.phase;
  show("phase-indicator", data.phase);
  show("raw", data.raw);
  show("uptime", formatUptime(data.uptime_ms));
  document.getElementById("throttle-meter").value = data.throttle_pct;
  document.getElementById("pwm-meter").value = data.pwm_pct;
  show("payload", JSON.stringify(data, null, 2));

  samples.push(data);
  if (samples.length > MAX_SAMPLES) samples.shift();
  updateChart();
  checkCommandState(data);
  lastReceived = performance.now();
  updateFreshness();
}

function validateLighting(data) {
  if (data === undefined) return true; // Older firmware has no lamp telemetry.
  if (!data || typeof data !== "object" || Array.isArray(data)) return false;
  const flags = [data.output_valid, data.fault_blink, data.servo_ready, data.servo_tracking, data.roll_valid];
  return flags.every(value => typeof value === "boolean") &&
    Number.isFinite(data.ambient_lux) && data.ambient_lux >= -1 &&
    Number.isInteger(data.duty_pct) && data.duty_pct >= 0 && data.duty_pct <= 100 &&
    Number.isFinite(data.roll_deg) && Math.abs(data.roll_deg) <= 180 &&
    [data.servo_cmd_deg, data.servo_target_deg].every(value => Number.isFinite(value) && value >= 0 && value <= 180) &&
    (!data.servo_tracking || (data.servo_ready && data.roll_valid));
}

function renderLighting(data) {
  if (!data) {
    show("lighting-freshness", "Firmware chưa gửi dữ liệu đèn");
    for (const id of ["headlight-state", "servo-angle", "roll-angle"]) show(id, "—");
    show("ambient-reading", "Chưa có số đo ánh sáng");
    show("headlight-duty", "Lệnh độ sáng: —");
    show("servo-status", "Chưa có dữ liệu");
    show("servo-target", "Góc mục tiêu: —");
    document.getElementById("servo-meter").value = 90;
    return;
  }
  show("lighting-freshness", "Cập nhật từ ESP32");
  show("headlight-state", !data.output_valid ? "Không có lệnh hợp lệ" : data.fault_blink
    ? "Nhấp nháy cảnh báo" : data.ambient_lux < 0 ? "Bật dự phòng" : data.duty_pct > 0 ? "Bật" : "Tắt");
  show("ambient-reading", data.ambient_lux < 0 ? "BH1750: chưa có số đo hợp lệ" : `Ánh sáng: ${data.ambient_lux.toFixed(1)} lux`);
  show("headlight-duty", data.output_valid ? `Lệnh độ sáng: ${data.duty_pct}%` : "Lệnh độ sáng: không xác nhận");
  show("servo-angle", data.servo_ready ? `${data.servo_cmd_deg.toFixed(1)}°` : "—");
  document.getElementById("servo-meter").value = data.servo_ready ? data.servo_cmd_deg : 90;
  show("servo-status", !data.servo_ready ? "PWM chưa sẵn sàng / lỗi" : data.servo_tracking ? "Bám theo góc nghiêng" : "Giữ / trở về giữa");
  show("servo-target", data.servo_ready ? `Góc mục tiêu: ${data.servo_target_deg.toFixed(1)}°` : "Góc mục tiêu: —");
  show("roll-angle", data.roll_valid ? `${data.roll_deg.toFixed(1)}°` : "—");
}

function validateSystemTelemetry(data) {
  if (!data || typeof data !== "object" || Array.isArray(data)) return false;
  const flags = [data.mpu_available, data.gps_available,
    data.power_available, data.eeprom_available, data.gps_valid];
  const coordinatesValid = !data.gps_valid ||
    (Number.isFinite(data.latitude_deg) && data.latitude_deg >= -90 && data.latitude_deg <= 90 &&
     Number.isFinite(data.longitude_deg) && data.longitude_deg >= -180 && data.longitude_deg <= 180);
  return flags.every(value => typeof value === "boolean") && coordinatesValid && validateLighting(data.lighting) &&
    (data.mpu_valid === undefined || (typeof data.mpu_valid === "boolean" &&
      (!data.mpu_valid || (data.mpu_available && Number.isFinite(data.tilt_deg))))) &&
    (data.eeprom_write_ok === undefined || (typeof data.eeprom_write_ok === "boolean" &&
      (!data.eeprom_write_ok || data.eeprom_available))) &&
    (data.power_valid === undefined || (typeof data.power_valid === "boolean" &&
      (!data.power_valid || (data.power_available && Number.isFinite(data.bus_voltage_v) && Number.isFinite(data.current_ma))))) &&
    (!data.gps_valid || data.gps_available) &&
    (data.satellites == null || (Number.isInteger(data.satellites) && data.satellites >= 0)) &&
    (data.speed_mps == null || (Number.isFinite(data.speed_mps) && data.speed_mps >= 0)) &&
    (data.tilt_deg === null || Number.isFinite(data.tilt_deg)) &&
    (data.bus_voltage_v === null || Number.isFinite(data.bus_voltage_v)) &&
    (data.current_ma === null || Number.isFinite(data.current_ma));
}

function renderSystemTelemetry(data) {
  renderLighting(data.lighting);
  lastSystemReceived = performance.now();
  lastSystemData = data;
  document.body.classList.remove("sensors-stale");
  const mpuValid = data.mpu_available && data.mpu_valid !== false && Number.isFinite(data.tilt_deg);
  show("mpu-status", !data.mpu_available ? "Không phát hiện" : !mpuValid
    ? (document.body.dataset.phase === "FAULT_LOCKED" ? "Tạm dừng đọc do khóa an toàn" : "MPU đang lỗi / phục hồi")
    : data.mpu_valid === undefined ? "Chưa xác nhận số đo (firmware cũ)" : "Đọc dữ liệu thành công");
  show("tilt", mpuValid
    ? `${data.tilt_deg.toFixed(1)}°` : "—");
  const gpsStates = {
    INIT_ERROR: ["LỖI UART", "Không khởi tạo được cổng GPS"],
    NO_DATA: ["CHƯA CÓ DỮ LIỆU", "Kiểm tra nguồn, GND và TX GPS → D16"],
    INVALID_NMEA: ["DỮ LIỆU KHÔNG HỢP LỆ", "Có byte UART; kiểm tra baud 9600 và dây tín hiệu"],
    NO_NAV: ["CHƯA CÓ GGA/RMC", "Nhận NMEA nhưng chưa có bản tin vị trí hợp lệ"],
    NO_FIX: ["CHƯA FIX", "Đang nhận GPS; đặt anten nơi thoáng trời"],
    FIX_OK: [`${data.satellites ?? 0} VỆ TINH`, "Có vị trí hợp lệ"],
    STALE: ["MẤT DỮ LIỆU GPS", "Không nhận byte GPS trong ít nhất 5 giây"],
    UART_ERROR: ["LỖI ĐỌC UART", "ESP32 không đọc được cổng GPS"]
  };
  const gpsState = Object.hasOwn(gpsStates, data.gps_status) ? gpsStates[data.gps_status] : null;
  show("gps-status", gpsState ? gpsState[1] : data.gps_available
    ? "UART đã khởi tạo; chưa có chẩn đoán đường truyền" : "Lỗi khởi tạo UART");
  show("gps-summary", gpsState ? gpsState[0] : !data.gps_available ? "LỖI UART" : data.gps_valid
    ? `${data.satellites ?? 0} VỆ TINH` : "CHƯA FIX");
  show("gps-fix", !data.gps_available ? "—" : data.gps_valid
    ? `${data.latitude_deg.toFixed(6)}, ${data.longitude_deg.toFixed(6)}`
    : "Chưa có vị trí");
  const powerValid = data.power_available && data.power_valid !== false &&
    Number.isFinite(data.bus_voltage_v) && Number.isFinite(data.current_ma);
  show("power-status", !data.power_available ? "Chưa khởi tạo được INA226" : !powerValid
    ? "Lỗi đọc INA226" : data.power_valid === undefined ? "Chưa có xác nhận số đo (firmware cũ)" : "Đọc dữ liệu thành công");
  show("power-reading", powerValid
    ? `${data.bus_voltage_v.toFixed(2)} V / ${data.current_ma.toFixed(0)} mA` : "—");
  show("eeprom-status", !data.eeprom_available ? "Không phát hiện" : data.eeprom_write_ok === false
    ? "Lỗi ghi EEPROM" : data.eeprom_write_ok === true ? "Ghi dữ liệu thành công" : "Chưa xác nhận ghi (firmware cũ)");
  updateMap(data);
  show("payload", JSON.stringify(data, null, 2));
}

function validateEvent(data) {
  if (!data || typeof data !== "object" || Array.isArray(data)) return false;
  if (!VALID_EVENTS.includes(data.event) || typeof data.gps_valid !== "boolean") return false;
  if (!data.gps_valid) {
    return data.latitude_deg === null && data.longitude_deg === null;
  }
  return Number.isFinite(data.latitude_deg) && data.latitude_deg >= -90 && data.latitude_deg <= 90 &&
    Number.isFinite(data.longitude_deg) && data.longitude_deg >= -180 && data.longitude_deg <= 180;
}

function renderEvent(data) {
  const labels = {
    crash_detected: "Phát hiện xe ngã",
    power_warning: "Cảnh báo nguồn",
    sensor_fault: "Lỗi cảm biến"
  };
  lastEvent = data;
  document.getElementById("event-panel").classList.add("has-event");
  show("event-status", labels[data.event]);
  show("event-location", data.gps_valid
    ? `${data.latitude_deg.toFixed(6)}, ${data.longitude_deg.toFixed(6)}`
    : "Chưa có vị trí GPS hợp lệ");
  show("event-time", new Intl.DateTimeFormat("vi-VN", {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit"
  }).format(new Date()));
  show("payload", JSON.stringify(data, null, 2));

  const location = data.gps_valid
    ? ` tại ${data.latitude_deg.toFixed(6)}, ${data.longitude_deg.toFixed(6)}`
    : "; chưa có vị trí GPS hợp lệ";
  const level = data.event === "crash_detected" || data.event === "sensor_fault"
    ? "danger" : "warning";
  setAlert(`${labels[data.event]}${location}.`, level);
}

initializeMap();
document.getElementById("map-follow").addEventListener("click", () => {
  followVehicle = !followVehicle;
  document.getElementById("map-follow").setAttribute("aria-pressed", String(followVehicle));
  show("map-follow", `Bám theo xe: ${followVehicle ? "Bật" : "Tắt"}`);
  if (followVehicle && gpsMarker) gpsMap.panTo(gpsMarker.getLatLng());
});
updateChart();
updateFreshness();
setInterval(updateFreshness, 1000);

if (typeof mqtt === "undefined") {
  setConnection("Không tải được thư viện MQTT · kiểm tra Internet rồi tải lại.", "offline");
  setAlert("Không thể tải thư viện MQTT.js từ CDN.", "danger");
} else {
  setConnection("Đang kết nối broker…", "connecting");
  show("broker-address", `${brokerHost}:9001`);
  const client = mqtt.connect(brokerUrl, {
    reconnectPeriod: 2000,
    connectTimeout: 10000,
    clean: true
  });

  client.on("connect", () => {
    setConnection("Đã nối broker · đang đăng ký topic…", "connecting");
    client.subscribe("vehicle/#", (error, granted) => {
      if (error || !granted?.length || granted[0].qos === 128) {
        setConnection("Broker đã nối nhưng không đăng ký được topic.", "offline");
        setAlert("Không nhận được quyền đọc vehicle/# từ broker.", "danger");
        return;
      }
      setConnection("Broker đã kết nối · đang nghe telemetry", "online");
    });
  });

  client.on("reconnect", () => setConnection("Đang kết nối lại broker…", "connecting"));
  client.on("offline", () => setConnection("Broker ngoại tuyến · đang thử kết nối lại…", "offline"));
  client.on("close", () => setConnection("Mất kết nối broker · đang thử lại…", "offline"));
  client.on("error", error => {
    setConnection(`Lỗi MQTT · ${error.message}`, "offline");
  });

  client.on("message", (topic, message) => {
    let data;
    try {
      data = JSON.parse(message.toString());
    } catch (error) {
      setAlert("Không thể đọc bản tin telemetry JSON.", "danger");
      return;
    }
    if (topic === "vehicle/motor") {
      if (!validateTelemetry(data)) {
        setAlert("Đã bỏ qua dữ liệu động cơ sai định dạng hoặc nằm ngoài giới hạn.", "danger");
        return;
      }
      renderTelemetry(data);
    } else if (topic === "vehicle/telemetry") {
      if (!validateSystemTelemetry(data)) {
        setAlert("Đã bỏ qua telemetry cảm biến sai định dạng.", "danger");
        return;
      }
      renderSystemTelemetry(data);
    } else if (topic === "vehicle/event") {
      if (!validateEvent(data)) {
        setAlert("Đã bỏ qua sự kiện sai định dạng.", "danger");
        return;
      }
      renderEvent(data);
    }
  });
}
