"use strict";

const MAX_SAMPLES = 60;
const STALE_AFTER_SECONDS = 5;
const VALID_PHASES = ["LOCKED", "READY", "RELAY_WAIT", "RUNNING", "FAULT_LOCKED"];
const samples = [];
const brokerHost = location.hostname || "127.0.0.1";
const brokerUrl = `ws://${brokerHost}:9001`;

const show = (id, value) => {
  document.getElementById(id).textContent = value;
};

let lastReceived = null;
let lastUptime = null;

function setConnection(message, state) {
  const badge = document.getElementById("connection");
  badge.className = `connection-badge is-${state}`;
  show("connection", message);
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
  const age = lastReceived === null ? null : (performance.now() - lastReceived) / 1000;
  const stale = age === null || age >= STALE_AFTER_SECONDS;
  document.body.classList.toggle("stale", stale);
  const freshness = document.getElementById("freshness");
  freshness.className = `freshness-value ${stale ? "warning" : "good"}`;
  show("freshness", age === null ? "Chưa nhận dữ liệu" : stale
    ? `Dữ liệu cũ · ${Math.floor(age)} giây`
    : `Đang cập nhật · ${Math.floor(age)} giây trước`);

  if (stale && age !== null) {
    setAlert("Mất telemetry quá 5 giây. Kiểm tra ESP32, Wi-Fi và kết nối MQTT.", "warning");
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
    VALID_PHASES.includes(data.phase);
}

function checkCommandState(data) {
  const isOffPhase = data.phase === "LOCKED" || data.phase === "READY" ||
    data.phase === "FAULT_LOCKED";
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

  if (data.phase === "FAULT_LOCKED") {
    setAlert("Hệ thống đang khóa an toàn. Xem Monitor để biết nguyên nhân và reset sau khi đã xử lý lỗi.", "danger");
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
  }
  lastUptime = data.uptime_ms;

  show("throttle", `${data.throttle_pct}%`);
  show("filtered", `${data.filtered_pct.toFixed(1)}%`);
  show("pwm", `${data.pwm_pct.toFixed(1)}%`);
  show("relay", data.relay_cmd ? "BẬT" : "TẮT");
  show("phase", data.phase.replace("_", " "));
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

function validateSystemTelemetry(data) {
  if (!data || typeof data !== "object" || Array.isArray(data)) return false;
  const flags = [data.mpu_available, data.gps_available,
    data.power_available, data.eeprom_available, data.gps_valid];
  return flags.every(value => typeof value === "boolean") &&
    (data.tilt_deg === null || Number.isFinite(data.tilt_deg)) &&
    (data.bus_voltage_v === null || Number.isFinite(data.bus_voltage_v)) &&
    (data.current_ma === null || Number.isFinite(data.current_ma));
}

function renderSystemTelemetry(data) {
  show("mpu-status", data.mpu_available ? "Đã kết nối" : "Không phát hiện");
  show("tilt", data.mpu_available && data.tilt_deg !== null
    ? `${data.tilt_deg.toFixed(1)}°` : "—");
  show("gps-status", data.gps_available ? "Đã khởi tạo" : "Không phát hiện");
  show("gps-fix", !data.gps_available ? "—" : data.gps_valid
    ? `${data.latitude_deg.toFixed(6)}, ${data.longitude_deg.toFixed(6)}`
    : "Chưa có vị trí");
  show("power-status", data.power_available ? "Đã kết nối" : "Không phát hiện");
  show("power-reading", data.power_available && data.bus_voltage_v !== null && data.current_ma !== null
    ? `${data.bus_voltage_v.toFixed(2)} V / ${data.current_ma.toFixed(0)} mA` : "—");
  show("eeprom-status", data.eeprom_available ? "Sẵn sàng" : "Không phát hiện");
}

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
    }
  });
}
