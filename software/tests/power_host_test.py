"""Run real INA219 driver with fake I2C: python tests/power_host_test.py [g++]."""
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "esp_err.h": """#pragma once
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_FAIL=-1, ESP_ERR_INVALID_ARG=1,
 ESP_ERR_INVALID_STATE=2, ESP_ERR_NOT_FOUND=3, ESP_ERR_TIMEOUT=4;
""",
    "freertos/FreeRTOS.h": "#pragma once\n#include <cstdint>\n#define pdMS_TO_TICKS(ms) (ms)\n#define portTICK_PERIOD_MS 1\n",
"nvs_flash.h": """#pragma once
#include \"esp_err.h\"
constexpr int ESP_ERR_NVS_NO_FREE_PAGES=5;
constexpr int ESP_ERR_NVS_NEW_VERSION_FOUND=6;
inline esp_err_t nvs_flash_init() { return ESP_OK; }
""",
"nvs.h": """#pragma once
#include \"esp_err.h\"
#include <cstdint>
using nvs_handle_t=uint32_t;
constexpr int NVS_READONLY=0, NVS_READWRITE=1;
inline esp_err_t nvs_open(const char*,int,nvs_handle_t*) { return ESP_OK; }
inline esp_err_t nvs_get_u32(nvs_handle_t,const char*,uint32_t*) { return ESP_ERR_NOT_FOUND; }
inline esp_err_t nvs_set_u32(nvs_handle_t,const char*,uint32_t) { return ESP_OK; }
inline esp_err_t nvs_commit(nvs_handle_t) { return ESP_OK; }
inline void nvs_close(nvs_handle_t) {}
""",
    "freertos/task.h": "#pragma once\n#include <cstdint>\ninline uint32_t& sohTestTicks() { static uint32_t ticks=0; return ticks; }\ninline void vTaskDelay(int) {}\ninline uint32_t xTaskGetTickCount() { return sohTestTicks(); }\n",
    "esp_log.h": "#pragma once\n#include \"esp_err.h\"\n#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\ninline const char* esp_err_to_name(esp_err_t err) { return err == 0 ? \"OK\" : \"ERROR\"; }\n",
    "BoardConfig.h": """#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#define DEMO_AUX_I2C_PORT 1
#define DEMO_INA219_ADDRESS 0x40
namespace BoardI2c {
esp_err_t initialize(int);
class LockGuard { public: LockGuard(int,int) {} bool isLocked() const { return true; } };
}
""",
    "driver/i2c.h": """#pragma once
#include <cstdint>
#include <cstddef>
using i2c_port_t=int;
namespace BoardI2c { int write(int,uint8_t,const uint8_t*,size_t,int); }
namespace BoardI2c { int writeRead(int,uint8_t,const uint8_t*,size_t,uint8_t*,size_t,int); }
""",
}
HARNESS = r'''
#include "PowerMonitor.h"
#include "BoardConfig.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
static uint16_t regs[256]={};
static int failRead=-1, failWrite=-1;
static bool busError=false;
static bool wrongReset=false, ignoreCalibration=false;
namespace BoardI2c { esp_err_t initialize(int port) { assert(port==1); return busError ? ESP_FAIL : ESP_OK; } }
int BoardI2c::write(int port,uint8_t addr,const uint8_t* d,size_t n,int) {
 assert(port==1 && addr==0x40 && n==3);
 assert(d[0]<=5);
 if(d[0]==failWrite) return ESP_FAIL;
 uint16_t value=uint16_t(d[1])*256+d[2];
 if(d[0]==0 && value==0x8000) {
   regs[0]=wrongReset ? 0 : 0x399F; regs[5]=0; return 0;
 }
 if(d[0]==5 && ignoreCalibration) return 0;
 regs[d[0]]=value; return 0;
}
int BoardI2c::writeRead(int port,uint8_t addr,const uint8_t* d,size_t n,uint8_t* out,size_t len,int) {
 assert(port==1 && addr==0x40 && n==1 && len==2);
 assert(d[0]<=5); // INA219 has no device ID register at 0xFF.
 if(d[0]==failRead) return ESP_ERR_TIMEOUT;
 out[0]=regs[d[0]]>>8; out[1]=regs[d[0]]&255; return 0;
}
void unknown(const PowerReading& p) {
 assert(std::isnan(p.busVoltageVolts) && std::isnan(p.currentMilliamps) && std::isnan(p.powerMilliwatts));
 assert(p.stateOfChargePercent==-1.0F);
 assert(p.stateOfHealthPercent==-1.0F);
}
int main() {
 PowerMonitor m; PowerReading p={12,100,1200,50,-1};
 assert(m.read(nullptr)==ESP_ERR_INVALID_ARG);
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 // Each failed initialization must leave reads disabled.
 busError=true; assert(m.initialize()==ESP_FAIL); busError=false;
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 for(int r:{0,5}) {
   failWrite=r; assert(m.initialize()==ESP_FAIL);
   assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 }
 failWrite=-1; wrongReset=true;
 assert(m.initialize()==ESP_ERR_NOT_FOUND);
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 wrongReset=false;
 ignoreCalibration=true; assert(m.initialize()==ESP_ERR_INVALID_STATE);
 ignoreCalibration=false;
 for(int r:{0,5}) {
   failRead=r; assert(m.initialize()==ESP_ERR_TIMEOUT);
 }
 failRead=-1; assert(m.initialize()==ESP_OK);
 assert(regs[0]==0x3F77);
 assert(((regs[0]>>7)&15)==14 && ((regs[0]>>3)&15)==14); // 64 samples/channel.
 assert(regs[5]==4194); // R100, 3.2 A; CAL=0.04096/(LSB*R), bit 0 reserved.
 const float lsb=0.04096F/(4194*0.1F);
 // A legitimate zero reading must remain valid.
 regs[2]=2; // Conversion ready, zero bus voltage.
 assert(m.read(&p)==ESP_OK && p.busVoltageVolts==0 && p.currentMilliamps==0);
 regs[2]=(3000<<3)|2; regs[4]=16384; regs[3]=100;
 assert(m.read(&p)==ESP_OK);
 assert(std::abs(p.busVoltageVolts-12.0F)<0.001F);
 assert(std::abs(p.currentMilliamps-16384*lsb*1000)<0.01F);
 assert(std::abs(p.powerMilliwatts-100*20*lsb*1000)<0.01F);
 regs[4]=8192; assert(m.read(&p)==ESP_OK);
 assert(std::abs(p.currentMilliamps-(16384*lsb*1000*0.65F + 8192*lsb*1000*0.35F))<0.01F);
 regs[4]=0; assert(m.read(&p)==ESP_OK && p.currentMilliamps==0);
 regs[4]=16384;
 regs[2]|=1; assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 regs[2]=(3000<<3); assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 regs[2]|=2;
 regs[5]=0; assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 regs[5]=4194;
 regs[4]=0xC000; assert(m.read(&p)==ESP_OK && p.currentMilliamps<0);
 for(int r:{2,4,3,5}) {
   failRead=r; assert(m.read(&p)==ESP_ERR_TIMEOUT); unknown(p);
   failRead=-1; assert(m.read(&p)==ESP_OK && std::abs(p.busVoltageVolts-12.0F)<0.001F);
 }
 regs[2]=(2400<<3)|2; assert(m.read(&p)==ESP_OK && p.stateOfChargePercent==0.0F);
 regs[2]=(2775<<3)|2; assert(m.read(&p)==ESP_OK && std::abs(p.stateOfChargePercent-50.0F)<0.01F);
 regs[2]=(3150<<3)|2; assert(m.read(&p)==ESP_OK && p.stateOfChargePercent==100.0F);
 regs[2]=(2300<<3)|2; assert(m.read(&p)==ESP_OK && p.stateOfChargePercent==0.0F);
 regs[2]=(3200<<3)|2; assert(m.read(&p)==ESP_OK && p.stateOfChargePercent==100.0F);
 regs[2]=(3000<<3)|2;
 assert(m.read(&p)==ESP_OK && p.stateOfHealthPercent==-1.0F);
 // A full-to-empty discharge cycle learns capacity once and remains in 0..100%.
 sohTestTicks()=0;
 regs[2]=(3150<<3)|2; regs[4]=0;
 assert(m.read(&p)==ESP_OK);
 sohTestTicks()=3600000;
 regs[2]=(2250<<3)|2; regs[4]=22530;
 assert(m.read(&p)==ESP_OK);
 std::printf("SOH calibrated test=%.3f\\n", p.stateOfHealthPercent);
 assert(std::abs(p.stateOfHealthPercent-100.0F)<0.1F);
 assert(m.read(&p)==ESP_OK);
 // Charging reversal aborts an active SOH cycle.
 sohTestTicks()=7200000;
 regs[2]=(3150<<3)|2; regs[4]=0;
 assert(m.read(&p)==ESP_OK);
 sohTestTicks()=10800000;
 regs[4]=0xC000; assert(m.read(&p)==ESP_OK);
 regs[2]=(2250<<3)|2; assert(m.read(&p)==ESP_OK);
 std::printf("SOH reversal test=%.3f\\n", p.stateOfHealthPercent);
 assert(std::abs(p.stateOfHealthPercent-100.0F)<0.1F);
 // Reinitialization failure must not reuse a previous calibration as readiness.
 busError=true; assert(m.initialize()==ESP_FAIL);
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 puts("PASS: INA219 config, calibration, voltage/current/power, overflow, not-ready, reset and I2C failures");
}
'''

compiler = sys.argv[1] if len(sys.argv) > 1 else shutil.which("g++")
# Use the actual board calibration instead of a separate mock calibration.
board_config = (ROOT / "components/board_config/include/BoardConfig.h").read_text(encoding="utf-8")
for setting in ("DEMO_INA219_SHUNT_MICRO_OHMS", "DEMO_INA219_MAX_CURRENT_MILLIAMPS",
                "DEMO_INA219_CURRENT_FILTER_PERCENT", "DEMO_INA219_DISCHARGE_SIGN",
                "DEMO_BATTERY_EMPTY_MILLIVOLTS", "DEMO_BATTERY_NOMINAL_MILLIVOLTS",
                "DEMO_BATTERY_FULL_MILLIVOLTS", "DEMO_BATTERY_CUTOFF_MILLIVOLTS",
                "DEMO_BATTERY_RATED_CAPACITY_MAH"):
    match = re.search(r"^#define\s+" + setting + r"\s+(\d+)\s*$", board_config, re.MULTILINE)
    if not match:
        raise SystemExit("Missing board setting: " + setting)
    HEADERS["BoardConfig.h"] += "\n#define " + setting + " " + match.group(1) + "\n"
if not compiler:
    raise SystemExit("Native g++ required; pass its path as the first argument.")
with tempfile.TemporaryDirectory(prefix="ina219-test-") as temporary:
    temp = Path(temporary)
    for name, content in HEADERS.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    harness = temp / "test.cpp"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = temp / ("power-test.exe" if os.name == "nt" else "power-test")
    environment = dict(os.environ)
    environment["PATH"] = str(Path(compiler).resolve().parent) + os.pathsep + environment.get("PATH", "")
    subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I" + str(temp), "-I" + str(ROOT / "components/power_monitor/include"),
                    str(ROOT / "components/power_monitor/PowerMonitor.cpp"), str(harness),
                    "-o", str(executable)], check=True, env=environment, timeout=60)
    subprocess.run([str(executable)], check=True, env=environment, timeout=10)
