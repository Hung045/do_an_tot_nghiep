"""Run real INA226 driver with fake I2C: python tests/power_host_test.py [g++]."""
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
    "freertos/FreeRTOS.h": "#pragma once\n#define pdMS_TO_TICKS(ms) (ms)\n",
    "freertos/task.h": "#pragma once\ninline void vTaskDelay(int) {}\n",
    "esp_log.h": "#pragma once\n#define ESP_LOGI(...) ((void)0)\n",
    "BoardConfig.h": """#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#define DEMO_AUX_I2C_PORT 1
#define DEMO_INA226_ADDRESS 0x40
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
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
static uint16_t regs[256]={};
static int failRead=-1, failWrite=-1;
static bool busError=false;
namespace BoardI2c { esp_err_t initialize(int port) { assert(port==1); return busError ? ESP_FAIL : ESP_OK; } }
int BoardI2c::write(int port,uint8_t addr,const uint8_t* d,size_t n,int) {
 assert(port==1 && addr==0x40 && n==3);
 if(d[0]==failWrite) return ESP_FAIL;
 regs[d[0]]=uint16_t(d[1])*256+d[2]; return 0;
}
int BoardI2c::writeRead(int port,uint8_t addr,const uint8_t* d,size_t n,uint8_t* out,size_t len,int) {
 assert(port==1 && addr==0x40 && n==1 && len==2);
 if(d[0]==failRead) return ESP_ERR_TIMEOUT;
 out[0]=regs[d[0]]>>8; out[1]=regs[d[0]]&255; return 0;
}
void unknown(const PowerReading& p) {
 assert(std::isnan(p.busVoltageVolts) && std::isnan(p.currentMilliamps) && std::isnan(p.powerMilliwatts));
}
int main() {
 PowerMonitor m; PowerReading p={12,100,1200,50};
 assert(m.read(nullptr)==ESP_ERR_INVALID_ARG);
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 // Each failed initialization must leave reads disabled.
 busError=true; assert(m.initialize()==ESP_FAIL); busError=false;
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 for(int r:{0,5}) {
   failWrite=r; assert(m.initialize()==ESP_FAIL);
   assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 }
 failWrite=-1; regs[255]=0x9999;
 assert(m.initialize()==ESP_ERR_NOT_FOUND);
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 regs[255]=0x2260; assert(m.initialize()==ESP_OK);
 assert(regs[5]==2097); // R020, 4 A range, CAL=0.00512/(Current_LSB*R).
 // A legitimate zero reading must remain valid.
 assert(m.read(&p)==ESP_OK && p.busVoltageVolts==0 && p.currentMilliamps==0);
 regs[2]=9600; regs[4]=16384; regs[3]=100;
 assert(m.read(&p)==ESP_OK);
 assert(std::abs(p.busVoltageVolts-12.0F)<0.001F);
 assert(std::abs(p.currentMilliamps-2000.0F)<0.01F);
 regs[4]=0xC000; assert(m.read(&p)==ESP_OK && p.currentMilliamps<0);
 for(int r:{2,4,3}) {
   failRead=r; assert(m.read(&p)==ESP_ERR_TIMEOUT); unknown(p);
   failRead=-1; assert(m.read(&p)==ESP_OK && p.busVoltageVolts==12.0F);
 }
 // Reinitialization failure must not reuse a previous calibration as readiness.
 busError=true; assert(m.initialize()==ESP_FAIL);
 assert(m.read(&p)==ESP_ERR_INVALID_STATE); unknown(p);
 puts("PASS: INA226 init failures, true zero, signed current, read errors and recovery");
}
'''

compiler = sys.argv[1] if len(sys.argv) > 1 else shutil.which("g++")
# Use the actual board calibration instead of a separate mock calibration.
board_config = (ROOT / "components/board_config/include/BoardConfig.h").read_text(encoding="utf-8")
for setting in ("DEMO_INA226_SHUNT_MICRO_OHMS", "DEMO_INA226_MAX_CURRENT_MILLIAMPS"):
    match = re.search(r"^#define\s+" + setting + r"\s+(\d+)\s*$", board_config, re.MULTILINE)
    if not match:
        raise SystemExit("Missing board setting: " + setting)
    HEADERS["BoardConfig.h"] += "\n#define " + setting + " " + match.group(1) + "\n"
if not compiler:
    raise SystemExit("Native g++ required; pass its path as the first argument.")
with tempfile.TemporaryDirectory(prefix="ina226-test-") as temporary:
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
