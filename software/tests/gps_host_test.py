"""Compile the real GPS driver against a fake UART/clock. Requires native g++.

Run: python tests/gps_host_test.py [path-to-g++]
No ESP32, GPS, network or serial port is used.
"""
import os
from pathlib import Path
import shutil
import subprocess
import sys
from host_test_temp import test_directory

ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "esp_err.h": """#pragma once
using esp_err_t = int;
constexpr int ESP_OK=0, ESP_FAIL=-1, ESP_ERR_NOT_FOUND=1,
 ESP_ERR_INVALID_ARG=2, ESP_ERR_INVALID_STATE=3;
""",
    "freertos/FreeRTOS.h": """#pragma once
#include <cstdint>
using TickType_t = uint32_t;
#define pdMS_TO_TICKS(ms) (static_cast<TickType_t>(ms))
#define portTICK_PERIOD_MS 1
""",
    "freertos/task.h": "#pragma once\n#include \"FreeRTOS.h\"\nTickType_t xTaskGetTickCount();\n",
    "BoardConfig.h": """#pragma once
#define DEMO_GPS_UART_PORT 2
#define DEMO_GPS_UART_TX_GPIO 17
#define DEMO_GPS_UART_RX_GPIO 16
#define DEMO_GPS_BAUD_RATE 9600
""",
    "esp_log.h": "#pragma once\n#define ESP_LOGI(...) ((void)0)\n",
    "driver/uart.h": """#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_err.h"
using uart_port_t = int;
constexpr int UART_DATA_8_BITS=8, UART_PARITY_DISABLE=0, UART_STOP_BITS_1=1,
 UART_HW_FLOWCTRL_DISABLE=0, UART_SCLK_DEFAULT=0, UART_PIN_NO_CHANGE=-1;
struct uart_config_t { int baud_rate,data_bits,parity,stop_bits,flow_ctrl,source_clk; };
int uart_param_config(int,const uart_config_t*);
int uart_set_pin(int,int,int,int,int);
bool uart_is_driver_installed(int);
int uart_driver_install(int,int,int,int,void*,int);
int uart_flush_input(int);
int uart_get_buffered_data_len(int,size_t*);
int uart_read_bytes(int,void*,size_t,int);
""",
}
HARNESS = r'''
#include "GpsTracker.h"
#include "driver/uart.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static TickType_t ticks=0;
static std::string rx;
static bool readError=false, initError=false;
TickType_t xTaskGetTickCount() { return ticks; }
int uart_param_config(int p,const uart_config_t* c) {
 assert(p==2 && c->baud_rate==9600); return initError ? ESP_FAIL : ESP_OK;
}
int uart_set_pin(int,int tx,int r,int,int) { assert(tx==17 && r==16); return 0; }
bool uart_is_driver_installed(int) { return true; }
int uart_driver_install(int,int,int,int,void*,int) { return 0; }
int uart_flush_input(int) { rx.clear(); return 0; }
int uart_get_buffered_data_len(int,size_t* n) { *n=rx.size(); return readError ? ESP_FAIL : 0; }
int uart_read_bytes(int,void* b,size_t n,int wait) {
 assert(wait==0); n=std::min(n,rx.size()); memcpy(b,rx.data(),n); rx.erase(0,n); return int(n);
}
std::string sentence(const std::string& body) {
 unsigned char sum=0; for(unsigned char c:body) sum^=c;
 char tail[8]; snprintf(tail,sizeof(tail),"*%02X\r\n",sum);
 return "$"+body+tail;
}
void feed(GpsTracker& g,GpsFix& f,const std::string& body) {
 rx+=sentence(body); g.readFix(&f);
}
int main() {
 GpsTracker g; GpsFix f={};
 assert(g.diagnostics().status==GpsStatus::InitError);
 initError=true; assert(g.initialize()==ESP_FAIL); initError=false;
 assert(g.initialize()==ESP_OK);
 assert(g.readFix(&f)==ESP_ERR_NOT_FOUND && !f.valid);
 assert(g.diagnostics().status==GpsStatus::NoData && g.diagnostics().rxAgeMs==-1);
 rx="garbage"; g.readFix(&f);
 assert(g.diagnostics().status==GpsStatus::InvalidNmea && g.diagnostics().receivedBytes==7);
 rx="$GPGGA,bad*00\r\n"; g.readFix(&f);
 assert(g.diagnostics().rejectedSentences==1);
 feed(g,f,"GPTXT,01,01,02,receiver started");
 assert(g.diagnostics().status==GpsStatus::NoNavigation);
 feed(g,f,"GPGGA,120000,,,,,0,00,99.9,,,,,,");
 assert(g.diagnostics().status==GpsStatus::NoFix && !f.valid);
 const std::string valid="GPGGA,120001,2058.6967,N,10547.4898,E,1,07,1.2,10.0,M,0.0,M,,";
 const auto wire=sentence(valid);
 rx=wire.substr(0,17); g.readFix(&f); assert(!f.valid);
 rx=wire.substr(17); g.readFix(&f);
 assert(f.valid && f.satellitesInView==7 && std::abs(f.latitudeDegrees-20.9782783)<0.000001);
 assert(g.diagnostics().status==GpsStatus::FixOk);
 feed(g,f,"GPRMC,120001,A,2058.6967,N,10547.4898,E,2.0,0,051126,,,A");
 assert(f.valid && f.speedMetersPerSecond>1.0F);
 // Explicit GGA no-fix must clear a previous fix immediately.
 feed(g,f,"GPGGA,120002,,,,,0,00,99.9,,,,,,");
 assert(!f.valid && g.diagnostics().status==GpsStatus::NoFix);
 feed(g,f,valid);
 feed(g,f,"GPRMC,120002,V,,,,,,,051126,,,N");
 assert(!f.valid);
 feed(g,f,valid); ticks=5000; g.readFix(&f);
 assert(!f.valid && f.satellitesInView==0 && g.diagnostics().status==GpsStatus::Stale);
 // Good NMEA but no navigation updates cannot refresh an old location.
 feed(g,f,"GPTXT,01,01,02,receiver started");
 assert(g.diagnostics().status==GpsStatus::NoNavigation);
 feed(g,f,valid); assert(f.valid);
 readError=true; assert(g.readFix(&f)==ESP_FAIL && !f.valid);
 assert(g.diagnostics().status==GpsStatus::UartError);
 readError=false; feed(g,f,valid); assert(f.valid);
 // Bad numbers must never enter JSON as NaN or infinity.
 assert(g.initialize()==ESP_OK);
 feed(g,f,"GPGGA,120003,nan,N,10547.4898,E,1,07,1.2,10,M,0,M,,");
 assert(!f.valid);
 feed(g,f,"GPGGA,120003,1e100,N,10547.4898,E,1,07,1.2,10,M,0,M,,");
 assert(!f.valid);
 feed(g,f,"GNRMC,120003,A,2058.6967,N,10547.4898,E,nan,0,051126,,,A");
 assert(f.valid && f.speedMetersPerSecond==0);
 // Tick wrap-around and recovery from overlong lines.
 ticks=UINT32_MAX-1000; assert(g.initialize()==ESP_OK); feed(g,f,valid);
 ticks=1000; g.readFix(&f); assert(f.valid);
 ticks=5000; g.readFix(&f); assert(!f.valid);
 rx="$"+std::string(200,'X')+"\r\n"+sentence(valid); g.readFix(&f);
 assert(f.valid && g.diagnostics().rejectedSentences==1);
 assert(g.diagnostics().validSentences==2);
 puts("PASS: GPS UART diagnostics, NMEA parsing, loss/recovery, timeouts and tick wrap");
}
'''

compiler = sys.argv[1] if len(sys.argv) > 1 else shutil.which("g++")
if not compiler:
    raise SystemExit("Native g++ not found; pass its path as the first argument.")
with test_directory(prefix="gps-test-") as temporary:
    temp = Path(temporary)
    for name, content in HEADERS.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    harness = temp / "test.cpp"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = temp / ("gps-test.exe" if os.name == "nt" else "gps-test")
    environment = dict(os.environ)
    environment["PATH"] = str(Path(compiler).resolve().parent) + os.pathsep + environment.get("PATH", "")
    subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    "-I" + str(temp),
                    "-I" + str(ROOT / "components/gps_tracker/include"),
                    "-I" + str(ROOT / "components/project_types/include"),
                    str(ROOT / "components/gps_tracker/GpsTracker.cpp"),
                    str(harness), "-o", str(executable)], check=True, env=environment, timeout=60)
    subprocess.run([str(executable)], check=True, env=environment, timeout=10)
