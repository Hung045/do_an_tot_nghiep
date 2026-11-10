"""Exercise real servo/MPU code with fake LEDC, I2C and clock; no hardware used.
Run: python tests/adaptive_servo_host_test.py [path-to-native-g++]
"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "esp_err.h": """#pragma once
using esp_err_t=int;
constexpr int ESP_OK=0,ESP_FAIL=-1,ESP_ERR_INVALID_STATE=1,ESP_ERR_INVALID_ARG=2,ESP_ERR_TIMEOUT=3;
const char* esp_err_to_name(int);
""",
    "freertos/FreeRTOS.h": """#pragma once
#include <cstdint>
using TickType_t=uint32_t;
#define pdMS_TO_TICKS(ms) (ms)
#define portTICK_PERIOD_MS 1
""",
    "freertos/task.h": "#pragma once\n#include \"FreeRTOS.h\"\nTickType_t xTaskGetTickCount();\n",
    "esp_log.h": "#pragma once\n#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n",
    "BoardConfig.h": """#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#define GPIO_NUM_23 23
#define I2C_NUM_0 0
namespace BoardI2c {
inline int initialize(int) { return 0; }
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
    "driver/ledc.h": """#pragma once
#include <cstdint>
using ledc_mode_t=int; using ledc_timer_t=int; using ledc_channel_t=int;
constexpr int LEDC_LOW_SPEED_MODE=0,LEDC_TIMER_2=2,LEDC_CHANNEL_2=2,
 LEDC_TIMER_16_BIT=16,LEDC_AUTO_CLK=0,LEDC_INTR_DISABLE=0;
struct ledc_timer_config_t { int speed_mode,duty_resolution,timer_num; uint32_t freq_hz; int clk_cfg; };
struct ledc_channel_config_t { int gpio_num,speed_mode,channel,intr_type,timer_sel; uint32_t duty,hpoint; };
int ledc_timer_config(const ledc_timer_config_t*);
int ledc_channel_config(const ledc_channel_config_t*);
int ledc_set_duty(int,int,uint32_t);
int ledc_update_duty(int,int);
int ledc_stop(int,int,uint32_t);
""",
}
HARNESS = r'''
#include "BoardConfig.h"
#include "AdaptiveServo.h"
#include "CrashDetector.h"
#include "driver/ledc.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
static bool pwmError=false,timerError=false,stopped=false,imuError=false;
static uint32_t ticks=0,duty=0;
static int16_t ax=0,ay=0,az=16384;
uint32_t xTaskGetTickCount(){return ticks;}
const char* esp_err_to_name(int){return "mock";}
int ledc_timer_config(const ledc_timer_config_t* t) {
 assert(t->timer_num==2 && t->freq_hz==50 && t->duty_resolution==16);
 return timerError?ESP_FAIL:ESP_OK;
}
int ledc_channel_config(const ledc_channel_config_t* c) {
 assert(c->gpio_num==23 && c->channel==2 && c->timer_sel==2);
 duty=c->duty; assert(duty==4915); return ESP_OK;
}
int ledc_set_duty(int,int ch,uint32_t d){assert(ch==2); duty=d; return pwmError?ESP_FAIL:ESP_OK;}
int ledc_update_duty(int,int){return ESP_OK;}
int ledc_stop(int,int,uint32_t idle){assert(idle==0);stopped=true;return ESP_OK;}
int BoardI2c::write(int,uint8_t,const uint8_t*,size_t,int){return 0;}
int BoardI2c::writeRead(int port,uint8_t,const uint8_t* reg,size_t,uint8_t* out,size_t n,int timeout){
 assert(port==0);
 if(*reg==0x3B) assert(timeout<=20);
 if(imuError)return ESP_FAIL;
 if(*reg==0x75){out[0]=0x68;return 0;}
 assert(*reg==0x3B && n==6);
 int16_t data[]={ax,ay,az};
 for(int i=0;i<3;i++){out[2*i]=static_cast<uint16_t>(data[i])>>8;out[2*i+1]=data[i]&255;}
 return 0;
}
int main(){
 AdaptiveServo s;
 assert(s.update(0,true,false,20)==ESP_ERR_INVALID_STATE);
 timerError=true;assert(s.initialize()==ESP_FAIL && !s.isReady());timerError=false;
 assert(s.initialize()==ESP_OK && s.angleDegrees()==90);
 for(int i=0;i<200;i++)assert(s.update(2,true,false,20)==ESP_OK);
 assert(s.angleDegrees()==90); // deadband
 float previous=90;
 for(int i=0;i<100;i++){
  assert(s.update(45,true,false,20)==ESP_OK);
  assert(s.angleDegrees()-previous<=1.201F);previous=s.angleDegrees();
 }
 assert(std::fabs(s.angleDegrees()-120)<0.002F && s.targetDegrees()==120);
 for(int i=0;i<150;i++)s.update(-45,true,false,20);
 assert(std::fabs(s.angleDegrees()-60)<0.002F && s.targetDegrees()==60);
 s.update(-45,true,true,20);assert(!s.tracking() && s.targetDegrees()==90);
 for(int i=0;i<100;i++)s.update(-45,true,true,20);
 assert(std::fabs(s.angleDegrees()-90)<0.002F);
 for(int i=0;i<100;i++)s.update(30,true,false,20);
 assert(s.angleDegrees()>90);
 s.update(30,false,false,20);assert(!s.tracking() && s.targetDegrees()==90);
 s.update(std::numeric_limits<float>::quiet_NaN(),true,false,20);assert(!s.tracking());
 s.update(65,true,false,20);assert(!s.tracking());
 previous=s.angleDegrees();s.update(-45,true,false,10000);
 assert(std::fabs(s.angleDegrees()-previous)<=3.001F);
 pwmError=true; assert(s.update(-45,true,false,20)==ESP_FAIL);
 assert(stopped && !s.isReady());
 // Signed roll comes from the same read; old crash tilt remains unsigned.
 CrashDetector m;float tilt=0;bool crash=false;
 assert(m.initialize()==ESP_OK && m.isAvailable());
 ay=8192;assert(m.update(&tilt,&crash)==ESP_OK);
 assert(m.rollDegrees()>26 && m.rollDegrees()<27 && !crash);
 const float rightTilt=tilt;
 ay=-8192;assert(m.update(&tilt,&crash)==ESP_OK);
 assert(m.rollDegrees()<-26 && std::fabs(tilt-rightTilt)<0.01F);
 ay=16384;az=4096;ticks=100;m.update(&tilt,&crash);assert(!crash);
 ticks=300;m.update(&tilt,&crash);assert(crash);
 imuError=true;assert(m.update(&tilt,&crash)==ESP_FAIL);imuError=false;
 ax=ay=az=0;assert(m.update(&tilt,&crash)==ESP_ERR_INVALID_STATE);
 // Fully inverted must not fold back to zero degrees.
 ax=ay=0;az=-16384;ticks=1000;m.update(&tilt,&crash);
 assert(tilt>179 && !crash);
 ticks=1200;m.update(&tilt,&crash);assert(crash);
 // A failed sample breaks the continuous crash-confirmation interval.
 imuError=true;ticks=1210;m.update(&tilt,&crash);assert(!crash);imuError=false;
 ticks=1400;m.update(&tilt,&crash);assert(!crash);
 ticks=1600;m.update(&tilt,&crash);assert(crash);
 puts("PASS: servo pulse/timer isolation, limits, smoothing, faults; signed MPU roll and crash threshold");
}
'''
board = (ROOT / "components/board_config/include/BoardConfig.h").read_text(encoding="utf-8")
for line in board.splitlines():
    if re.match(r"#define DEMO_(SERVO_|HEADLIGHT_SERVO_|MPU_ROLL_|MPU_I2C_PORT\b|MPU6050_ADDRESS\b)", line):
        HEADERS["BoardConfig.h"] += line + "\n"
compiler = sys.argv[1] if len(sys.argv) > 1 else shutil.which("g++")
if not compiler:
    raise SystemExit("Native g++ required")
with tempfile.TemporaryDirectory(prefix="servo-test-") as temporary:
    temp = Path(temporary)
    for name, content in HEADERS.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    harness = temp / "test.cpp"
    harness.write_text(HARNESS, encoding="utf-8")
    executable = temp / ("test.exe" if os.name == "nt" else "test")
    environment = dict(os.environ)
    environment["PATH"] = str(Path(compiler).resolve().parent) + os.pathsep + environment.get("PATH", "")
    subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-I"+str(temp),
                    "-I"+str(ROOT/"components/headlight_controller/include"),
                    "-I"+str(ROOT/"components/crash_detector/include"),
                    str(ROOT/"components/headlight_controller/AdaptiveServo.cpp"),
                    str(ROOT/"components/crash_detector/CrashDetector.cpp"), str(harness),
                    "-o", str(executable)], check=True, env=environment, timeout=60)
    subprocess.run([str(executable)], check=True, env=environment, timeout=10)
