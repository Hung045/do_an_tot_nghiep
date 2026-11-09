"""Run real lamp/EEPROM code with a fake I2C bus. Requires native g++."""
import ast
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
# Reuse only literal mock headers; do not execute the servo test on import.
tree = ast.parse((ROOT / 'tests/adaptive_servo_host_test.py').read_text(encoding='utf-8'))
headers = next(ast.literal_eval(n.value) for n in tree.body if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == 'HEADERS' for t in n.targets))
headers['esp_err.h'] += '\nconstexpr int ESP_ERR_NOT_FOUND=4, ESP_ERR_INVALID_CRC=5;\n'
headers['esp_log.h'] += '\n#define ESP_LOGE(...) ((void)0)\n'
headers['freertos/task.h'] += '\nvoid vTaskDelay(TickType_t);\n'
headers['driver/i2c.h'] += '\nnamespace BoardI2c { int read(int,uint8_t,uint8_t*,size_t,int); }\n'
headers['driver/ledc.h'] += '\nusing ledc_timer_bit_t=int; constexpr int LEDC_TIMER_1=1,LEDC_CHANNEL_1=1;\n'
headers['BoardConfig.h'] = headers['BoardConfig.h'].replace('LockGuard(int,int)', 'LockGuard(int,int=0)')
headers['BoardConfig.h'] += '\n#define I2C_NUM_1 1\n#define GPIO_NUM_26 26\n'
board = (ROOT / 'components/board_config/include/BoardConfig.h').read_text(encoding='utf-8')
for line in board.splitlines():
    if re.match(r'#define DEMO_(AUX_I2C_PORT|BH1750_ADDRESS|AT24C256_|HEADLIGHT_PWM_GPIO|PWM_)', line):
        headers['BoardConfig.h'] += line + '\n'
headers['esp_rom_crc.h'] = '''#pragma once
#include <cstdint>
uint32_t esp_rom_crc32_le(uint32_t,const uint8_t*,uint32_t);
'''
HARNESS = r'''
#include "BoardConfig.h"
#include "HeadlightController.h"
#include "DataLogger.h"
#include "driver/ledc.h"
#include <cassert>
#include <cstdio>
#include <cstring>
static uint32_t ticks=0, pwm=0;
static bool lightFail=false, writeFail=false;
static uint8_t memory[32768]={};
uint32_t xTaskGetTickCount(){return ticks;}
void vTaskDelay(uint32_t n){ticks+=n;}
const char* esp_err_to_name(int){return "mock";}
uint32_t esp_rom_crc32_le(uint32_t crc,const uint8_t* p,uint32_t n){
 crc=~crc;for(uint32_t i=0;i<n;i++){crc^=p[i];for(int b=0;b<8;b++)crc=(crc>>1)^((crc&1)?0xedb88320U:0);}return ~crc;
}
int ledc_timer_config(const ledc_timer_config_t*){return 0;}
int ledc_channel_config(const ledc_channel_config_t*){return 0;}
int ledc_set_duty(int,int,uint32_t value){pwm=value;return 0;}
int ledc_update_duty(int,int){return 0;}
int BoardI2c::write(int,uint8_t addr,const uint8_t* data,size_t n,int){
 if(addr==0x50){if(writeFail)return ESP_FAIL;assert(n>=2);size_t pos=(data[0]<<8)|data[1];assert(pos+n-2<=sizeof(memory));memcpy(memory+pos,data+2,n-2);}return 0;
}
int BoardI2c::writeRead(int,uint8_t addr,const uint8_t* data,size_t,uint8_t* out,size_t n,int){
 assert(addr==0x50);size_t pos=(data[0]<<8)|data[1];assert(pos+n<=sizeof(memory));memcpy(out,memory+pos,n);return 0;
}
int BoardI2c::read(int,uint8_t,uint8_t* out,size_t,int){
 if(lightFail)return ESP_FAIL;
 out[0]=0;out[1]=240;return 0; // 200 lux
}
int main(){
 HeadlightController lamp;assert(lamp.initialize()==ESP_OK);
 float lux=0;uint32_t duty=0;
 ticks+=200;lamp.updateAutoHeadlight(false,&lux,&duty);assert(lux>199 && lux<201 && duty==0);
 lightFail=true;ticks+=200;lamp.updateAutoHeadlight(false,&lux,&duty);
 assert(lux<0 && duty==100 && pwm==1023);
 // Valid data resumes automatically after the bus recovers.
 lightFail=false;for(int i=0;i<25;i++){ticks+=200;lamp.updateAutoHeadlight(false,&lux,&duty);}
 assert(lux>199 && lux<201 && duty==0);
 DataLogger logger;assert(logger.initialize()==ESP_OK);
 TelemetryRecord record={};record.tiltDegrees=5;record.crashDetected=true;
 assert(logger.append(record)==ESP_OK);assert(memory[29]==1);
 DataLogger restarted;assert(restarted.initialize()==ESP_OK);
 TelemetryRecord restored={};assert(restarted.readLatest(&restored)==ESP_OK);
 assert(restored.crashDetected && restored.tiltDegrees==5);
 writeFail=true;record.crashDetected=false;assert(restarted.append(record)!=ESP_OK);
 assert(restarted.readLatest(&restored)==ESP_OK && restored.crashDetected);
 writeFail=false;assert(restarted.append(record)==ESP_OK);
 assert(restarted.readLatest(&restored)==ESP_OK && !restored.crashDetected);
 puts("PASS: BH1750 loss/fallback/recovery; EEPROM latched crash survives reset and failed write");
}
'''
compiler = sys.argv[1]
with tempfile.TemporaryDirectory(prefix='sensor-recovery-') as temporary:
    temp = Path(temporary)
    for name, content in headers.items():
        path = temp / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
    harness = temp / 'test.cpp'
    harness.write_text(HARNESS, encoding='utf-8')
    exe = temp / 'test.exe'
    environment = dict(os.environ)
    environment['PATH'] = str(Path(compiler).resolve().parent) + os.pathsep + environment.get('PATH', '')
    components = ['headlight_controller', 'data_logger', 'project_types']
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I'+str(temp),
                    *['-I'+str(ROOT/'components'/c/'include') for c in components],
                    str(ROOT/'components/headlight_controller/HeadlightController.cpp'),
                    str(ROOT/'components/data_logger/DataLogger.cpp'), str(harness), '-o', str(exe)],
                   env=environment, check=True, timeout=60)
    subprocess.run([str(exe)], env=environment, check=True, timeout=10)
