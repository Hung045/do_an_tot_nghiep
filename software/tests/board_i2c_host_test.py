"""Exercise real BoardI2c transport with fake new-driver handles and a 10 ms RTOS tick."""
import os
from pathlib import Path
import re
import subprocess
import sys
from host_test_temp import test_directory

ROOT=Path(__file__).resolve().parents[1]
HEADERS={
 'esp_err.h': '''#pragma once
using esp_err_t=int;
constexpr int ESP_OK=0,ESP_FAIL=-1,ESP_ERR_INVALID_ARG=1,ESP_ERR_NO_MEM=2,ESP_ERR_TIMEOUT=3,ESP_ERR_INVALID_STATE=4;
''',
 'esp_log.h':'#pragma once\n#define ESP_LOGI(...) ((void)0)\n#define ESP_LOGE(...) ((void)0)\n',
 'freertos/FreeRTOS.h':'''#pragma once
#include <cstdint>
using TickType_t=uint32_t;
#define portTICK_PERIOD_MS 10
#define pdMS_TO_TICKS(ms) ((ms)/10)
#define pdTRUE 1
''',
 'freertos/task.h':'#pragma once\n#include "FreeRTOS.h"\nTickType_t xTaskGetTickCount();\n',
 'freertos/semphr.h':'''#pragma once
#include "FreeRTOS.h"
struct Mutex { bool held=false; };
using SemaphoreHandle_t=Mutex*;
inline auto xSemaphoreCreateMutex(){return new Mutex;}
inline void vSemaphoreDelete(Mutex* m){delete m;}
inline int xSemaphoreTake(Mutex* m,TickType_t){if(m->held)return 0;m->held=true;return 1;}
inline void xSemaphoreGive(Mutex* m){m->held=false;}
''',
 'driver/gpio.h':'#pragma once\nusing gpio_num_t=int;\n',
 'driver/i2c_master.h':'''#pragma once
#include <cstddef>
#include <cstdint>
#include "esp_err.h"
using i2c_port_t=int;
using i2c_master_bus_handle_t=void*;
using i2c_master_dev_handle_t=void*;
constexpr int I2C_NUM_0=0,I2C_NUM_1=1,I2C_CLK_SRC_DEFAULT=0,I2C_ADDR_BIT_LEN_7=7;
struct i2c_master_bus_config_t { int i2c_port,sda_io_num,scl_io_num,clk_source,glitch_ignore_cnt; struct {bool enable_internal_pullup;}flags; };
struct i2c_device_config_t {int dev_addr_length;uint16_t device_address;uint32_t scl_speed_hz;};
int i2c_new_master_bus(const i2c_master_bus_config_t*,void**);
int i2c_master_bus_add_device(void*,const i2c_device_config_t*,void**);
int i2c_master_transmit(void*,const uint8_t*,size_t,int);
int i2c_master_receive(void*,uint8_t*,size_t,int);
int i2c_master_transmit_receive(void*,const uint8_t*,size_t,uint8_t*,size_t,int);
'''
}
board=(ROOT/'components/board_config/include/BoardConfig.h').read_text(encoding='utf-8')
for pin in sorted(set(re.findall(r'GPIO_NUM_(\d+)',board))):
    HEADERS['driver/gpio.h']+=f'constexpr int GPIO_NUM_{pin}={pin};\n'
HARNESS=r'''
#include "BoardConfig.h"
#include <cassert>
#include <cstdio>
static uint32_t ticks=0;
static int busCalls=0,addCalls=0,transfers=0,lastTimeout=0,result=0;
static bool initFail=true;
TickType_t xTaskGetTickCount(){return ticks;}
int i2c_new_master_bus(const i2c_master_bus_config_t* c,void** out){
 ++busCalls;if(initFail)return ESP_FAIL;
 assert(c->glitch_ignore_cnt==7 && c->flags.enable_internal_pullup);
 assert(c->sda_io_num==(c->i2c_port==0?21:18));
 *out=reinterpret_cast<void*>(uintptr_t(c->i2c_port+1));return 0;
}
int i2c_master_bus_add_device(void* bus,const i2c_device_config_t* c,void** out){
 ++addCalls;assert(c->scl_speed_hz==100000 && c->dev_addr_length==7);
 *out=reinterpret_cast<void*>(uintptr_t(bus)*128+c->device_address);return 0;
}
int transfer(int timeout){++transfers;lastTimeout=timeout;return result;}
int i2c_master_transmit(void*,const uint8_t*,size_t,int t){return transfer(t);}
int i2c_master_receive(void*,uint8_t*,size_t,int t){return transfer(t);}
int i2c_master_transmit_receive(void*,const uint8_t*,size_t,uint8_t*,size_t,int t){return transfer(t);}
int main(){
 uint8_t byte=0;
 assert(BoardI2c::initialize(2)==ESP_ERR_INVALID_ARG);
 assert(BoardI2c::initialize(0)==ESP_FAIL);
 initFail=false;assert(BoardI2c::initialize(0)==0);assert(BoardI2c::initialize(0)==0);
 assert(busCalls==2);assert(BoardI2c::initialize(1)==0);
 BoardI2c::LockGuard mpu(0,0),aux(1,0);assert(mpu.isLocked() && aux.isLocked());
 {BoardI2c::LockGuard duplicate(0,0);assert(!duplicate.isLocked());}
 assert(BoardI2c::writeRead(0,0x68,&byte,1,&byte,1,2)==0);assert(lastTimeout==20 && addCalls==1);
 assert(BoardI2c::read(0,0x68,&byte,1,2)==0);assert(addCalls==1);
 result=ESP_ERR_TIMEOUT;assert(BoardI2c::read(0,0x68,&byte,1,2)==ESP_ERR_TIMEOUT);
 int previous=transfers;result=0;
 assert(BoardI2c::read(0,0x68,&byte,1,2)==ESP_ERR_TIMEOUT);assert(transfers==previous);
 // The AUX bus is unaffected by a MPU timeout.
 assert(BoardI2c::write(1,0x23,&byte,1,3)==0);assert(lastTimeout==30);
 ticks=1;assert(BoardI2c::read(0,0x68,&byte,1,2)==ESP_ERR_TIMEOUT);
 ticks=2;assert(BoardI2c::read(0,0x68,&byte,1,2)==0);
 // Bad arguments never reach the driver; timeouts are finite, in milliseconds.
 previous=transfers;assert(BoardI2c::read(2,0x68,&byte,1,2)==ESP_ERR_INVALID_ARG);
 assert(BoardI2c::read(0,0xff,&byte,1,2)==ESP_ERR_INVALID_ARG);
 assert(BoardI2c::read(0,0x68,nullptr,1,2)==ESP_ERR_INVALID_ARG);assert(transfers==previous);
 assert(BoardI2c::read(0,0x68,&byte,1,UINT32_MAX)==0);assert(lastTimeout==1000);
 // Cooldown is bounded even when RTOS ticks wrap.
 ticks=UINT32_MAX;result=ESP_ERR_TIMEOUT;BoardI2c::read(0,0x68,&byte,1,2);
 result=0;ticks=0;assert(BoardI2c::read(0,0x68,&byte,1,2)==ESP_ERR_TIMEOUT);
 ticks=1;assert(BoardI2c::read(0,0x68,&byte,1,2)==0);
 puts("PASS: new I2C handle lifecycle, bus isolation, bounded timeouts, cooldown and wrap");
}
'''
compiler=sys.argv[1]
environment=dict(os.environ)
environment['PATH']=str(Path(compiler).resolve().parent)+os.pathsep+environment.get('PATH','')
with test_directory('board-i2c-') as temporary:
    temp=Path(temporary)
    for name,content in HEADERS.items():
        p=temp/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(content,encoding='utf-8')
    (temp/'test.cpp').write_text(HARNESS,encoding='utf-8')
    exe=temp/'test.exe'
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-I'+str(temp),
                    '-I'+str(ROOT/'components/board_config/include'),
                    str(ROOT/'components/board_config/BoardConfig.cpp'),str(temp/'test.cpp'),'-o',str(exe)],
                   check=True,env=environment,timeout=60)
    subprocess.run([str(exe)],check=True,env=environment,timeout=10)
