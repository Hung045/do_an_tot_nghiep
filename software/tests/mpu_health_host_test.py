"""Regression tests for transient MPU failure, stable recovery and fault latch."""
import os
from pathlib import Path
import subprocess
import sys
from host_test_temp import test_directory

ROOT = Path(__file__).resolve().parents[1]
HARNESS = r'''
#include "MpuHealth.h"
#include <cassert>
#include <cstdio>
int main() {
 using S=MpuHealth::State;
 MpuHealth h;
 assert(h.update(true,0)==S::Healthy);
 assert(h.update(false,20)==S::Recovering);
 assert(h.update(true,40)==S::Recovering);
 assert(h.update(true,60)==S::Recovering);
 assert(h.update(true,80)==S::Healthy);
 // Catch-up bursts cannot masquerade as 40 ms of stable samples.
 assert(h.update(false,100)==S::Recovering);
 for(int i=0;i<10;i++)assert(h.update(true,120)==S::Recovering);
 assert(h.update(true,160)==S::Healthy);
 // An intermittent good sample must not keep a failing sensor alive indefinitely.
 h.update(false,200);h.update(true,220);h.update(false,240);h.update(true,260);
 assert(h.update(false,399)==S::Recovering);
 assert(h.update(true,400)==S::Fault);
 assert(h.update(true,1000)==S::Fault);
 // Unsigned subtraction works across the uptime counter wrap.
 MpuHealth wrap;
 assert(wrap.update(false,UINT32_MAX-50)==S::Recovering);
 assert(wrap.update(false,148)==S::Recovering);
 assert(wrap.update(false,149)==S::Fault);
 MpuHealth recover;
 recover.update(false,UINT32_MAX-50);
 recover.update(true,UINT32_MAX-30);recover.update(true,UINT32_MAX-10);
 assert(recover.update(true,10)==S::Healthy);
 puts("PASS: MPU transient failure, stable recovery, deadline, latch and clock wrap");
}
'''
compiler = sys.argv[1]
environment = dict(os.environ)
environment['PATH'] = str(Path(compiler).resolve().parent)+os.pathsep+environment.get('PATH','')
with test_directory('mpu-health-') as temporary:
    path=Path(temporary)
    (path/'test.cpp').write_text(HARNESS,encoding='utf-8')
    exe=path/'test.exe'
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',
                    '-I'+str(ROOT/'components/crash_detector/include'),str(path/'test.cpp'),'-o',str(exe)],
                   env=environment,check=True,timeout=60)
    subprocess.run([str(exe)],env=environment,check=True,timeout=10)
