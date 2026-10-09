#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Запускает мок-сервер страницы поз на :8090, гоняет test_pose_api.py, гасит."""
import os
import shutil
import subprocess
import sys
import time

SCR = os.path.dirname(os.path.abspath(__file__))
shutil.copy2('/home/sg/robot-dog/ros2_ws/src/dog_bringup/config/servos.yaml',
             os.path.join(SCR, 'servos_apply_test.yaml'))   # копия для теста, репо не трогаем
srv = subprocess.Popen(
    [sys.executable, os.path.join(SCR, 'servo_pose_server.py'), '8090',
     os.path.join(SCR, 'servos_apply_test.yaml')],
    cwd=SCR, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
rc = 1
try:
    time.sleep(1.2)
    rc = subprocess.call([sys.executable, os.path.join(SCR, 'test_pose_api.py')])
finally:
    srv.terminate()
    try:
        out, _ = srv.communicate(timeout=5)
    except Exception:                                   # noqa: BLE001
        srv.kill()
        out, _ = srv.communicate()
    print('== лог сервера ==')
    print(out)
print('rc=%d' % rc)
sys.exit(rc)
