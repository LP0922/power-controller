@echo off
rem One-click launcher for the actuator control page (single process).
rem Stop server: close this window, or run: taskkill /F /IM python.exe
cd /d %~dp0
set PYTHONPATH=E:\XiangMu\powder\server\src
set PYTHONIOENCODING=utf-8
"C:\Users\86182\AppData\Local\Programs\Python\Python312\python.exe" app.py
pause
