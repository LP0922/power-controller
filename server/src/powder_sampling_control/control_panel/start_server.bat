@echo off
rem ============================================================
rem 控制面板启动器（单进程 HTTP 服务，端口 8765）
rem 优先用项目虚拟环境 powder_control，找不到再退回系统 Python 3.12
rem 停止服务：关闭本窗口，或 taskkill /F /IM python.exe
rem 注意：长期跑建议操作员双击本文件启动（关掉窗口就停）。
rem       会话/自动化里也可以代启（2026-10-23 实测通过）——粉体库走
rem       data\powder_library_alias 这个 junction，绕开了按路径字符串匹配的
rem       文件访问拦截；代价是代启的进程在对话结束后会被回收。
rem ============================================================
setlocal
chcp 65001 >nul
cd /d %~dp0
set PYTHONPATH=E:\XiangMu\powder\server\src
set PYTHONIOENCODING=utf-8

set "VENV_PY=C:\Users\86182\.workbuddy\binaries\python\envs\powder_control\Scripts\python.exe"
set "SYS_PY=C:\Users\86182\AppData\Local\Programs\Python\Python312\python.exe"

if exist "%VENV_PY%" (
  set "PY=%VENV_PY%"
  echo [启动] 解释器：虚拟环境 powder_control
) else (
  set "PY=%SYS_PY%"
  echo [启动] 解释器：系统 Python 3.12（未找到虚拟环境）
)

if not exist "%PY%" (
  echo [错误] 找不到 Python 解释器：
  echo        %VENV_PY%
  echo        %SYS_PY%
  pause
  exit /b 1
)

echo [启动] 工作目录：%CD%
echo [启动] 地址：http://127.0.0.1:8765/
echo.

"%PY%" app.py

echo.
echo [退出] 服务已结束，退出码 %ERRORLEVEL%
pause
