@echo off
rem Run a Python script with the Isaac Lab environment (torch, isaaclab, rsl_rl, h5py, cv2 ...).
rem Usage: tools\isaacpy.cmd script.py [args]   |   tools\isaacpy.cmd -c "print(1)"
rem Override the install location with the ISAACLAB_PATH environment variable.
if "%ISAACLAB_PATH%"=="" set "ISAACLAB_PATH=C:\isaaclab"
call "%ISAACLAB_PATH%\isaaclab.bat" -p %*
