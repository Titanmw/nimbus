@echo off
powershell -Command "$ip = (wsl hostname -I).Split(' ')[0]; netsh interface portproxy delete v4tov4 listenport=5000 listenaddress=0.0.0.0; netsh interface portproxy add v4tov4 listenport=5000 listenaddress=0.0.0.0 connectport=5000 connectaddress=$ip"
echo.
echo Portweiterleitung für WSL wurde gesetzt.
pause
