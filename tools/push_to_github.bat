@echo off
rem Double-click to push the local branch to the group repository.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0push_to_github.ps1"
pause
