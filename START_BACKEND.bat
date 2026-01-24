@echo off
echo ========================================
echo Starting Backend Server
echo ========================================
cd backend
echo.
echo Activating virtual environment...
call .venv\Scripts\activate.bat
echo.
echo Starting uvicorn on http://localhost:8000...
echo API docs will be available at http://localhost:8000/docs
echo.
.venv\Scripts\python.exe -m uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
pause
