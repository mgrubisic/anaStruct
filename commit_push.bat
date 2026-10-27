@echo off
setlocal EnableDelayedExpansion
REM ============================================================
REM  commit_push.bat - commit and push anaStruct_KIMI to GitHub
REM  Target: https://github.com/mgrubisic/anaStruct (master)
REM
REM  Usage:
REM    commit_push.bat                 - default commit message
REM    commit_push.bat "My message"    - custom commit message
REM
REM  First run: initialises a local repo here and ADOPTS the
REM  remote master history (no force-push, working tree is never
REM  overwritten). Every run shows the pending changes and asks
REM  for confirmation before committing and pushing.
REM ============================================================
cd /d "%~dp0"

set "REMOTE_URL=https://github.com/mgrubisic/anaStruct.git"
set "BRANCH=master"
set "DEFAULT_MSG=Add imposed displacement/rotation of supports and thermal loads on elements"

REM commit message: first argument or default (no second prompt -
REM multiple "set /p" prompts do not work reliably with
REM piped/redirected input)
if "%~1"=="" (
    set "MSG=%DEFAULT_MSG%"
) else (
    set "MSG=%~1"
)

where git >nul 2>nul
if errorlevel 1 (
    echo [ERROR] git is not on PATH.
    pause
    exit /b 1
)

REM ---- first-time setup: adopt remote history ----
if not exist ".git\" (
    echo [INFO] No .git in this folder - initialising repo and fetching %REMOTE_URL% ...
    git init -b %BRANCH%
    if errorlevel 1 goto :error
    git remote add origin %REMOTE_URL%
    if errorlevel 1 goto :error
    git fetch origin %BRANCH%
    if errorlevel 1 goto :error
    REM Point local master at the fetched remote master WITHOUT touching local files
    git update-ref refs/heads/%BRANCH% origin/%BRANCH%
    if errorlevel 1 goto :error
    git reset
    if errorlevel 1 goto :error
    echo.
    echo [INFO] Local branch %BRANCH% now points at origin/%BRANCH%.
    echo [INFO] Local files were NOT modified - they are listed as pending changes below.
)

REM ---- review ----
echo.
echo Commit message: !MSG!
echo.
echo Pending changes:
echo --------------------------------------------------
git status --short
echo --------------------------------------------------
echo.
set /p "CONFIRM=Commit and push these changes to %REMOTE_URL% (%BRANCH%)? [y/N] "
if /i not "!CONFIRM!"=="y" (
    echo Aborted - nothing was changed.
    pause
    exit /b 0
)

git add -A
if errorlevel 1 goto :error

git diff --cached --quiet
if not errorlevel 1 (
    echo [INFO] Nothing to commit - working tree already matches origin/%BRANCH%.
    pause
    exit /b 0
)

git commit -m "!MSG!"
if errorlevel 1 goto :error

git push -u origin %BRANCH%
if errorlevel 1 goto :error

echo.
echo [OK] Committed and pushed to %REMOTE_URL% (%BRANCH%).
pause
exit /b 0

:error
echo.
echo [ERROR] A git command failed - see the messages above. Nothing was pushed.
pause
exit /b 1
