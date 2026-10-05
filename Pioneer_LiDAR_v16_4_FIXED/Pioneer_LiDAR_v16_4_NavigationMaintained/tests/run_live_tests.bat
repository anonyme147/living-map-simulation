@echo off
cd /d %~dp0\..
python tests\run_tests.py --live %*
