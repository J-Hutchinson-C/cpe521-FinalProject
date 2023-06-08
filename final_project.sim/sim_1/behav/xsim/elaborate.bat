@echo off
REM ****************************************************************************
REM Vivado (TM) v2020.2 (64-bit)
REM
REM Filename    : elaborate.bat
REM Simulator   : Xilinx Vivado Simulator
REM Description : Script for elaborating the compiled design
REM
REM Generated by Vivado on Tue Jun 06 13:01:23 -0700 2023
REM SW Build 3064766 on Wed Nov 18 09:12:45 MST 2020
REM
REM Copyright 1986-2020 Xilinx, Inc. All Rights Reserved.
REM
REM usage: elaborate.bat
REM
REM ****************************************************************************
REM elaborate design
echo "xelab -wto 3c2cd1beb0dd49338d2ed37565099e8b --incr --debug typical --relax --mt 2 -L xil_defaultlib -L uvm -L unisims_ver -L unimacro_ver -L secureip --snapshot clock_tb_behav xil_defaultlib.clock_tb xil_defaultlib.glbl -log elaborate.log"
call xelab  -wto 3c2cd1beb0dd49338d2ed37565099e8b --incr --debug typical --relax --mt 2 -L xil_defaultlib -L uvm -L unisims_ver -L unimacro_ver -L secureip --snapshot clock_tb_behav xil_defaultlib.clock_tb xil_defaultlib.glbl -log elaborate.log
if "%errorlevel%"=="0" goto SUCCESS
if "%errorlevel%"=="1" goto END
:END
exit 1
:SUCCESS
exit 0