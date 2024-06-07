".\srec_cat.exe" ..\Release\SPIM_DMM_RUN_CODE.hex -intel -crop 0x00000000 0x000FFFFF -o SPIM_DMM_RUN_CODE_aprom.hex -intel -obs=16
".\srec_cat.exe" SPIM_DMM_RUN_CODE_aprom.hex -intel -offset -0x00000000 -o SPIM_DMM_RUN_CODE_aprom.bin -binary
".\srec_cat.exe" ..\Release\SPIM_DMM_RUN_CODE.hex -intel -crop 0x00100000 0x020FFFFF -o SPIM_DMM_RUN_CODE_spim.hex  -intel -obs=16

".\srec_cat.exe" SPIM_DMM_RUN_CODE_spim.hex -intel -offset -0x00100000 -o SPIM_DMM_RUN_CODE_spim.bin -binary
pause
