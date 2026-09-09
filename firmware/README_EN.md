# MPU6050/F9P Navigation Firmware

[Project home](../README_EN.md) | [中文](README.md) | English

This directory contains STM32F103C8T6 firmware for synchronized MPU6050/ZED-F9P acquisition. TIM2_CH1 captures the F9P time pulse on PA0 and TIM2_CH2 captures MPU6050 DATA_RDY on PA1 in the same 1 MHz timer domain. USART2 receives UBX while USART1 emits GPS-timestamped IMU, GNSS, and sky-view records at 460800 bit/s.

## Current configuration

- Target: STM32F103C8T6
- I2C: PB6/SCL and PB7/SDA
- System clock: 8 MHz HSE multiplied to 72 MHz
- GNSS PPS: PA0/TIM2_CH1, rising-edge input capture
- Data ready: PA1/TIM2_CH2, rising-edge input capture
- GNSS UART: PA2/USART2_TX and PA3/USART2_RX at 115200 bit/s
- Output: PA9/USART1_TX, 460800 bit/s
- RTCM input: PA10/USART1_RX at 460800 bit/s, interrupt-forwarded to PA2/F9P
- Nominal output rate: approximately 100 Hz
- Startup: acquisition begins automatically after power-up or reset

## Wiring

| Peripheral | STM32 pin |
| --- | --- |
| GY-521 VCC | 3.3V |
| GY-521 GND | GND |
| GY-521 SCL | PB6 |
| GY-521 SDA | PB7 |
| GY-521 INT | PA1/TIM2_CH2 |
| C099 TP | PA0/TIM2_CH1 |
| C099 RX_ZED | PA2/USART2_TX |
| C099 TX_ZED | PA3/USART2_RX |
| C099 GND | GND |
| USB-TTL RX | PA9 |
| USB-TTL TX (3.3 V TTL) | PA10 |
| USB-TTL GND | GND |
| ST-LINK SWDIO | PA13/SWDIO |
| ST-LINK SWCLK | PA14/SWCLK |
| ST-LINK GND | GND |
| ST-LINK 3.3V | 3.3V |

Keep BOOT0 low and use a common ground. USB-TTL TX must connect to PA10 for RTCM injection. Avoid feeding the board VCC from multiple power sources.

Place exactly one C099 J4 jumper in `ARDUINO MODE` (pins 7-8, silkscreen `ARD`) so PA2 can drive ZED-F9P RXD. Do not populate the `UART1` or `UART3` routing jumpers at the same time.

At every boot the MCU configures F9P UART1 for 115200 bit/s, UBX/RTCM3 input and UBX-only output. Internal measurements/navigation remain at 10 Hz; UBX-NAV-PVT, UBX-NAV-SAT, and UBX-TIM-TP are each output at 1 Hz. TIMEPULSE uses the GPS grid at 1 Hz with a 100 ms active-high pulse aligned to integer TOW.

## Build

RTCM bridging opens only after startup configuration/retries, preventing UBX configuration bytes from interleaving with corrections. A 2048-byte interrupt-driven queue forwards PA10 to PA2. Use the new Qt credit-controlled sender, not unrestricted BNC output to COM7. F9P performs RTK; STM32 does not. Startup settings are RAM-only and do not require changing native USB configuration.

Every approximately 100 ms, PA9 outputs a comment record; the v3 data/CSV formats are unchanged:

```text
#RTCM,uptime_ms,ready,rx_bytes,tx_bytes,dropped_bytes,uart_errors,gnss_rx_overruns,f9p_frames,f9p_used,f9p_crc_errors,station_id,msg_type,last_rx_age_ms
```

Counters are cumulative 32-bit values since boot. `tx_bytes` counts writes to the UART data register, not receiver confirmation. `f9p_*` are UBX-RXM-RTCM reports; age `4294967295` means none received. Qt limits outstanding bytes to 1024 and stops on errors or two-second confirmation/status timeouts. CLI capture ignores these comment records; no fourth CSV is created. Receiver acceptance/use does not guarantee an RTK fix. See the [u-blox interface specification](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf).

CMake, Ninja, and the GNU Arm Embedded Toolchain are required. Add the tool bundles installed by the STM32Cube VS Code extension to the current PowerShell session:

    $ninjaDir = "$env:LOCALAPPDATA\stm32cube\bundles\ninja\1.13.2+st.1\bin"
    $gccDir = "$env:LOCALAPPDATA\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin"
    $env:Path = "$ninjaDir;$gccDir;$env:Path"

Build Release from the repository root:

    Push-Location firmware
    cmake --preset Release
    cmake --build --preset Release
    Pop-Location

Replace Release with Debug for a debug build. Main output:

    firmware\build\Release\mpu6050_f9p_navigation.elf

The build/ directory is reproducible and excluded from Git.

## Flash

Select the ELF in STM32CubeProgrammer, or run this from the repository root:

    & "$env:LOCALAPPDATA\stm32cube\bundles\programmer\2.23.0\bin\STM32_Programmer_CLI.exe" -c port=SWD mode=UR reset=HWrst -w "firmware\build\Release\mpu6050_f9p_navigation.elf" -v -rst

After reset, PA9 emits protocol-v3 synchronization, navigation, sky-view, and RAWX records:

    IMU,sample,gps_week,gps_tow_us,time_valid,timer_us,ax_raw,ay_raw,az_raw,temp_raw,gx_raw,gy_raw,gz_raw
    GNSS,gps_week,gps_tow_ms,time_valid,rx_timer_us,fix,num_sv,flags,flags2,carr_soln,lat_e7,lon_e7,hmsl_mm,h_acc_mm,v_acc_mm,vel_n_mms,vel_e_mms,vel_d_mms,g_speed_mms,s_acc_mms,pdop_x100
    SAT,gps_week,gps_tow_ms,time_valid,gnss_id,sv_id,cno_dbhz,elev_deg,azim_deg,used
    SAT_END,gps_week,gps_tow_ms,time_valid,num_svs
    RAWX,gps_week,rcv_tow_f64hex,leap_s,rec_stat,num_meas,total_meas,rx_timer_us
    RAWX_MEAS,gnss_id,sv_id,sig_id,freq_id,pr_f64hex,cp_f64hex,do_f32hex,lock_ms,cno,pr_std,cp_std,do_std,trk_stat
    RAWX_END,num_meas

`IMU` records are produced at 100 Hz. Each `gps_tow_us` value is derived directly from the MPU6050 DATA_RDY and F9P 1PPS edges captured in the same TIM2 clock domain. GPS time is usable only when `time_valid=1`; week and TOW are zero before the first valid TIM-TP/PPS association.

`GNSS` records are produced at 1 Hz. `rx_timer_us` is the STM32 local microsecond time when a complete NAV-PVT frame passes checksum validation. Latitude/longitude use `1e-7 deg`; height and position accuracy use mm; NED/ground velocity and speed accuracy use mm/s; PDOP uses a 0.01 scale. `flags` and `flags2` preserve the NAV-PVT quality bits, while `carr_soln` extracts `flags[7:6]` (0=no carrier solution, 1=RTK float, 2=RTK fixed). `SAT`/`SAT_END` provide a 1 Hz sky-view snapshot. A `# sync` diagnostic line is also emitted for every PPS.

`RAWX` is also 1 Hz. Pseudorange, carrier phase, and Doppler are transported as exact IEEE-754 bit-pattern hex together with signal IDs and quality flags. Up to 96 observations are stored, and one RAWX record is emitted per IMU epoch to keep the 460800-bit/s logger responsive.

## How it works

TIM2 runs freely at 1 MHz and is extended to 48 bits with an overflow counter. CH1 and CH2 latch PPS and DATA_RDY in hardware, placing both signals in the same microsecond time domain. Interrupts only transfer capture values; the main loop performs I2C, UBX parsing, and serial output.

## Troubleshooting

- No IMU data: check INT→PA1, PA9→USB-TTL RX, common ground, and 460800 baud.
- GNSS receive works but configuration does not: move the C099 J4 jumper to `ARD` and remove the `UART1/UART3` jumpers.
- Ninja or GCC not found: update the versioned bundle paths above.
- Flash failure: verify BOOT0=0, ST-LINK wiring and driver, then retry at a lower SWD frequency.
- Wrong sample rate: make sure INT is connected to PA1 and that the main loop is not blocked.

Continue with the [Qt monitor](../host/README_EN.md) and [tools guide](../tools/README_EN.md).
