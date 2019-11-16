# 8556_backlight

Use STM32 as main chip to control  back light board's brightness. The back light panel is driven by LP8556 and TPS61199.

- STM32 use UART to receive commands from PC.
- STM32 use i2c or PWM to control LP8556.
- STM32 use PWM to control TPS61199.
- STM32 can save the configuration to its flash.

