# linSen firmware (huch linear sensor array - HUB)

## Some small hints
download arm toolchain as unix tar file from:
NOTE: the 2011.09 won't do it the easy way 2011.03 works fine for me
https://sourcery.mentor.com/sgpp/lite/arm/portal/subscription3053
and copy it where ever you want. Make it known to the project by setting
TC variable in Makefile.common to that path.
Of course you can install it also by copiing into the system path()...

STM32F10x standard peripheral library(STSW-STM32054) was downloaded from:
http://www.st.com/web/en/catalog/tools/PF257890
Make it known to the project by setting the STMLIB variable in Makefile.common right.

You'll also need a flashing tool. E.g. stm32flash from here:
http://code.google.com/p/stm32flash/downloads/list
compile it and use it:
stm32flash -w FILE -v -g 0x0 /dev/ttyWHATEVER
the -g 0x0 option starts the program on the device after writing it.
Otherwise you'll have to reset the device manually to get the code running.


## hardware in use overview

Timer2, Timer3, Timer4 used for generation of pixel clock and start impuls for sensor

ADC1 reads analog sensor values

DMA1C&2 used for reading adc values and copy them to "user space"

USART1 communication with host - mainly for debug and development

I2C2 communication with host - as use case
