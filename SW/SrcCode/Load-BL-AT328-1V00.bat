"C:\Program Files (x86)\Arduino\hardware\tools\avr/bin/avrdude" -C"C:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -F -v -patmega328p -cstk500v1 -PCOM13 -b19200 -e -Ulock:w:0x3F:m -Uefuse:w:0xFD:m -Uhfuse:w:0xDE:m -Ulfuse:w:0xFF:m 
"C:\Program Files (x86)\Arduino\hardware\tools\avr/bin/avrdude" -C"C:\Program Files (x86)\Arduino\hardware\tools\avr/etc/avrdude.conf" -F -v -patmega328p -cstk500v1 -PCOM13 -b19200 -Uflash:w:"C:\Program Files (x86)\Arduino\hardware\arduino\avr/bootloaders/optiboot/optiboot_atmega328.hex":i -Ulock:w:0x0F:m 

