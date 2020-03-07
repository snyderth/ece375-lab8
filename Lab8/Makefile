
FlashTx: 
	avra Thomas_Snyder_And_Jesudas_Joseph_Tx_sourcecode.asm
	avrdude -c usbasp -p Atmega128 -U flash:w:Thomas_Snyder_And_Jesudas_Joseph_Tx_sourcecode.hex

FlashRx: receiver
	avrdude -c usbasp -p Atmega128 -U flash:w:receiver.hex

flash: asm
	avrdude -c usbasp -p Atmega128 -U flash:w:Robot.hex

receiver: receiver.asm
	avra receiver.asm

asm: Robot.asm
	avra Robot.asm

challenge:
	avra Thomas_Snyder_Lab8_challengecode.asm
	avrdude -c usbasp -p Atmega128 -U flash:w:Thomas_Snyder_Lab8_challengecode.hex

test: 
	avra timer_test.asm
	avrdude -c usbasp -p Atmega128 -U flash:w:timer_test.hex