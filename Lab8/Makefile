
FlashTx: 
	avra Thomas_Snyder_And_Jesudas_Joseph_Tx_sourcecode.asm
	avrdude -c usbasp -p Atmega128 -U flash:w:Thomas_Snyder_And_Jesudas_Joseph_Tx_sourcecode.hex

FlashRx: receiver
	avrdude -c usbasp -p Atmega128 -U flash:w:Thomas_Snyder_And_Jesudas_Joseph_Rx_sourcecode.hex

receiver: Thomas_Snyder_And_Jesudas_Joseph_Rx_sourcecode.asm.asm
	avra Thomas_Snyder_And_Jesudas_Joseph_Rx_sourcecode.asm


RxChallenge: Thomas_Snyder_And_Jesudas_Joseph_Rx_challengecode.asm
	avra Thomas_Snyder_And_Jesudas_Joseph_Rx_challengecode.asm
	avrdude -c usbasp -p Atmega128 -U flash:w:Thomas_Snyder_And_Jesudas_Joseph_Rx_challengecode.hex


asm: Robot.asm
	avra Robot.asm

challenge:
	avra Thomas_Snyder_Lab8_challengecode.asm
	avrdude -c usbasp -p Atmega128 -U flash:w:Thomas_Snyder_Lab8_challengecode.hex

test: 
	avra timer_test.asm
	avrdude -c usbasp -p Atmega128 -U flash:w:timer_test.hex