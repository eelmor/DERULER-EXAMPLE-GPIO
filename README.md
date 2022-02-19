# DERULER-EXAMPLE-GPIO

Example project for DERULER (www.deruler.com)

 This example shows how to read and write the state of a pin. The pins used in this example are called GPIO (general purpose input/output) meaning they can be configured as either input or output depending on your preference. Each pin can have one of two states, 1 or 0. These states can also be referred to as high/low or set/reset. A state of 1 means the pin voltage is near the supply voltage (3.3V in our circuit). A state of 0 means the pin voltage is near ground (0V). You can alter the state of the pin and measure the pin voltage with a multimeter to confirm.

In this example we configure the pins connected to the buttons as inputs to be able to read if they are pressed and configure the pins connected to the LED as outputs for adjusting the color. When a button is pressed, it creates a short circuit from the pin to ground (state 0). When the button isn't pressed, the pin is not directly connected to anything and thus undefined. This state is also referred to as floating or Hi-Z (high impedance). When the pin is floating, it may read either 1 or 0. To reliably detect if the button is pressed we have to make sure the voltage of the pin is near the supply voltage when the button is not pressed for it to read as 1. We can achieve this by enabling the built-in pull-up resistor on the pin. This connects a 25-65 kohm resistor from the pin to the supply voltage.
 
