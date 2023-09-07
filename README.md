# ESP32-Weather-Station-Microcontroller-Stuff
Here's a brief description of the code structure for an ESP32 project written in Visual Studio Code using the PlatformIO extension, which monitors air quality using a DHT11 temperature and humidity sensor, as well as a dust sensor. It displays temperature and humidity on an LCD display, and air quality (measured by the dust sensor) is indicated using RGB LEDs. The code also includes a button interrupt to toggle between displaying temperature/humidity and air quality on the LCD.

Libraries and Pin Definitions: Necessary libraries have been included, and pin definitions for sensors, LEDs, and the LCD display have been specified.

Global Variables: Variables for storing previous sensor readings, LCD display status flags, and the current air quality level have been declared.

Functions: Several helper functions have been defined for various tasks, such as setting the RGB LED color based on air quality, scrolling text on the LCD display, printing dust values, and clearing the display.

Initialization: The setup() function initializes essential components, including serial communication, sensor initialization, LCD display setup, and button configuration.

Main Loop: The loop() function contains the main program loop. If the isDustOnTop flag is set, the DustOnTop() function is called to monitor air quality on the top row of the LCD. Otherwise, the TempOnTop() function is called to monitor temperature and humidity on the top row of the LCD.

