# HOW TO START THE SIMULATOR
Follow these steps to set up and run the line follower simulation:
1. **Download and Install CoppeliaSim Edu Edition**  
   [CoppeliaSim Edu Edition](https://www.coppeliarobotics.com/)

2. **Install Python**  
   Make sure [Python](https://www.python.org/downloads/) is installed on your system.

3. **Download the Repository**  
   [tir-lineFollower GitHub Repository](https://github.com/IRS-tecMty/tir-lineFollower)

4. **Install Required Python Libraries**  
   Run the following command in your terminal:
   ```bash
   pip install coppeliasim_zmqremoteapi_client numpy pynput
   ```

5. **Open the Simulation File in CoppeliaSim**  
   Open:
   ```
   tir-lineFollower\CoppeliaSim2\linefollower.ttt
   ```

6. **Run the Python Script**  
   Execute:
   ```
   tir-lineFollower\CoppeliaSim2\line_follower.py
   ```
   The robot should now start moving in the CoppeliaSim window.




## Hello world to ESP32-C6 Dev Module


### Install the ESP32-C6 Dev Module Board in Arduino by **Expressif**


1.-Go to your IDE Arduino-> Tools -> Board -> Board Manager 

2.-Search **esp32** by **Exoressif Systems** 

3.-Install the latest version of it


### Install NeoPixel

1.- Go to your IDE Arduino-> Tools -> Manage Libraries
2.-Search **Adafruit NeoPixel** by Adafruit
3.-Install the latest version


Use the next script to test it


```
#include <Adafruit_NeoPixel.h>

#define LED_PIN       8    // pin del LED integrado
#define NUMPIXELS     1    // un solo LED

Adafruit_NeoPixel pixels(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();
  pixels.show(); // Apaga inicialmente
}

void loop() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // rojo
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // azul
  pixels.show();
  delay(2500);
  pixels.setPixelColor(0,pixels.Color(255,255,255));
  pixels.show();
  delay(500);
}

```


