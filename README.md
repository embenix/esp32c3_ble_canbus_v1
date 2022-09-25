# ESP32C3 BM-Lite
Interfacing the ESP32-C3-DevKitM-1 with BM-Lite fingerprint sensor from Fingerprint AB.
> Note: This is the work still in progress. However, the fingerprint sensor is initialized and helping functions work just fine.

## Product Links
1. [ESP32-C3-DevKitM-1](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html "ESP32-C3-DevKitM-1")
2. [BM-Lite @ Digikey](https://www.digikey.se/product-detail/en/fingerprint-cards-ab/100018754/2304-100018754-ND/11480144)
3. [BM-Lite @ Fingerprint AB Sweden](https://www.fingerprints.com/solutions/access/bm-lite-development-kit/)


## How to get started:
### Software Requirements:
1. Install the ESP-IDF (Espressif IoT Development Framework) from Espressif. Follow the instructions [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html#installation-step-by-step).
2. Once the ESP-IDF is setup clone this repository by using git: `git clone --recursive https://github.com/embenix/esp32c3_bmlite.git`
3. Use `idf.py build` to build the project and `idf.py -p COMxx flash monitor` to upload the firmware to the dev-kit. 
> Note: **`xx`** is the number of the COM port on your computer

### Hardware Reference:
#### 1. The pinout of the ESP32-C3-DevKitM-1:
<div align="center"> <img src="./information/esp32-c3-devkitm-1-v1-pinout.jpg"/></div>


#### 2. Connection between Fingerprint sensor BM-Lite and ESP32-C3:

| BM-Lite IO Pins      | ESP32-C3 DevKit    |
|:---------------------|:-------------------|
| 01- GND              | GND                |
| 02- SPICLK           | GPIO6              |
| 03- MISO             | GPIO2              |
| 04- MOSI             | GPIO7              |
| 05- CS_N             | GPIO10             |
| 06- IRQ              | GPIO1              |
| 07- RST_N            | GPIO3              |
| 08- UART_RX          | NOT USED           |
| 09- UART_TX          | NOT USED           |
| 10- VDDIO            | 3.3v               |

### Special Notes:
Please do read the following special notes in order to successfully build and test the project.

#### 1. BM-Lite SPI Interface
When using the SPI interface, the UART RX signal should be held at a fixed state to avoid unintentional interference on the UART interface.

<div align="center"><img src="./information/note1_BM-Lite_spi_interface.jpg"/></div>
