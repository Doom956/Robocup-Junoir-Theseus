# Electronics and Sensor Bill of Materials

This BOM consolidates the parts listed in `BoM for worlds.xlsx` with the component details and links recorded in `Electronics_Sensor Bill of materials..docx`.

## Reading this BOM

- **Required** is the quantity needed for one competition robot, as recorded in the spreadsheet.
- **Existing** is the quantity already installed on the robots.
- **Spare** is uninstalled inventory.
- **Status** is marked **reused** where the spreadsheet records existing or spare inventory. Verify the physical inventory before submission.
- **Kit/custom-built** identifies purchased components as **kit/off-the-shelf** and team-made assemblies as **custom-built**.
- Prices were checked on **June 12, 2026** and exclude tax and shipping.
- Where the exact model is still unknown, a price is included only when a clearly labeled representative assumption can be made. Other entries remain **TBD** rather than using an unreliable estimate.

## Bill of materials

| Category | Component name / description (part number where known) | Supplier / source | Status | Kit or custom-built | Required | Existing | Spare | Unit price (USD) |
|---|---|---|---|---|---:|---:|---:|---:|
| Processing | Processor/controller board (exact model TBD) | Supplier TBD | Reused; existing inventory | Kit/off-the-shelf | 0 | 2 | 1 | TBD - exact model required |
| Sensor | Adafruit VL53L0X time-of-flight distance sensor breakout, PID 3317 | [Adafruit product page](https://www.adafruit.com/product/3317) | Reused; existing inventory | Kit/off-the-shelf | 7 | 14 | 1 | $14.95 |
| Sensor | Adafruit TCS34725 color sensor breakout, PID 1334 | [Adafruit product page](https://www.adafruit.com/product/1334); discontinued | Reused; existing inventory | Kit/off-the-shelf | 1 | 2 | 0 | $7.95 listed; discontinued |
| Sensor | Adafruit BNO055 absolute-orientation sensor breakout, PID 2472 | [Adafruit product page](https://www.adafruit.com/product/2472) | Reused; existing inventory | Kit/off-the-shelf | 1 | 2 | 0 | $34.95 |
| Sensor | Motor encoders (assumption: Pololu 20D magnetic encoder pair kit, item 3499; spreadsheet quantities are packs) | [Pololu product page](https://www.pololu.com/product/3499); confirm installed encoder model | Reused; spare inventory; installed quantity unknown | Kit/off-the-shelf | 2 packs | Unknown | 1 pack | $9.95 per pair kit |
| Vision | OpenMV Cam H7 Plus, SKU OMV-CAM-H7-PLUS-V1 | [OpenMV product page](https://openmv.io/products/openmv-cam-h7-plus); sold out / production-run order only | Reused; existing inventory | Kit/off-the-shelf | 2 | 4 | 0 | $135.00 |
| Actuator | Pololu gearmotor with extended shaft; spreadsheet says 150:1, linked item 3493 is 195:1 | [Pololu item 3493](https://www.pololu.com/product/3493); confirm ratio/model before purchase | Reused; existing inventory | Kit/off-the-shelf | 4 | 8 | 1 | $32.95 for linked 195:1 model |
| Actuator | 28BYJ-style 5 V geared stepper motor (representative assumption: Adafruit PID 858) | [Adafruit product page](https://www.adafruit.com/product/858); out of stock; confirm exact model | Reused; existing and spare inventory | Kit/off-the-shelf | 1 | 2 | 4 (stored at team member's house) | $4.95 representative |
| Display | LCD screen (exact model and size TBD) | Supplier TBD | Reused; existing and spare inventory | Kit/off-the-shelf | 1 | 1 | 1 (stored at team member's house) | TBD |
| Lighting | LED strips (type/length TBD) | Supplier TBD | Reused; existing inventory | Kit/off-the-shelf | 3 | 3 | 0 | TBD |
| Mechanical | M4 suspension screws (length/type TBD) | Supplier TBD | Reused; existing inventory | Kit/off-the-shelf | 1 set | 2 sets | 0 | TBD |
| Mechanical | Ball bearings (size/model TBD) | Supplier TBD | Reused; existing and spare inventory | Kit/off-the-shelf | 2 | 4 | 5 | TBD |
| Mechanical | Motor brackets (exact model TBD) | Supplier TBD | Reused; existing and spare inventory | Kit/off-the-shelf | 4 | 8 | 6 | TBD |
| Electronics | Protoboard (representative assumption: Adafruit half-size Perma-Proto, PID 1609) | [Adafruit product page](https://www.adafruit.com/product/1609); confirm dimensions | Reused; existing and spare inventory | Kit/off-the-shelf | 1 | 2 | 1 | $4.50 representative |
| Wiring | Qwiic cables (assumption: 100 mm JST-SH 4-pin cable, Adafruit PID 4210) | [Adafruit product page](https://www.adafruit.com/product/4210); confirm lengths | Reused; existing inventory | Kit/off-the-shelf | 10 | 18 | 0 | $0.95 each |
| Safety/electronics | Circuit breaker (rating/model TBD) | Supplier TBD | Reused; existing and spare inventory | Kit/off-the-shelf | 1 | 1 | 3 | TBD |
| Wiring | Wire clips (type and quantity TBD) | Supplier TBD | Inventory status TBD | Kit/off-the-shelf | TBD | TBD | TBD | TBD |
| Wiring | JST-PH connectors/cables (assumption: Pololu 6-pin female-female 10 cm cable, item 5643) | [Pololu 6-pin JST-PH cable listing](https://www.pololu.com/category/361/6-pin-jst-ph-style-cables); confirm pin count/length | Reused; existing and spare inventory | Kit/off-the-shelf | 4 | 8 | 2 | $3.23 representative |
| Power | Pololu D24V50F5 5 V step-down voltage regulator, item 2851 | [Pololu product page](https://www.pololu.com/product/2851) | Reused; existing inventory; verify possible school spares | Kit/off-the-shelf | 1 | 1 | 0 confirmed | $29.95 |
| Control | CAROBOT Motor Shield V3, SKU 2337 | [Canada Robotix product page](https://www.canadarobotix.com/products/2337) | Reused; quantity must be verified | Kit/off-the-shelf | TBD | TBD | TBD | CAD $19.99; USD price TBD |
| Control | SparkFun Qwiic Mux Breakout - 8 Channel (TCA9548A), BOB-16784 | [SparkFun product page](https://www.sparkfun.com/sparkfun-qwiic-mux-breakout-8-channel-tca9548a.html) | Reused; quantity must be verified | Kit/off-the-shelf | TBD | TBD | TBD | $6.95 |
| Power | LiPo battery (voltage, capacity, and model TBD) | Supplier TBD | Reused; quantity and condition must be verified | Kit/off-the-shelf | TBD | TBD | TBD | TBD |
| Assembly | Robot mechanical structure and wiring assembly | Team designed and assembled | Reused; verify current configuration | Custom-built | 1 assembly | TBD | TBD | Material costs TBD |

## Pricing summary

- **Partial replacement subtotal: $631.07 USD**, based only on rows with known required quantities and traceable USD prices or explicitly stated representative assumptions.
- The subtotal excludes tax, shipping, the CAD-priced CAROBOT shield, all rows with unknown required quantities, and all rows still marked **TBD**.
- Listed prices and availability can change; recheck them before ordering.

## Required follow-up before submission

1. Confirm exact models/specifications and enter a traceable supplier and current unit price for every remaining **TBD** entry.
2. Confirm the exact controller board, encoders, LCD, LED strips, screws, bearings, brackets, protoboard, circuit breaker, wire clips, JST-PH connectors, battery, and stepper-motor model.
3. Resolve the spreadsheet's gearmotor description (**150:1**) against the existing Word document's Pololu reference, which points to a **195:1** gearmotor.
4. Verify all physical quantities, especially encoders, wire clips, motor shields, multiplexers, batteries, and the custom-built assembly.
5. Record whether any replacement parts will be purchased new; the current status is based only on the recorded inventory.
