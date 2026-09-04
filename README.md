# ESP32 + SSD1306 OLED + Servo

ESP32-D0WD devkit driving a 0.96" SSD1306 OLED (the dual-colour kind with a yellow
top band) and an SG90-class hobby servo.

## Wiring

| Signal | Pin |
| --- | --- |
| OLED VCC | 3.3V |
| OLED GND | GND |
| OLED SDA | GPIO21 |
| OLED SCL | GPIO22 |
| Servo VCC | 5V (external supply preferred) |
| Servo GND | GND (common with the ESP32) |
| Servo signal | GPIO23 |

The panel is probed on **both** pin orders (SDA21/SCL22 and SDA22/SCL21) and the
working one is used, so a swapped pair still comes up — the serial log says which
order answered. If nothing answers the bus is rescanned every 2 s while the servo
keeps running, so the display can be wired up with the board powered.

Do not feed the servo from the 3.3V rail: its stall current browns out the ESP32.

## Display

* **Yellow band (top 16 px)** — mode (`SWEEP`/`MANUAL`) and the current angle.
* **Blue area** — a 180° dial: outer arc for the full travel range, an inner arc
  showing the travelled portion, ticks every 30° (longer at 0/90/180), and a
  needle at the current angle. Pulse width in µs at the left.

## Motion

It boots straight into `SWEEP` and runs on its own — no button, no host, nothing
to press. Serial is only there if you want to take over.

* `SWEEP` — 0° → 180° → 0°, 2.5 s each way with a 400 ms dwell at both ends. The
  travel is cosine-eased, so the horn accelerates and decelerates instead of
  slamming into the end stops.
* `MANUAL` — moves to a commanded angle at a 150 °/s slew limit.

Servo output is refreshed at 50 Hz, the panel redraws at 20 Hz; the loop is
non-blocking (no `delay()` in `loop()`), so serial stays responsive during motion.

## Serial commands (115200 baud)

| Command | Effect |
| --- | --- |
| `0`–`180` | move to that angle (switches to MANUAL) |
| `s` | resume the sweep from the current position |
| `c` | centre at 90° |
| `i` | rescan the I2C bus and report every address found |
| `p` | pin hunt: drive each candidate GPIO in turn, 3 s each, twitching 50°↔130° |
| `u<gpio>` | move the servo signal to that GPIO and resume sweeping |
| `?` | print mode, angle, pulse width, servo pin, panel address |

## If the servo does not move

Split the board from the servo before guessing at either.

**`m` settles the board half.** It reads the servo pin back through the GPIO matrix
— no jumper, no scope — and prints pulse width, period and rate. Healthy output on
this build looks like:

```
measure GPIO23 (servo on GPIO23): 25 pulses / 500 ms, high=1445 us, period=20000 us (50.0 Hz), commanded=1450 us
```

Pulse width tracking the commanded angle at 50.0 Hz means the firmware, the pin and
the timer are all fine, and the fault is downstream. `m19` measures a different pin
instead, for when you want a jumper-verified second opinion.

**`p` settles the wrong-hole case** — it walks the signal across candidate GPIOs,
3 s each, twitching the horn 50°↔130° while the pin number fills the screen. Watch
for the twitch, then `u<gpio>` to pin it there. Lead colours on a TowerPro SG90 are
brown = GND, red = +5V, signal = orange **or yellow** depending on the batch.

**What does not work as a test:** turning the horn by hand. A micro servo's gear
train is stiff to backdrive with the power off too, so "it resists, therefore it is
powered" is not a valid inference — it cost an hour here.

A better free signal: watch for a brownout. A servo that is genuinely trying to move
draws hundreds of mA on each start, which sags a USB-fed 5V rail enough to reset the
ESP32 or blink the panel. Full-range commands that produce no disturbance at all
mean the servo is drawing nothing — no power reaching it, or a dead servo. On this
desk it was a dead servo: correct wiring, verified pulses, no current draw, no motion.

## Build

```bash
arduino-cli lib install "U8g2" "ESP32Servo"
arduino-cli compile --fqbn esp32:esp32:esp32 .
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32 .
```

Tested on an ESP32-D0WD rev1 devkit (CP2102N USB bridge), esp32 Arduino core 3.3.0,
U8g2 2.36.19, ESP32Servo 3.2.1.

## Pulse range

`US_MIN`/`US_MAX` are set to 500–2400 µs, which is the usual SG90 range. If the
horn buzzes or grinds at an end of travel, narrow it (e.g. 600–2300) — the servo
is being commanded past its mechanical stop.
