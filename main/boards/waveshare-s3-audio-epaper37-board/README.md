=====================
# 1. Our Goal
Integrate a 3.7-inch 240×416 SPI e‑paper display into an existing ESP32‑S3 project using:

• ESP-IDF (not Arduino)
• LVGL
• A unified C++ display class system (same architecture as existing 1.54-inch driver)
• Pure SPI + GPIO control
• The vendor’s STM32 reference driver as the protocol source

Ultimate objective:
Make the 3.7” display behave like the 1.54” display inside the project architecture, while respecting the display's different hardware protocol.

===========================================================
# 2. What Hardware We Have
Our board
• ESP32-S3 with 8MB PSRAM (Octal, internal MSPI bus)
• I2S audio (already used but NOT blocking SPI)
• No other SPI peripherals
• SD card likely using SDMMC (not SPI)
→ SPI2_HOST or SPI3_HOST is available for e‑paper.

The e‑paper display
• Resolution: 240 × 416
• Color: 1-bit black/white
• Interface: 4-wire SPI (MOSI + SCLK + CS + DC)
• Additional: RESET, BUSY pins
• No MISO
• Uses internal controller with simple waveform commands
• Supports full refresh, fast refresh, and partial refresh, but partial refresh still requires sending full frame.

===========================================================
# 3. Reference Hardware / Code
A. Vendor STM32 driver
• Very simple — no pixel windowing, no complex LUT
• Full-frame writes only
• Commands: 0x10, 0x13, 0x04, 0x12, and init sequences
• Buffer size: 12480 bytes (240/8 × 416)
• “Partial refresh” only changes waveform, but still transmits full frame
• Uses two 1-bit buffers:

oldImage[] = previous frame
image[] = new frame
B. Working 1.54-inch driver already in our project
• Fully integrated into project
• Uses C++ class system LcdDisplay
• Uses LVGL flush callback
• LVGL draws RGB565 frame buffer
• Driver converts LVGL RGB565 → 1-bit pixel writes
• Supports windowed drawing and partial updates (this is different from 3.7")
• Uses same class structure we must follow

===========================================================
# 4. Architecture of Existing Project
It uses a unified C++ Display class system:
Base class: LcdDisplay
Concrete class (for each display):
• CustomLcdDisplay (1.54")
• Epaper37Display (our new one)

Every display object must:

• Initialize SPI
• Initialize GPIOs
• Initialize LVGL
• Allocate LVGL frame buffer
• Provide a flush callback to LVGL
• Manage its own internal 1-bit buffer
• Perform display updates independent of LVGL internal formats

This is already working for 1.54".

LVGL side:
• LVGL always renders in RGB565 (2 bytes per pixel)
• LVGL expects a framebuffer of width * height * 2 bytes
• But the display driver does NOT send this RGB565 buffer to the panel
• The flush_cb examines the given buffer (color_p), converts it to pixels, and writes into the 1-bit buffer

===========================================================
# 5. Similarities Between 1.54" and 3.7"
✔ Both are SPI e‑paper
✔ Both need: MOSI, SCLK, CS, DC, RST, BUSY
✔ Both use 1-bit buffers
✔ Both use LVGL (RGB565 → black/white conversion)
✔ Both require full-frame refresh operations
✔ Same C++ architecture
✔ Same flush callback registration
✔ Same memory (buffer) management principles
✔ Same SPI bus usage (no MISO, 1 direction only)
✔ Same initialization ordering (SPI → LVGL → EPD init → clear → base image)
✔ Same use of heap_caps_malloc for SPIRAM
✔ Same pattern of EPD_DrawColorPixel processing

===========================================================
# 6. Differences Between 1.54" and 3.7"
A. Protocol
1.54":
• Rich EPD controller
• SetWindows, SetCursor, LUT tables
• Supports partial window updates
• Complex waveforms
• Two-phase update (0x24 + 0x26)
• Pixel-level control

3.7":
• Much simpler controller
• No SetWindows, no windowing
• No LUT tables
• Writes must ALWAYS be full-frame
• Two buffers: old_buffer and buffer
• Partial refresh = same size writes but different waveform
• Commands limited to: 0x00,0x10,0x13,0x04,0x12,0xE0,0xE5 etc.

B. LVGL flush behavior
1.54":
• flush_cb must iterate pixels and set them individually
• partial refresh is small-area friendly

3.7":
• flush_cb ignores area
• Always send entire frame
• pixel iteration unnecessary unless custom UI writes directly to 1-bit buffer

C. Resolution and buffer size
1.54": 200×200 → 5000 bytes
3.7": 240×416 → 12480 bytes

D. SPI transfer size constraints
CRITICAL DIFFERENCE

1.54" uses small transfers (≤5000 bytes) → fits ESP-IDF defaults
3.7" uses 12,480-byte transfers → must increase SPI max_transfer_sz

===========================================================
# 7. Target Architecture (What Our Final Driver Must Be)
Here is the correct and final architectural shape we must implement:

1. LVGL
• LVGL framebuffer: RGB565 (Width × Height × 2 bytes)
• LVGL flush_cb registered in constructor
• flush_cb MUST:

Either convert RGB565 → 1-bit pixel map, OR
Ignore color_p & use pre-rendered buffer
ALWAYS call EPD_Display() then EPD_Update()
2. SPI
• Use SPI2_HOST or SPI3_HOST
• DMA enabled
• max_transfer_sz = 12800 or larger (>= 12480 + overhead)
• Only MOSI + SCLK used
• CS, DC, RST, BUSY handled with GPIO

3. E-paper buffers
• Allocate two 1-bit buffers:

buffer (new frame)
old_buffer (previous frame)
• Size = 12480 bytes (calculated from Width/8 * Height)
4. Update logic
• Always full-frame writes
• Commands must follow STM32 sequence exactly
• EPD_Update() must toggle waveform correctly
• For partial refresh:

Use FastInit() or PartInit()
Still send full buffer
5. Pixel mapping
• index = y * (Width/8) + (x >> 3)
• exactly 30 bytes per line for 240px width

===========================================================
# 8. What Works and What Does NOT Work
✔ CONFIRMED WORKING
• LVGL initialize
• LVGL buffer allocation (when fixed)
• SPI bus initializes
• GPIO config works
• E-paper reset works
• Command sending through SPI works
• The e-paper responds enough to flicker
• read_busy() works
• Constructor structure identical to 1.54

❌ CONFIRMED PROBLEMS
• SPI crashes with txdata > host maximum
→ max_transfer_sz too small
→ LVGL buffer or EPD writes exceed SPI limit

• Wrong earlier LVGL buffer size caused memory corruption
→ Now fixed

• Some commands send beyond declared buffer length
→ Must verify writeBytes() usage

• Platform mis-match:
1.54 driver sends raw RGB565 pixels to EPD frame buffer
3.7 driver must NOT do this

===========================================================
# 9. Final Takeaways
The 1.54" driver is NOT a direct template for commands
But it IS the correct template for architecture.

The 3.7" driver MUST follow the STM32 command sequence
Because e-paper protocols differ massively by controller.

LVGL requires a RAM framebuffer in RGB565
Even though the e-paper uses 1-bit data.

SPI must be configured for the largest transaction
Your current transaction is ~12.5 KB.

flush callback must ALWAYS send the full frame
Regardless of LVGL "area" parameter.

Partial refresh only changes waveform — still full-frame write.
===========================================================