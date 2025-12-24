# NOTES.md - RTIC OLED Project Development Log

This document tracks all the compilation errors encountered during development and their solutions. Useful reference for debugging similar issues.

---

## Initial Compilation Errors (6 errors)

### Error 1: BufferedGraphicsMode Missing Generic Parameter

**Error Message:**

```
error[E0423]: expected value, found struct `BufferedGraphicsMode`
error[E0107]: missing generics for struct `ssd1306::mode::BufferedGraphicsMode`
```

**Problem:** `BufferedGraphicsMode` requires a generic parameter specifying the display size.

**Wrong:**

```rust
display: Ssd1306<DisplayInterfaceType, DisplaySize128x32, BufferedGraphicsMode>,
```

**Fixed:**

```rust
display: Ssd1306<DisplayInterfaceType, DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>,
```

**Why:** The display mode needs to know the buffer size at compile time for static allocation.

---

### Error 2: RCC cfgr Field Access

**Error Message:**

```
error[E0616]: field `cfgr` of struct `stm32f4xx_hal::pac::rcc::RegisterBlock` is private
```

**Problem:** In newer HAL versions, `cfgr` is a method, not a field.

**Wrong:**

```rust
let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();
```

**Fixed:**

```rust
let clocks = rcc.cfgr().sysclk(48.MHz()).freeze();
```

**Why:** HAL API changed to use builder pattern with method calls.

---

### Error 3: Ambiguous Numeric Type for Hz

**Error Message:**

```
error[E0689]: can't call method `Hz` on ambiguous numeric type `{float}`
```

**Problem:** Rust can't infer the type of `0.5` - could be f32, f64, etc.

**Wrong:**

```rust
timer.start(0.5.Hz()).unwrap();
```

**Fixed:**

```rust
timer.start(2_u32.Hz()).unwrap();  // 2 Hz = 0.5 second period
```

**Why:** Need to specify frequency (2 Hz) not period (0.5 Hz). Timer expects integer Hz values.

---

### Error 4: Missing Write Trait Import

**Error Message:**

```
error[E0599]: cannot write into `heapless::String<32>`
note: must implement `io::Write`, `fmt::Write`, or have a `write_fmt` method
```

**Problem:** The `write!` macro requires `core::fmt::Write` trait to be in scope.

**Wrong:**

```rust
write!(line, "Timer tick: {}", *cx.local.counter).ok();
// Missing import
```

**Fixed:**

```rust
use core::fmt::Write;  // Add at top of file

write!(line, "Timer tick: {}", *cx.local.counter).ok();
```

**Why:** Rust traits must be imported to use their methods.

---

### Error 5: Text Style Ownership

**Error Message:**

```
error[E0599]: the method `draw` exists for struct `Text<'_, &mut MonoTextStyle<'static, BinaryColor>>`, but its trait bounds were not satisfied
```

**Problem:** Passing `&mut MonoTextStyle` when `&MonoTextStyle` is needed.

**Wrong:**

```rust
Text::new(&line, Point::new(0, 8), cx.local.text_style).draw(display).ok();
```

**Fixed:**

```rust
Text::new(&line, Point::new(0, 8), *cx.local.text_style).draw(display).ok();
```

**Why:** `text_style` is already a reference from `cx.local`, need to dereference it.

---

### Error 6: BufferedGraphicsMode Constructor

**Error Message:**

```
error[E0624]: associated function `new` is private
error[E0308]: expected `DisplayRotation`, found `BufferedGraphicsMode<_>`
```

**Problem:** Can't construct `BufferedGraphicsMode` directly; must use `.into_buffered_graphics_mode()`.

**Wrong:**

```rust
let mut display = Ssd1306::new(interface, DisplaySize128x32, BufferedGraphicsMode::new())
```

**Fixed:**

```rust
let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
    .into_buffered_graphics_mode();
```

**Why:** The SSD1306 driver uses a builder pattern for mode transitions.

---

## HAL Version Compatibility Issues

### Problem: HAL 0.23.0 vs 0.21.0 API Differences

When using `stm32f4xx-hal = "0.23"`, encountered multiple API incompatibilities:

1. **GPIO split() requires &mut rcc parameter**
2. **Different I2C initialization signature**
3. **Timer API changes**

**Solution:** Downgraded to version 0.21.0 which has stable, well-documented API:

```toml
stm32f4xx-hal = { version = "0.21", features = ["stm32f446"] }
```

**Note:** Version 0.21.0 does not have an `rt` feature (runtime support comes from `cortex-m-rt` instead).

---

## Display Interface Type Confusion

### Error: I2CDisplayInterface vs I2CInterface

**Error Message:**

```
error[E0107]: struct takes 0 generic arguments but 1 generic argument was supplied
note: struct defined here, with 0 generic parameters
pub struct I2CDisplayInterface(());
```

**Problem:** `ssd1306` v0.8 has multiple interface types with confusing names:

- `I2CDisplayInterface` - Legacy, takes no generics
- `I2CInterface<I2C>` - Current, takes I2C type as generic

**Wrong:**

```rust
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
type DisplayInterfaceType = I2CDisplayInterface<I2cProxy>;
```

**Fixed:**

```rust
use ssd1306::{prelude::*, Ssd1306};  // I2CInterface is in prelude
// Use I2CInterface directly, don't create type alias
display: Ssd1306<I2CInterface<I2cProxy>, DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>,
```

**Constructor:**

```rust
let interface = I2CInterface::new(bus.acquire_i2c(), 0x3C, 0x40);
// 0x3C = I2C address
// 0x40 = Data/command byte indicator
```

---

## Hardware-Specific Issues

### Issue: Wrong GPIO Pin for LED

**Problem:** Used PD12 (STM32F4 Discovery LED pin) instead of PA5 (Nucleo LED pin).

**Symptoms:**

- Code compiles and runs
- Timer interrupts fire
- OLED updates correctly
- LED doesn't blink

**Discovery Board (STM32F407VG):**

- LEDs on PD12, PD13, PD14, PD15 (green, orange, red, blue)

**Nucleo Board (STM32F446RE):**

- LED on PA5 (LD2, green, active-low)

**Fixed:**

```rust
// Wrong (Discovery):
let gpiod = dp.GPIOD.split();
let led = gpiod.pd12.into_push_pull_output();

// Correct (Nucleo):
let gpioa = dp.GPIOA.split();
let led = gpioa.pa5.into_push_pull_output();
```

**Additional Fix:** Added HSE (external crystal) configuration:

```rust
let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();
```

---

## Timer Configuration Issues

### Error: clear_flags() Expects Flag, Not Event

**Error Message:**

```
error[E0277]: the trait bound `BitFlags<Flag, u32>: From<Event>` is not satisfied
```

**Problem:** Timer interrupt clearing uses `Flag` enum, not `Event` enum.

**Wrong:**

```rust
cx.local.timer.clear_flags(Event::Update);
```

**Fixed:**

```rust
use stm32f4xx_hal::timer::Flag;  // Add import

cx.local.timer.clear_flags(Flag::Update);
```

**Why:** `Event` is for configuring interrupt sources, `Flag` is for clearing interrupt flags.

---

## Rust 2024 Edition Compatibility Warnings

### Warning: static_mut_refs

**Warning Message:**

```
warning: creating a shared reference to mutable static
= note: shared references to mutable statics are dangerous
```

**Problem:** Rust 2024 edition prep warns about `static mut` usage patterns.

**Original Code:**

```rust
static mut I2C_BUS: Option<I2cBus> = None;
let bus = unsafe {
    I2C_BUS = Some(BusManagerSimple::new(i2c));
    I2C_BUS.as_ref().unwrap()  // ← Warning here
};
```

**Solution Options:**

1. **Suppress Warning (Quick fix):**

   ```rust
   #![allow(static_mut_refs)]
   ```

2. **Use MaybeUninit with Raw Pointers (Future-proof):**

   ```rust
   use core::mem::MaybeUninit;

   static mut I2C_BUS: MaybeUninit<I2cBus> = MaybeUninit::uninit();
   let bus = unsafe {
       let bus_ptr = I2C_BUS.as_mut_ptr();
       bus_ptr.write(BusManagerSimple::new(i2c));
       &*bus_ptr
   };
   ```

**Note:** For embedded projects in 2021 edition, the warning can be safely ignored or suppressed. The code is sound because `init()` runs exactly once before any interrupts are enabled.

---

## Shared Bus Configuration

### Why We Need Shared Bus

**Problem:** I2C peripheral can only be owned by one driver at a time, but we might want multiple devices on the same bus (OLED, sensors, etc.).

**Solution:** Use `shared-bus` crate with `NullMutex` for interrupt-free critical sections.

**Setup:**

```rust
use shared_bus::BusManagerSimple;

type I2cBus = BusManagerSimple<I2c<pac::I2C1>>;
type I2cProxy = shared_bus::I2cProxy<'static, shared_bus::NullMutex<I2c<pac::I2C1>>>;

// In init():
static mut I2C_BUS: MaybeUninit<I2cBus> = MaybeUninit::uninit();
let bus = unsafe { /* ... */ };

// Each device gets a proxy:
let interface = I2CInterface::new(bus.acquire_i2c(), 0x3C, 0x40);
```

**Note:** `NullMutex` is safe because we're only accessing I2C from one task (tim2_handler).

---

## Common Pitfalls & Tips

### 1. RTIC Resource Syntax

Resources in RTIC 2.x use a different syntax than 1.x:

**RTIC 1.x (old):**

```rust
#[resources]
struct Resources {
    led: PA5<Output<PushPull>>,
}
```

**RTIC 2.x (current):**

```rust
#[local]
struct Local {
    led: Pin<'A', 5, Output>,
}

#[shared]
struct Shared {
    // Shared between tasks
}
```

### 2. Task Binding Syntax

**Correct:**

```rust
#[task(binds = TIM2, local = [led, timer, display])]
fn tim2_handler(cx: tim2_handler::Context) { }
```

**Common mistakes:**

- Wrong: `#[task(bind = TIM2)]` (singular)
- Wrong: `#[interrupt]` (RTIC 1.x syntax)

### 3. Type-Level Pin Encoding

HAL uses const generics for type-safe pin handling:

```rust
Pin<'A', 5, Output>  // Port A, Pin 5, Output mode
Pin<'D', 12, Output>  // Port D, Pin 12, Output mode
```

These are **different types** - compiler prevents pin mixups!

### 4. Active-Low vs Active-High LEDs

**Nucleo boards:** LED is active-low

```rust
led.set_low();   // LED ON
led.set_high();  // LED OFF
```

**Arduino/Discovery:** Often active-high

```rust
led.set_high();  // LED ON
led.set_low();   // LED OFF
```

Always check your board schematic!

---

## Useful Debug Techniques

### 1. RTT Logging with defmt

Add debug prints to trace execution:

```rust
defmt::println!("Timer interrupt #{}", counter);
defmt::println!("LED state: {}", led.is_set_high());
```

### 2. Initialization Markers

Test hardware during init:

```rust
led.set_low();
defmt::println!("LED should be ON now");
// Look at the board - is LED actually on?
```

### 3. Verify Clock Configuration

```rust
defmt::println!("SYSCLK: {} MHz", clocks.sysclk().to_Hz() / 1_000_000);
defmt::println!("PCLK1: {} MHz", clocks.pclk1().to_Hz() / 1_000_000);
```

### 4. I2C Scanner (if OLED not working)

Can add a simple I2C scanner in init to find devices:

```rust
for addr in 0x00..=0x7F {
    if i2c.write(addr, &[]).is_ok() {
        defmt::println!("Found device at 0x{:02X}", addr);
    }
}
```

---

## Performance Considerations

### Memory Usage

- **Framebuffer**: 128x32 = 4096 bits = 512 bytes
- **Stack**: Each task needs stack space for locals
- **Static**: I2C bus manager is ~100 bytes

### Timing

- **Interrupt latency**: ~1-2 µs (depends on clock speed)
- **Display update**: ~5-10 ms (I2C at 400 kHz)
- **Timer accuracy**: Crystal-dependent (±50 ppm with HSE)

### Optimization Tips

1. Use `--release` for production (10x smaller, faster)
2. Enable LTO in Cargo.toml for size optimization
3. Consider DMA for I2C if doing frequent updates
4. Use `wfi` in idle loop to save power

---

## Future Enhancements Checklist

- [ ] Add button input with debouncing
- [ ] Implement menu system on OLED
- [ ] Add multiple timer tasks
- [ ] Integrate LoRa module (SX1276/RFM95)
- [ ] Add shared SPI bus for multiple devices
- [ ] Implement message queue between tasks
- [ ] Add low-power sleep modes
- [ ] Create reusable library crate

---

## References

- [RTIC Book](https://rtic.rs/2/book/en/)
- [stm32f4xx-hal v0.21 docs](https://docs.rs/stm32f4xx-hal/0.21.0/)
- [ssd1306 driver docs](https://docs.rs/ssd1306/0.8.4/)
- [shared-bus docs](https://docs.rs/shared-bus/0.3.1/)
- [STM32F446RE Reference Manual](https://www.st.com/resource/en/reference_manual/dm00135183.pdf)
- [Nucleo-F446RE User Manual](https://www.st.com/resource/en/user_manual/dm00105823.pdf)

---

## Debugging Hardware Connection Issues

### Error: JtagNoDeviceConnected

**Symptoms:**

```
WARN probe_rs::probe::stlink: send_jtag_command 242 failed: JtagNoDeviceConnected
Error: Connecting to the chip was unsuccessful.
Caused by:
    0: An ARM specific error occurred.
    1: The debug probe encountered an error.
    2: An error which is specific to the debug probe in use occurred.
    3: Command failed with status JtagNoDeviceConnected.
```

**What This Means:**
The ST-Link debugger can't communicate with the STM32 chip. This usually happens when:

1. The chip is in a locked or corrupted state
2. Previous flash operation was interrupted
3. Code crashed and left debug interface in bad state
4. WFI (Wait For Interrupt) sleep mode is too deep

**Solution:**
Perform a chip erase with connect-under-reset:

```bash
probe-rs erase --chip STM32F446RETx --protocol swd --connect-under-reset
```

**What This Does:**

- `--protocol swd`: Use Serial Wire Debug (2-wire protocol, not JTAG's 5-wire)
- `--connect-under-reset`: Holds chip in reset while connecting
- Erases flash and resets debug interface to known good state

**After Erase:**
You can now flash normally again:

```bash
cargo run --release
```

**Prevention Tips:**

1. **Always use WFI carefully**: Deep sleep modes can make debug connection difficult
2. **Add timeout before WFI**: Give debugger time to connect
   ```rust
   // In idle loop, instead of immediate wfi:
   for _ in 0..1000 { cortex_m::asm::nop(); }  // Small delay
   cortex_m::asm::wfi();
   ```
3. **Use BOOT0 jumper**: Can force chip into bootloader mode if really stuck
4. **Keep reset button accessible**: Holding reset while connecting can help

**When It Happens Most:**

- After uploading buggy code that crashes hard
- When switching between projects with different sleep modes
- After power cycling while debugger was connected
- When code disables SWD pins accidentally (rare but possible)

**Alternative Recovery Methods:**

1. **STM32CubeProgrammer** (if probe-rs doesn't work):

   ```bash
   # Download from ST website, then:
   STM32_Programmer_CLI -c port=SWD mode=UR -e all
   ```

2. **OpenOCD** (alternative toolchain):

   ```bash
   openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; reset halt; flash erase_sector 0 0 last; exit"
   ```

3. **Physical BOOT0 Jumper**:
   - Set BOOT0 to HIGH (3.3V)
   - Press RESET
   - Connect with debugger
   - Flash new code
   - Set BOOT0 back to LOW

**Pro Tip:**
If this happens frequently during development, create a shell alias:

```bash
# Add to ~/.bashrc or ~/.zshrc
alias fix-stlink='probe-rs erase --chip STM32F446RETx --protocol swd --connect-under-reset'

# Then just run:
fix-stlink
```

---

## Version History

- **v0.1.0** - Initial working version with LED blink and OLED display
- Uses HAL 0.21.0 for stability
- RTIC 2.1 framework
- Rust 2021 edition (with 2024 compatibility warnings)
