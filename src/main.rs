//! # RTIC OLED Bringup
//!
//! This example demonstrates Real-Time Interrupt-driven Concurrency (RTIC) on STM32F446RE.
//! 
//! ## What This Code Does
//! - Blinks an LED at 2 Hz using a timer interrupt (TIM2)
//! - Updates an OLED display with a counter on each interrupt
//! - Uses interrupt-driven execution (no busy loops)
//! - Demonstrates safe resource sharing in embedded Rust
//!
//! ## Hardware
//! - Board: STM32 Nucleo-F446RE
//! - LED: LD2 (green) on PA5 (active-low)
//! - Display: SSD1306 OLED 128x32 via I2C (SCL=PB8, SDA=PB9)
//!
//! ## RTIC Key Concepts
//! 1. **Tasks**: Functions that run in response to interrupts
//! 2. **Resources**: Hardware or data shared between tasks
//! 3. **Context**: Provides access to resources within a task
//! 4. **Init**: One-time hardware initialization
//! 5. **Idle**: Low-power loop that runs when no interrupts are pending

// ============================================================================
// COMPILER DIRECTIVES
// ============================================================================

// Don't use the Rust standard library (not available in embedded systems)
#![no_std]

// Don't use the standard main() entry point (we define our own with RTIC)
#![no_main]

// ============================================================================
// PANIC AND DEBUG INFRASTRUCTURE
// ============================================================================

// When a panic occurs (like .unwrap() on None), halt the CPU
// In production, consider using panic-probe for better debugging
use panic_probe as _;

// Enable defmt logging over RTT (Real-Time Transfer)
// RTT is faster and more efficient than semihosting
use defmt_rtt as _;

// ============================================================================
// RTIC FRAMEWORK
// ============================================================================

// RTIC (Real-Time Interrupt-driven Concurrency) is a framework for building
// concurrent embedded applications. It provides:
// - Zero-cost abstractions (no runtime overhead)
// - Compile-time checking of resource access (prevents data races)
// - Priority-based task scheduling
// - Static memory allocation (no heap)
use rtic::app;

// ============================================================================
// STM32F4 HARDWARE ABSTRACTION LAYER (HAL)
// ============================================================================

// The HAL provides safe, high-level abstractions over hardware peripherals
use stm32f4xx_hal::{
    prelude::*,                    // Extension traits for HAL types
    gpio::{Output, Pin},            // Type-safe GPIO pin types
    pac,                            // Peripheral Access Crate (raw registers)
    timer::{CounterHz, Event, Timer, Flag},  // Timer types and configuration
    i2c::I2c,                       // I2C peripheral driver
};

// ============================================================================
// DISPLAY DRIVER AND GRAPHICS
// ============================================================================

// SSD1306 OLED driver - supports I2C and SPI interfaces
use ssd1306::{prelude::*, Ssd1306};

// Display mode that uses a framebuffer in RAM (allows double-buffering)
use ssd1306::mode::BufferedGraphicsMode;

// Embedded graphics library for drawing text, shapes, images
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},  // Font and text styling
    pixelcolor::BinaryColor,                        // On/off pixels for monochrome displays
    prelude::*,                                     // Common graphics traits
    text::Text,                                     // Text rendering
};

// ============================================================================
// UTILITY CRATES
// ============================================================================

// Thread-safe bus sharing for I2C (allows multiple devices on same bus)
use shared_bus::BusManagerSimple;

// Static string type (no heap allocation)
use heapless::String;

// Required for write!() macro with heapless::String
use core::fmt::Write;

// For proper static initialization without triggering 2024 edition warnings
use core::mem::MaybeUninit;

// ============================================================================
// TYPE ALIASES FOR READABILITY
// ============================================================================

// Type alias for our I2C bus manager
// BusManagerSimple uses NullMutex (simple critical sections)
type I2cBus = BusManagerSimple<I2c<pac::I2C1>>;

// Type alias for I2C proxy (handle to the shared bus)
// 'static lifetime means it lives for the entire program
type I2cProxy = shared_bus::I2cProxy<'static, shared_bus::NullMutex<I2c<pac::I2C1>>>;

// ============================================================================
// RTIC APPLICATION
// ============================================================================

// #[app] macro generates:
// - Interrupt vector table
// - Resource management code
// - Task dispatchers
// - Context types
#[app(
    device = stm32f4xx_hal::pac,    // Use STM32F4 peripheral access crate
    peripherals = true,              // Give us ownership of all peripherals
    dispatchers = [EXTI0]            // Software task dispatcher (not used yet)
)]
mod app {
    // Import all types from parent scope
    use super::*;

    // ========================================================================
    // SHARED RESOURCES
    // ========================================================================
    
    // Shared resources can be accessed by multiple tasks
    // RTIC ensures safe access through:
    // - Automatic locking mechanisms
    // - Compile-time deadlock detection
    // - Priority-based resource ceiling protocol
    //
    // Currently empty - all our resources are task-local
    #[shared]
    struct Shared {}

    // ========================================================================
    // LOCAL RESOURCES
    // ========================================================================
    
    // Local resources are owned by a single task
    // No locking overhead - direct access
    // Perfect for peripherals used by only one task
    #[local]
    struct Local {
        // LED pin - 'A' = Port A, 5 = Pin 5, Output mode
        // Type system prevents using wrong pin at compile time!
        led: Pin<'A', 5, Output>,
        
        // Hardware timer configured as a frequency counter
        // CounterHz allows setting frequency in Hz (e.g., 2.Hz())
        timer: CounterHz<pac::TIM2>,
        
        // OLED display driver with:
        // - I2CInterface<I2cProxy>: Communication interface
        // - DisplaySize128x32: Physical display dimensions
        // - BufferedGraphicsMode: Uses RAM framebuffer for double-buffering
        display: Ssd1306<I2CInterface<I2cProxy>, DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>,
        
        // Text style for rendering (font, color)
        // 'static lifetime means the font data is in Flash, not RAM
        text_style: MonoTextStyle<'static, BinaryColor>,
        
        // Simple counter to display on screen
        counter: u32,
    }

    // ========================================================================
    // INITIALIZATION TASK
    // ========================================================================
    
    // #[init] runs exactly once at system startup
    // It has the highest priority and cannot be preempted
    // Must return (Shared, Local) tuple with all resources
    //
    // Execution flow:
    // 1. Power on / Reset
    // 2. cortex-m-rt startup code (copy .data, zero .bss, etc.)
    // 3. init() executes
    // 4. Interrupts are enabled
    // 5. idle() starts running
    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        // Log initialization start (visible via RTT)
        defmt::println!("Starting init...");

        // ====================================================================
        // STEP 1: TAKE OWNERSHIP OF PERIPHERALS
        // ====================================================================
        
        // Context provides access to device peripherals
        // This ensures exclusive ownership - prevents multiple drivers
        // from accessing the same hardware
        let dp = cx.device;

        // ====================================================================
        // STEP 2: CONFIGURE SYSTEM CLOCKS
        // ====================================================================
        
        // Convert raw RCC peripheral to HAL type with safe methods
        // .constrain() consumes the raw type, preventing direct register access
        let rcc = dp.RCC.constrain();
        
        // Configure clock tree:
        // 1. use_hse(8.MHz()) - Use external 8 MHz crystal (more accurate than internal RC)
        // 2. sysclk(48.MHz()) - Set system clock to 48 MHz
        // 3. freeze() - Lock configuration and calculate all clock frequencies
        //
        // Why HSE (external crystal)?
        // - UART: Needs precise timing for baud rates
        // - USB: Requires exact 48 MHz (impossible with internal RC)
        // - Better stability: ±50 ppm vs ±1% for internal oscillator
        //
        // Clock tree (simplified):
        //   HSE 8 MHz → PLL → SYSCLK 48 MHz → AHB 48 MHz → APB1/APB2 → Peripherals
        let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(48.MHz()).freeze();

        // Log actual clock frequency for verification
        defmt::println!("System initialized at {} MHz", clocks.sysclk().to_Hz() / 1_000_000);

        // ====================================================================
        // STEP 3: CONFIGURE GPIO PINS
        // ====================================================================
        
        // Split GPIOA into individual pin objects
        // After .split(), each pin (PA0-PA15) has its own type
        // This provides compile-time guarantees that we're using the right pin
        let gpioa = dp.GPIOA.split();
        
        // Configure PA5 as push-pull output for LED
        // Push-pull: Can drive both high (3.3V) and low (0V) actively
        // Alternative: OpenDrain (only pulls low, needs external pull-up)
        let mut led = gpioa.pa5.into_push_pull_output();
        
        // Test LED during initialization
        // Nucleo board has ACTIVE-LOW LED: LOW=on, HIGH=off
        led.set_low();
        defmt::println!("LED should be ON now (active-low)");

        // ====================================================================
        // STEP 4: CONFIGURE I2C FOR OLED
        // ====================================================================
        
        // Split GPIOB to access I2C pins
        let gpiob = dp.GPIOB.split();
        
        // Configure I2C pins as alternate function with open-drain
        // I2C requires open-drain outputs with external pull-ups
        // - PB8 = SCL (clock)
        // - PB9 = SDA (data)
        let scl = gpiob.pb8.into_alternate().set_open_drain();
        let sda = gpiob.pb9.into_alternate().set_open_drain();
        
        // Initialize I2C peripheral
        // - I2C1: Use I2C peripheral 1
        // - 400.kHz(): Fast mode I2C (standard is 100 kHz, fast is 400 kHz)
        // - &clocks: Needed to calculate baud rate dividers
        let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

        // ====================================================================
        // STEP 5: CREATE SHARED I2C BUS
        // ====================================================================
        
        // Problem: Only one owner can have an I2C peripheral at a time
        // Solution: Shared bus manager creates proxies for multiple devices
        //
        // Why static? The bus manager must live for entire program lifetime
        // Why MaybeUninit? Safer initialization pattern for Rust 2024 edition
        //
        // Safety: This is safe because:
        // 1. init() runs exactly once before any interrupts
        // 2. We only access I2C from one task (tim2_handler)
        // 3. NullMutex provides critical sections when needed
        static mut I2C_BUS: MaybeUninit<I2cBus> = MaybeUninit::uninit();
        let bus = unsafe {
            // Get raw pointer to uninitialized static
            let bus_ptr = I2C_BUS.as_mut_ptr();
            
            // Initialize the bus manager in-place
            bus_ptr.write(BusManagerSimple::new(i2c));
            
            // Return a reference with 'static lifetime
            // This is safe because I2C_BUS lives forever
            &*bus_ptr
        };

        // ====================================================================
        // STEP 6: INITIALIZE OLED DISPLAY
        // ====================================================================
        
        // Create I2C interface for display
        // - bus.acquire_i2c(): Get a proxy to the shared I2C bus
        // - 0x3C: I2C address of SSD1306 (typical value, some use 0x3D)
        // - 0x40: Command/data byte indicator for I2C protocol
        let interface = I2CInterface::new(bus.acquire_i2c(), 0x3C, 0x40);
        
        // Create display driver and convert to buffered mode
        // - DisplaySize128x32: Physical dimensions (128 pixels wide, 32 tall)
        // - DisplayRotation::Rotate0: No rotation (use Rotate180 to flip)
        // - into_buffered_graphics_mode(): Allocate framebuffer in RAM
        //
        // Buffered mode allows:
        // - Drawing without visible tearing
        // - Only updating display when .flush() is called
        // - Faster graphics operations (RAM is faster than I2C)
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        
        // Send initialization sequence to display
        // This configures charge pump, display on/off, etc.
        display.init().ok();
        
        // Create text style for rendering
        // - FONT_6X10: 6 pixels wide, 10 pixels tall ASCII font
        // - BinaryColor::On: White pixels (for black background)
        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);

        // ====================================================================
        // STEP 7: CONFIGURE HARDWARE TIMER
        // ====================================================================
        
        // Create timer from TIM2 peripheral
        // counter_hz() configures it as a frequency counter
        let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
        
        // Set timer frequency to 2 Hz (fires every 500ms)
        // Calculation: 2 Hz = 2 interrupts per second = 0.5 second period
        //
        // To change blink rate:
        // - 1_u32.Hz()    → 1 second period
        // - 10_u32.Hz()   → 100ms period
        // - 1000_u32.Hz() → 1ms period (1 kHz)
        timer.start(2_u32.Hz()).unwrap();
        
        // Enable timer interrupt generation
        // Event::Update fires when timer counter reaches zero
        timer.listen(Event::Update);

        defmt::println!("Init complete, entering main loop");

        // ====================================================================
        // RETURN RESOURCES TO RTIC
        // ====================================================================
        
        // RTIC takes ownership of all resources
        // They will be provided to tasks through context parameters
        (
            Shared {},  // Empty - no shared resources
            Local {
                led,
                timer,
                display,
                text_style,
                counter: 0,
            }
        )
    }

    // ========================================================================
    // TIMER INTERRUPT HANDLER
    // ========================================================================
    
    // #[task] defines an interrupt handler
    // - binds = TIM2: This task handles TIM2 interrupts
    // - local = [...]: List of resources this task can access
    //
    // When TIM2 interrupt fires:
    // 1. CPU stops executing idle()
    // 2. Saves context (registers) to stack
    // 3. Calls tim2_handler()
    // 4. Restores context and returns to idle()
    //
    // Interrupt priority: Default is lowest (unless specified)
    #[task(binds = TIM2, local = [led, timer, display, text_style, counter])]
    fn tim2_handler(cx: tim2_handler::Context) {
        // ====================================================================
        // CLEAR INTERRUPT FLAG
        // ====================================================================
        
        // CRITICAL: Must clear interrupt flag or we'll get stuck in infinite loop
        // Timer hardware sets Flag::Update when counter expires
        // Software must clear it to acknowledge the interrupt
        //
        // Note: Use Flag::Update, not Event::Update
        // - Event: For configuring what triggers interrupts
        // - Flag: For clearing interrupt status
        cx.local.timer.clear_flags(Flag::Update);

        // ====================================================================
        // TOGGLE LED
        // ====================================================================
        
        // Toggle LED state (on→off or off→on)
        // Remember: Nucleo LED is active-low
        // - LOW = LED on (current flows)
        // - HIGH = LED off (no current)
        cx.local.led.toggle();

        // ====================================================================
        // INCREMENT COUNTER
        // ====================================================================
        
        // Update counter for display
        // * dereferences the mutable reference from cx.local
        *cx.local.counter += 1;

        // ====================================================================
        // UPDATE OLED DISPLAY
        // ====================================================================
        
        // Get mutable reference to display
        let display = cx.local.display;
        
        // Clear framebuffer (set all pixels to off/black)
        // .ok() converts Result to Option, ignoring errors
        // In production, you might want to handle errors!
        display.clear(BinaryColor::Off).ok();

        // Format counter into a string
        // heapless::String<32> can hold up to 32 characters on the stack
        let mut line = String::<32>::new();
        
        // write!() macro formats text like println!() but into a String
        // Requires `use core::fmt::Write;` at top of file
        write!(line, "Tick: {}", *cx.local.counter).ok();
        
        // Render text to framebuffer
        // - &line: The text to draw
        // - Point::new(0, 8): Position (x=0, y=8 pixels from top)
        // - *cx.local.text_style: Dereference the style reference
        // - .draw(display): Render to the display framebuffer
        Text::new(&line, Point::new(0, 8), *cx.local.text_style).draw(display).ok();

        // Send framebuffer to display over I2C
        // This is when pixels actually appear on screen
        // Takes ~5-10ms at 400 kHz I2C
        display.flush().ok();

        // Log to RTT for debugging
        defmt::println!("Timer interrupt #{}", *cx.local.counter);
    }

    // ========================================================================
    // IDLE TASK (LOW-POWER LOOP)
    // ========================================================================
    
    // #[idle] runs when no interrupts are pending
    // It has the lowest priority (0) and can be preempted by any interrupt
    //
    // The -> ! return type means this function never returns
    // In embedded systems, programs run forever until power is removed
    #[idle]
    fn idle(_: idle::Context) -> ! {
        // Infinite loop - embedded systems never exit
        loop {
            // WFI = Wait For Interrupt
            // CPU enters low-power sleep until next interrupt
            //
            // Benefits:
            // - Reduces power consumption (important for battery devices)
            // - Reduces heat generation
            // - CPU instantly wakes on interrupt (no polling delay)
            //
            // Alternative: cortex_m::asm::nop() (do nothing, stay awake)
            cortex_m::asm::wfi();
        }
    }
}

// ============================================================================
// PROGRAM EXECUTION FLOW
// ============================================================================
//
// 1. Power On / Reset
//    ↓
// 2. cortex-m-rt startup code
//    - Initialize stack pointer
//    - Copy .data from Flash to RAM
//    - Zero .bss section
//    ↓
// 3. init() executes
//    - Configure clocks
//    - Initialize GPIO
//    - Setup I2C and OLED
//    - Configure timer
//    - Enable interrupts
//    ↓
// 4. idle() starts
//    - CPU sleeps (wfi)
//    ↓
// 5. Timer interrupt fires (every 500ms)
//    - CPU wakes
//    - tim2_handler() executes
//      - Toggle LED
//      - Update display
//      - Return
//    - CPU returns to idle
//    ↓
// 6. Repeat from step 5 forever
//
// ============================================================================

// ============================================================================
// MEMORY LAYOUT
// ============================================================================
//
// Flash (512 KB):
// ├── Vector Table (startup code)
// ├── Program Code (~8-10 KB for this example)
// ├── Constants (fonts, etc.)
// └── Unused (~502 KB available)
//
// RAM (128 KB):
// ├── Stack (~8-16 KB, grows down)
// ├── Static Variables:
// │   ├── I2C_BUS (~100 bytes)
// │   └── OLED framebuffer (512 bytes for 128x32)
// ├── Task Locals (a few hundred bytes)
// └── Unused (~125 KB available)
//
// No heap - all allocations are static!
//
// ============================================================================