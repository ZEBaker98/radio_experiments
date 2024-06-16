#![no_std]
#![no_main]

use panic_probe as _;

mod radio;

use rfm69::Rfm69;
use rp_pico::hal::{self, pac, gpio::{self, FunctionSio, PullUp, SioInput, SioOutput, PullDown, FunctionPio0, FunctionSpi}};

type Neopixel = ws2812_pio::Ws2812Direct<
    hal::pac::PIO0,
    hal::pio::SM0,
    gpio::Pin<gpio::bank0::Gpio4, FunctionPio0, PullDown>,
>;

// GPIO Types
type PullUpInput<P> = gpio::Pin<P, FunctionSio<SioInput>, PullUp>;
type PushPullOutput<P> = gpio::Pin<P, FunctionSio<SioOutput>, PullDown>;

type Button1 = PullUpInput<gpio::bank0::Gpio0>;
type RxOrTx = PullUpInput<gpio::bank0::Gpio1>;
type OnboardLED = PushPullOutput<gpio::bank0::Gpio13>;
type ExternalLED = PushPullOutput<gpio::bank0::Gpio2>;
type RadioCS = PushPullOutput<gpio::bank0::Gpio16>;

// Spi Types
type SpiMosi = gpio::Pin<gpio::bank0::Gpio15, FunctionSpi, PullDown>;
type SpiMiso = gpio::Pin<gpio::bank0::Gpio8, FunctionSpi, PullDown>;
type SpiClock = gpio::Pin<gpio::bank0::Gpio14, FunctionSpi, PullDown>;

type Spi = hal::spi::Spi<hal::spi::Enabled, pac::SPI1, (SpiMosi, SpiMiso, SpiClock)>;

// Radio Type
type Radio = Rfm69<RadioCS, Spi>;
                

#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [SW0_IRQ, SW1_IRQ],
    peripherals = true,
)]
mod app {

    use fugit::RateExtU32;
    use rp_pico::hal::{
        self, clocks,
        gpio::{self, FunctionPio0, FunctionSio, FunctionSpi, PullDown, SioOutput},
        pio::PIOExt,
        sio::Sio,
        watchdog::Watchdog,
        Clock,
    };

    use crate::radio::config_radio;

    use rp_pico::XOSC_CRYSTAL_FREQ;

    use embedded_hal::{
        digital::v2::{InputPin, OutputPin, ToggleableOutputPin},
        spi::MODE_0,
    };
    use rtic_monotonics::rp2040::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        neopixel: crate::Neopixel,
        button1: crate::Button1,
        rx_or_tx: crate::RxOrTx,
        onboard_led: crate::OnboardLED,
        external_led: crate::ExternalLED,
        radio: crate::Radio,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Initialize the interrupt for the RP2040 timer and obtain the token
        // proving that we have.
        let rp2040_timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        Timer::start(ctx.device.TIMER, &ctx.device.RESETS, rp2040_timer_token); // default rp2040 clock-rate is 125MHz
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Init GPIO
        let sio = Sio::new(ctx.device.SIO);
        let pins = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );
        
        // Init LEDs
        let mut onboard_led = pins.gpio13.into_push_pull_output();
        onboard_led.set_low().unwrap();

        let external_led = pins.gpio2.into_push_pull_output();
        onboard_led.set_low().unwrap();

        // Init Button
        let button1 = pins.gpio0.into_pull_up_input();
        button1.set_schmitt_enabled(true);
        button1.set_interrupt_enabled(gpio::Interrupt::EdgeLow, true);

        let rx_or_tx = pins.gpio1.into_pull_up_input();

        // Init SPI
        let mosi = pins.gpio15.into_function();
        let miso = pins.gpio8.into_function();
        let sck = pins.gpio14.into_function();
        let cs = pins.gpio16.into_function();

        let spi: hal::spi::Spi<_, _, _, 8> = hal::spi::Spi::new(ctx.device.SPI1, (mosi, miso, sck)).init(
            &mut ctx.device.RESETS,
            125_000_000u32.Hz(),
            16_000_000u32.Hz(),
            MODE_0,
        );

        let mut radio = crate::Rfm69::new(spi, cs);

        config_radio(&mut radio).ok();

        // Init RGB LED
        let (mut pio0, sm0, _, _, _) = ctx.device.PIO0.split(&mut ctx.device.RESETS);
        let neopixel = ws2812_pio::Ws2812Direct::new(
            pins.gpio4.into_function(),
            &mut pio0,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        // Spawn tasks
        heartbeat::spawn().ok();
        rainbow::spawn().ok();

        // Setup core for interrupt driven operation
        ctx.core.SCB.set_sleeponexit();

        // Return resources and timer
        (
            Shared {},
            Local {
                neopixel,
                button1,
                rx_or_tx,
                onboard_led,
                external_led,
                radio,
            },
        )
    }

    #[idle()]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(local = [onboard_led], priority = 1)]
    async fn heartbeat(ctx: heartbeat::Context) {
        loop {
            ctx.local.onboard_led.toggle().unwrap();
            Timer::delay(1000.millis()).await;
        }
    }

    #[task(local = [neopixel], priority = 2)]
    async fn rainbow(ctx: rainbow::Context) {
        use smart_leds::{
            hsv::{hsv2rgb, Hsv},
            SmartLedsWrite,
        };

        let mut hue: u8 = 0;
        loop {
            let color = hsv2rgb(Hsv {
                hue,
                sat: 230,
                val: 40,
            });

            let _ = ctx.local.neopixel.write([color].iter().copied());
            Timer::delay(10.millis()).await;
            hue = hue.wrapping_add(1);
        }
    }

    #[task(binds = IO_IRQ_BANK0, local = [button1, external_led])]
    fn button_handler(ctx: button_handler::Context) {
        if ctx.local.button1.interrupt_status(gpio::Interrupt::EdgeLow) {
            ctx.local.external_led.toggle().unwrap();
            ctx.local.button1.clear_interrupt(gpio::Interrupt::EdgeLow);
        }
    }
}
