#![no_std]
#![no_main]

#[rtic::app(
    device = rp_pico::hal::pac,
    dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use rp_pico::hal::{
        self,
        clocks,
        gpio::{self, FunctionPio0, FunctionSio, PullDown, SioOutput},
        pio::PIOExt,
        sio::Sio,
        watchdog::Watchdog,
        Clock
    };
    use smart_leds::{SmartLedsWrite, RGB8};

    use rp_pico::XOSC_CRYSTAL_FREQ;

    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use rtic_monotonics::rp2040::*;

    use panic_probe as _;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<gpio::bank0::Gpio13, FunctionSio<SioOutput>, PullDown>,
        neopixel: ws2812_pio::Ws2812Direct<hal::pac::PIO0, hal::pio::SM0, gpio::Pin<gpio::bank0::Gpio4, FunctionPio0, PullDown>>,
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

        // Init LED pin
        let sio = Sio::new(ctx.device.SIO);
        let gpioa = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );
        let mut led = gpioa.gpio13.into_push_pull_output();
        led.set_low().unwrap();

        // Init neopixel PIO
        let (mut pio0, sm0, _, _, _) = ctx.device.PIO0.split(&mut ctx.device.RESETS);
        let neopixel = ws2812_pio::Ws2812Direct::new(
            gpioa.gpio4.into_function(),
            &mut pio0,
            sm0, 
            clocks.peripheral_clock.freq());
        // Spawn heartbeat task
        heartbeat::spawn().ok();
        rainbow::spawn().ok();

        // Return resources and timer
        (Shared {}, Local { led, neopixel })
    }

    #[task(local = [led])]
    async fn heartbeat(ctx: heartbeat::Context) {
        // Loop forever.
        //
        // It is important to remember that tasks that loop
        // forever should have an `await` somewhere in that loop.
        //
        // Without the await, the task will never yield back to
        // the async executor, which means that no other lower or
        // equal  priority task will be able to run.
        loop {
            // Flicker the built-in LED
            _ = ctx.local.led.toggle();

            // Congrats, you can use your i2c and have access to it here,
            // now to do something with it!

            // Delay for 1 second
            Timer::delay(1000.millis()).await;
        }
    }

    #[task(local = [neopixel])]
    async fn rainbow(ctx: rainbow::Context) {
        use smart_leds::hsv::{Hsv, hsv2rgb};

        let mut hue: u8 = 0;
        loop {
            let color = hsv2rgb(Hsv{hue, sat: 230, val: 40});

            ctx.local.neopixel.write([color].iter().copied());
            Timer::delay(10.millis()).await;
            hue = hue.wrapping_add(1);
        }
    }
}