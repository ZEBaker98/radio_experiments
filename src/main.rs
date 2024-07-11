#![no_std]
#![no_main]

//use defmt as _;
//use defmt_rtt as _;
use panic_probe as _;

mod radio_cfg;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rtic::app(
    device = rp2040_hal::pac,
    dispatchers = [SW0_IRQ, SW1_IRQ],
    peripherals = true,
)]
mod app {
    use crate::radio_cfg;
    use embedded_hal::digital::{InputPin, OutputPin};
    use embedded_hal_bus::spi::ExclusiveDevice;
    use fugit::RateExtU32;
    use hal::gpio::{self, bank0, FunctionSio, FunctionSpi, PullDown, SioOutput};
    use hal::pac;
    use hal::pio::PIOExt;
    use hal::spi::Spi;
    use hal::Clock;
    use rp2040_hal as hal;
    use rtic_monotonics::rp2040::prelude::*;

    const XTAL_FREQ_HZ: u32 = 12_000_000;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<bank0::Gpio13, FunctionSio<SioOutput>, PullDown>,
        radio: rfm69::Rfm69<
            ExclusiveDevice<
                Spi<
                    rp2040_hal::spi::Enabled,
                    rp2040_pac::SPI1,
                    (
                        gpio::Pin<bank0::Gpio15, FunctionSpi, PullDown>,
                        gpio::Pin<bank0::Gpio8, FunctionSpi, PullDown>,
                        gpio::Pin<bank0::Gpio14, FunctionSpi, PullDown>,
                    ),
                >,
                gpio::Pin<bank0::Gpio16, FunctionSio<SioOutput>, PullDown>,
                Mono,
            >,
        >,
        neopixel: ws2812_pio::Ws2812Direct<
            pac::PIO0,
            rp2040_hal::pio::SM0,
            gpio::Pin<bank0::Gpio4, gpio::FunctionPio0, PullDown>,
        >,
    }
    rp2040_timer_monotonic!(Mono);

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .unwrap();

        let mut delay =
            cortex_m::delay::Delay::new(ctx.core.SYST, clocks.system_clock.freq().to_Hz());
        Mono::start(ctx.device.TIMER, &ctx.device.RESETS);

        let sio = hal::Sio::new(ctx.device.SIO);
        let pins = gpio::bank0::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        // Init LED
        let mut led = pins.gpio13.into_push_pull_output();
        led.set_low().unwrap();

        let mut radio_rst = pins.gpio18.into_push_pull_output();
        radio_rst.set_high().unwrap();

        // Init RGB LED
        let (mut pio0, sm0, _, _, _) = ctx.device.PIO0.split(&mut ctx.device.RESETS);
        let neopixel = ws2812_pio::Ws2812Direct::new(
            pins.gpio4.into_function(),
            &mut pio0,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        // Init mode in pin
        let mut mode_in = pins.gpio0.into_pull_up_input();

        // Init SPI bus pins
        let spi_pins = (
            pins.gpio15.into_function::<FunctionSpi>(),
            pins.gpio8.into_function::<FunctionSpi>(),
            pins.gpio14.into_function::<FunctionSpi>(),
        );

        // Init Spi bus
        let radio_spi_bus: Spi<_, _, _, 8> = Spi::new(ctx.device.SPI1, spi_pins).init(
            &mut ctx.device.RESETS,
            clocks.peripheral_clock.freq(),
            1u32.MHz(),
            embedded_hal::spi::MODE_0,
        );

        // Init Radio CS
        let mut cs = pins.gpio16.into_function();
        cs.set_high().unwrap();

        // Init Radio Spi Device
        let radio_spi_device = ExclusiveDevice::new(radio_spi_bus, cs, Mono).unwrap();

        // Init Radio
        let mut radio = rfm69::Rfm69::new(radio_spi_device);
        radio_rst.set_low().unwrap();
        delay.delay_ms(10);
        radio_cfg::configure(&mut radio).unwrap();

        // Get Tx or Rx mode
        let tx_mode = mode_in.is_high().unwrap();

        // Start tasks
        heartbeat::spawn().unwrap();
        rainbow::spawn(tx_mode).unwrap();

        // Enable sleep on exit ISR
        ctx.core.SCB.set_sleeponexit();

        // Return resources and timer
        (
            Shared {},
            Local {
                led,
                radio,
                neopixel,
            },
        )
    }

    #[idle()]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(local = [led], priority = 2)]
    async fn heartbeat(ctx: heartbeat::Context) {
        use embedded_hal::digital::StatefulOutputPin;
        loop {
            ctx.local.led.toggle().unwrap();
            Mono::delay(1000.millis()).await;
        }
    }

    #[task(local = [neopixel, radio], priority = 1)]
    async fn rainbow(ctx: rainbow::Context, tx_mode: bool) {
        use smart_leds::{
            hsv::{hsv2rgb, Hsv},
            SmartLedsWrite,
        };
        if tx_mode {
            let mut hsv = Hsv {
                hue: 0,
                sat: 230,
                val: 40,
            };
            loop {
                let rgb = hsv2rgb(hsv);
                ctx.local.neopixel.write([rgb].iter().copied()).unwrap();
                ctx.local.radio.send(&[rgb.r, rgb.g, rgb.b]).unwrap();
                Mono::delay(5.millis()).await;
                hsv.hue = hsv.hue.wrapping_add(1);
            }
        } else {
            loop {
                let mut buffer = [0; 3];
                ctx.local.radio.recv(&mut buffer).unwrap();
                let rgb = smart_leds::RGB::new(buffer[0], buffer[1], buffer[2]);
                ctx.local.neopixel.write([rgb].iter().copied()).unwrap();
            }
        }
    }
}
