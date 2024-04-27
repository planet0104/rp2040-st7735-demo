#![no_std]
#![no_main]

use core::f32::consts::PI;

use cortex_m_rt::entry;
use panic_halt as _;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use defmt_rtt as _;
use fugit::RateExtU32;

use embedded_graphics::image::{Image, ImageRaw, ImageRawLE};
use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::Rgb565;
use rp_pico::hal::rom_data::reset_to_usb_boot;
use rp_pico::hal::{prelude::*, spi};
use st7735_lcd;
use st7735_lcd::Orientation;

use rp_pico::hal::pac;
use rp_pico::hal;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    /*
    GND <=> GND
    VCC <=> 3V3
    SCL <=> SCLK(GPIO6)
    SDA <=> MOSI(GPIO7)
    RES <=> RST(GPIO14)
    DC  <=> DC(GPIO13)
    CS  <=> GND
    BLK <=> 不连接
     */

    // These are implicitly used by the spi driver if they are in the correct mode
    let spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();
    let spi_mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi = spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    let dc = pins.gpio13.into_push_pull_output();
    let rst = pins.gpio14.into_push_pull_output();

    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        64_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 161, 81);

    disp.init(&mut delay).unwrap();
    disp.set_orientation(&Orientation::Landscape).unwrap();
    disp.set_offset(0, 25);

    // 清空屏幕为红色
    disp.clear(Rgb565::new(255, 0, 0)).unwrap();
    delay.delay_ms(1000);

    // 清空屏幕为蓝色
    disp.clear(Rgb565::new(0, 0, 255)).unwrap();
    delay.delay_ms(1000);
    
    // 画一条斜线
    let mut x = 0;
    let mut y = 0;
    
    for _ in 0..50{
        disp.set_pixel(x, y, 255).unwrap();
        x += 1;
        y += 1;
    }
    
    //画一个圆
    let radius = 30.;
    let center_x = 80; // 屏幕中心X坐标（对于128x64屏幕）
    let center_y = 40; // 屏幕中心Y坐标（对于128x64屏幕）
    // 按照角度增量遍历整个圆
    for angle in 0..=360 {
        let radian = angle as f32 * PI / 180.0;
        let pixel_x = (center_x as f32 + radius * libm::cos(radian.into()) as f32) as u8;
        let pixel_y = (center_y as f32 + radius * libm::sin(radian.into()) as f32) as u8;

        // 检查像素点是否在屏幕范围内，避免超出边界
        if pixel_x < 161 && pixel_y < 80 {
            disp.set_pixel(pixel_x as u16, pixel_y as u16, 255).unwrap();
        }
    }

    // 绘制ferris
    let image_raw: ImageRawLE<Rgb565> =
        ImageRaw::new(include_bytes!("../assets/ferris.raw"), 86);

    let image: Image<_> = Image::new(&image_raw, Point::new(0, 0));

    image.draw(&mut disp).unwrap();

    // 10秒后重启到USB模式（方便下次cargo run）    
    delay.delay_ms(15000);

    reset_to_usb_boot(0, 0);

    loop { continue; }
}

// End of file