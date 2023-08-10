//! The starter code slowly blinks the LED, sets up
//! USB logging, and creates a UART driver using pins
//! 14 and 15. The UART baud rate is [`UART_BAUD`].
//!
//! Despite targeting the Teensy 4.0, this starter code
//! also works on the Teensy 4.1.

#![no_std]
#![no_main]

use bsp::board;
use teensy4_bsp as bsp;
use teensy4_panic as _;

use bsp::hal::timer::Blocking;

// spi imports ( without_pins() )
use bsp::hal::lpspi::Lpspi;
use imxrt_iomuxc as iomuxc;

// import the rfm95w driver
use sx127x_lora;

// trait that implements delay_ms for a timer
use embedded_hal::blocking::delay::DelayMs;

// trait implementin Transfer and Write for spi
//use embedded_hal::blocking::spi::{Transfer, Write};

/// Milliseconds to delay before toggling the LED
/// and writing text outputs.
const SHORT_DELAY: u32 = 100;
const LONG_DELAY: u32 = 2000;
const FREQUENCY: i64 = 915;

#[bsp::rt::entry]
fn main() -> ! {
    // These are peripheral instances. Let the board configure these for us.
    // This function can only be called once!
    let instances = board::instances();

    // Driver resources that are configured by the board. For more information,
    // see the `board` documentation.
    let board::Resources {
        // `pins` has objects that represent the physical pins. The object
        // for pin 13 is `p13`.
        pins,
        // This is a hardware timer. We'll use it for blocking delays.
        mut gpt1,
        // Another hardware timer :)
        mut gpt2,
        // These are low-level USB resources. We'll pass these to a function
        // that sets up USB logging.
        usb,
        // This is the GPIO2 port. We need this to configure the LED as a
        // GPIO output.
        mut gpio2,
        // This is the SPI port. We'll use it to configure the spi ourselves
        lpspi4,
        
        ..
    } = board::t40(instances);

    // This configures the LED as a GPIO output.
    // This is an alternate initialization function for gpio pins
    // In this case it simplifies led initialization
    let led = gpio2.output(pins.p8);

    // Configures the GPT1 timer to run at GPT1_FREQUENCY. See the
    // constants below for more information.
    gpt1.disable();
    gpt1.set_divider(GPT1_DIVIDER);
    gpt1.set_clock_source(GPT1_CLOCK_SOURCE);

    gpt2.disable();
    gpt2.set_divider(GPT1_DIVIDER);
    gpt2.set_clock_source(GPT1_CLOCK_SOURCE);

    // Convenience for blocking delays.
    let mut delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt1);
    let lora_delay = Blocking::<_, GPT1_FREQUENCY>::from_gpt(gpt2);

    // When this returns, you can use the `log` crate to write text
    // over USB. Use either `screen` (macOS, Linux) or PuTTY (Windows)
    // to visualize the messages from this example.
    bsp::LoggingFrontend::default_log().register_usb(usb);
    delay.delay_ms(LONG_DELAY); // allows for usb to initialize properly

    // setup the SPI
    let mut spi = Lpspi::without_pins(lpspi4);

    // sets the spi baud rate and clock frequency
    spi.disabled(|spi| {
        spi.set_clock_hz(board::LPSPI_FREQUENCY, 1_000_000);
    });

    // intialize serial data pins for spi
    let mut sdo = pins.p11;
    iomuxc::lpspi::prepare(&mut sdo);
    let mut sdi = pins.p12; 
    iomuxc::lpspi::prepare(&mut sdi);
    let mut sck = pins.p13; 
    iomuxc::lpspi::prepare(&mut sck);
    
    // Reset and CS pin for the RFM95W (requried by the library)
    let reset = gpio2.output(pins.p9);
    let cs = gpio2.output(pins.p10);

    // initialize lora module
    let mut lora = match sx127x_lora::LoRa::new(spi, cs, reset, FREQUENCY, lora_delay) {
        Ok(sx127x) => sx127x,
        Err(error) => match error {
            sx127x_lora::Error::VersionMismatch(version) => panic!("Version Mismatch Error. Version{:?}", version),
            sx127x_lora::Error::CS(_) => panic!("Chip select issue"),
            sx127x_lora::Error::Reset(_) => panic!("Reset issue"),
            sx127x_lora::Error::SPI(_) => panic!("SPI problem"),
            sx127x_lora::Error::Transmitting => panic!("Error during spi transmission"),
            sx127x_lora::Error::Uninformative => panic!("Uninformative error RIP"),
        }
    };

    // wait for a little bit after initializing radio
    delay.delay_ms(SHORT_DELAY);
    log::info!("LoRa succesfully initalized with {FREQUENCY} MHz");   

    // set transmit power for LoRa using PA BOOST pin
    match lora.set_tx_power(17,1) {
        Ok(_) => log::info!("Power has been adjusted!"),
        Err(_) => panic!("Error setting tx power!"),
    };

    // encode hello world message into array: [u8: 255]
    let message = "Hello, world!";
    let mut buffer = [0;255];
    for (i,c) in message.chars().enumerate() {
        buffer[i] = c as u8;
    }

    // infinite loop
    loop {
        // turn LED ON while message is being transmitted
        led.set();
        log::info!("Sending...");

        // transmits the message and wait until message is fully transmitted
        let transmit = lora.transmit_payload_busy(buffer, message.len());
        match transmit {
            Ok(packet_size) => log::info!("Sent packet with size: {}", packet_size),
            Err(_) => panic!("Error sendig packet!"),
        }
        // add an extra delay
        delay.block_ms(SHORT_DELAY);

        // turn LED OFF after succesfully sending the packet
        led.clear();
        // wait 10 seconds before sending another packet
        delay.block_ms(5 * LONG_DELAY);
    }
}

// We're responsible for configuring our timers.
// This example uses PERCLK_CLK as the GPT1 clock source,
// and it configures a 1 KHz GPT1 frequency by computing a
// GPT1 divider.
use bsp::hal::gpt::ClockSource;

/// The intended GPT1 frequency (Hz).
const GPT1_FREQUENCY: u32 = 1_000;
/// Given this clock source...
const GPT1_CLOCK_SOURCE: ClockSource = ClockSource::HighFrequencyReferenceClock;
/// ... the root clock is PERCLK_CLK. To configure a GPT1 frequency,
/// we need a divider of...
const GPT1_DIVIDER: u32 = board::PERCLK_FREQUENCY / GPT1_FREQUENCY;
