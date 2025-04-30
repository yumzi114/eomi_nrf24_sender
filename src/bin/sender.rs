#![no_std]
#![no_main]
use cortex_m_rt::entry;
use defmt::*;
use embassy_stm32::{bind_interrupts, gpio::{ Level, Output,  Speed}, i2c::{self, I2c}, peripherals, spi, time::{mhz, Hertz}};
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::{Executor, Spawner};
// use embedded_hal::spi::SpiBus;
use embassy_time::{ Duration, Timer};
use embassy_time::Delay;


const R_REGISTER: u8     = 0x00;
const W_REGISTER: u8     = 0x20;
const W_TX_PAYLOAD: u8   = 0xA0;
const FLUSH_TX: u8       = 0xE1;
const STATUS: u8         = 0x07;
const CONFIG: u8         = 0x00;
const EN_AA: u8          = 0x01;
const SETUP_RETR: u8     = 0x04;
const RF_CH: u8          = 0x05;
const RF_SETUP: u8       = 0x06;
const TX_ADDR: u8        = 0x10;
const FIFO_STATUS: u8    = 0x17;

// STATUS bits
const TX_DS: u8 = 1 << 5;
const MAX_RT: u8 = 1 << 4;
const RX_DR: u8 = 1 << 6;

// I2C ADDR
const ADDRESS: u8 = 0x48;
bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner)  {
    // info!("Hello World!");
    let config={
        use embassy_stm32::rcc::*;
        let mut config = embassy_stm32::Config::default();
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL192,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV4),
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.plli2s = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL384,
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV5),
        });
        config.enable_debug_during_sleep = true;

        config
    };
    let p = embassy_stm32::init(config); 

    let mut rf_spi_config = spi::Config::default();
    rf_spi_config.mode = embassy_stm32::spi::MODE_0;
    rf_spi_config.frequency = mhz(10);
    let delay = Delay;
    let mut ce: Output = Output::new(p.PA3, Level::High, Speed::Medium);
    ce.set_low();
    let mut nss = Output::new(p.PA4, Level::High, Speed::Medium);
    let mut spi: spi::Spi<embassy_stm32::mode::Async>= spi::Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH2,p.DMA2_CH0,rf_spi_config);
    let mut i2c: I2c<embassy_stm32::mode::Async> = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB7,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH0,
        Hertz(100_000),
        Default::default(),
    );
    let channels = [0b100, 0b101, 0b110, 0b111];
    let mut adc_data=[0_i16;4];
    // let mut adc_data=[0_f32;4]  ;
    // let mut i2c = I2c::new_blocking(p.I2C1, p.PB8, p.PB7, Hertz(100_000), Default::default());
    //클리어
    let addr: [u8; 5] = [0xE7, 0xE7, 0xE7, 0xE7, 0xE7];
    // let addr  = "eomi".as_bytes();
    Timer::after(Duration::from_millis(5)).await;
    write_register(&mut spi, &mut nss, 0x07, &[0b01110000]).await;
    write_register(&mut spi, &mut nss,CONFIG, &[0b0000_1010]).await; // PWR_UP + PRIM_TX
    write_register(&mut spi, &mut nss,EN_AA, &[0x01]).await;         // Pipe 0 Auto-ACK
    write_register(&mut spi, &mut nss,SETUP_RETR, &[0x3F]).await;    // Auto Retransmit 15 times, delay 1500us
    write_register(&mut spi, &mut nss,RF_CH, &[0x02]).await;         // Channel
    // 8. CE 핀 HIGH → 수신 시작

    // 최장거리 통신	0x1E (250Kbps, 0dBm)
    //빠른 속도 통신	0x2E (2Mbps, 0dBm)
    //기본 안정성	0x0E (1Mbps, 0dBm)
    write_register(&mut spi, &mut nss,RF_SETUP, &[0b0000_1110]).await; 
    
    write_register(&mut spi, &mut nss,TX_ADDR, &addr).await;         // TX 주소
    Timer::after(Duration::from_nanos(10)).await;

    loop {
        for (index,mux) in channels.iter().enumerate() {
            let config = 0x8000
                       | ((*mux as u16) << 12)
                       | (0b001 << 9)
                       | (0b110 << 5)
                       | 0x0003;
            i2c.write(ADDRESS, &[0x01, (config >> 8) as u8, config as u8]).await.unwrap();
            // 변환 대기
            Timer::after_millis(4).await;
            let mut buffer = [0u8; 2];
            match i2c.blocking_write_read(ADDRESS,  &[0x00],&mut buffer) {
                Ok(()) => {
                    let raw = ((buffer[0] as i16) << 8) | (buffer[1] as i16);
                    let voltage = (raw as f32) * 0.1875 / 1000.0;
                    adc_data[index]=raw;
                    // info!("data: {}", raw);
                },
                Err(_) => error!("Operation timed out"),
                Err(e) => error!("I2c Error: {:?}", e),
            }
        }
        sender(&mut spi, &mut nss,&mut ce,&adc_data).await;
        Timer::after(Duration::from_millis(50)).await;
    }
}


async fn write_register(
    spi: &mut spi::Spi<'_, embassy_stm32::mode::Async>, 
    nss: &mut Output<'_>, 
    reg: u8, 
    value: &[u8])

{
    let len = 1 + value.len();
    let mut tx_buffer = [0_u8; 32];
    let mut rx_buffer = [0_u8; 32];
    tx_buffer[0] = W_REGISTER | (reg & 0x1F);
    tx_buffer[1..(1 + value.len())].copy_from_slice(value);
    nss.set_low();
    spi.transfer(&mut rx_buffer[..len], &mut tx_buffer[..len]).await.unwrap();
    nss.set_high();
    // Timer::after(Duration::from_millis(10)).await;
}

async fn sender(
    spi: &mut spi::Spi<'_, embassy_stm32::mode::Async>,
    nss: &mut Output<'_>, 
    ce: &mut Output<'_>, 
    adc_data:&[i16; 4],
){
    let mut adc_payload = [0_u8;8];
    adc_data.iter().enumerate().for_each(|(ind,num)|{
        let temp: [u8; 2] = num.to_be_bytes();
        adc_payload[ind * 2] = temp[0]; 
        adc_payload[ind * 2 + 1] = temp[1]; 
    });
    info!("{}",adc_payload);
    // let payload: [u8; 5] = *b"HELLO";
    let mut dummy_read = [0u8; 9]; 
    let mut spi_write = [0u8; 9];
    spi_write[0] = W_TX_PAYLOAD; 
    spi_write[1..].copy_from_slice(&adc_payload);
    nss.set_low();

    spi.transfer(&mut dummy_read, &spi_write).await.unwrap();
    nss.set_high();

    ce.set_high();
    Timer::after(Duration::from_micros(1)).await;
    ce.set_low();
    

    // 상태 읽기
    let mut status_buf = [0u8; 2];
    let mut tx_buf = [R_REGISTER | STATUS, 0xFF];
    nss.set_low();
    spi.transfer(&mut status_buf, &tx_buf).await.unwrap();
    nss.set_high();
    let status = status_buf[1];
    // info!("STATUS: {:08b}", status);

    if status & TX_DS != 0 {
        info!("Transmission successful!");
        // TX_DS 클리어: 쓰기 1 클리어
        write_register(spi, nss, STATUS, &[TX_DS]).await;
    } else if status & MAX_RT != 0 {
        info!("Transmission failed: MAX_RT reached!");
        // MAX_RT 클리어
        write_register(spi, nss, STATUS, &[MAX_RT]).await;
        // 플러시 TX FIFO
        // flush_tx(spi, nss).await;
        write_register(spi, nss, FLUSH_TX, &[]).await;

    } else {
        info!("Transmission status unknown");
    }

}

