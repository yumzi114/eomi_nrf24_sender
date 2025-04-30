#![no_std]
#![no_main]
use cortex_m::delay::Delay as CortexDelay;
use cortex_m_rt::entry;
use defmt::*;
use embassy_stm32::{gpio::{Input, Level, Output, Pull, Speed}, spi, time::mhz};
use embedded_nrf24l01_async::NRF24L01;
use {defmt_rtt as _, panic_probe as _};
use embassy_executor::{Executor, Spawner};
use embedded_hal_1::{delay::DelayNs, spi::{Operation, SpiBus, SpiDevice}};
// use embedded_hal::spi::SpiBus;
use embassy_stm32::Config;
use embedded_hal_bus::spi::{ExclusiveDevice};
use embassy_time::{ Duration, Timer};
use embassy_stm32::rcc::{PllSource,AHBPrescaler,APBPrescaler,PllPDiv,PllQDiv,Pll,Sysclk};
use embassy_time::Delay;
use heapless::Vec;


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

#[embassy_executor::main]
async fn main(spawner: Spawner)  {
    // info!("Hello World!");
    let mut config = Config::default();
    // let rcc_cfg = Config {};
    config.rcc.hsi = true; // 내부 16MHz 클럭 사용
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.pll_src = PllSource::HSI;
    config.rcc.pll = Some(Pll {
        prediv: 16.into(),           // 16MHz / 16 = 1MHz
        mul: embassy_stm32::rcc::PllMul(192), 
        divp: Some(PllPDiv::DIV2),
        divq: Some(PllQDiv::DIV4),
        // divp: Some(PllPDiv::DIV2),        // SYSCLK = 192 / 2 = 96MHz
        // divq: Some(PllQDiv::DIV4),        // 48MHz (USB, SDIO 등에 사용 가능)
        divr: None,
    });
    config.rcc.sys = Sysclk::PLL1_P;
    config.rcc.ahb_pre = AHBPrescaler::DIV1;  
    config.rcc.apb1_pre = APBPrescaler::DIV2;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    
    let p = embassy_stm32::init(config); 

    let mut rf_spi_config = spi::Config::default();
    rf_spi_config.mode = embassy_stm32::spi::MODE_0;
    rf_spi_config.frequency = mhz(8);
    let delay = Delay;
    let mut ce: Output = Output::new(p.PA3, Level::High, Speed::Medium);
    ce.set_low();
    let mut nss = Output::new(p.PA4, Level::High, Speed::Medium);
    // let mut mosi: Output = Output::new(p.PA7, Level::High, Speed::Low);
    // mosi.set_high();
    // let mut miso = Input::new(p.PA6, Pull::Up);    
    let mut spi: spi::Spi<embassy_stm32::mode::Async>= spi::Spi::new(p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH2,p.DMA2_CH0,rf_spi_config);
    // let mut spi_device = ExclusiveDevice::new(spi, nss,delay).unwrap();
    
    // // let mut txbuf = [0x20_u16, 0x08];
    let mut txbuf = [0x20u8, 0x0B];
    let mut rxbuf = [0_u8; 2];
    // spi_device.transaction(&mut [
    //     Operation::Transfer(&mut txbuf, &mut rxbuf),
    // ]).unwrap();
    // nss.set_low();
    
    // spi.transfer( &mut rxbuf, &mut txbuf).await.unwrap();
    // // Timer::after(Duration::from_millis(1)).await;
    // nss.set_high();
    // info!("STATUS: {:08b}, CONFIG: {:08b}", rxbuf[0], rxbuf[1]);
    // write_register(&mut spi, &mut nss, 0x00, &[0b0000_1100]).await;
    // Timer::after(Duration::from_millis(10)).await;

    // read_register(&mut spi, &mut nss, 0x00, 1).await;
    // write_register(&mut spi, &mut nss, 0x00, &[0b00001111]).await;
    // Timer::after(Duration::from_millis(2)).await;
    // read_register(&mut spi, &mut nss, 0x07, 1).await;
    // write_register(&mut spi, &mut nss, 0x07, &[0x0E]).await;
    // Timer::after(Duration::from_millis(5)).await;

    // // CONFIG 다시 설정 (PWR_UP=1, PRIM_RX=1)
    // write_register(&mut spi, &mut nss, 0x00, &[0x03]).await;
    // Timer::after(Duration::from_millis(5)).await;
    //클리어
    let addr: [u8; 5] = [0xE7, 0xE7, 0xE7, 0xE7, 0xE7];
    Timer::after(Duration::from_millis(5)).await;
    write_register(&mut spi, &mut nss, 0x07, &[0b01110000]).await;
    write_register(&mut spi, &mut nss,CONFIG, &[0b0000_1010]).await; // PWR_UP + PRIM_TX
    write_register(&mut spi, &mut nss,EN_AA, &[0x01]).await;         // Pipe 0 Auto-ACK
    write_register(&mut spi, &mut nss,SETUP_RETR, &[0x3F]).await;    // Auto Retransmit 15 times, delay 1500us
    write_register(&mut spi, &mut nss,RF_CH, &[0x02]).await;         // Channel
    write_register(&mut spi, &mut nss,RF_SETUP, &[0x0F]).await;      // 2Mbps, 0dBm
    write_register(&mut spi, &mut nss,TX_ADDR, &addr).await;         // TX 주소
    // 5. CE 핀 HIGH → 수신 시작
    Timer::after(Duration::from_millis(150)).await;
    // 8. CE 핀 HIGH → 수신 시작


    // read_register(&mut spi, &mut nss, 0x07, 1).await; // STATUS
    // read_register(&mut spi, &mut nss, 0x00, 1).await; // CONFIG
    // sender(&mut spi, &mut nss,&mut ce).await;
    // read_register(&mut spi, &mut nss, 0x05, 1).await;

    // let mut tx = nrf24.tx().await.unwrap();
    loop {
        sender(&mut spi, &mut nss,&mut ce).await;
        Timer::after(Duration::from_millis(10)).await;
    }
}


async fn write_register(
    spi: &mut spi::Spi<'_, embassy_stm32::mode::Async>, 
    nss: &mut Output<'_>, 
    reg: u8, 
    value: &[u8])
// where
    // SPI: Transfer<u8>,
    // CSN: Output,
{
    let len = 1 + value.len();
    // const asd: /* Type */ = value.len()+1;
    // let asd: usize = value.len()+1;
    let mut tx_buffer = [0_u8; 32];
    let mut rx_buffer = [0_u8; 32];
    // let mut buffer = Vec::<_, 12>::new();
    // buffer.push(0b0010_0000 | (reg & 0x1F)).unwrap();

    tx_buffer[0] = W_REGISTER | (reg & 0x1F);
    tx_buffer[1..(1 + value.len())].copy_from_slice(value);
    // buffer[0] = 0b0010_0000 | (reg & 0x1F); 
    // buffer[1..].copy_from_slice(value);
    nss.set_low();
    spi.transfer(&mut rx_buffer[..len], &mut tx_buffer[..len]).await.unwrap();
    nss.set_high();
    Timer::after(Duration::from_millis(10)).await;

}

async fn read_register(
    spi: &mut spi::Spi<'_, embassy_stm32::mode::Async>,
    nss: &mut Output<'_>,
    reg: u8,
    len: usize,
) {
    let mut tx_buffer = [0u8; 32];
    let mut rx_buffer = [0u8; 32];

    // R_REGISTER 명령어 + 레지스터 주소
    tx_buffer[0] = R_REGISTER | (reg & 0x1F);

    for i in 1..=len {
        tx_buffer[i] = 0xFF; // dummy write
    }

    nss.set_low();
    spi.transfer(&mut rx_buffer[..=len], &mut tx_buffer[..=len]).await.unwrap();
    nss.set_high();

    info!("READ {:02X} => {:?}", reg, &rx_buffer[1..=len]);
    info!("STATUS: {:08b}, CONFIG: {:08b}", rx_buffer[0], rx_buffer[1]);
}

async fn sender(
    spi: &mut spi::Spi<'_, embassy_stm32::mode::Async>,
    nss: &mut Output<'_>, 
    ce: &mut Output<'_>, 
){
    let payload: [u8; 5] = *b"HELLO";
    let mut dummy_read = [0u8; 6]; 
    let mut spi_write = [0u8; 6];
    spi_write[0] = W_TX_PAYLOAD; 
    spi_write[1..].copy_from_slice(&payload);
    nss.set_low();

    spi.transfer(&mut dummy_read, &spi_write).await.unwrap();
    nss.set_high();

    ce.set_high();
    Timer::after(Duration::from_micros(15)).await;
    ce.set_low();
    

    // 상태 읽기
    let mut status_buf = [0u8; 2];
    let mut tx_buf = [R_REGISTER | STATUS, 0xFF];
    nss.set_low();
    spi.transfer(&mut status_buf, &tx_buf).await.unwrap();
    nss.set_high();
    let status = status_buf[1];
    info!("STATUS: {:08b}", status);

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