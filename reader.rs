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

mod nrf_conf;

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


const EN_RXADDR: u8    = 0x02;
const RX_ADDR_P0: u8   = 0x0A;
const RX_PW_P0: u8     = 0x11;
const R_RX_PAYLOAD: u8 = 0x61;

const FLUSH_RX: u8 = 0xE2;
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
  
    let addr: [u8; 5] = [0xE7, 0xE7, 0xE7, 0xE7, 0xE7];
    Timer::after(Duration::from_millis(5)).await;
    write_register(&mut spi, &mut nss, STATUS, &[0x70]).await;

    // 수신 모드 설정 (PWR_UP=1, PRIM_RX=1)
    write_register(&mut spi, &mut nss, CONFIG, &[0b0000_1011]).await;
    write_register(&mut spi, &mut nss, EN_AA, &[0x01]).await;

    // 파이프 0 활성화
    write_register(&mut spi, &mut nss, EN_RXADDR, &[0x01]).await;

    // 채널 설정
    write_register(&mut spi, &mut nss, RF_CH, &[0x02]).await;

    // 데이터 속도 및 출력 세기 설정
    write_register(&mut spi, &mut nss, RF_SETUP, &[0x0F]).await;

    // 수신 주소 설정
    write_register(&mut spi, &mut nss, RX_ADDR_P0, &addr).await;

    // 페이로드 길이 설정 (5바이트 예시)
    write_register(&mut spi, &mut nss, RX_PW_P0, &[0x05]).await;

    // 5. CE 핀 HIGH → 수신 시작
    ce.set_high();
    Timer::after(Duration::from_millis(150)).await;
    // 8. CE 핀 HIGH → 수신 시작


    read_register(&mut spi, &mut nss, 0x07, 1).await; // STATUS
    read_register(&mut spi, &mut nss, 0x00, 1).await; // CONFIG
    // sender(&mut spi, &mut nss,&mut ce).await;
    // read_register(&mut spi, &mut nss, 0x05, 1).await;

    // let mut tx = nrf24.tx().await.unwrap();
    loop {
        Timer::after(Duration::from_millis(10)).await;
        let mut status = [0u8; 2];
        // if status[0]==0{
        //     // write_register(&mut spi, &mut nss, STATUS, &[0x70]).await;
        //     // write_register(&mut spi, &mut nss, RX_ADDR_P0, &addr).await;
        //     write_register(&mut spi, &mut nss, STATUS, &[RX_DR]).await;
        // }
        // info!("{}",status);
        read_register(&mut spi, &mut nss, STATUS, 1).await;
        nss.set_low();
        spi.transfer(&mut status, &mut [R_REGISTER | STATUS, 0xFF]).await.unwrap();
        nss.set_high();
        if status[1] & RX_DR != 0 {
            // 데이터 수신
            let mut tx_buf = [R_RX_PAYLOAD];
            let mut rx_buf = [0u8; 6]; // 1 바이트 명령어 + 5바이트 데이터

            nss.set_low();
            spi.transfer(&mut rx_buf, &mut [R_RX_PAYLOAD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]).await.unwrap();
            nss.set_high();

            
            if let Ok(str)=str::from_utf8(&rx_buf){
                info!("수신한 데이터: {:?}", str);
            }
            // let asd = str::from_utf8(&rx_buf).unwrap();
            

            // STATUS의 RX_DR 비트 클리어
            write_register(&mut spi, &mut nss, STATUS, &[RX_DR]).await;
            // flush_rx(&mut spi, &mut nss).await;

        }

        // sender(&mut spi, &mut nss,&mut ce).await;
        // Timer::after(Duration::from_millis(1000)).await;
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

    // info!("READ {:02X} => {:?}", reg, &rx_buffer[1..=len]);
    // info!("STATUS: {:08b}, CONFIG: {:08b}", rx_buffer[0], rx_buffer[1]);
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

async fn flush_rx(
    spi: &mut spi::Spi<'_, embassy_stm32::mode::Async>,
    nss: &mut Output<'_>,
) {
    let mut buf = [FLUSH_RX];
    let mut rx_buf = [0x00];
    nss.set_low();
    spi.transfer(&mut rx_buf, &mut buf).await.unwrap();
    nss.set_high();
}