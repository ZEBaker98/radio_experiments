
use embedded_hal::digital::v2::OutputPin;
use rfm69::{
  registers::{
    DataMode, Modulation, ModulationShaping, ModulationType, PacketConfig, PacketDc, PacketFiltering, PacketFormat, InterPacketRxDelay
  }, ReadWrite, Rfm69
};

const MODULATION: Modulation = Modulation {
  data_mode: DataMode::Packet,
  modulation_type: ModulationType::Fsk,
  shaping: ModulationShaping::Shaping00,
};
const BIT_RATE: u32 = 4800;
const FREQ_DEVIATION: u32 = 5000; // 5kHz
const FREQUENCY: u32 = 915_000_000; // 915 Mhz
const PREAMBLE_LEN: u16 = 4;
const SYNC_WORD: u16 = 0x0c81;
const PACKET_CONFIG: PacketConfig = PacketConfig {
  format: PacketFormat::Fixed(4),
  dc: PacketDc::Whitening,
  crc: true,
  filtering: PacketFiltering::None,
  interpacket_rx_delay: InterPacketRxDelay::Delay8Bits,
  auto_rx_restart: true,
};
const AES_SECRET: [u8; 16] = [
  0xe6, 0xd9, 0xbb, 0xd0,
  0x74, 0x6e, 0xbc, 0x60,
  0xd3, 0x64, 0xa0, 0xbd,
  0x1e, 0xaf, 0xed, 0x46
];

pub fn config_radio<T, S, Ecs, Espi>(radio: &mut Rfm69<T, S>) -> Result<(), rfm69::Error<Ecs, Espi>>
where
  T: OutputPin<Error = Ecs>,
  S: ReadWrite<Error = Espi>,
{
  radio.modulation(MODULATION)?;
  radio.bit_rate(BIT_RATE)?;
  radio.fdev(FREQ_DEVIATION)?;
  radio.frequency(FREQUENCY)?;
  radio.preamble(PREAMBLE_LEN)?;
  radio.sync(&SYNC_WORD.to_ne_bytes())?;
  radio.packet(PACKET_CONFIG)?;
  radio.aes(&AES_SECRET)?;

  Ok(())
}