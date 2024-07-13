pub fn configure<S, E>(radio: &mut rfm69::Rfm69<S>) -> Result<(), rfm69::Error<E>>
where
    S: rfm69::ReadWrite<Error = E>,
{
    use rfm69::registers::{LnaConfig, LnaGain, LnaImpedance};
    radio.lna(LnaConfig {
        gain_select: LnaGain::AgcLoop,
        zin: LnaImpedance::Ohm200,
    })?;

    use rfm69::registers::{DccCutoff, RxBw, RxBwFsk};
    radio.rx_bw(RxBw {
        dcc_cutoff: DccCutoff::Percent4,
        rx_bw: RxBwFsk::Khz10dot4,
    })?;
    radio.rx_afc_bw(RxBw {
        dcc_cutoff: DccCutoff::Percent1,
        rx_bw: RxBwFsk::Khz50dot0,
    })?;

    radio.write(rfm69::registers::Registers::DioMapping2, 0x07)?; // Disable ClkOut;

    radio.rssi_threshold(0xE4)?;

    radio.sync(&[0x42, 0x24, 0xBD, 0xDB])?;

    use rfm69::registers::FifoMode;
    radio.fifo_mode(FifoMode::NotEmpty)?;

    use rfm69::registers::ContinuousDagc;
    radio.continuous_dagc(ContinuousDagc::ImprovedMarginAfcLowBetaOn0)?;

    use rfm69::registers::{DataMode, Modulation, ModulationShaping, ModulationType};
    radio.modulation(Modulation {
        data_mode: DataMode::Packet,
        modulation_type: ModulationType::Fsk,
        shaping: ModulationShaping::Shaping00,
    })?;

    use rfm69::registers::{
        InterPacketRxDelay, PacketConfig, PacketDc, PacketFiltering, PacketFormat,
    };
    radio.packet(PacketConfig {
        format: PacketFormat::Fixed(3),
        dc: PacketDc::Whitening,
        crc: true,
        filtering: PacketFiltering::None,
        interpacket_rx_delay: InterPacketRxDelay::Delay1Bit,
        auto_rx_restart: true,
    })?;

    use rfm69::registers::{DioMapping, DioMode, DioPin, DioType};
    radio.dio_mapping(DioMapping {
        pin: DioPin::Dio0,
        dio_mode: DioMode::Rx,
        dio_type: DioType::Dio01,
    })
}
