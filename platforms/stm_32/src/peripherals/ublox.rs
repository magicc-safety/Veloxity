use embassy_stm32::mode::Async;
use embassy_stm32::usart;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embassy_time::Instant;
use embassy_time::with_timeout;

use super::pps;
use veloxity_core::errors;
use veloxity_core::packets;

const BUFFER_LEN: usize = 512;

fn unix_seconds_from_utc(year: u16, month: u8, day: u8, hour: u8, min: u8, sec: u8) -> i64 {
    let mut y = year as i32;
    let m = month as i32;
    y -= (m <= 2) as i32;
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = y - era * 400;
    let mp = m + if m > 2 { -3 } else { 9 };
    let doy = (153 * mp + 2) / 5 + day as i32 - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146097 + doe - 719468;
    days as i64 * 86_400 + hour as i64 * 3_600 + min as i64 * 60 + sec as i64
}

pub static GNSS_SIGNAL: Signal<
    CriticalSectionRawMutex,
    Result<packets::GNSSPacket, errors::SensorError>,
> = Signal::<CriticalSectionRawMutex, Result<packets::GNSSPacket, errors::SensorError>>::new();

#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct PvtPayload {
    pub i_tow: u32, // ms, GPS time of week
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub min: u8,
    pub sec: u8, // UTC

    pub valid: u8,    // validity flags
    pub t_acc: u32,   // ns, time accuracy estimate
    pub nano: i32,    // ns, Fraction of second -1e9 to 1e9 (UTC)
    pub fix_type: u8, // 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNS+dead reckoning combined, 5 time only fix

    pub flags: u8,
    pub flags2: u8,
    pub num_sv: u8, // satellites used in solution
    pub lon: i32,
    pub lat: i32, // degx10^-7
    pub height: i32,
    pub h_msl: i32, // mm
    pub h_acc: u32,
    pub v_acc: u32, // mm
    pub vel_n: i32,
    pub vel_e: i32,
    pub vel_d: i32,
    pub g_speed: i32,  // mm/s velocity
    pub head_mot: i32, // degx10^-5
    pub s_acc: u32,    // mm/s speed accuracy estimate
    pub head_acc: u32, // degx10^-5
    pub p_dop: u16,    // 0.01 (percent)

    flags3: u16,
    pub reserved1: u8,
    pub head_veh: i32, // degx10^-5, vehicle heading
    pub mag_dec: i16,  // degx 10^-2
    pub mag_acc: u32,  // degx 10^-2
}

#[repr(C, packed)]
#[derive(Copy, Clone)]
pub union PvtUnion {
    pub packet: PvtPayload,
    pub payload: [u8; 92],
}

#[repr(u32)]
#[derive(Clone, Copy)]
pub enum Bitrate {
    Baud9600 = 9600u32,
    Baud38400 = 38400u32,
    Baud57600 = 57600u32,
    Baud115200 = 115200u32,
    Baud230400 = 230400u32,
}

pub enum Protocol {
    M8,
    M9,
}
static UBX_MAX_PAYLOAD_BYTES: usize = 256;

#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct UbxFrame {
    cl: u8,
    id: u8,
    length: usize,
    a: u8, // checksum
    b: u8, // checksum
    payload: [u8; UBX_MAX_PAYLOAD_BYTES],
}

pub struct UbloxSensor {
    pub uart: usart::Uart<'static, Async>,
    pub protocol: Protocol,
    pub baudrate: Bitrate,
    pub nav_period_ms: u16,
}

static PPS_PERIOD_US: u32 = 1000000u32;

// checksum is over class, id, length, payload only
fn checksum(buffer: &[u8]) -> (u8, u8) {
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;

    for &byte in buffer {
        ck_a = ck_a.wrapping_add(byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }

    (ck_a, ck_b)
}

fn make_packet(class: u8, id: u8, payload: &[u8], buffer: &mut [u8]) -> bool {
    let length = payload.len();

    // Check for payload too big
    if length + 8 > buffer.len() {
        return false;
    }

    // header
    buffer[0] = 0xB5;
    buffer[1] = 0x62;
    buffer[2] = class;
    buffer[3] = id;
    buffer[4..6].copy_from_slice(&(length as u16).to_le_bytes());

    // payload
    buffer[6..length + 6].copy_from_slice(payload);

    // checksum
    let (ck_a, ck_b) = checksum(&buffer[2..length + 6]);
    buffer[length + 6] = ck_a;
    buffer[length + 7] = ck_b;

    true
}

impl UbloxSensor {
    async fn tx(&mut self, class: u8, id: u8, payload: &[u8]) -> bool {
        let mut buffer = [0u8; BUFFER_LEN]; // largest ubx packet length we will support
        // Make packet
        if make_packet(class, id, payload, &mut buffer) {
            // Make the expected ack packet
            let mut ack: [u8; 10] = [0u8; 10];
            let _result = make_packet(0x05, 0x01, &[class, id], &mut ack); // ack packet is class=0x05 id=0x01

            // send packet
            let result = self.uart.write(&buffer[0..payload.len() + 8]).await;
            if let Ok(_size) = result {
                // check it it was successful
                let result = self.look_for_ack(&ack).await;
                return result;
            }
        }
        false
    }

    async fn cfg_prt(&mut self, _baud: u32) -> bool {
        match self.protocol {
            Protocol::M8 => {
                let mut payload = [0u8; 20]; // #define CFG_PRT_LENGTH 20
                payload[0] = 0x01; // Port 1 is the UART
                //payload[1] = 0x00; // Reserved
                //payload[2] = 0x00; // txReady
                //payload[3] = 0x00; // txReady
                payload[4] = 0xC0; // mode 1100 0000 (8-bit character length)
                payload[5] = 0x08; // mode 0000 1000 (No parity, 1 stop bit)
                //payload[6] = 0x00; // mode
                //payload[7] = 0x00; // mode
                payload[8..12].copy_from_slice(&(self.baudrate as u32).to_le_bytes()); // meas rate
                payload[12] = 0x01; // inProtoMask (ubx)
                //payload[13] = 0x00; // inProtoMask
                payload[14] = 0x01; // outProtoMask (ubx)
                //payload[15] = 0x00; // outProtoMask
                //payload[16] = 0x00; // flags
                //payload[17] = 0x00; // flags
                //payload[18] = 0x00; // reserved2
                //payload[19] = 0x00; // reserved2

                self.tx(0x06, 0x00, &payload).await
            }
            Protocol::M9 => {
                let mut payload = [0u8; 34];
                //payload[0] = 0x00; // Message Version (1 bytes)
                payload[1] = 0x01; // Write to RAM bit 1 is ram, 2 is bbr layer, 3 is flash (1 bytes)
                //payload[2] = 0x00; // transaction/action (1 bytes)
                //payload[3] = 0x00; // reserved0 (1 bytes)

                // Key-Value pairs
                // baud rate (8 bytes)
                payload[4..8].copy_from_slice(&0x40520001u32.to_le_bytes());
                payload[8..12].copy_from_slice(&(self.baudrate as u32).to_le_bytes());

                // output rate in milliseconds (6 bytes)
                payload[12..16].copy_from_slice(&0x30210001u32.to_le_bytes());
                payload[16..18].copy_from_slice(&self.nav_period_ms.to_le_bytes());

                // 1 data output per nav measurement (6 bytes)
                payload[18..22].copy_from_slice(&0x30210002u32.to_le_bytes());
                payload[22..24].copy_from_slice(&1u16.to_le_bytes());

                // CFG-NAVSPG-DYNMODEL 8 = 4G Airborne (5 bytes)
                payload[24..28].copy_from_slice(&0x20110021u32.to_le_bytes());
                payload[28] = 8;

                // CFG-NAVSPG-FIXMODE 3 = Auto 2/3D (5 bytes)
                payload[29..33].copy_from_slice(&0x20110011u32.to_le_bytes());
                payload[33] = 3u8;

                self.tx(0x06, 0x8A, &payload).await
            }
        }
    }

    async fn cfg_rate(&mut self) -> bool {
        let mut payload = [0u8; 6]; // #define CFG_RATE_LENGTH 6
        payload[0..2].copy_from_slice(&self.nav_period_ms.to_le_bytes()); // meas rate
        payload[2..4].copy_from_slice(&0x0001u16.to_le_bytes()); // nav rate = meas rate
        payload[4..6].copy_from_slice(&0x0000u16.to_le_bytes()); // UTC time reference

        self.tx(0x06, 0x08, &payload).await
    }

    async fn cfg_tp5(&mut self) -> bool {
        let mut payload = [0u8; 32]; // #define SFG_TP5_LENGTH 32
        payload[0] = 0; // Timepulse pin 0
        payload[1] = 1; // Version 1
        //payload[2] = 0; // reserved
        //payload[3] = 0; // reserved
        //payload[4..6].copy_from_slice(& 0u16.to_le_bytes()); // antenna delay
        //payload[6..8].copy_from_slice(& 0u16.to_le_bytes()); // rf group delay
        let pps_period_us = PPS_PERIOD_US; //(pps_period_ms as u32)*1000u32;
        payload[8..12].copy_from_slice(&pps_period_us.to_le_bytes()); // pulse period
        payload[12..16].copy_from_slice(&pps_period_us.to_le_bytes()); // pulse period if locked other set
        let pulse_len_us = 1000u32;
        payload[16..20].copy_from_slice(&pulse_len_us.to_le_bytes()); // pulse high time
        payload[20..24].copy_from_slice(&pulse_len_us.to_le_bytes()); // pulse high time
        // payload[24..28].copy_from_slice(& 0u32.to_le_bytes()); // pulse high time
        payload[28..32].copy_from_slice(&0x01F7u32.to_le_bytes()); // pulse high time

        self.tx(0x06, 0x31, &payload).await
    }

    async fn cfg_nav5(&mut self) -> bool {
        let mut payload = [0u8; 36]; // #define CFG_NAV5_LENGTH 36
        payload[0] = 5; // Parameters bitmask
        payload[1] = 8; // Airbourne navigatin < 4G's
        payload[2] = 3; // Auto 2d/3d fix mode
        self.tx(0x06, 0x24, &payload).await
    }

    async fn cfg_msg(&mut self, class: u8, id: u8, decimation_rate: u8) -> bool {
        let payload = [class, id, decimation_rate];
        self.tx(0x06, 0x01, &payload).await
    }

    async fn look_for_ack(&mut self, ack: &[u8]) -> bool {
        let mut buffer = [0u8; 256];

        // Expected ack packet [0xB5u8, 0x62u8, 0x05u8, 0x01u8, 0x02u8, 0x00u8, 0x06u8, 0x00u8, 0x0Eu8, 0x37u8,]
        // Read data block with 2 second timeout
        match with_timeout(
            Duration::from_secs(2),
            self.uart.read_until_idle(&mut buffer),
        )
        .await
        {
            Ok(Ok(_size)) => {
                for subarray in buffer.windows(ack.len()) {
                    if ack == subarray {
                        return true;
                    }
                }
            }
            Ok(Err(_read_err)) => {
                // underlying UART/read error from self.uart.read(...)
            }
            Err(_timeout_err) => {
                // with_timeout timed out
            }
        }
        false
    }

    async fn sync_baudrate(&mut self) -> bool {
        // Determine baud rate
        let bauds = [
            Bitrate::Baud9600 as u32,
            Bitrate::Baud38400 as u32,
            Bitrate::Baud57600 as u32,
            Bitrate::Baud115200 as u32,
            Bitrate::Baud230400 as u32,
        ]; //, 460800u32, 921600u32];

        for _retries in 0..30 {
            for baud in bauds {
                // try baud rate
                // set stm32 baud rate
                let _result: Result<(), usart::ConfigError> = self.uart.set_baudrate(baud);

                // set ublox the desired baud rate
                let result = self.cfg_prt(self.baudrate as u32).await;

                if result {
                    let _result: Result<(), usart::ConfigError> =
                        self.uart.set_baudrate(self.baudrate as u32);
                    return true;
                }
            }
        }
        false
    }

    pub async fn run(&mut self) {
        let _synced = self.sync_baudrate().await;

        // Disable these messages
        self.cfg_msg(0x0A, 0x09, 0).await; // MON-HW
        self.cfg_msg(0x0A, 0x0B, 0).await; // MON-HW2
        self.cfg_msg(0x01, 0x04, 0).await; // NAV-DOP
        self.cfg_msg(0x01, 0x03, 0).await; // NAV-STATUS
        self.cfg_msg(0x01, 0x35, 0).await; // NAV-SAT
        self.cfg_msg(0x01, 0x20, 0).await; // NAV-TIMEGPS
        self.cfg_msg(0x01, 0x01, 0).await; // NAV-POSECEF (length 20)
        self.cfg_msg(0x01, 0x11, 0).await; // NAV-VELECEF (length 20)

        // These are needed if you want ECEF, but disable for now
        self.cfg_msg(0x01, 0x20, 0).await; // NAV-TIMEGPS (length 16)
        self.cfg_msg(0x01, 0x01, 0).await; // NAV-POSECEF (length 20)
        self.cfg_msg(0x01, 0x11, 0).await; // NAV-VELECEF (length 20)

        // Enable this messages
        self.cfg_msg(0x01, 0x07, 1).await; // NAV-PVT (length 92)

        // Set GPS Configuration (already done in cfg_prt() for UBX_M9)
        if let Protocol::M8 = self.protocol {
            self.cfg_rate().await;
            self.cfg_tp5().await;
            self.cfg_nav5().await;
        }
        pub struct UbxFrame {
            cl: u8,
            id: u8,
            length: usize,
            a: u8, // checksum
            b: u8, // checksum
            payload: [u8; UBX_MAX_PAYLOAD_BYTES],
        }

        let mut p: UbxFrame = UbxFrame {
            cl: 0u8,
            id: 0u8,
            length: 0usize,
            a: 0u8,
            b: 0u8,
            payload: [0u8; UBX_MAX_PAYLOAD_BYTES],
        };
        let mut n = 0usize;
        let mut pps_timestamp = 0;
        loop {
            // get most recent pps timestamp
            match pps::PPS_SIGNAL.try_take() {
                Some(packet) => {
                    pps_timestamp = packet.header.timestamp;
                }
                None => {}
            }

            let mut buffer = [0u8; BUFFER_LEN];

            let result = self.uart.read_until_idle(&mut buffer).await;
            if let Ok(size) = result {
                // This could be a function, but might as well just chug through this here
                for &c in buffer[0..size].iter() {
                    // special case where we get 0xB5 randomly duplicated at the start (DMA wierdness).

                    if (c == 0xB5) && (n == 1) {
                        n = 0;
                    }

                    if n == 0
                    // header byte 1 "mu" character
                    {
                        if c == 0xB5 {
                            n += 1;
                        } else {
                            n = 0;
                        }
                    } else if n == 1
                    // header byte 2
                    {
                        if c == 0x62 {
                            n += 1;
                        } else if c == 0xB5 {
                            n = 1;
                        }
                        // repeated 'mu'
                        else {
                            n = 0;
                        }
                    } else if n == 2
                    // Class
                    {
                        p.a = 0;
                        p.b = 0; // Reset the checksum calculation
                        p.cl = c;
                        n += 1;
                        p.a = p.a.wrapping_add(c);
                        p.b = p.b.wrapping_add(p.a);
                    } else if n == 3
                    // ID, allow all
                    {
                        p.id = c;
                        n += 1;
                        p.a = p.a.wrapping_add(c);
                        p.b = p.b.wrapping_add(p.a);
                    } else if n == 4
                    // length LSB
                    {
                        p.length = c as usize;
                        n += 1;
                        p.a = p.a.wrapping_add(c);
                        p.b = p.b.wrapping_add(p.a);
                    } else if n == 5
                    // length MSB
                    {
                        p.length |= (c as usize) << 8;
                        if p.length > UBX_MAX_PAYLOAD_BYTES {
                            n = 0;
                        } else {
                            n += 1;
                            p.a = p.a.wrapping_add(c);
                            p.b = p.b.wrapping_add(p.a);
                        }
                    } else if n < p.length + 6
                    // Packet Payload bytes and first byte of checksum.
                    {
                        p.payload[n - 6] = c;
                        n += 1;
                        p.a = p.a.wrapping_add(c);
                        p.b = p.b.wrapping_add(p.a);
                    } else if n == p.length + 6
                    // Checksum A
                    {
                        if p.a != c {
                            n = 0;
                        } else {
                            n += 1;
                        }
                    } else {
                        n = 0;
                        if p.b == c {
                            // we found a valid packet
                            if (p.cl == 0x01) || (p.id == 0x07)
                            // pvt packet
                            {
                                let end_of_packet_timestamp = Instant::now();

                                // map the payload into the pvt union
                                let mut payload = [0u8; 92];
                                payload.copy_from_slice(&p.payload[0..92]);
                                let pvt = PvtUnion { payload };

                                // build up the device specific status register.
                                // (from Bitfield valid) [0] = validDate, [1] = validTime, [2] = fullyResolved, [3] validMag, none ignored
                                // (from Bitfield flags) [4] = gnssFixOK , [5] = diffSoln, [6] = psmState, [7] = headVehValid, [8] = carrSoln, none ignored
                                // (from Bitfield flags2) [9] = confirmedAvai, [10] = confirmedDate, [11] = confirmedTime, none ignored
                                // (from Bitfield flags3) [12] = invalidLlh, [13] = lastCorrectionAge

                                let mut status = 0u16;
                                let valid = unsafe { pvt.packet }.valid;
                                let flags = unsafe { pvt.packet }.flags;
                                let flags2 = unsafe { pvt.packet }.flags2;
                                let flags3 = unsafe { pvt.packet }.flags3;

                                if (valid & 0x01) != 0 {
                                    status |= 0x0001
                                }; // validDate
                                if (valid & 0x02) != 0 {
                                    status |= 0x0002
                                }; // validTime
                                if (valid & 0x04) != 0 {
                                    status |= 0x0004
                                }; // fullyResolved
                                if (valid & 0x08) != 0 {
                                    status |= 0x0008
                                }; // validMag

                                if (flags & 0x01) != 0 {
                                    status |= 0x0010
                                }; // gnssFixOK
                                if (flags & 0x02) != 0 {
                                    status |= 0x0020
                                }; // diffSoln
                                if (flags & 0x10) != 0 {
                                    status |= 0x0020
                                }; // pmsState
                                if (flags & 0x20) != 0 {
                                    status |= 0x0040
                                }; // headVehValid
                                if (flags & 0x80) != 0 {
                                    status |= 0x0080
                                }; // carrSoln

                                if (flags2 & 0x20) != 0 {
                                    status |= 0x0100
                                }; // confirmedAvai
                                if (flags2 & 0x40) != 0 {
                                    status |= 0x0200
                                }; // confirmedDate
                                if (flags2 & 0x80) != 0 {
                                    status |= 0x0400
                                }; // confirmedTime

                                if (flags3 & 0x0001) != 0 {
                                    status |= 0x0800
                                }; // invalidLlh
                                if (flags3 & 0x0010) != 0 {
                                    status |= 0x1000
                                }; // lastCorrectionAge

                                // put in terms of microseconds
                                let t0 = pps_timestamp as u64; // top of seconds
                                let t1 = end_of_packet_timestamp.as_micros();
                                let nav_dt = (self.nav_period_ms as u64) * 1000;

                                // phase offset from t0, doesn't matter if its a little old
                                // we are just counting off how many nav times we are behind the time pulse.
                                let dt = ((t1 - t0) / nav_dt) * nav_dt;

                                let timestamp = Instant::from_micros(t0 + dt);

                                let header = packets::RosflightPacketHeader {
                                    timestamp: timestamp.as_micros(),
                                    status,
                                };

                                let _fix_type =
                                    packets::GNSSFixType::from_u8(unsafe { pvt.packet }.fix_type);

                                let _pi = 3.141592654;
                                let pvt_packet = packets::GNSSPacket {
                                    header: header,
                                    unix_seconds: unix_seconds_from_utc(
                                        unsafe { pvt.packet }.year,
                                        unsafe { pvt.packet }.month,
                                        unsafe { pvt.packet }.day,
                                        unsafe { pvt.packet }.hour,
                                        unsafe { pvt.packet }.min,
                                        unsafe { pvt.packet }.sec,
                                    ),
                                    unix_nanos: unsafe { pvt.packet }.nano,
                                    lat: (unsafe { pvt.packet }.lat as f64) * 1.7453292519943296e-9,
                                    lon: (unsafe { pvt.packet }.lon as f64) * 1.7453292519943296e-9,
                                    height: (unsafe { pvt.packet }.height as f32) / 1000.0,
                                    vel_n: (unsafe { pvt.packet }.vel_n as f32) / 1000.0,
                                    vel_e: (unsafe { pvt.packet }.vel_e as f32) / 1000.0,
                                    vel_d: (unsafe { pvt.packet }.vel_d as f32) / 1000.0,
                                    h_acc: (unsafe { pvt.packet }.h_acc as f32) / 1000.0,
                                    v_acc: (unsafe { pvt.packet }.v_acc as f32) / 1000.0,
                                    s_acc: (unsafe { pvt.packet }.s_acc as f32) / 1000.0,
                                    month: unsafe { pvt.packet }.month,
                                    day: unsafe { pvt.packet }.day,
                                    year: unsafe { pvt.packet }.year,
                                    hour: unsafe { pvt.packet }.hour,
                                    min: unsafe { pvt.packet }.min,
                                    sec: unsafe { pvt.packet }.sec,
                                    nano: unsafe { pvt.packet }.nano,
                                    fix_type: packets::GNSSFixType::from_u8(
                                        unsafe { pvt.packet }.fix_type,
                                    ),
                                    num_sats: unsafe { pvt.packet }.num_sv,
                                    mag_dec: (unsafe { pvt.packet }.mag_dec as f32)
                                        * 1.7453292519943296e-4,
                                    time_correction: dt,
                                };
                                GNSS_SIGNAL.signal(Ok(pvt_packet));
                            }
                        }
                    }
                }
            }
        }
    }
}

#[embassy_executor::task]
pub async fn task(mut ublox: UbloxSensor) {
    ublox.run().await;
}
