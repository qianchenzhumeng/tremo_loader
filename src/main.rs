use clap::SubCommand;
use clap::{crate_authors, crate_name, crate_version, App, Arg};
use crc::{Crc, CRC_32_ISO_HDLC};
use regex::Regex;
use serialport::SerialPort;
use shadow_rs::shadow;
use std::fs;
use std::thread;
use std::time::Duration;

shadow!(build);

#[macro_use]
extern crate structure;

const BAUDRATE: u32 = 921600;

const CMD_SYNC: u8 = 1;
const CMD_FLASH: u8 = 3;
const CMD_ERASE: u8 = 4;
const CMD_VERIFY: u8 = 5;

enum BootMode {
    Flash,
    Run,
}

struct TremoLoader {
    port: Box<dyn SerialPort>,
}

impl TremoLoader {
    fn wait_response(&mut self) -> Result<(u8, Vec<u8>), String> {
        let crc: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);
        let mut pkt: Vec<u8> = vec![0; 0];
        let mut header: Vec<u8> = vec![0; 4];
        match self.port.read_exact(header.as_mut_slice()) {
            Ok(_) => {
                pkt.extend_from_slice(&header);
                match structure!("<BBH").unpack(header) {
                    Ok((start, status, rsp_data_len)) => {
                        if start != 0xFE {
                            return Err(String::from("Wrong response packet"));
                        }
                        if rsp_data_len > 512 {
                            return Err(String::from("Wrong response packet length"));
                        }

                        let mut remain_data: Vec<u8> = vec![0; rsp_data_len as usize + 5];
                        if let Ok(_) = self.port.read_exact(remain_data.as_mut_slice()) {
                            pkt.extend_from_slice(&remain_data);
                            println!("recv Packed Value: {:02X?}", pkt);
                            match structure!("<IB").unpack(&pkt[4 + rsp_data_len as usize..]) {
                                Ok((checksum_in_pkt, end)) => {
                                    if end != 0xEF {
                                        return Err(format!(
                                            "Wrong response packet: invalid EOF(0x{:02X})",
                                            end
                                        ));
                                    }
                                    let checksum = crc.checksum(&pkt[..4 + rsp_data_len as usize]);
                                    if checksum != checksum_in_pkt {
                                        return Err(format!("Response crc error: (0x{:08X} in frame and 0x{:08X} expected)", checksum_in_pkt, checksum));
                                    }
                                    return Ok((
                                        status,
                                        pkt[4..4 + rsp_data_len as usize].to_vec(),
                                    ));
                                }
                                Err(e) => {
                                    return Err(format!("Wrong response packet: {}", e));
                                }
                            }
                        } else {
                            return Err(format!("Read response data timeout"));
                        }
                    }
                    Err(e) => {
                        return Err(format!("Read response head timeout: {}", e));
                    }
                }
            }
            Err(e) => {
                return Err(format!("Read response header timeout: {}", e));
            }
        }
    }

    fn requeset(&mut self, cmd: u8, data: Option<Vec<u8>>) {
        let format = structure!("<BBH"); // 小端
        let crc: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);
        match data {
            Some(v) => {
                if let Ok(buf) = format.pack(0xFE, cmd, v.len() as u16) {
                    let mut frame: Vec<u8> = buf.to_vec();
                    frame.extend_from_slice(&v);
                    let checksum = crc.checksum(&frame).to_le_bytes(); // 小端
                    frame.extend_from_slice(&checksum.to_vec());
                    frame.extend_from_slice(&vec![0xEF; 1]);
                    println!("send Packed Value: {:02X?}", frame);
                    let _ = self.port.write(&frame);
                }
            }
            None => {
                if let Ok(buf) = format.pack(0xFE, cmd, 0) {
                    let mut frame: Vec<u8> = buf.to_vec();
                    let checksum = crc.checksum(&frame).to_le_bytes(); // 小端
                    frame.extend_from_slice(&checksum.to_vec());
                    frame.push(0xEF);
                    println!("send Packed Value: {:02X?}", frame);
                    let _ = self.port.write(&frame);
                }
            }
        }
    }

    fn sync(&mut self) -> Result<(), ()> {
        self.requeset(CMD_SYNC, None);
        if let Ok(_) = self.wait_response() {
            return Ok(());
        } else {
            return Err(());
        }
    }

    fn hw_reset(&mut self, mode: BootMode) {
        match mode {
            BootMode::Flash => {
                let _ = self.port.write_data_terminal_ready(true); // gpio2 0
                let _ = self.port.write_request_to_send(true); // rst 0
                thread::sleep(Duration::from_millis(100));
                let _ = self.port.write_data_terminal_ready(false); // gpio2 1
                thread::sleep(Duration::from_millis(100));
                let _ = self.port.write_request_to_send(false); // rst 1
            }
            BootMode::Run => {
                let _ = self.port.write_data_terminal_ready(true); // gpio2 0
                let _ = self.port.write_request_to_send(true); // rst 0
                thread::sleep(Duration::from_millis(100));
                let _ = self.port.write_request_to_send(false); // rst 1
                thread::sleep(Duration::from_millis(100));
                let _ = self.port.write_data_terminal_ready(false); // gpio2 1
            }
        }
    }

    fn connect(&mut self, retry: u8) {
        println!("Connecting...");
        for _ in 0..retry {
            self.hw_reset(BootMode::Flash);
            if let Ok(_) = self.sync() {
                println!("Connected");
                break;
            }
        }
    }

    fn erase(&mut self, addr: u32, size: usize) -> Result<(), String> {
        match structure!("<II").pack(addr, size as u32) {
            Ok(v) => {
                self.requeset(CMD_ERASE, Some(v));
                match self.wait_response() {
                    Ok((ret, _)) => {
                        if ret != 0 {
                            return Err(String::from("Erase error"));
                        } else {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        return Err(e);
                    }
                }
            }
            Err(e) => {
                return Err(e.to_string());
            }
        }
    }

    fn flash(&mut self, addr: u32, line_data: &Vec<u8>) -> Result<(), String> {
        match structure!("<II").pack(addr, line_data.len() as u32) {
            Ok(mut v) => {
                v.extend_from_slice(line_data);
                self.requeset(CMD_FLASH, Some(v));
                match self.wait_response() {
                    Ok((ret, _)) => {
                        if ret != 0 {
                            return Err(String::from("Flash error"));
                        } else {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        return Err(e);
                    }
                }
            }
            Err(e) => {
                return Err(e.to_string());
            }
        }
    }

    fn verify(&mut self, addr: u32, size: usize, checksum: u32) -> Result<(), String> {
        match structure!("<III").pack(addr, size as u32, checksum) {
            Ok(v) => {
                self.requeset(CMD_VERIFY, Some(v));
                match self.wait_response() {
                    Ok((ret, _)) => {
                        if ret != 0 {
                            return Err(String::from("Verify error"));
                        } else {
                            return Ok(());
                        }
                    }
                    Err(e) => {
                        return Err(e);
                    }
                }
            }
            Err(e) => {
                return Err(e.to_string());
            }
        }
    }
}

impl TremoLoader {
    pub fn tremo_flash(&mut self, download_file: &str, address: u32) {
        let crc: Crc<u32> = Crc::<u32>::new(&CRC_32_ISO_HDLC);
        self.connect(3);
        match fs::read(download_file) {
            Ok(mut file) => {
                let image_checksum = crc.checksum(&file);
                let image_size = file.len();
                match self.erase(address, image_size) {
                    Ok(_) => {
                        let mut flash_addr = address;
                        let mut line_data: Vec<u8>;
                        let mut l = 0;
                        loop {
                            if file.len() >= 512 {
                                line_data = file.drain(..512).collect();
                            } else {
                                line_data = file.drain(..file.len()).collect();
                            }
                            match self.flash(flash_addr, &line_data) {
                                Ok(_) => {
                                    l += line_data.len();
                                    println!("send: {}", l);
                                }
                                Err(e) => {
                                    println!("{}", e);
                                    return;
                                }
                            }
                            if file.len() != 0 {
                                flash_addr += line_data.len() as u32;
                            } else {
                                break;
                            }
                        }
                    }
                    Err(e) => {
                        println!("{}", e);
                        return;
                    }
                }
                if let Err(e) = self.verify(address, image_size, image_checksum) {
                    println!("{}", e);
                    return;
                }
                self.hw_reset(BootMode::Run);

                println!("Download files successfully");
            }
            Err(e) => {
                println!("打开文件错误：{}", e);
            }
        }
    }

    pub fn tremo_reboot(&mut self, mode: u8) {
        match mode {
            0 => {
                self.hw_reset(BootMode::Flash);
            }
            1 => {
                self.hw_reset(BootMode::Run);
            }
            _ => {}
        }
    }
}

fn main() {
    let matches = App::new(crate_name!())
        .version(crate_version!())
        .long_version(build::version().as_str())
        .author(crate_authors!())
        .arg(
            Arg::with_name("port")
                .short("p")
                .long("port")
                .takes_value(true)
                .required(true)
                .help("指定端口。"),
        )
        .subcommand(
            SubCommand::with_name("flash")
                .about("erase and write the flash")
                .arg(
                    Arg::with_name("address")
                        .short("a")
                        .long("address")
                        .takes_value(true)
                        .required(true)
                        .help("指定烧录地址，例如：0x08000000。"),
                )
                .arg(
                    Arg::with_name("file")
                        .short("f")
                        .long("file")
                        .takes_value(true)
                        .required(true)
                        .help("指定烧录文件。"),
                )
                .help("erase and write the flash"),
        )
        .subcommand(
            SubCommand::with_name("reboot")
                .about("reboot")
                .arg(
                    Arg::with_name("mode")
                        .short("m")
                        .long("mode")
                        .takes_value(true)
                        .required(true)
                        .help("reboot"),
                )
                .help(
                    "hardware reset.\n\
                     MODES:\n    \
                     0\tflash or erase\n    \
                     1\trun",
                ),
        )
        .before_help("ASR6601 Download Tool")
        .after_help("Pins Connection:\n    \
                     DTS <-> gpio 2\n    \
                     RTS <-> rst"
        )
        .get_matches();

    if let Some(port_name) = matches.value_of("port") {
        if let Ok(port) = serialport::new(port_name, BAUDRATE)
            .timeout(Duration::from_secs(5))
            .open()
        {
            let mut loader = TremoLoader { port: port };
            if let Some(sub_m) = matches.subcommand_matches("flash") {
                if let Some(download_file) = sub_m.value_of("file") {
                    if let Some(address) = sub_m.value_of("address") {
                        println!("address: {}", address);
                        if let Ok(re) = Regex::new(r"0[xX][[0-9a-fA-F]]+") {
                            if re.is_match(address) {
                                if let Ok(a) = u32::from_str_radix(&address[2..], 16) {
                                    loader.tremo_flash(download_file, a);
                                }
                            } else {
                                println!("invalid address!");
                            }
                        }
                    }
                }
            }
            if let Some(sub_m) = matches.subcommand_matches("reboot") {
                if let Some(mode) = sub_m.value_of("mode") {
                    match mode {
                        "0" => {
                            loader.tremo_reboot(0);
                        }
                        "1" => {
                            loader.tremo_reboot(1);
                        }
                        _ => {
                            println!("invalid mode!");
                        }
                    }
                }
            }
        } else {
            println!("端口不存在！");
            return ();
        }
    }
}
