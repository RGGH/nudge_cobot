use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::thread::sleep;
use std::time::Duration;

const ADDR: &str = "127.0.0.1:9000";
const SERIAL_PORT: &str = "/dev/ttyAMA0";
const BAUD: &str = "1000000";

/// Build a myCobot frame: FE FE <len> <cmd> <data...> FA
fn frame(cmd: u8, data: &[u8]) -> Vec<u8> {
    let mut f = vec![0xFE, 0xFE, (data.len() + 2) as u8, cmd];
    f.extend_from_slice(data);
    f.push(0xFA);
    f
}

struct Cobot {
    s: TcpStream,
}

impl Cobot {
    /// Connect and do the 3-message handshake (serial port, baud, timeout).
    fn connect(addr: &str, serial: &str, baud: &str) -> io::Result<Self> {
        let s = TcpStream::connect(addr)?;
        s.set_read_timeout(Some(Duration::from_secs(3)))?;
        s.set_nodelay(true)?;
        let mut c = Cobot { s };
        for item in [serial, baud, "0.1"] {
            let reply = c.send_recv(item.as_bytes())?;
            if reply != b"1" {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("handshake failed, server said {:?}", reply),
                ));
            }
        }
        Ok(c)
    }

    /// One message out, one reply in (the server always sends exactly one reply).
    fn send_recv(&mut self, msg: &[u8]) -> io::Result<Vec<u8>> {
        self.s.write_all(msg)?;
        let mut buf = [0u8; 1024];
        let n = self.s.read(&mut buf)?;
        if n == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "server closed",
            ));
        }
        Ok(buf[..n].to_vec())
    }

    /// This server version wants the frame as the TEXT of a list: "[254, 254, 2, 32, 250]".
    /// Rust's {:?} on a Vec<u8> prints exactly that.
    fn request(&mut self, cmd: u8, data: &[u8]) -> io::Result<Vec<u8>> {
        let text = format!("{:?}", frame(cmd, data));
        self.send_recv(text.as_bytes())
    }

    fn power_on(&mut self) -> io::Result<()> {
        self.request(0x10, &[])?;
        Ok(())
    }

    /// Read the 6 joint angles in degrees. The server only waits 50 ms for the
    /// robot, so the answer can arrive one request late: retry and search for
    /// the reply header (FE FE 0E 20) instead of trusting the first bytes.
    fn get_angles(&mut self) -> io::Result<[f32; 6]> {
        const HEADER: [u8; 4] = [0xFE, 0xFE, 0x0E, 0x20];
        for _ in 0..5 {
            let reply = self.request(0x20, &[])?;
            if let Some(i) = reply.windows(4).position(|w| w == HEADER) {
                if reply.len() >= i + 16 {
                    let mut out = [0f32; 6];
                    for (j, slot) in out.iter_mut().enumerate() {
                        let k = i + 4 + 2 * j;
                        *slot = i16::from_be_bytes([reply[k], reply[k + 1]]) as f32 / 100.0;
                    }
                    return Ok(out);
                }
            }
            sleep(Duration::from_millis(100));
        }
        Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "robot never answered get_angles (check power, wiring, baud)",
        ))
    }

    /// Move to the given joint angles (degrees) at speed 0-100.
    /// The reply to motion commands is meaningless; confirm by reading angles back.
    fn send_angles(&mut self, angles: [f32; 6], speed: u8) -> io::Result<()> {
        let mut data = Vec::with_capacity(13);
        for a in angles {
            data.extend_from_slice(&((a * 100.0).round() as i16).to_be_bytes());
        }
        data.push(speed.min(100));
        self.request(0x22, &data)?;
        Ok(())
    }
}

fn main() -> io::Result<()> {
    let mut c = Cobot::connect(ADDR, SERIAL_PORT, BAUD)?;

    let start = c.get_angles()?;
    println!("start: {:?}", start);

    c.power_on()?;
    sleep(Duration::from_millis(500));

    // Small, slow test move: joint 2 (shoulder, index 1) 5 degrees toward 0
    // (arm vertical, "toward the sky"). signum() keeps it moving toward the
    // sky whichever side of vertical the angle is on. Keep the arm clear.
    let mut target = start;
    target[1] -= target[1].signum() * 5.0;
    c.send_angles(target, 20)?;

    sleep(Duration::from_secs(2));
    println!("end:   {:?}", c.get_angles()?);
    Ok(())
}
