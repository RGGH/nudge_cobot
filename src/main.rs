use std::env;
use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::thread::sleep;
use std::time::{Duration, Instant};

const DEFAULT_ADDR: &str = "192.168.1.236:9000";
const SERIAL_PORT: &str = "/dev/ttyAMA0";
const BAUD: &str = "1000000";
const SERIAL_TIMEOUT: &str = "0.1";

const FOOTER: u8 = 0xFA;

/// Command bytes from the myCobot protocol.
mod cmd {
    pub const POWER_ON: u8 = 0x10;
    pub const GET_ANGLES: u8 = 0x20;
    pub const SEND_ANGLES: u8 = 0x22;
}

type Angles = [f32; 6];

/// Joint limits in degrees. These are the myCobot 280 values; adjust for
/// other models (the 320, for example, differs).
const JOINT_LIMITS: [(f32, f32); 6] = [
    (-168.0, 168.0),
    (-135.0, 135.0),
    (-150.0, 150.0),
    (-145.0, 145.0),
    (-165.0, 165.0),
    (-180.0, 180.0),
];

/// Build a myCobot frame: FE FE <len> <cmd> <data...> FA
fn frame(cmd: u8, data: &[u8]) -> Vec<u8> {
    let len = u8::try_from(data.len() + 2).expect("payload too long for one frame");
    let mut f = Vec::with_capacity(data.len() + 5);
    f.extend_from_slice(&[0xFE, 0xFE, len, cmd]);
    f.extend_from_slice(data);
    f.push(FOOTER);
    f
}

/// Find a complete get_angles reply anywhere in `reply` and decode it.
/// Reply layout: FE FE 0E 20 <6 x i16 big-endian, centidegrees> FA (17 bytes).
/// Searching (rather than trusting offset 0) copes with stale bytes from an
/// earlier request arriving in front of the answer.
fn parse_angles(reply: &[u8]) -> Option<Angles> {
    const HEADER: [u8; 4] = [0xFE, 0xFE, 0x0E, cmd::GET_ANGLES];
    const FRAME_LEN: usize = 17;

    reply
        .windows(FRAME_LEN)
        .find(|w| w[..4] == HEADER && w[FRAME_LEN - 1] == FOOTER)
        .map(|w| {
            let mut out = [0f32; 6];
            for (j, slot) in out.iter_mut().enumerate() {
                *slot = i16::from_be_bytes([w[4 + 2 * j], w[5 + 2 * j]]) as f32 / 100.0;
            }
            out
        })
}

/// Reject targets outside the joint limits (NaN fails too) before they reach the robot.
fn check_limits(angles: &Angles) -> io::Result<()> {
    for (i, (a, (lo, hi))) in angles.iter().zip(JOINT_LIMITS).enumerate() {
        if !(lo..=hi).contains(a) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("joint {} angle {a} is outside [{lo}, {hi}]", i + 1),
            ));
        }
    }
    Ok(())
}

/// Move `angle` up to `step` degrees toward 0 without overshooting past it.
fn step_toward_zero(angle: f32, step: f32) -> f32 {
    if angle.abs() <= step {
        0.0
    } else {
        angle - angle.signum() * step
    }
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
        for item in [serial, baud, SERIAL_TIMEOUT] {
            let reply = c.send_recv(item.as_bytes())?;
            if reply != b"1" {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!(
                        "handshake failed on {item:?}, server said {:?}",
                        String::from_utf8_lossy(&reply)
                    ),
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

    /// Throw away any late replies still sitting in the socket, so they can't be
    /// mistaken for the answer to the next request.
    fn drain(&mut self) -> io::Result<()> {
        self.s.set_nonblocking(true)?;
        let mut buf = [0u8; 1024];
        let result = loop {
            match self.s.read(&mut buf) {
                Ok(0) => {
                    break Err(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "server closed",
                    ));
                }
                Ok(_) => continue,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break Ok(()),
                Err(e) => break Err(e),
            }
        };
        self.s.set_nonblocking(false)?;
        result
    }

    /// This server version wants the frame as the TEXT of a list: "[254, 254, 2, 32, 250]".
    /// Rust's {:?} on a Vec<u8> prints exactly that.
    fn request(&mut self, cmd: u8, data: &[u8]) -> io::Result<Vec<u8>> {
        let text = format!("{:?}", frame(cmd, data));
        self.send_recv(text.as_bytes())
    }

    fn power_on(&mut self) -> io::Result<()> {
        self.request(cmd::POWER_ON, &[])?;
        Ok(())
    }

    /// Read the 6 joint angles in degrees. The server only waits 50 ms for the
    /// robot, so the answer can arrive one request late: retry until a complete,
    /// valid frame shows up.
    fn get_angles(&mut self) -> io::Result<Angles> {
        const ATTEMPTS: usize = 5;
        for attempt in 0..ATTEMPTS {
            if attempt > 0 {
                sleep(Duration::from_millis(100));
            }
            self.drain()?;
            let reply = self.request(cmd::GET_ANGLES, &[])?;
            if let Some(angles) = parse_angles(&reply) {
                return Ok(angles);
            }
        }
        Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "robot never answered get_angles (check power, wiring, baud)",
        ))
    }

    /// Move to the given joint angles (degrees) at speed 0-100.
    /// Targets outside the joint limits are rejected before anything is sent.
    /// The reply to motion commands is meaningless; confirm with `wait_until_reached`.
    fn send_angles(&mut self, angles: Angles, speed: u8) -> io::Result<()> {
        check_limits(&angles)?;
        let mut data = Vec::with_capacity(13);
        for a in angles {
            data.extend_from_slice(&((a * 100.0).round() as i16).to_be_bytes());
        }
        data.push(speed.min(100));
        self.request(cmd::SEND_ANGLES, &data)?;
        Ok(())
    }

    /// Poll the real joint angles until every joint is within `tolerance` degrees
    /// of `target`, or fail after `timeout`. Returns the final reading.
    fn wait_until_reached(
        &mut self,
        target: &Angles,
        tolerance: f32,
        timeout: Duration,
    ) -> io::Result<Angles> {
        let deadline = Instant::now() + timeout;
        loop {
            let now = self.get_angles()?;
            if now
                .iter()
                .zip(target)
                .all(|(a, b)| (a - b).abs() <= tolerance)
            {
                return Ok(now);
            }
            if Instant::now() >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("target not reached within {timeout:?}; last reading {now:?}"),
                ));
            }
            sleep(Duration::from_millis(100));
        }
    }
}

fn main() -> io::Result<()> {
    // Usage: cobot [ADDR]   (defaults to DEFAULT_ADDR)
    let addr = env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_ADDR.to_string());
    let mut c = Cobot::connect(&addr, SERIAL_PORT, BAUD)?;

    let start = c.get_angles()?;
    println!("start: {start:?}");

    c.power_on()?;
    sleep(Duration::from_millis(500));

    // Small, slow test move: joint 2 (shoulder, index 1) up to 5 degrees toward 0
    // (arm vertical, "toward the sky"), never past it. Keep the arm clear.
    let mut target = start;
    target[1] = step_toward_zero(start[1], 5.0);
    c.send_angles(target, 20)?;

    let end = c.wait_until_reached(&target, 1.0, Duration::from_secs(5))?;
    println!("end:   {end:?}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_matches_protocol_example() {
        assert_eq!(frame(cmd::GET_ANGLES, &[]), vec![254, 254, 2, 32, 250]);
        assert_eq!(format!("{:?}", frame(0x20, &[])), "[254, 254, 2, 32, 250]");
    }

    #[test]
    fn parse_angles_skips_leading_junk() {
        let mut reply = vec![0xFE, 0xFE, 0x02, 0x10, 0xFA]; // stale power_on reply
        reply.extend_from_slice(&[0xFE, 0xFE, 0x0E, 0x20]);
        for a in [100i16, -250, 0, 9000, -9000, 1] {
            reply.extend_from_slice(&a.to_be_bytes());
        }
        reply.push(0xFA);
        assert_eq!(
            parse_angles(&reply),
            Some([1.0, -2.5, 0.0, 90.0, -90.0, 0.01])
        );
    }

    #[test]
    fn parse_angles_rejects_truncated_or_unterminated_frames() {
        assert_eq!(parse_angles(&[0xFE, 0xFE, 0x0E, 0x20, 0, 0]), None);
        let mut bad = vec![0xFE, 0xFE, 0x0E, 0x20];
        bad.extend_from_slice(&[0; 12]);
        bad.push(0x00); // wrong footer
        assert_eq!(parse_angles(&bad), None);
    }

    #[test]
    fn step_toward_zero_never_overshoots() {
        assert_eq!(step_toward_zero(30.0, 5.0), 25.0);
        assert_eq!(step_toward_zero(-30.0, 5.0), -25.0);
        assert_eq!(step_toward_zero(2.0, 5.0), 0.0);
        assert_eq!(step_toward_zero(0.0, 5.0), 0.0);
    }

    #[test]
    fn limits_are_enforced() {
        assert!(check_limits(&[0.0; 6]).is_ok());
        assert!(check_limits(&[0.0, 136.0, 0.0, 0.0, 0.0, 0.0]).is_err());
        assert!(check_limits(&[f32::NAN, 0.0, 0.0, 0.0, 0.0, 0.0]).is_err());
    }
}
