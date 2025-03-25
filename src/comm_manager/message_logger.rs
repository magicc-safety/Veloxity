pub const LOG_MESSAGE_SIZE: usize = 50;
pub const LOG_BUFFER_SIZE: usize = 25;

#[derive(Copy, Clone)]
pub enum LogSeverity {
    LogInfo,
    LogWarning,
    LogError,
    LogCritical,
}

#[derive(Copy, Clone)]
pub struct LogMessage {
    pub msg: [u8; LOG_MESSAGE_SIZE],
    pub msg_len: usize,
    pub severity: LogSeverity,
}

impl LogMessage {
    pub fn construct_msg(msg: &str) -> [u8; 50] {
        let mut arr = [0u8; 50];
        for (i, b) in msg.as_bytes().iter().enumerate().take(LOG_MESSAGE_SIZE) {
            arr[i] = *b;
        }

        arr
    }

    pub fn construct_str(&self) -> &str {
        let str_slice = core::str::from_utf8(&self.msg[..self.msg_len]).unwrap();
        str_slice
    }
}

pub struct LogMessageBuffer {
    pub buffer: [LogMessage; LOG_BUFFER_SIZE],
    oldest_idx: usize,
    next_idx: usize,
    length: usize,
}

impl LogMessageBuffer {
    pub fn new() -> Self {
        LogMessageBuffer {
            buffer: [LogMessage {
                msg: [0u8; LOG_MESSAGE_SIZE],
                msg_len: 0,
                severity: LogSeverity::LogInfo,
            }; LOG_BUFFER_SIZE],
            oldest_idx: 0,
            next_idx: 0,
            length: 0,
        }
    }

    pub fn add_message(&mut self, msg: LogMessage) {
        self.buffer[self.next_idx] = msg;
        self.next_idx = (self.next_idx + 1) % LOG_BUFFER_SIZE;

        // quietly over-write old messages (what else can we do?)
        self.length += 1;
        if self.length > LOG_BUFFER_SIZE {
            self.length = LOG_BUFFER_SIZE;
            self.oldest_idx = (self.oldest_idx + 1) % LOG_BUFFER_SIZE;
        }
    }

    pub fn size(&self) -> usize {
        self.length
    }

    pub fn empty(&self) -> bool {
        self.length > 0
    }

    pub fn full(&self) -> bool {
        self.length == LOG_BUFFER_SIZE
    }

    pub fn oldest(&self) -> Option<&LogMessage> {
        if self.length > 0 {
            Some(&self.buffer[self.oldest_idx])
        } else {
            None
        }
    }

    pub fn pop(&mut self) {
        if self.length > 0 {
            self.length -= 1;
            self.oldest_idx = (self.oldest_idx + 1) % LOG_BUFFER_SIZE;
        }
    }
}

#[cfg(test)]
mod test_logger {
    use super::*;

    #[test]
    fn test_add() {
        let mut l = LogMessageBuffer::new();
        let m = LogMessage {
            msg: LogMessage::construct_msg("Hello World!"),
            msg_len: "Hello World!".as_bytes().len(),
            severity: LogSeverity::LogInfo,
        };

        l.add_message(m.clone());

        assert_eq!(l.size(), 1);
    }

    #[test]
    fn test_pop() {
        let mut l = LogMessageBuffer::new();
        let m = LogMessage {
            msg: LogMessage::construct_msg("Hello World"),
            msg_len: "Hello World".as_bytes().len(),
            severity: LogSeverity::LogInfo,
        };

        l.add_message(m.clone());
        l.add_message(m.clone());
        l.add_message(m.clone());
        l.add_message(m.clone());
        l.add_message(m.clone());

        assert_eq!(l.size(), 5);

        l.pop();

        assert_eq!(l.size(), 4);

        l.pop();

        assert_eq!(l.size(), 3)
    }

    #[test]
    fn test_fill() {
        let mut l = LogMessageBuffer::new();
        let m = LogMessage {
            msg: LogMessage::construct_msg("Hello World"),
            msg_len: "Hello World".as_bytes().len(),
            severity: LogSeverity::LogInfo,
        };

        for i in 0..30 {
            l.add_message(m.clone())
        }

        assert_eq!(l.size(), 25)
    }

    #[test]
    fn test_remove_empty() {
        let mut l = LogMessageBuffer::new();

        for _ in 0..26 {
            l.pop();
        }
    }

    #[test]
    fn test_retrieve() {
        let mut l = LogMessageBuffer::new();
        let m = LogMessage {
            msg: LogMessage::construct_msg("Hello World"),
            msg_len: "Hello World".as_bytes().len(),
            severity: LogSeverity::LogInfo,
        };

        let m2 = LogMessage {
            msg: LogMessage::construct_msg("Foo Bar!"),
            msg_len: "Foo Bar!".as_bytes().len(),
            severity: LogSeverity::LogInfo,
        };

        l.add_message(m);
        l.add_message(m2);

        assert_eq!(l.oldest().unwrap().construct_str(), "Hello World");

        l.pop();

        assert_eq!(l.oldest().unwrap().construct_str(), "Foo Bar!");

        l.pop();
    }
}
