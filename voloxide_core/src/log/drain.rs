use crate::{
    comm::messages::messages::StatustextMsg,
    events::{COMM_RESPONSE_QUEUE_CAPACITY, CommResponse},
    log::Logger,
    ports::EventEmitPort,
};

const MAX_LOGS_PER_DRAIN: usize = 1;

pub struct LogDrainCtx<'a> {
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    pub connected: bool,
}

pub fn drain_logs_to_comm_responses(mut ctx: LogDrainCtx<'_>) -> usize {
    if !ctx.connected {
        return 0;
    }

    let mut drained = 0;

    while drained < MAX_LOGS_PER_DRAIN {
        let Some(entry) = Logger::pop() else {
            break;
        };

        let mut text = [0u8; 50];
        let bytes = entry.message.as_str().as_bytes();
        let len = bytes.len().min(text.len());
        text[..len].copy_from_slice(&bytes[..len]);

        if ctx
            .responses
            .emit(CommResponse::Statustext(StatustextMsg {
                severity: entry.severity,
                text,
            }))
            .is_err()
        {
            break;
        }

        drained += 1;
    }

    drained
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        comm::messages::enums::Severity,
        events::{COMM_RESPONSE_QUEUE_CAPACITY, EventQueue},
        log_info, log_warn,
    };

    #[test]
    fn drain_logs_queues_statustext_responses() {
        while Logger::pop().is_some() {}

        log_info!("hello");
        log_warn!("world");

        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();
        let drained = drain_logs_to_comm_responses(LogDrainCtx {
            responses: EventEmitPort::new(&mut responses),
            connected: true,
        });

        assert_eq!(drained, 1);
        match responses.pop().unwrap() {
            CommResponse::Statustext(msg) => {
                assert!(matches!(msg.severity, Severity::Info));
                assert_eq!(&msg.text[..5], b"hello");
            }
            _ => panic!("expected statustext response"),
        }

        let drained = drain_logs_to_comm_responses(LogDrainCtx {
            responses: EventEmitPort::new(&mut responses),
            connected: true,
        });

        assert_eq!(drained, 1);
        match responses.pop().unwrap() {
            CommResponse::Statustext(msg) => {
                assert!(matches!(msg.severity, Severity::Warning));
                assert_eq!(&msg.text[..5], b"world");
            }
            _ => panic!("expected statustext response"),
        }
    }

    #[test]
    fn drain_logs_waits_for_companion_connection() {
        while Logger::pop().is_some() {}

        log_info!("buffered");

        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();
        let drained = drain_logs_to_comm_responses(LogDrainCtx {
            responses: EventEmitPort::new(&mut responses),
            connected: false,
        });

        assert_eq!(drained, 0);
        assert!(responses.is_empty());

        let drained = drain_logs_to_comm_responses(LogDrainCtx {
            responses: EventEmitPort::new(&mut responses),
            connected: true,
        });

        assert_eq!(drained, 1);
    }
}
