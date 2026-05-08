use crate::{
    comm_manager::str_to_fixed_bytes,
    comm_messages::{enums::ParamIdentifier, messages::ParamValueMsg},
    events::{
        COMM_RESPONSE_QUEUE_CAPACITY, CommResponse, PARAM_CHANGED_QUEUE_CAPACITY,
        PARAM_LIST_REQUEST_QUEUE_CAPACITY, PARAM_READ_REQUEST_QUEUE_CAPACITY,
        PARAM_SET_REQUEST_QUEUE_CAPACITY, ParamChanged, ParamListRequested, ParamReadRequested,
        ParamSetRequested,
    },
    params::{PARAM_DEFINITIONS, PARAMS_COUNT, ParamId},
    ports::{EventDrainPort, EventEmitPort, ParamsReadPort, ParamsWritePort},
};

#[derive(Default)]
pub struct ParamListState {
    next_index: Option<usize>,
}

impl ParamListState {
    pub fn is_active(&self) -> bool {
        self.next_index.is_some()
    }
}

pub struct ParamApplyCtx<'a> {
    pub params: ParamsWritePort<'a>,
    pub requests: EventDrainPort<'a, ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>,
    pub changes: EventEmitPort<'a, ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

pub struct ParamListCtx<'a> {
    pub params: ParamsReadPort<'a>,
    pub state: &'a mut ParamListState,
    pub requests: EventDrainPort<'a, ParamListRequested, PARAM_LIST_REQUEST_QUEUE_CAPACITY>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

pub struct ParamReadCtx<'a> {
    pub params: ParamsReadPort<'a>,
    pub requests: EventDrainPort<'a, ParamReadRequested, PARAM_READ_REQUEST_QUEUE_CAPACITY>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

pub fn apply_param_requests(mut ctx: ParamApplyCtx<'_>) {
    while let Some(req) = ctx.requests.next() {
        let Some(id) = param_id_from_name_bytes(req.param_id_bytes) else {
            continue;
        };

        let old = ctx.params.get(id);
        ctx.params.set(id, req.value);
        let new = ctx.params.get(id);

        let changed = ParamChanged {
            id,
            old,
            new,
            param_id_bytes: req.param_id_bytes,
        };
        ctx.changes.emit_or_log(changed, "param changed event");

        let response = ParamValueMsg {
            param_id: req.param_id_bytes,
            param_value: new,
            param_count: PARAMS_COUNT as u16,
            param_index: id as u16,
        };
        ctx.responses
            .emit_or_log(CommResponse::ParamValue(response), "param set response");
    }
}

pub fn service_param_read_requests(mut ctx: ParamReadCtx<'_>) {
    while let Some(req) = ctx.requests.next() {
        let Some(id) = param_id_from_identifier(req.identifier) else {
            continue;
        };
        let def = &PARAM_DEFINITIONS[id as usize];
        let response = ParamValueMsg {
            param_id: str_to_fixed_bytes(def.name),
            param_value: ctx.params.get(id),
            param_count: PARAMS_COUNT as u16,
            param_index: id as u16,
        };
        ctx.responses
            .emit_or_log(CommResponse::ParamValue(response), "param read response");
    }
}

pub fn service_param_list_requests(mut ctx: ParamListCtx<'_>) {
    while ctx.requests.next().is_some() {
        ctx.state.next_index = Some(0);
    }

    let Some(index) = ctx.state.next_index else {
        return;
    };

    let Some(def) = PARAM_DEFINITIONS.get(index) else {
        ctx.state.next_index = None;
        return;
    };

    let response = ParamValueMsg {
        param_id: str_to_fixed_bytes(def.name),
        param_value: ctx.params.get(def.id),
        param_count: PARAMS_COUNT as u16,
        param_index: def.id as u16,
    };
    ctx.responses
        .emit_or_log(CommResponse::ParamValue(response), "param list response");

    let next = index + 1;
    ctx.state.next_index = (next < PARAMS_COUNT).then_some(next);
}

fn param_id_from_identifier(identifier: ParamIdentifier) -> Option<ParamId> {
    match identifier {
        ParamIdentifier::INDEX(index) if index >= 0 => {
            PARAM_DEFINITIONS.get(index as usize).map(|def| def.id)
        }
        ParamIdentifier::INDEX(_) => None,
        ParamIdentifier::ID(bytes) => param_id_from_name_bytes(bytes),
    }
}

fn param_id_from_name_bytes(bytes: [u8; 16]) -> Option<ParamId> {
    let len = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    let name = core::str::from_utf8(&bytes[..len]).ok()?;
    PARAM_DEFINITIONS
        .iter()
        .find(|def| def.name == name)
        .map(|def| def.id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        events::{EventQueue, ParamListRequested, ParamReadRequested, ParamSetRequested},
        params::{ParamId, ParamValue, Params},
        ports::{EventDrainPort, EventEmitPort, ParamsReadPort, ParamsWritePort},
    };

    #[test]
    fn apply_param_requests_mutates_params_and_defers_ack() {
        let mut params = Params::new();
        let mut requests = EventQueue::<ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>::new();
        let mut changes = EventQueue::<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let request = ParamSetRequested {
            value: ParamValue::Int(42),
            param_id_bytes: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
        };
        let _ = requests.push(request);

        apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut params),
            requests: EventDrainPort::new(&mut requests),
            changes: EventEmitPort::new(&mut changes),
            responses: EventEmitPort::new(&mut responses),
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );

        let change = changes.pop().unwrap();
        assert_eq!(change.id, ParamId::PARAM_SYSTEM_ID);
        assert_eq!(change.old, ParamValue::Int(1));
        assert_eq!(change.new, ParamValue::Int(42));

        match responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_SYSTEM_ID as u16);
                assert_eq!(response.param_value, ParamValue::Int(42));
            }
            _ => panic!("expected param value response"),
        }
    }

    #[test]
    fn service_param_list_requests_streams_one_param_per_call() {
        let params = Params::new();
        let mut state = ParamListState::default();
        let mut requests =
            EventQueue::<ParamListRequested, PARAM_LIST_REQUEST_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(ParamListRequested);

        service_param_list_requests(ParamListCtx {
            params: ParamsReadPort::new(&params),
            state: &mut state,
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
        });

        match responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_BAUD_RATE as u16);
                assert_eq!(response.param_value, ParamValue::Int(921600));
            }
            _ => panic!("expected param value response"),
        }
        assert!(state.is_active());

        service_param_list_requests(ParamListCtx {
            params: ParamsReadPort::new(&params),
            state: &mut state,
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
        });

        match responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_SERIAL_DEVICE as u16);
            }
            _ => panic!("expected param value response"),
        }
    }

    #[test]
    fn service_param_read_requests_responds_by_index_and_id() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_SYSTEM_ID, ParamValue::Int(42));
        let mut requests =
            EventQueue::<ParamReadRequested, PARAM_READ_REQUEST_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(ParamReadRequested {
            identifier: ParamIdentifier::INDEX(0),
        });
        let _ = requests.push(ParamReadRequested {
            identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        });

        service_param_read_requests(ParamReadCtx {
            params: ParamsReadPort::new(&params),
            requests: EventDrainPort::new(&mut requests),
            responses: EventEmitPort::new(&mut responses),
        });

        match responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_BAUD_RATE as u16);
                assert_eq!(response.param_value, ParamValue::Int(921600));
            }
            _ => panic!("expected param value response"),
        }

        match responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_SYSTEM_ID as u16);
                assert_eq!(response.param_value, ParamValue::Int(42));
            }
            _ => panic!("expected param value response"),
        }
    }
}
