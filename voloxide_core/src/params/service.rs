use crate::{
    comm::messages::{enums::ParamIdentifier, messages::ParamValueMsg},
    comm::str_to_fixed_bytes,
    events::{
        COMM_RESPONSE_QUEUE_CAPACITY, CommResponse, PARAM_CHANGED_QUEUE_CAPACITY,
        PARAM_LIST_REQUEST_QUEUE_CAPACITY, PARAM_READ_REQUEST_QUEUE_CAPACITY,
        PARAM_SET_REQUEST_QUEUE_CAPACITY, ParamChanged, ParamListRequested, ParamReadRequested,
        ParamSetRequested,
    },
    params::{PARAM_DEFINITIONS, PARAMS_COUNT, ParamId, ParamValue},
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

        let def = &PARAM_DEFINITIONS[id as usize];
        if !same_param_type(def.default, req.value) {
            continue;
        }

        let old = ctx.params.get(id);
        if old == req.value {
            if is_mixer_choice_param(id) {
                crate::mixer::matrix::sync_reflected_mixer_params(ctx.params.raw_mut(), id);
                emit_reflected_mixer_param_responses(&mut ctx.responses, &ctx.params, id);
                emit_param_value_response(&mut ctx.responses, ctx.params.get(id), id);
            }
            continue;
        }

        ctx.params.set(id, req.value);
        crate::mixer::matrix::sync_reflected_mixer_params(ctx.params.raw_mut(), id);
        let new = ctx.params.get(id);

        let changed = ParamChanged {
            id,
            old,
            new,
            param_id_bytes: req.param_id_bytes,
        };
        ctx.changes.emit_or_log(changed, "param changed event");

        emit_reflected_mixer_param_responses(&mut ctx.responses, &ctx.params, id);
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

fn is_mixer_choice_param(id: ParamId) -> bool {
    matches!(
        id,
        ParamId::PARAM_PRIMARY_MIXER | ParamId::PARAM_SECONDARY_MIXER
    )
}

fn emit_param_value_response(
    responses: &mut EventEmitPort<'_, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    value: ParamValue,
    id: ParamId,
) {
    let def = &PARAM_DEFINITIONS[id as usize];
    let response = ParamValueMsg {
        param_id: str_to_fixed_bytes(def.name),
        param_value: value,
        param_count: PARAMS_COUNT as u16,
        param_index: id as u16,
    };
    responses.emit_or_log(CommResponse::ParamValue(response), "param value response");
}

fn emit_reflected_mixer_param_responses(
    responses: &mut EventEmitPort<'_, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    params: &ParamsWritePort<'_>,
    changed: ParamId,
) {
    match changed {
        ParamId::PARAM_PRIMARY_MIXER => {
            emit_param_range(
                responses,
                params,
                ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0,
                NUM_MIXER_OUTPUT_PARAMS,
            );
            emit_param_range(
                responses,
                params,
                ParamId::PARAM_PRIMARY_MIXER_PWM_RATE_0,
                NUM_MIXER_OUTPUT_PARAMS,
            );
            emit_param_range(
                responses,
                params,
                ParamId::PARAM_PRIMARY_MIXER_0_0,
                NUM_MIXER_MATRIX_PARAMS,
            );
        }
        ParamId::PARAM_SECONDARY_MIXER => {
            emit_param_range(
                responses,
                params,
                ParamId::PARAM_SECONDARY_MIXER_0_0,
                NUM_MIXER_MATRIX_PARAMS,
            );
        }
        _ => {}
    }
}

const NUM_MIXER_OUTPUT_PARAMS: usize = 10;
const NUM_MIXER_MATRIX_PARAMS: usize = 100;

fn emit_param_range(
    responses: &mut EventEmitPort<'_, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
    params: &ParamsWritePort<'_>,
    first_id: ParamId,
    len: usize,
) {
    let first_index = first_id as usize;
    for offset in 0..len {
        let Some(id) = ParamId::from_index(first_index + offset) else {
            return;
        };
        emit_param_value_response(responses, params.get(id), id);
    }
}

fn same_param_type(lhs: ParamValue, rhs: ParamValue) -> bool {
    matches!(
        (lhs, rhs),
        (ParamValue::Float(_), ParamValue::Float(_))
            | (ParamValue::Int(_), ParamValue::Int(_))
            | (ParamValue::Uint(_), ParamValue::Uint(_))
            | (ParamValue::Bool(_), ParamValue::Bool(_))
    )
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
    fn apply_param_requests_ignores_wrong_type_and_unchanged_value() {
        let mut params = Params::new();
        let mut requests = EventQueue::<ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>::new();
        let mut changes = EventQueue::<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(ParamSetRequested {
            value: ParamValue::Float(42.0),
            param_id_bytes: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
        });
        let _ = requests.push(ParamSetRequested {
            value: ParamValue::Int(1),
            param_id_bytes: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
        });

        apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut params),
            requests: EventDrainPort::new(&mut requests),
            changes: EventEmitPort::new(&mut changes),
            responses: EventEmitPort::new(&mut responses),
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert!(changes.is_empty());
        assert!(responses.is_empty());
    }

    #[test]
    fn apply_param_requests_refreshes_mixer_reflection_for_unchanged_mixer_choice() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(10));
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0, ParamValue::Int(2));
        let mut requests = EventQueue::<ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>::new();
        let mut changes = EventQueue::<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(ParamSetRequested {
            value: ParamValue::Int(10),
            param_id_bytes: *b"PRIMARY_MIXER\0\0\0",
        });

        apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut params),
            requests: EventDrainPort::new(&mut requests),
            changes: EventEmitPort::new(&mut changes),
            responses: EventEmitPort::new(&mut responses),
        });

        assert_eq!(
            params.get_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0),
            ParamValue::Int(1)
        );
        assert!(changes.is_empty());
        assert!(!responses.is_empty());
    }

    #[test]
    fn apply_param_requests_emits_mixer_reflection_before_choice_ack() {
        let mut params = Params::new();
        let mut requests = EventQueue::<ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>::new();
        let mut changes = EventQueue::<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let _ = requests.push(ParamSetRequested {
            value: ParamValue::Int(10),
            param_id_bytes: *b"PRIMARY_MIXER\0\0\0",
        });

        apply_param_requests(ParamApplyCtx {
            params: ParamsWritePort::new(&mut params),
            requests: EventDrainPort::new(&mut requests),
            changes: EventEmitPort::new(&mut changes),
            responses: EventEmitPort::new(&mut responses),
        });

        match responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(
                    response.param_index,
                    ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0 as u16
                );
                assert_eq!(response.param_value, ParamValue::Int(1));
            }
            _ => panic!("expected reflected mixer param response"),
        }

        for _ in 1..(NUM_MIXER_OUTPUT_PARAMS * 2 + NUM_MIXER_MATRIX_PARAMS) {
            let _ = responses.pop().unwrap();
        }

        match responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_PRIMARY_MIXER as u16);
                assert_eq!(response.param_value, ParamValue::Int(10));
            }
            _ => panic!("expected primary mixer acknowledgement"),
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
