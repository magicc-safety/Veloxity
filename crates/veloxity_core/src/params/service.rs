use crate::{
    comm::messages::{enums::ParamIdentifier, messages::ParamValueMsg},
    comm::str_to_fixed_bytes,
    events::{
        CommEventQueues, CommResponse, EventQueue, PARAM_CHANGED_QUEUE_CAPACITY, ParamChanged,
        ParamEventQueues,
    },
    params::{PARAM_DEFINITIONS, PARAMS_COUNT, ParamId, ParamValue, Params},
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

pub struct ParamServiceCtx<'a> {
    pub params: &'a mut Params,
    pub state: &'a mut ParamListState,
    pub events: &'a mut ParamEventQueues,
    pub comm_events: &'a mut CommEventQueues,
}

pub fn service_param_events(ctx: &mut ParamServiceCtx<'_>) {
    service_param_read_requests(ctx);
    service_param_list_requests(ctx);
    apply_param_requests(ctx);
}

pub fn apply_param_requests(ctx: &mut ParamServiceCtx<'_>) {
    while let Some(req) = ctx.events.set_requests.pop() {
        let Some(id) = param_id_from_name_bytes(req.param_id_bytes) else {
            continue;
        };

        let def = &PARAM_DEFINITIONS[id as usize];
        if !same_param_type(def.default, req.value) {
            continue;
        }

        let old = ctx.params.get_by_id(id);
        if old == req.value {
            if is_mixer_choice_param(id) {
                crate::mixer::matrix::sync_reflected_mixer_params(ctx.params, id);
                emit_reflected_mixer_param_responses(ctx.comm_events, ctx.params, id);
                emit_param_value_response(ctx.comm_events, ctx.params.get_by_id(id), id);
            }
            continue;
        }

        set_param_and_emit_change(ctx.params, &mut ctx.events.changes, id, req.value);
        crate::mixer::matrix::sync_reflected_mixer_params(ctx.params, id);
        let new = ctx.params.get_by_id(id);

        emit_reflected_mixer_param_responses(ctx.comm_events, ctx.params, id);
        let response = ParamValueMsg {
            param_id: req.param_id_bytes,
            param_value: new,
            param_count: PARAMS_COUNT as u16,
            param_index: id as u16,
        };
        ctx.comm_events
            .responses
            .push_or_log(CommResponse::ParamValue(response), "param set response");
    }
}

pub fn set_param_and_emit_change(
    params: &mut Params,
    changes: &mut EventQueue<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
    id: ParamId,
    value: ParamValue,
) {
    let old = params.get_by_id(id);
    if old == value {
        return;
    }
    params.set_by_id(id, value);
    let new = params.get_by_id(id);
    changes.push_or_log(
        ParamChanged {
            id,
            old,
            new,
            param_id_bytes: str_to_fixed_bytes(PARAM_DEFINITIONS[id as usize].name),
        },
        "param changed event",
    );
}

pub fn mark_all_params_changed(events: &mut ParamEventQueues) {
    events.full_refresh = true;
}

fn is_mixer_choice_param(id: ParamId) -> bool {
    matches!(
        id,
        ParamId::PARAM_PRIMARY_MIXER | ParamId::PARAM_SECONDARY_MIXER
    )
}

fn emit_param_value_response(comm_events: &mut CommEventQueues, value: ParamValue, id: ParamId) {
    let def = &PARAM_DEFINITIONS[id as usize];
    let response = ParamValueMsg {
        param_id: str_to_fixed_bytes(def.name),
        param_value: value,
        param_count: PARAMS_COUNT as u16,
        param_index: id as u16,
    };
    comm_events
        .responses
        .push_or_log(CommResponse::ParamValue(response), "param value response");
}

fn emit_reflected_mixer_param_responses(
    comm_events: &mut CommEventQueues,
    params: &Params,
    changed: ParamId,
) {
    match changed {
        ParamId::PARAM_PRIMARY_MIXER => {
            emit_param_range(
                comm_events,
                params,
                ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0,
                NUM_MIXER_OUTPUT_PARAMS,
            );
            emit_param_range(
                comm_events,
                params,
                ParamId::PARAM_PRIMARY_MIXER_PWM_RATE_0,
                NUM_MIXER_OUTPUT_PARAMS,
            );
            emit_param_range(
                comm_events,
                params,
                ParamId::PARAM_PRIMARY_MIXER_0_0,
                NUM_MIXER_MATRIX_PARAMS,
            );
        }
        ParamId::PARAM_SECONDARY_MIXER => {
            emit_param_range(
                comm_events,
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
    comm_events: &mut CommEventQueues,
    params: &Params,
    first_id: ParamId,
    len: usize,
) {
    let first_index = first_id as usize;
    for offset in 0..len {
        let Some(id) = ParamId::from_index(first_index + offset) else {
            return;
        };
        emit_param_value_response(comm_events, params.get_by_id(id), id);
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

pub fn service_param_read_requests(ctx: &mut ParamServiceCtx<'_>) {
    while let Some(req) = ctx.events.read_requests.pop() {
        let Some(id) = param_id_from_identifier(req.identifier) else {
            continue;
        };
        let def = &PARAM_DEFINITIONS[id as usize];
        let response = ParamValueMsg {
            param_id: str_to_fixed_bytes(def.name),
            param_value: ctx.params.get_by_id(id),
            param_count: PARAMS_COUNT as u16,
            param_index: id as u16,
        };
        ctx.comm_events
            .responses
            .push_or_log(CommResponse::ParamValue(response), "param read response");
    }
}

pub fn service_param_list_requests(ctx: &mut ParamServiceCtx<'_>) {
    while ctx.events.list_requests.pop().is_some() {
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
        param_value: ctx.params.get_by_id(def.id),
        param_count: PARAMS_COUNT as u16,
        param_index: def.id as u16,
    };
    ctx.comm_events
        .responses
        .push_or_log(CommResponse::ParamValue(response), "param list response");

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
        .find(|def| def.name == name || str_to_fixed_bytes(def.name) == bytes)
        .map(|def| def.id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        events::{
            CommEventQueues, ParamEventQueues, ParamListRequested, ParamReadRequested,
            ParamSetRequested,
        },
        params::{ParamId, ParamValue, Params},
    };

    fn test_ctx<'a>(
        params: &'a mut Params,
        state: &'a mut ParamListState,
        events: &'a mut ParamEventQueues,
        comm_events: &'a mut CommEventQueues,
    ) -> ParamServiceCtx<'a> {
        ParamServiceCtx {
            params,
            state,
            events,
            comm_events,
        }
    }

    #[test]
    fn apply_param_requests_mutates_params_and_defers_ack() {
        let mut params = Params::new();
        let mut events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut state = ParamListState::default();

        let request = ParamSetRequested {
            value: ParamValue::Int(42),
            param_id_bytes: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
        };
        let _ = events.set_requests.push(request);

        apply_param_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(42)
        );

        let change = events.changes.pop().unwrap();
        assert_eq!(change.id, ParamId::PARAM_SYSTEM_ID);
        assert_eq!(change.old, ParamValue::Int(1));
        assert_eq!(change.new, ParamValue::Int(42));

        match comm_events.responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_SYSTEM_ID as u16);
                assert_eq!(response.param_value, ParamValue::Int(42));
            }
            _ => panic!("expected param value response"),
        }
    }

    #[test]
    fn apply_param_requests_matches_fixed_width_long_param_names() {
        let mut params = Params::new();
        let mut events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut state = ParamListState::default();

        let request = ParamSetRequested {
            value: ParamValue::Int(0x0f),
            param_id_bytes: crate::comm::str_to_fixed_bytes("MOTOR_OUTPUT_MASK"),
        };
        let _ = events.set_requests.push(request);

        apply_param_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        assert_eq!(
            params.get_by_id(ParamId::PARAM_MOTOR_OUTPUT_MASK),
            ParamValue::Int(0x0f)
        );
    }

    #[test]
    fn apply_param_requests_ignores_wrong_type_and_unchanged_value() {
        let mut params = Params::new();
        let mut events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut state = ParamListState::default();

        let _ = events.set_requests.push(ParamSetRequested {
            value: ParamValue::Float(42.0),
            param_id_bytes: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
        });
        let _ = events.set_requests.push(ParamSetRequested {
            value: ParamValue::Int(1),
            param_id_bytes: *b"SYS_ID\0\0\0\0\0\0\0\0\0\0",
        });

        apply_param_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        assert_eq!(
            params.get_by_id(ParamId::PARAM_SYSTEM_ID),
            ParamValue::Int(1)
        );
        assert!(events.changes.is_empty());
        assert!(comm_events.responses.is_empty());
    }

    #[test]
    fn apply_param_requests_refreshes_mixer_reflection_for_unchanged_mixer_choice() {
        let mut params = Params::new();
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(10));
        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0, ParamValue::Int(2));
        let mut events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut state = ParamListState::default();

        let _ = events.set_requests.push(ParamSetRequested {
            value: ParamValue::Int(10),
            param_id_bytes: *b"PRIMARY_MIXER\0\0\0",
        });

        apply_param_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        assert_eq!(
            params.get_by_id(ParamId::PARAM_PRIMARY_MIXER_OUTPUT_0),
            ParamValue::Int(1)
        );
        assert!(events.changes.is_empty());
        assert!(!comm_events.responses.is_empty());
    }

    #[test]
    fn apply_param_requests_emits_mixer_reflection_before_choice_ack() {
        let mut params = Params::new();
        let mut events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut state = ParamListState::default();

        let _ = events.set_requests.push(ParamSetRequested {
            value: ParamValue::Int(10),
            param_id_bytes: *b"PRIMARY_MIXER\0\0\0",
        });

        apply_param_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        match comm_events.responses.pop().unwrap() {
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
            let _ = comm_events.responses.pop().unwrap();
        }

        match comm_events.responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_PRIMARY_MIXER as u16);
                assert_eq!(response.param_value, ParamValue::Int(10));
            }
            _ => panic!("expected primary mixer acknowledgement"),
        }
    }

    #[test]
    fn service_param_list_requests_streams_one_param_per_call() {
        let mut params = Params::new();
        let mut state = ParamListState::default();
        let mut events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();

        let _ = events.list_requests.push(ParamListRequested);

        service_param_list_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        match comm_events.responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_BAUD_RATE as u16);
                assert_eq!(response.param_value, ParamValue::Int(921600));
            }
            _ => panic!("expected param value response"),
        }
        assert!(state.is_active());

        service_param_list_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        match comm_events.responses.pop().unwrap() {
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
        let mut events = ParamEventQueues::default();
        let mut comm_events = CommEventQueues::default();
        let mut state = ParamListState::default();

        let _ = events.read_requests.push(ParamReadRequested {
            identifier: ParamIdentifier::INDEX(0),
        });
        let _ = events.read_requests.push(ParamReadRequested {
            identifier: ParamIdentifier::ID(*b"SYS_ID\0\0\0\0\0\0\0\0\0\0"),
        });

        service_param_read_requests(&mut test_ctx(
            &mut params,
            &mut state,
            &mut events,
            &mut comm_events,
        ));

        match comm_events.responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_BAUD_RATE as u16);
                assert_eq!(response.param_value, ParamValue::Int(921600));
            }
            _ => panic!("expected param value response"),
        }

        match comm_events.responses.pop().unwrap() {
            CommResponse::ParamValue(response) => {
                assert_eq!(response.param_index, ParamId::PARAM_SYSTEM_ID as u16);
                assert_eq!(response.param_value, ParamValue::Int(42));
            }
            _ => panic!("expected param value response"),
        }
    }
}
