use crate::{
    comm_messages::messages::ParamValueMsg,
    events::{
        CommResponse, COMM_RESPONSE_QUEUE_CAPACITY, PARAM_CHANGED_QUEUE_CAPACITY,
        PARAM_SET_REQUEST_QUEUE_CAPACITY, ParamChanged, ParamSetRequested,
    },
    params2::PARAMS_COUNT,
    ports::{EventDrainPort, EventEmitPort, ParamsWritePort},
};

pub struct ParamApplyCtx<'a> {
    pub params: ParamsWritePort<'a>,
    pub requests: EventDrainPort<'a, ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>,
    pub changes: EventEmitPort<'a, ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
    pub responses: EventEmitPort<'a, CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>,
}

pub fn apply_param_requests(mut ctx: ParamApplyCtx<'_>) {
    while let Some(req) = ctx.requests.next() {
        let old = ctx.params.get(req.id);
        ctx.params.set(req.id, req.value);
        let new = ctx.params.get(req.id);

        let changed = ParamChanged {
            id: req.id,
            old,
            new,
            param_id_bytes: req.param_id_bytes,
        };
        let _ = ctx.changes.emit(changed);

        let response = ParamValueMsg {
            param_id: req.param_id_bytes,
            param_value: new,
            param_count: PARAMS_COUNT as u16,
            param_index: req.id as u16,
        };
        let _ = ctx.responses.emit(CommResponse::ParamValue(response));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        events::{EventQueue, ParamSetRequested},
        params2::{ParamId, ParamValue, Params},
        ports::{EventDrainPort, EventEmitPort, ParamsWritePort},
    };

    #[test]
    fn apply_param_requests_mutates_params_and_defers_ack() {
        let mut params = Params::new();
        let mut requests = EventQueue::<ParamSetRequested, PARAM_SET_REQUEST_QUEUE_CAPACITY>::new();
        let mut changes = EventQueue::<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>::new();
        let mut responses = EventQueue::<CommResponse, COMM_RESPONSE_QUEUE_CAPACITY>::new();

        let request = ParamSetRequested {
            id: ParamId::PARAM_SYSTEM_ID,
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
        }
    }
}
