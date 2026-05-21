use crate::{
    command::CommandManager,
    events::{PARAM_CHANGED_QUEUE_CAPACITY, ParamChanged},
    params::ParamId,
    ports::{EventReadPort, ParamsReadPort},
    rc::Rc,
    state_machine::StateManager,
};

pub struct RcParamChangedCtx<'a> {
    pub rc: &'a mut Rc,
    pub params: ParamsReadPort<'a>,
    pub changes: EventReadPort<'a, ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
}

pub fn rc_on_param_changed(ctx: RcParamChangedCtx<'_>) {
    for change in ctx.changes.iter() {
        ctx.rc.param_change_callback(change.id, ctx.params.raw());
    }
}

pub struct CommandParamChangedCtx<'a> {
    pub command: &'a mut CommandManager,
    pub state: &'a mut StateManager,
    pub params: ParamsReadPort<'a>,
    pub changes: EventReadPort<'a, ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>,
}

pub fn command_on_param_changed(ctx: CommandParamChangedCtx<'_>) {
    for change in ctx.changes.iter() {
        match change.id {
            ParamId::PARAM_FAILSAFE_THROTTLE | ParamId::PARAM_FIXED_WING => {
                ctx.command
                    .update_failsafe_config(ctx.params.raw(), ctx.state);
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        command::CommandManager,
        events::EventQueue,
        params::{ParamValue, Params},
        state_machine::ErrorFlag,
    };

    #[test]
    fn command_reacts_only_to_failsafe_related_param_changes() {
        let mut command = CommandManager::new();
        let mut state = StateManager::new();
        let mut params = Params::new();
        let mut changes = EventQueue::<ParamChanged, PARAM_CHANGED_QUEUE_CAPACITY>::new();

        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(2.0));
        let _ = changes.push(ParamChanged {
            id: ParamId::PARAM_BAUD_RATE,
            old: ParamValue::Int(921600),
            new: ParamValue::Int(57600),
            param_id_bytes: [0; 16],
        });

        command_on_param_changed(CommandParamChangedCtx {
            command: &mut command,
            state: &mut state,
            params: ParamsReadPort::new(&params),
            changes: EventReadPort::new(&changes),
        });

        assert!(!state.get_errors().contains(ErrorFlag::INVALID_FAILSAFE));

        let _ = changes.push(ParamChanged {
            id: ParamId::PARAM_FAILSAFE_THROTTLE,
            old: ParamValue::Float(0.0),
            new: ParamValue::Float(2.0),
            param_id_bytes: [0; 16],
        });

        command_on_param_changed(CommandParamChangedCtx {
            command: &mut command,
            state: &mut state,
            params: ParamsReadPort::new(&params),
            changes: EventReadPort::new(&changes),
        });

        assert!(state.get_errors().contains(ErrorFlag::INVALID_FAILSAFE));
    }
}
