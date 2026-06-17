use crate::{
    command::CommandManager,
    control::ControlPipelineResource,
    controller::Controller,
    estimator::Estimator,
    events::ParamEventQueues,
    math::FlightFloat,
    mixer::Mixer,
    params::{ParamId, Params},
    rc::Rc,
    state_machine::{ErrorFlag, Event, StateManager},
};

pub struct ParamReactionCtx<'a, E, C, M, R>
where
    E: Estimator<R>,
    C: Controller<R, State = E::State>,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    R: FlightFloat,
{
    pub events: &'a mut ParamEventQueues,
    pub params: &'a Params,
    pub rc: &'a mut Rc,
    pub command: &'a mut CommandManager,
    pub state: &'a mut StateManager,
    pub estimator: &'a mut E,
    pub controller: &'a mut C,
    pub mixer: &'a mut M,
    pub control_pipeline: &'a mut ControlPipelineResource<E::State, M::ActuatorCommands, R>,
}

pub fn apply_param_reactions<E, C, M, R>(ctx: &mut ParamReactionCtx<'_, E, C, M, R>)
where
    E: Estimator<R>,
    C: Controller<R, State = E::State>,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    R: FlightFloat,
{
    let full_refresh = ctx.events.full_refresh;
    let has_param_changes = ctx.events.changes.iter().next().is_some();
    if full_refresh {
        mixer_refresh_params(ctx);
        ctx.rc.init(ctx.params);
        ctx.command.update_failsafe_config(ctx.params, ctx.state);
    } else {
        mixer_on_param_changed(ctx);
        rc_on_param_changed(ctx);
        command_on_param_changed(ctx);
    }

    if full_refresh || has_param_changes {
        ctx.estimator.update_params(ctx.params);
        ctx.controller.update_gains(ctx.params);
    }

    ctx.events.changes.clear();
    ctx.events.full_refresh = false;
}

fn mixer_refresh_params<E, C, M, R>(ctx: &mut ParamReactionCtx<'_, E, C, M, R>)
where
    E: Estimator<R>,
    C: Controller<R, State = E::State>,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    R: FlightFloat,
{
    let Some(status) = ctx.mixer.refresh_params(ctx.params) else {
        return;
    };
    ctx.control_pipeline.invalidate_pwm_rates();
    match status {
        crate::mixer::MixerStatus::Healthy => ctx
            .state
            .update(Event::ERROR_CLEARED(ErrorFlag::INVALID_MIXER), ctx.params),
        crate::mixer::MixerStatus::InvalidMixer => ctx
            .state
            .update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER), ctx.params),
    }
}

fn mixer_on_param_changed<E, C, M, R>(ctx: &mut ParamReactionCtx<'_, E, C, M, R>)
where
    E: Estimator<R>,
    C: Controller<R, State = E::State>,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    R: FlightFloat,
{
    for change in ctx.events.changes.iter() {
        let Some(status) = ctx.mixer.on_param_changed(ctx.params, change.id) else {
            continue;
        };
        ctx.control_pipeline.invalidate_pwm_rates();
        match status {
            crate::mixer::MixerStatus::Healthy => ctx
                .state
                .update(Event::ERROR_CLEARED(ErrorFlag::INVALID_MIXER), ctx.params),
            crate::mixer::MixerStatus::InvalidMixer => ctx
                .state
                .update(Event::ERROR_OCCURRED(ErrorFlag::INVALID_MIXER), ctx.params),
        }
    }
}

pub fn rc_on_param_changed<E, C, M, R>(ctx: &mut ParamReactionCtx<'_, E, C, M, R>)
where
    E: Estimator<R>,
    C: Controller<R, State = E::State>,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    R: FlightFloat,
{
    for change in ctx.events.changes.iter() {
        ctx.rc.param_change_callback(change.id, ctx.params);
    }
}

pub fn command_on_param_changed<E, C, M, R>(ctx: &mut ParamReactionCtx<'_, E, C, M, R>)
where
    E: Estimator<R>,
    C: Controller<R, State = E::State>,
    M: Mixer<R, MixerInput = C::ControlOutput>,
    R: FlightFloat,
{
    for change in ctx.events.changes.iter() {
        match change.id {
            ParamId::PARAM_FAILSAFE_THROTTLE | ParamId::PARAM_FIXED_WING => {
                ctx.command.update_failsafe_config(ctx.params, ctx.state);
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
        control::ControlPipelineResource,
        events::{ParamChanged, ParamEventQueues},
        params::{ParamValue, Params},
        rc::Rc,
        state_machine::ErrorFlag,
        vehicle::quadrotor,
    };

    fn test_ctx<'a>(
        events: &'a mut ParamEventQueues,
        params: &'a Params,
        rc: &'a mut Rc,
        command: &'a mut CommandManager,
        state: &'a mut StateManager,
        estimator: &'a mut quadrotor::Estimator<f64>,
        controller: &'a mut quadrotor::Controller<f64>,
        mixer: &'a mut quadrotor::Mixer<f64>,
        control_pipeline: &'a mut ControlPipelineResource<
            <quadrotor::Estimator<f64> as Estimator<f64>>::State,
            <quadrotor::Mixer<f64> as Mixer<f64>>::ActuatorCommands,
            f64,
        >,
    ) -> ParamReactionCtx<
        'a,
        quadrotor::Estimator<f64>,
        quadrotor::Controller<f64>,
        quadrotor::Mixer<f64>,
        f64,
    > {
        ParamReactionCtx {
            events,
            params,
            rc,
            command,
            state,
            estimator,
            controller,
            mixer,
            control_pipeline,
        }
    }

    #[test]
    fn command_reacts_only_to_failsafe_related_param_changes() {
        let mut command = CommandManager::new();
        let mut state = StateManager::new();
        let mut params = Params::new();
        let mut events = ParamEventQueues::default();
        let mut rc = Rc::new();
        let mut estimator = quadrotor::Estimator::<f64>::default();
        let mut controller = quadrotor::Controller::<f64>::default();
        let mut mixer = quadrotor::mixer(&params);
        let mut control_pipeline = ControlPipelineResource::default();

        params.set_by_id(ParamId::PARAM_FAILSAFE_THROTTLE, ParamValue::Float(2.0));
        let _ = events.changes.push(ParamChanged {
            id: ParamId::PARAM_BAUD_RATE,
            old: ParamValue::Int(921600),
            new: ParamValue::Int(57600),
            param_id_bytes: [0; 16],
        });

        command_on_param_changed(&mut test_ctx(
            &mut events,
            &params,
            &mut rc,
            &mut command,
            &mut state,
            &mut estimator,
            &mut controller,
            &mut mixer,
            &mut control_pipeline,
        ));

        assert!(!state.get_errors().contains(ErrorFlag::INVALID_FAILSAFE));

        let _ = events.changes.push(ParamChanged {
            id: ParamId::PARAM_FAILSAFE_THROTTLE,
            old: ParamValue::Float(0.0),
            new: ParamValue::Float(2.0),
            param_id_bytes: [0; 16],
        });

        command_on_param_changed(&mut test_ctx(
            &mut events,
            &params,
            &mut rc,
            &mut command,
            &mut state,
            &mut estimator,
            &mut controller,
            &mut mixer,
            &mut control_pipeline,
        ));

        assert!(state.get_errors().contains(ErrorFlag::INVALID_FAILSAFE));
    }

    #[test]
    fn full_param_refresh_reacts_once_without_expanding_param_changes() {
        let mut command = CommandManager::new();
        let mut state = StateManager::new();
        let mut params = Params::new();
        let mut events = ParamEventQueues::default();
        let mut rc = Rc::new();
        let mut estimator = quadrotor::Estimator::<f64>::default();
        let mut controller = quadrotor::Controller::<f64>::default();
        let mut mixer = quadrotor::mixer(&params);
        let mut control_pipeline = ControlPipelineResource::default();

        params.set_by_id(ParamId::PARAM_PRIMARY_MIXER, ParamValue::Int(99));
        events.full_refresh = true;

        apply_param_reactions(&mut test_ctx(
            &mut events,
            &params,
            &mut rc,
            &mut command,
            &mut state,
            &mut estimator,
            &mut controller,
            &mut mixer,
            &mut control_pipeline,
        ));

        assert!(!events.full_refresh);
        assert!(events.changes.is_empty());
        assert!(state.get_errors().contains(ErrorFlag::INVALID_MIXER));
    }
}
