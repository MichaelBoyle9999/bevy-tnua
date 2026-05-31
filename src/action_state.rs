use bevy_tnua_physics_integration_layer::data_for_backends::TnuaMotor;
use serde::{Deserialize, Serialize};

use crate::{TnuaAction, TnuaActionContext, TnuaBasis};
use crate::{TnuaActionLifecycleDirective, TnuaActionLifecycleStatus, TnuaBasisContext};

/// The full state of a single [`TnuaAction`].
///
/// These are used in the variants of the [action state enum](crate::TnuaScheme::ActionState),
/// created automatically by [the `TnuaScheme` derive](bevy_tnua_macros::TnuaScheme).
#[derive(Serialize, Deserialize)]
pub struct TnuaActionState<A: TnuaAction<B>, B: TnuaBasis> {
    /// The data that the user control system feeds to the
    /// [`TnuaController`](crate::TnuaController).
    pub input: A,
    /// The action configuration, retrieved from an asset.
    pub config: A::Config,
    /// Initiated when the action starts, and gets updated by the action itself.
    pub memory: A::Memory,
}

impl<A: TnuaAction<B>, B: TnuaBasis> TnuaActionState<A, B> {
    pub fn new(input: A, config: &A::Config) -> Self {
        Self {
            input,
            config: config.clone(),
            memory: Default::default(),
        }
    }
}

impl<A: TnuaAction<B> + Clone, B: TnuaBasis> Clone for TnuaActionState<A, B>
where
    A::Config: Clone,
    A::Memory: Clone,
{
    fn clone(&self) -> Self {
        Self {
            input: self.input.clone(),
            config: self.config.clone(),
            memory: self.memory.clone(),
        }
    }
}

impl<A: TnuaAction<B> + PartialEq, B: TnuaBasis> PartialEq for TnuaActionState<A, B>
where
    A::Config: PartialEq,
    A::Memory: PartialEq,
{
    fn eq(&self, other: &Self) -> bool {
        self.input == other.input && self.config == other.config && self.memory == other.memory
    }
}

impl<A: TnuaAction<B> + std::fmt::Debug, B: TnuaBasis> std::fmt::Debug for TnuaActionState<A, B>
where
    A::Config: std::fmt::Debug,
    A::Memory: std::fmt::Debug,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TnuaActionState")
            .field("input", &self.input)
            .field("config", &self.config)
            .field("memory", &self.memory)
            .finish()
    }
}

#[doc(hidden)]
pub trait TnuaActionStateInterface<B: TnuaBasis> {
    fn apply(
        &mut self,
        sensors: &B::Sensors<'_>,
        ctx: TnuaActionContext<B>,
        lifecycle_status: TnuaActionLifecycleStatus,
        motor: &mut TnuaMotor,
    ) -> TnuaActionLifecycleDirective;

    fn influence_basis(
        &self,
        ctx: TnuaBasisContext,
        basis_input: &B,
        basis_config: &B::Config,
        basis_memory: &mut B::Memory,
    );
}

impl<A: TnuaAction<B>, B: TnuaBasis> TnuaActionStateInterface<B> for TnuaActionState<A, B> {
    fn apply(
        &mut self,
        sensors: &B::Sensors<'_>,
        ctx: TnuaActionContext<B>,
        lifecycle_status: TnuaActionLifecycleStatus,
        motor: &mut TnuaMotor,
    ) -> TnuaActionLifecycleDirective {
        self.input.apply(
            &self.config,
            &mut self.memory,
            sensors,
            ctx,
            lifecycle_status,
            motor,
        )
    }

    fn influence_basis(
        &self,
        ctx: TnuaBasisContext,
        basis_input: &B,
        basis_config: &<B as TnuaBasis>::Config,
        basis_memory: &mut <B as TnuaBasis>::Memory,
    ) {
        self.input.influence_basis(
            &self.config,
            &self.memory,
            ctx,
            basis_input,
            basis_config,
            basis_memory,
        );
    }
}
