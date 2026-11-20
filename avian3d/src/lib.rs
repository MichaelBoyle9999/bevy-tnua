//! # avian3d Integration for bevy-tnua
//!
//! In addition to the instruction in bevy-tnua's documentation:
//!
//! * Add [`TnuaAvian3dPlugin`] to the Bevy app.
//! * Optionally: Add [`TnuaAvian3dSensorShape`] to either entity of the character controller by
//!   Tnua or to the to the sensor entities. If exists, the shape on the sensor entity overrides
//!   the one on the character entity for that specific sensor.
mod spatial_ext;

use avian3d::{prelude::*, schedule::PhysicsStepSystems};
use bevy::ecs::schedule::{InternedScheduleLabel, ScheduleLabel};
use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy_tnua_physics_integration_layer::math::AsF32;
use bevy_tnua_physics_integration_layer::math::Float;
use bevy_tnua_physics_integration_layer::math::Vector3;
use bevy_tnua_physics_integration_layer::math::{AdjustPrecision, Quaternion};
pub use spatial_ext::TnuaSpatialExtAvian3d;

use bevy_tnua_physics_integration_layer::TnuaPipelineSystems;
use bevy_tnua_physics_integration_layer::TnuaSystems;
use bevy_tnua_physics_integration_layer::data_for_backends::TnuaGravity;
use bevy_tnua_physics_integration_layer::data_for_backends::TnuaToggle;
use bevy_tnua_physics_integration_layer::data_for_backends::{TnuaGhostPlatform, TnuaNotPlatform};
use bevy_tnua_physics_integration_layer::data_for_backends::{TnuaGhostSensor, TnuaSensorOf};
use bevy_tnua_physics_integration_layer::data_for_backends::{
    TnuaMotor, TnuaProximitySensor, TnuaProximitySensorOutput, TnuaRigidBodyTracker,
};
use bevy_tnua_physics_integration_layer::obstacle_radar::TnuaObstacleRadar;

pub mod prelude {
    pub use crate::{
        TnuaAvian3dPlugin, TnuaAvian3dSensorCasts, TnuaAvian3dSensorShape, TnuaSpatialExtAvian3d,
    };
}

/// Add this plugin to use avian3d as a physics backend.
///
/// This plugin should be used in addition to `TnuaControllerPlugin`.
/// Note that you should make sure both of these plugins use the same schedule.
/// This should usually be `PhysicsSchedule`, which by default is `FixedUpdate`.
///
/// # Example
///
/// ```ignore
/// App::new()
///     .add_plugins((
///         DefaultPlugins,
///         PhysicsPlugins::default(),
///         TnuaControllerPlugin::new(PhysicsSchedule),
///         TnuaAvian3dPlugin::new(PhysicsSchedule),
///     ));
/// ```
pub struct TnuaAvian3dPlugin {
    schedule: InternedScheduleLabel,
}

impl TnuaAvian3dPlugin {
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Plugin for TnuaAvian3dPlugin {
    fn build(&self, app: &mut App) {
        app.configure_sets(
            self.schedule,
            TnuaSystems
                // Need to run _before_ `First`, not after it. The documentation is misleading. See
                // https://github.com/Jondolf/avian/issues/675
                .before(PhysicsStepSystems::First)
                .run_if(|physics_time: Res<Time<Physics>>| !physics_time.is_paused()),
        );
        app.add_systems(
            self.schedule,
            (
                update_rigid_body_trackers_system,
                update_proximity_sensors_system,
                update_obstacle_radars_system
                    // Both use SpatialQuery, which uses a ResMut
                    .ambiguous_with(update_proximity_sensors_system),
            )
                .in_set(TnuaPipelineSystems::Sensors),
        );
        app.add_systems(
            self.schedule,
            apply_motors_system.in_set(TnuaPipelineSystems::Motors),
        );
        app.register_required_components_with::<TnuaGravity, GravityScale>(|| GravityScale(0.0));
    }
}

/// Add this component to make [`TnuaProximitySensor`] cast a shape instead of a ray.
#[derive(Component)]
pub struct TnuaAvian3dSensorShape(pub Collider);

#[allow(clippy::type_complexity)]
fn update_rigid_body_trackers_system(
    gravity: Res<Gravity>,
    mut query: Query<(
        &Position,
        &Rotation,
        &LinearVelocity,
        &AngularVelocity,
        &mut TnuaRigidBodyTracker,
        Option<&TnuaToggle>,
        Option<&TnuaGravity>,
    )>,
) {
    for (
        position,
        rotation,
        linaer_velocity,
        angular_velocity,
        mut tracker,
        tnua_toggle,
        tnua_gravity,
    ) in query.iter_mut()
    {
        match tnua_toggle.copied().unwrap_or_default() {
            TnuaToggle::Disabled => continue,
            TnuaToggle::SenseOnly => {}
            TnuaToggle::Enabled => {}
        }
        *tracker = TnuaRigidBodyTracker {
            translation: position.adjust_precision(),
            rotation: rotation.adjust_precision(),
            velocity: linaer_velocity.0.adjust_precision(),
            angvel: angular_velocity.0.adjust_precision(),
            gravity: tnua_gravity.map(|g| g.0).unwrap_or(gravity.0),
        };
    }
}

/// How a hit along a proximity sensor's cast counts.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SensedHitKind {
    Ground,
    Ghost,
}

#[allow(clippy::type_complexity)]
struct SensedHit<'a> {
    kind: SensedHitKind,
    kinematic_data: Option<(
        &'a Position,
        &'a LinearVelocity,
        &'a AngularVelocity,
        Option<&'a RigidBody>,
    )>,
}

/// The ground a [`TnuaProximitySensor`] would count along a ray, by the rule its casts sense
/// with.
///
/// Use [`ground_proximity_along_ray`](Self::ground_proximity_along_ray) to find it, such as
/// under a sensor's cast origin when it casts a shape.
#[derive(SystemParam)]
pub struct TnuaAvian3dSensorCasts<'w, 's> {
    casts: SensorCasts<'w, 's>,
    sensor_query: Query<'w, 's, (&'static TnuaProximitySensor, &'static TnuaSensorOf)>,
}

impl TnuaAvian3dSensorCasts<'_, '_> {
    /// The distance, along a ray from `sensor`'s cast origin in its cast direction and within its
    /// cast range, to the nearest hit `sensor` would count as ground for the entity it senses
    /// for. `None` when no hit counts, or `sensor` is no positioned entity's sensor.
    pub fn ground_proximity_along_ray(&self, sensor: Entity) -> Option<Float> {
        let (sensor, &TnuaSensorOf(owner)) = self.sensor_query.get(sensor).ok()?;
        self.casts.ground_proximity_along_ray(owner, sensor)
    }
}

/// The casts [`TnuaProximitySensor`]s sense their surroundings with, and the rule that decides
/// which hits along them count as ground.
#[derive(SystemParam)]
struct SensorCasts<'w, 's> {
    spatial_query: SpatialQuery<'w, 's>,
    #[allow(clippy::type_complexity)]
    owner_query: Query<
        'w,
        's,
        (
            &'static Position,
            &'static Rotation,
            Option<&'static Collider>,
            Option<&'static TnuaAvian3dSensorShape>,
            Option<&'static TnuaToggle>,
        ),
    >,
    collision_layers_query: Query<'w, 's, &'static CollisionLayers>,
    #[allow(clippy::type_complexity)]
    other_object_query: Query<
        'w,
        's,
        (
            Option<(
                &'static Position,
                &'static LinearVelocity,
                &'static AngularVelocity,
                Option<&'static RigidBody>,
            )>,
            Option<&'static CollisionLayers>,
            Option<&'static ColliderOf>,
            Has<TnuaGhostPlatform>,
            Has<Sensor>,
            Has<TnuaNotPlatform>,
        ),
    >,
}

impl SensorCasts<'_, '_> {
    fn ground_proximity_along_ray(
        &self,
        owner: Entity,
        sensor: &TnuaProximitySensor,
    ) -> Option<Float> {
        let (position, rotation, collider, _, _) = self.owner_query.get(owner).ok()?;
        let cast_origin = cast_origin(position, rotation, collider, sensor);
        let owner_layers = self.owner_layers(owner);
        let mut nearest: Option<Float> = None;
        self.ray_hits(owner, cast_origin, sensor, |ray_hit_data| {
            if self
                .classify_hit(owner, owner_layers, ray_hit_data.entity)
                .is_some_and(|hit| hit.kind == SensedHitKind::Ground)
            {
                nearest = Some(nearest.map_or(ray_hit_data.distance, |nearest| {
                    nearest.min(ray_hit_data.distance)
                }));
            }
        });
        nearest
    }

    fn owner_layers(&self, owner: Entity) -> CollisionLayers {
        self.collision_layers_query
            .get(owner)
            .ok()
            .copied()
            .unwrap_or_default()
    }

    fn ray_hits(
        &self,
        owner: Entity,
        cast_origin: Vector3,
        sensor: &TnuaProximitySensor,
        mut callback: impl FnMut(RayHitData),
    ) {
        self.spatial_query.ray_hits_callback(
            cast_origin,
            sensor.cast_direction,
            sensor.cast_range,
            true,
            &query_filter(owner),
            |ray_hit_data| {
                callback(ray_hit_data);
                true
            },
        );
    }

    fn classify_hit(
        &self,
        owner: Entity,
        owner_layers: CollisionLayers,
        entity: Entity,
    ) -> Option<SensedHit<'_>> {
        let Ok((
            mut kinematic_data,
            mut collision_layers,
            collider_of,
            mut is_ghost,
            mut is_sensor,
            mut is_not_platform,
        )) = self.other_object_query.get(entity)
        else {
            return None;
        };

        if let Some(collider_of) = collider_of {
            let parent_entity = collider_of.body;

            // Collider is child of our rigid body. ignore.
            if parent_entity == owner {
                return None;
            }

            if let Ok((
                parent_kinematic_data,
                parent_collision_layers,
                _,
                parent_is_ghost,
                parent_is_sensor,
                parent_is_not_platform,
            )) = self.other_object_query.get(parent_entity)
            {
                if kinematic_data.is_none() {
                    kinematic_data = parent_kinematic_data;
                }
                if collision_layers.is_none() {
                    collision_layers = parent_collision_layers;
                }
                is_ghost = is_ghost || parent_is_ghost;
                is_sensor = is_sensor || parent_is_sensor;
                is_not_platform = is_not_platform || parent_is_not_platform;
            }
        }

        if is_not_platform {
            return None;
        }

        let kind = if is_ghost {
            SensedHitKind::Ghost
        } else if !is_sensor
            && owner_layers.interacts_with(collision_layers.copied().unwrap_or_default())
        {
            SensedHitKind::Ground
        } else {
            return None;
        };
        Some(SensedHit {
            kind,
            kinematic_data,
        })
    }
}

fn query_filter(owner: Entity) -> SpatialQueryFilter {
    SpatialQueryFilter::from_excluded_entities([owner])
}

fn cast_origin(
    position: &Position,
    rotation: &Rotation,
    collider: Option<&Collider>,
    sensor: &TnuaProximitySensor,
) -> Vector3 {
    let transform = Transform {
        translation: position.0.f32(),
        rotation: rotation.0.f32(),
        scale: collider
            .map(|collider| collider.scale().f32())
            .unwrap_or(Vec3::ONE),
    };

    // TODO: is there any point in doing these transformations as f64 when that feature
    // flag is active?
    transform
        .transform_point(sensor.cast_origin.f32())
        .adjust_precision()
}

fn update_proximity_sensors_system(
    casts: SensorCasts,
    mut sensor_query: Query<(
        &mut TnuaProximitySensor,
        &TnuaSensorOf,
        Option<&TnuaAvian3dSensorShape>,
        Option<&mut TnuaGhostSensor>,
    )>,
) {
    sensor_query.par_iter_mut().for_each(
        |(mut sensor, &TnuaSensorOf(owner_entity), shape, mut ghost_sensor)| {
            let Ok((position, rotation, collider, owner_shape, tnua_toggle)) =
                casts.owner_query.get(owner_entity)
            else {
                return;
            };
            let shape = shape.or(owner_shape);
            match tnua_toggle.copied().unwrap_or_default() {
                TnuaToggle::Disabled => return,
                TnuaToggle::SenseOnly => {}
                TnuaToggle::Enabled => {}
            }

            let cast_origin = cast_origin(position, rotation, collider, &*sensor);
            let cast_direction = sensor.cast_direction;

            struct CastResult {
                entity: Entity,
                proximity: Float,
                intersection_point: Vector3,
                normal: Dir3,
            }

            let owner_layers = casts.owner_layers(owner_entity);

            let mut final_sensor_output = None;
            if let Some(ghost_sensor) = ghost_sensor.as_mut() {
                ghost_sensor.0.clear();
            }
            let mut apply_cast = |cast_result: CastResult| {
                let CastResult {
                    entity,
                    proximity,
                    intersection_point,
                    normal,
                } = cast_result;

                let Some(SensedHit {
                    kind,
                    kinematic_data,
                }) = casts.classify_hit(owner_entity, owner_layers, entity)
                else {
                    return;
                };

                let entity_linvel;
                let entity_angvel;
                if let Some((
                    entity_position,
                    entity_linear_velocity,
                    entity_angular_velocity,
                    rigid_body,
                )) = kinematic_data
                {
                    if rigid_body == Some(&RigidBody::Static) {
                        entity_angvel = Vector3::ZERO;
                        entity_linvel = Vector3::ZERO;
                    } else {
                        entity_angvel = entity_angular_velocity.0.adjust_precision();
                        entity_linvel = entity_linear_velocity.0.adjust_precision()
                            + if 0.0 < entity_angvel.length_squared() {
                                let relative_point =
                                    intersection_point - entity_position.adjust_precision();
                                // NOTE: no need to project relative_point on the
                                // rotation plane, it will not affect the cross
                                // product.
                                entity_angvel.cross(relative_point)
                            } else {
                                Vector3::ZERO
                            };
                    }
                } else {
                    entity_angvel = Vector3::ZERO;
                    entity_linvel = Vector3::ZERO;
                }
                let sensor_output = TnuaProximitySensorOutput {
                    entity,
                    proximity,
                    normal,
                    entity_linvel,
                    entity_angvel,
                };

                match kind {
                    SensedHitKind::Ghost => {
                        if let Some(ghost_sensor) = ghost_sensor.as_mut() {
                            ghost_sensor.0.push(sensor_output);
                        }
                    }
                    SensedHitKind::Ground => {
                        if final_sensor_output.as_ref().is_none_or(
                            |nearest: &TnuaProximitySensorOutput| {
                                sensor_output.proximity < nearest.proximity
                            },
                        ) {
                            final_sensor_output = Some(sensor_output);
                        }
                    }
                }
            };

            if let Some(TnuaAvian3dSensorShape(shape)) = shape {
                // TODO: can I bake `owner_rotation` into
                // `sensor.cast_shape_rotation`?
                let owner_rotation = Quaternion::from_axis_angle(
                    cast_direction.adjust_precision(),
                    rotation
                        .to_scaled_axis()
                        .dot(cast_direction.adjust_precision()),
                );
                casts.spatial_query.shape_hits_callback(
                    shape,
                    cast_origin,
                    owner_rotation.mul_quat(sensor.cast_shape_rotation.adjust_precision()),
                    cast_direction,
                    &ShapeCastConfig {
                        max_distance: sensor.cast_range,
                        ignore_origin_penetration: true,
                        ..default()
                    },
                    &query_filter(owner_entity),
                    |shape_hit_data| {
                        apply_cast(CastResult {
                            entity: shape_hit_data.entity,
                            proximity: shape_hit_data.distance,
                            intersection_point: shape_hit_data.point1,
                            normal: Dir3::new(shape_hit_data.normal1.f32())
                                .unwrap_or_else(|_| -cast_direction),
                        });
                        true
                    },
                );
            } else {
                casts.ray_hits(owner_entity, cast_origin, &*sensor, |ray_hit_data| {
                    apply_cast(CastResult {
                        entity: ray_hit_data.entity,
                        proximity: ray_hit_data.distance,
                        intersection_point: cast_origin
                            + ray_hit_data.distance * cast_direction.adjust_precision(),
                        normal: Dir3::new(ray_hit_data.normal.f32())
                            .unwrap_or_else(|_| -cast_direction),
                    });
                });
            }
            if let Some(ghost_sensor) = ghost_sensor.as_mut() {
                if let Some(nearest) = final_sensor_output.as_ref() {
                    ghost_sensor
                        .0
                        .retain(|ghost_hit| ghost_hit.proximity < nearest.proximity);
                }
                ghost_sensor
                    .0
                    .sort_by(|a, b| a.proximity.total_cmp(&b.proximity));
            }
            sensor.output = final_sensor_output;
        },
    );
}

fn update_obstacle_radars_system(
    spatial_query: SpatialQuery,
    gravity: Res<Gravity>,
    mut radars_query: Query<(Entity, &mut TnuaObstacleRadar, &Position)>,
) {
    if radars_query.is_empty() {
        return;
    }
    for (radar_owner_entity, mut radar, radar_position) in radars_query.iter_mut() {
        radar.pre_marking_update(
            radar_owner_entity,
            radar_position.0,
            Dir3::new(gravity.0.f32()).unwrap_or(Dir3::Y),
        );
        spatial_query.shape_intersections_callback(
            &Collider::cylinder(radar.radius, radar.height),
            radar_position.0,
            Default::default(),
            &SpatialQueryFilter::DEFAULT,
            |obstacle_entity| {
                if radar_owner_entity == obstacle_entity {
                    return true;
                }
                radar.mark_seen(obstacle_entity);
                true
            },
        );
    }
}

#[allow(clippy::type_complexity)]
fn apply_motors_system(
    mut query: Query<(
        &TnuaMotor,
        &ComputedMass,
        &ComputedAngularInertia,
        Forces,
        Option<&TnuaToggle>,
        Option<&TnuaGravity>,
    )>,
) {
    for (motor, mass, inertia, mut forces, tnua_toggle, tnua_gravity) in query.iter_mut() {
        match tnua_toggle.copied().unwrap_or_default() {
            TnuaToggle::Disabled | TnuaToggle::SenseOnly => {
                // *external_force = Default::default();
                return;
            }
            TnuaToggle::Enabled => {}
        }
        if motor.lin.boost.is_finite() {
            forces.apply_linear_impulse(motor.lin.boost * mass.value());
        }
        if motor.lin.acceleration.is_finite() {
            forces.apply_linear_acceleration(motor.lin.acceleration);
        }
        if motor.ang.boost.is_finite() {
            forces.apply_angular_impulse(inertia.value() * motor.ang.boost);
        }
        if motor.ang.acceleration.is_finite() {
            // NOTE: I did not actually verify that this is correct. Nothing uses angular
            // acceleration yet - only angular impulses.
            forces.apply_angular_acceleration(motor.ang.acceleration);
        }
        if let Some(gravity) = tnua_gravity {
            forces.apply_force(gravity.0 * mass.value());
        }
    }
}
