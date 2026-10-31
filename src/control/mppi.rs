//! Model-predictive path integral control.
//!
//! At each step the controller holds a nominal sequence of bicycle inputs
//! `(acceleration, steering rate)` over a horizon of `H` steps. It draws `K`
//! perturbations, rolls each one out with [`BicycleModel::step`](crate::motion::BicycleModel::step),
//! and replaces the nominal with the cost-weighted average of the perturbations:
//!
//! ```text
//! w_k = exp(-(S_k - S_min) / λ) / Σ_j exp(-(S_j - S_min) / λ)
//! u_t ← u_t + Σ_k w_k ε_{k,t}
//! ```
//!
//! `S_k` is the sum of the attached [`Critic`](crate::control::Critic) scores
//! and `λ` is the temperature. The first input is applied and the sequence
//! shifts forward by one step.
//!
//! Two perturbation models are available. [`NoiseModel::Independent`] draws
//! each step independently. [`NoiseModel::ActionDerivative`] draws white noise
//! on the control derivative and integrates it, which is the smooth sampling
//! scheme of arXiv:2112.09988: the injected sequence is temporally correlated
//! without a separate smoothing filter. The derivative standard deviation is
//! `σ / sqrt(H)`, so the random walk stays on the order of the configured `σ`.
//!
//! Rollouts are deterministic for a given seed. With the `rayon` feature,
//! [`MppiConfig::parallel`] evaluates rollout costs across a thread pool. The
//! noise is drawn first, so the parallel and sequential updates match.
//!
//! The hot loop reuses the noise, cost, and nominal buffers. A few thousand
//! samples of a horizon of a few dozen steps is a few million bicycle
//! integrations, which is the budget this layout is aimed at.

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::control::critics::Rollout;
use crate::control::{Critic, VehicleCommand};
use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Pose2d};
use crate::grid::GridMap;
use crate::motion::{BicycleInput, BicycleModel, BicycleState};

/// How control perturbations are drawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoiseModel {
    /// Independent Gaussian noise at each step.
    Independent,
    /// Integrated white noise on the control derivative (arXiv:2112.09988).
    ActionDerivative,
}

/// Sampling and timing parameters of [`Mppi`].
#[derive(Clone, Debug)]
pub struct MppiConfig {
    /// Number of control steps in the horizon.
    pub horizon: usize,
    /// Seconds per step.
    pub dt: f64,
    /// Number of noisy rollouts per update.
    pub samples: usize,
    /// Softmax temperature `λ`. Smaller values concentrate weight on the best sample.
    pub temperature: f64,
    /// Standard deviation of acceleration noise, in meters per second squared.
    pub accel_noise: f64,
    /// Standard deviation of steering-rate noise, in radians per second.
    pub steering_rate_noise: f64,
    /// Independent steps, or integrated action derivatives.
    pub noise: NoiseModel,
    /// Seed of the perturbation generator.
    pub seed: u64,
    /// Speed passed through to critics as [`Rollout::reference_speed`].
    pub reference_speed: f64,
    /// Evaluate rollouts in parallel when the `rayon` feature is enabled.
    pub parallel: bool,
}

impl Default for MppiConfig {
    fn default() -> Self {
        Self {
            horizon: 20,
            dt: 0.1,
            samples: 256,
            temperature: 1.0,
            accel_noise: 0.6,
            steering_rate_noise: 0.4,
            noise: NoiseModel::Independent,
            seed: 1,
            reference_speed: 1.0,
            parallel: false,
        }
    }
}

/// One MPPI update.
#[derive(Clone, Debug)]
pub struct MppiOutput {
    /// Input applied to the bicycle model for this step.
    pub input: BicycleInput,
    /// State after applying `input` for one `dt`.
    pub state: BicycleState,
    /// Speed and steering implied by `state`.
    pub command: VehicleCommand,
    /// Nominal predicted poses after the update, including the current pose.
    pub predicted: Vec<Pose2d>,
}

/// Sampling-based predictive controller.
pub struct Mppi {
    model: BicycleModel,
    config: MppiConfig,
    nominal: Vec<BicycleInput>,
    noise_accel: Vec<f64>,
    noise_steer: Vec<f64>,
    costs: Vec<f64>,
    weights: Vec<f64>,
    rng: StdRng,
    spare_gaussian: Option<f64>,
    critics: Vec<Box<dyn Critic>>,
}

impl Mppi {
    /// Controller with a zero nominal sequence and no critics yet.
    pub fn new(model: BicycleModel, config: MppiConfig) -> Result<Self, PathPlannerError> {
        validate(&config)?;
        let nominal = vec![BicycleInput::zero(); config.horizon];
        let n = config.samples * config.horizon;
        Ok(Self {
            model,
            nominal,
            noise_accel: vec![0.0; n],
            noise_steer: vec![0.0; n],
            costs: vec![0.0; config.samples],
            weights: vec![0.0; config.samples],
            rng: StdRng::seed_from_u64(config.seed),
            spare_gaussian: None,
            critics: Vec::new(),
            config,
        })
    }

    /// Attach a cost term. Terms are summed in the order they were added.
    pub fn add_critic(&mut self, critic: impl Critic + 'static) {
        self.critics.push(Box::new(critic));
    }

    /// Bicycle model used for rollouts.
    pub fn model(&self) -> &BicycleModel {
        &self.model
    }

    /// Sampling configuration.
    pub fn config(&self) -> &MppiConfig {
        &self.config
    }

    /// Draw perturbations, update the nominal sequence, and apply its first input.
    pub fn step(
        &mut self,
        state: BicycleState,
        map: &GridMap,
        footprint: &Footprint,
        path: &[Pose2d],
        goal: Pose2d,
    ) -> MppiOutput {
        self.draw_noise();
        self.score_rollouts(state, map, footprint, path, goal);
        self.incorporate_weights();
        let input = clamp_input(self.nominal[0], &self.model);
        let next = self.model.step(state, input, self.config.dt);
        let predicted = self.predict(state);
        self.shift_nominal();
        let curvature = self.model.curvature_from_steering(next.steering);
        MppiOutput {
            input,
            state: next,
            command: VehicleCommand {
                velocity: next.velocity,
                steering_angle: next.steering,
                curvature,
                angular_velocity: next.velocity * curvature,
            },
            predicted,
        }
    }

    fn draw_noise(&mut self) {
        let horizon = self.config.horizon;
        let samples = self.config.samples;
        let accel_std = self.config.accel_noise;
        let steer_std = self.config.steering_rate_noise;
        let derivative_scale = 1.0 / (horizon as f64).sqrt();
        for sample in 0..samples {
            let mut integrated_accel = 0.0;
            let mut integrated_steer = 0.0;
            for step in 0..horizon {
                let index = sample * horizon + step;
                match self.config.noise {
                    NoiseModel::Independent => {
                        self.noise_accel[index] = self.gaussian() * accel_std;
                        self.noise_steer[index] = self.gaussian() * steer_std;
                    }
                    NoiseModel::ActionDerivative => {
                        integrated_accel += self.gaussian() * accel_std * derivative_scale;
                        integrated_steer += self.gaussian() * steer_std * derivative_scale;
                        self.noise_accel[index] = integrated_accel;
                        self.noise_steer[index] = integrated_steer;
                    }
                }
            }
        }
    }

    fn score_rollouts(
        &mut self,
        state: BicycleState,
        map: &GridMap,
        footprint: &Footprint,
        path: &[Pose2d],
        goal: Pose2d,
    ) {
        let parallel = self.config.parallel && cfg!(feature = "rayon");
        if parallel {
            self.score_parallel(state, map, footprint, path, goal);
        } else {
            self.score_sequential(state, map, footprint, path, goal);
        }
    }

    fn score_sequential(
        &mut self,
        state: BicycleState,
        map: &GridMap,
        footprint: &Footprint,
        path: &[Pose2d],
        goal: Pose2d,
    ) {
        let mut states = vec![state; self.config.horizon + 1];
        let mut inputs = vec![BicycleInput::zero(); self.config.horizon];
        let mut saturations = vec![0.0; self.config.horizon];
        for sample in 0..self.config.samples {
            self.costs[sample] = rollout_cost(RolloutInputs {
                sample,
                state,
                model: &self.model,
                config: &self.config,
                nominal: &self.nominal,
                noise_accel: &self.noise_accel,
                noise_steer: &self.noise_steer,
                critics: &self.critics,
                map,
                footprint,
                path,
                goal,
                states: &mut states,
                inputs: &mut inputs,
                saturations: &mut saturations,
            });
        }
    }

    fn score_parallel(
        &mut self,
        state: BicycleState,
        map: &GridMap,
        footprint: &Footprint,
        path: &[Pose2d],
        goal: Pose2d,
    ) {
        #[cfg(feature = "rayon")]
        {
            use std::cell::RefCell;

            use rayon::prelude::*;
            let model = &self.model;
            let config = &self.config;
            let nominal = &self.nominal;
            let noise_accel = &self.noise_accel;
            let noise_steer = &self.noise_steer;
            let critics = &self.critics;
            self.costs.par_iter_mut().enumerate().for_each(|(sample, cost)| {
                thread_local! {
                    static STATES: RefCell<Vec<BicycleState>> = const { RefCell::new(Vec::new()) };
                    static INPUTS: RefCell<Vec<BicycleInput>> = const { RefCell::new(Vec::new()) };
                    static SAT: RefCell<Vec<f64>> = const { RefCell::new(Vec::new()) };
                }
                *cost = STATES.with(|states| {
                    INPUTS.with(|inputs| {
                        SAT.with(|sat| {
                            let mut states = states.borrow_mut();
                            let mut inputs = inputs.borrow_mut();
                            let mut sat = sat.borrow_mut();
                            states.resize(config.horizon + 1, state);
                            inputs.resize(config.horizon, BicycleInput::zero());
                            sat.resize(config.horizon, 0.0);
                            rollout_cost(RolloutInputs {
                                sample,
                                state,
                                model,
                                config,
                                nominal,
                                noise_accel,
                                noise_steer,
                                critics,
                                map,
                                footprint,
                                path,
                                goal,
                                states: &mut states,
                                inputs: &mut inputs,
                                saturations: &mut sat,
                            })
                        })
                    })
                });
            });
        }
        #[cfg(not(feature = "rayon"))]
        {
            let _ = (state, map, footprint, path, goal);
        }
    }

    fn incorporate_weights(&mut self) {
        let temperature = self.config.temperature.max(1e-6);
        let min_cost = self.costs.iter().copied().fold(f64::INFINITY, f64::min);
        if !min_cost.is_finite() {
            return;
        }
        let mut sum = 0.0;
        for (weight, cost) in self.weights.iter_mut().zip(self.costs.iter()) {
            let value = (-(cost - min_cost) / temperature).exp();
            *weight = if value.is_finite() { value } else { 0.0 };
            sum += *weight;
        }
        if sum <= 0.0 || !sum.is_finite() {
            return;
        }
        let horizon = self.config.horizon;
        for (sample, weight) in self.weights.iter().copied().enumerate() {
            let scale = weight / sum;
            if scale == 0.0 {
                continue;
            }
            for step in 0..horizon {
                let index = sample * horizon + step;
                self.nominal[step].acceleration += scale * self.noise_accel[index];
                self.nominal[step].steering_rate += scale * self.noise_steer[index];
            }
        }
        for input in &mut self.nominal {
            *input = clamp_input(*input, &self.model);
        }
    }

    fn shift_nominal(&mut self) {
        let horizon = self.nominal.len();
        if horizon <= 1 {
            return;
        }
        for step in 0..horizon - 1 {
            self.nominal[step] = self.nominal[step + 1];
        }
        self.nominal[horizon - 1] = self.nominal[horizon - 2];
    }

    fn predict(&self, state: BicycleState) -> Vec<Pose2d> {
        let mut poses = Vec::with_capacity(self.config.horizon + 1);
        let mut cursor = state;
        poses.push(cursor.pose);
        for input in &self.nominal {
            cursor = self.model.step(cursor, *input, self.config.dt);
            poses.push(cursor.pose);
        }
        poses
    }

    fn gaussian(&mut self) -> f64 {
        if let Some(spare) = self.spare_gaussian.take() {
            return spare;
        }
        let mut u1 = self.rng.random::<f64>();
        if u1 < 1e-12 {
            u1 = 1e-12;
        }
        let u2 = self.rng.random::<f64>();
        let magnitude = (-2.0 * u1.ln()).sqrt();
        let angle = std::f64::consts::TAU * u2;
        self.spare_gaussian = Some(magnitude * angle.sin());
        magnitude * angle.cos()
    }
}

struct RolloutInputs<'a> {
    sample: usize,
    state: BicycleState,
    model: &'a BicycleModel,
    config: &'a MppiConfig,
    nominal: &'a [BicycleInput],
    noise_accel: &'a [f64],
    noise_steer: &'a [f64],
    critics: &'a [Box<dyn Critic>],
    map: &'a GridMap,
    footprint: &'a Footprint,
    path: &'a [Pose2d],
    goal: Pose2d,
    states: &'a mut [BicycleState],
    inputs: &'a mut [BicycleInput],
    saturations: &'a mut [f64],
}

fn rollout_cost(job: RolloutInputs<'_>) -> f64 {
    let RolloutInputs {
        sample,
        state,
        model,
        config,
        nominal,
        noise_accel,
        noise_steer,
        critics,
        map,
        footprint,
        path,
        goal,
        states,
        inputs,
        saturations,
    } = job;
    states[0] = state;
    let mut cursor = state;
    for step in 0..config.horizon {
        let index = sample * config.horizon + step;
        let raw = BicycleInput {
            acceleration: nominal[step].acceleration + noise_accel[index],
            steering_rate: nominal[step].steering_rate + noise_steer[index],
        };
        let applied = clamp_input(raw, model);
        saturations[step] = (raw.acceleration - applied.acceleration).abs()
            + (raw.steering_rate - applied.steering_rate).abs();
        inputs[step] = applied;
        cursor = model.step(cursor, applied, config.dt);
        states[step + 1] = cursor;
    }
    let rollout = Rollout {
        states: &states[..=config.horizon],
        inputs: &inputs[..config.horizon],
        saturations: &saturations[..config.horizon],
        dt: config.dt,
        map,
        footprint,
        path,
        goal,
        reference_speed: config.reference_speed,
    };
    critics.iter().map(|critic| critic.evaluate(&rollout)).sum()
}

fn clamp_input(input: BicycleInput, model: &BicycleModel) -> BicycleInput {
    let params = model.params();
    BicycleInput {
        acceleration: finite(input.acceleration)
            .clamp(-params.max_acceleration, params.max_acceleration),
        steering_rate: finite(input.steering_rate)
            .clamp(-params.max_steering_rate, params.max_steering_rate),
    }
}

fn finite(value: f64) -> f64 {
    if value.is_finite() { value } else { 0.0 }
}

fn validate(config: &MppiConfig) -> Result<(), PathPlannerError> {
    if config.horizon == 0 || config.samples == 0 {
        return Err(PathPlannerError::InvalidConfiguration(
            "MPPI horizon and sample count must be non-zero".into(),
        ));
    }
    if !config.dt.is_finite() || config.dt <= 0.0 {
        return Err(PathPlannerError::InvalidConfiguration(
            "MPPI timestep must be finite and positive".into(),
        ));
    }
    if !config.temperature.is_finite() || config.temperature <= 0.0 {
        return Err(PathPlannerError::InvalidConfiguration(
            "MPPI temperature must be finite and positive".into(),
        ));
    }
    for (name, value) in [
        ("accel_noise", config.accel_noise),
        ("steering_rate_noise", config.steering_rate_noise),
    ] {
        if !value.is_finite() || value < 0.0 {
            return Err(PathPlannerError::InvalidConfiguration(format!(
                "MPPI {name} must be finite and non-negative"
            )));
        }
    }
    if !config.reference_speed.is_finite() {
        return Err(PathPlannerError::InvalidConfiguration(
            "MPPI reference speed must be finite".into(),
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::{GoalCritic, ObstacleCritic, PathTrackingCritic};
    use crate::motion::BicycleParams;

    fn model() -> BicycleModel {
        BicycleModel::new(BicycleParams {
            wheelbase: 2.0,
            max_steering_angle: 0.5,
            max_steering_rate: 0.8,
            min_speed: -1.0,
            max_speed: 2.0,
            max_acceleration: 1.5,
        })
        .unwrap()
    }

    fn controller(seed: u64, samples: usize) -> Mppi {
        let mut controller = Mppi::new(
            model(),
            MppiConfig {
                horizon: 12,
                dt: 0.1,
                samples,
                temperature: 0.5,
                accel_noise: 1.0,
                steering_rate_noise: 0.3,
                seed,
                reference_speed: 1.0,
                parallel: false,
                ..MppiConfig::default()
            },
        )
        .unwrap();
        controller.add_critic(GoalCritic {
            position_weight: 4.0,
            heading_weight: 1.0,
        });
        controller.add_critic(PathTrackingCritic {
            lateral_weight: 2.0,
            heading_weight: 0.5,
            speed_weight: 0.2,
            progress_weight: 1.5,
        });
        controller.add_critic(ObstacleCritic {
            weight: 2.0,
            collision_cost: 1.0e4,
        });
        controller
    }

    #[test]
    fn same_seed_produces_the_same_command() {
        let map = GridMap::new(40, 20, 0.5, 0.0, 0.0).unwrap();
        let body = Footprint::Circle { radius: 0.3 };
        let path = [Pose2d::new(0.0, 5.0, 0.0), Pose2d::new(15.0, 5.0, 0.0)];
        let state = BicycleState::new(Pose2d::new(1.0, 5.0, 0.0), 0.0, 0.0);
        let mut first = controller(7, 48);
        let mut second = controller(7, 48);
        let a = first.step(state, &map, &body, &path, Pose2d::new(12.0, 5.0, 0.0));
        let b = second.step(state, &map, &body, &path, Pose2d::new(12.0, 5.0, 0.0));
        assert!((a.command.velocity - b.command.velocity).abs() < 1e-12);
        assert!((a.command.steering_angle - b.command.steering_angle).abs() < 1e-12);
    }

    #[test]
    fn goal_ahead_produces_forward_progress() {
        let map = GridMap::new(50, 20, 0.5, 0.0, 0.0).unwrap();
        let body = Footprint::Circle { radius: 0.3 };
        let path: Vec<_> = (0..20).map(|i| Pose2d::new(i as f64, 5.0, 0.0)).collect();
        let mut controller = controller(3, 64);
        let mut state = BicycleState::new(Pose2d::new(1.0, 5.0, 0.0), 0.0, 0.0);
        for _ in 0..12 {
            let output = controller.step(state, &map, &body, &path, Pose2d::new(16.0, 5.0, 0.0));
            state = output.state;
            assert!(state.velocity <= model().params().max_speed + 1e-9);
            assert!(state.steering.abs() <= model().params().max_steering_angle + 1e-9);
        }
        assert!(state.pose.x > 1.4, "x={}", state.pose.x);
    }

    #[test]
    fn a_wall_ahead_suppresses_forward_speed() {
        let mut map = GridMap::new(40, 20, 0.5, 0.0, 0.0).unwrap();
        map.set_aabb_occupied(
            crate::geometry::Point::new(1.7, 3.5),
            crate::geometry::Point::new(2.4, 6.5),
        );
        let body = Footprint::Circle { radius: 0.3 };
        let path = [Pose2d::new(0.0, 5.0, 0.0), Pose2d::new(10.0, 5.0, 0.0)];
        let mut controller = controller(5, 96);
        let state = BicycleState::new(Pose2d::new(0.5, 5.0, 0.0), 1.2, 0.0);
        let output = controller.step(state, &map, &body, &path, Pose2d::new(10.0, 5.0, 0.0));
        assert!(
            output.state.velocity < state.velocity,
            "velocity {}",
            output.state.velocity
        );
    }

    #[test]
    fn action_derivative_noise_is_deterministic() {
        let map = GridMap::new(20, 20, 0.5, 0.0, 0.0).unwrap();
        let body = Footprint::Circle { radius: 0.2 };
        let make = || {
            Mppi::new(
                model(),
                MppiConfig {
                    horizon: 8,
                    samples: 20,
                    noise: NoiseModel::ActionDerivative,
                    seed: 11,
                    ..MppiConfig::default()
                },
            )
            .unwrap()
        };
        let mut a = make();
        let mut b = make();
        a.add_critic(GoalCritic {
            position_weight: 1.0,
            heading_weight: 0.2,
        });
        b.add_critic(GoalCritic {
            position_weight: 1.0,
            heading_weight: 0.2,
        });
        let state = BicycleState::new(Pose2d::new(0.0, 0.0, 0.0), 0.0, 0.0);
        let left = a.step(state, &map, &body, &[], Pose2d::new(4.0, 0.0, 0.0));
        let right = b.step(state, &map, &body, &[], Pose2d::new(4.0, 0.0, 0.0));
        assert!((left.input.acceleration - right.input.acceleration).abs() < 1e-12);
    }

    #[cfg(feature = "rayon")]
    #[test]
    fn parallel_rollouts_match_sequential_ones() {
        let map = GridMap::new(30, 20, 0.5, 0.0, 0.0).unwrap();
        let body = Footprint::Circle { radius: 0.3 };
        let path = [Pose2d::new(0.0, 2.0, 0.0), Pose2d::new(8.0, 2.0, 0.0)];
        let make = |parallel| {
            let mut controller = Mppi::new(
                model(),
                MppiConfig {
                    horizon: 10,
                    samples: 32,
                    seed: 21,
                    parallel,
                    ..MppiConfig::default()
                },
            )
            .unwrap();
            controller.add_critic(GoalCritic {
                position_weight: 2.0,
                heading_weight: 0.5,
            });
            controller.add_critic(ObstacleCritic {
                weight: 1.0,
                collision_cost: 100.0,
            });
            controller
        };
        let state = BicycleState::new(Pose2d::new(1.0, 2.0, 0.0), 0.2, 0.0);
        let sequential = make(false).step(state, &map, &body, &path, Pose2d::new(7.0, 2.0, 0.0));
        let parallel = make(true).step(state, &map, &body, &path, Pose2d::new(7.0, 2.0, 0.0));
        assert!((sequential.command.velocity - parallel.command.velocity).abs() < 1e-9);
        assert!((sequential.input.steering_rate - parallel.input.steering_rate).abs() < 1e-9);
    }
}
