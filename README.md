# Taylor-Informed PCAC for Quadrotors

MATLAB implementation of the Taylor-Informed Predictive Cost Adaptive Control (PCAC) framework for quadrotors with online gravity-trim and mass estimation.

The implementation uses:

- Exact nonlinear quadrotor dynamics as the simulated plant.
- Taylor-informed row-wise RLS identification (D = 1, 2, 3).
- Jacobian-frozen LPV–ARX prediction.
- PCAC solved using `quadprog`.
- Automatic comparison of all identification degrees for multiple simulation cases. 

---

## Folder Structure

```
main_pcac_quadrotor.m
pcac_parameters.m

quadrotor_dynamics.m
rk4_step.m

taylor_dictionary.m
rowwise_rls.m
common_vrf_update.m
identified_map_jacobian.m
estimate_trim_mass.m

solve_pcac_qp.m

generate_reference.m

plot_results.m
animate_quadrotor.m
performance_metrics.m

results/
```

---

## Quick Start

1. Open MATLAB.
2. Set the project folder as the current directory.
3. Run

```matlab
main_pcac_quadrotor
```

The script automatically:

- runs Case 1 (abrupt mass change);
- runs Case 2 (aggressive helix tracking);
- evaluates D = 1, D = 2, and D = 3 under identical conditions;
- generates figures, animation, and performance metrics;
- saves results to the `results/` folder.

---

## Requirements

- MATLAB
- Optimization Toolbox (`quadprog`)
