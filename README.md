# Taylor-Informed PCAC for Quadrotors

Supporting MATLAB implementation for the paper **“Taylor-Informed Predictive Cost Adaptive Control for Quadrotors with Online Gravity-Trim Adaptation.”** The paper is the authoritative technical description; this repository provides the code used to reproduce its two deterministic simulation cases, figures, and performance tables.

Repository: <https://github.com/tamwng/taylor-informed-pcac-quadrotor>

## Method summary

The implementation combines an exact nonlinear quadrotor plant, sparse degree-1/2/3 Taylor-informed sampled-data dictionaries, row-wise RLS with shared variable-rate forgetting, online mass and gravity-trim estimation, a Jacobian-frozen affine predictor, and PCAC solved with `quadprog`.

## Requirements

- MATLAB with Optimization Toolbox (`quadprog`).
- The workflow has been verified with MATLAB R2026a. This is a verified environment, not a stated minimum version.

## Reproduce the paper

From the repository directory, run:

```matlab
main_pcac_quadrotor
verify_paper_results
```

The default workflow executes exactly six simulations: Case 1 and Case 2 for Taylor degrees (D=1,2,3). It does not run a Monte Carlo study. `verify_paper_results` checks the generated metrics, dictionary sizes, paper parameters, solver status, and constraints against the tracked paper expectations.

## Expected numerical results

Case 1 — abrupt mass change from 4.34 kg to 5.60 kg:

| (D) | Position RMSE (m) | Final mass error (kg) | Final trim-thrust error (N) |
|---:|---:|---:|---:|
| 1 | 0.348 | 0.052 | 0.028 |
| 2 | 0.062 | 0.066 | 0.017 |
| 3 | 0.069 | 0.031 | 0.007 |

Case 2 — aggressive helix tracking at \(m=m_0=4.34\) kg:

| (D) | Position RMSE (m) | Prediction RMSE (m) | Input variation | Maximum tracking error (m) |
|---:|---:|---:|---:|---:|
| 1 | 0.0519 | 0.001281 | 154.73 | 0.1695 |
| 2 | 0.0288 | 0.000559 | 69.27 | 0.1664 |
| 3 | 0.0281 | 0.000483 | 59.41 | 0.1664 |

The machine-readable expectations are in [`validation/paper_expected_metrics.csv`](validation/paper_expected_metrics.csv). QP solve times are intentionally excluded from reproducibility comparisons because they depend on the platform and runtime state.

## Generated outputs

Generated artifacts are written to the ignored `results/` directory:

- `pcac_quadrotor_results.mat` — complete run data and parameters;
- `pcac_quadrotor_metrics.csv` — numerical metrics and diagnostics;
- `case1_tracking.*` and `case1_estimates_vrf.*` — paper Figs. 1–2;
- `case2_trajectory.*` and `case2_prediction_attitude.*` — paper Figs. 3–4.

## Paper-to-code map

| Paper content | Repository file |
|---|---|
| Complete workflow | `main_pcac_quadrotor.m` |
| Table III and case parameters | `pcac_parameters.m` |
| Exact nonlinear plant, Eqs. (11)–(12) | `quadrotor_dynamics.m` |
| Fixed-step RK4 integration | `rk4_step.m` |
| Tables I–II Taylor dictionaries | `taylor_dictionary.m` |
| Row-wise RLS, Eqs. (43)–(50) | `rowwise_rls.m` |
| Shared variable-rate forgetting | `common_vrf_update.m` |
| Identified map and frozen Jacobian | `identified_map_jacobian.m` |
| Mass and gravity-trim estimates, Eqs. (52)–(56) | `estimate_trim_mass.m` |
| PCAC optimization, Eqs. (57)–(75) | `solve_pcac_qp.m` |
| Case reference trajectories | `generate_reference.m` |
| Tables IV–V metrics | `performance_metrics.m` |
| Figures 1–4 | `plot_results.m` |
| Reproduction checks | `verify_paper_results.m` |

## Optional features

Animations are optional and disabled by default. Set `p.graphics.make_animation = true` in `pcac_parameters.m` to generate comparison videos after the simulations.

The source also contains implementation-only numerical safeguards that are not paper tuning parameters: RLS covariance jitter (10^{-12}), QP Hessian regularization (10^{-9}), an Euler-chart singularity guard, and a QP safety tolerance. These protect finite-precision execution without changing the reported scientific configuration.

## Citation

Until the arXiv record is available, cite the work as:

> Tam W. Nguyen, “Taylor-Informed Predictive Cost Adaptive Control for Quadrotors with Online Gravity-Trim Adaptation,” unpublished L-CSS/ACC manuscript, 2026.

Software citation metadata are provided in [`CITATION.cff`](CITATION.cff). The preferred-citation metadata will be updated when the arXiv record is posted.

## License

This software is released under the BSD 3-Clause License; see [`LICENSE`](LICENSE).
