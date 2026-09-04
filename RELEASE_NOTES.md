# v1.0.0 - Stable reproducibility release

This release accompanies the stable preprint:

> T. W. Nguyen, “Taylor-Informed Predictive Cost Adaptive Control for Quadrotors with Online Gravity-Trim Adaptation,” arXiv:2609.03351, 2026, doi: 10.48550/arXiv.2609.03351.

- Paper: <https://arxiv.org/abs/2609.03351>
- DOI: <https://doi.org/10.48550/arXiv.2609.03351>

## Included

- MATLAB implementation of the Taylor-informed PCAC formulation.
- Deterministic Case 1 and Case 2 simulations for Taylor degrees 1, 2, and 3.
- Machine-readable expected metrics from Tables IV and V.
- Automated checks for reported metrics, model dimensions, paper parameters, solver status, and constraints.
- The six-page manuscript and machine-readable citation metadata.

## Reproduction

Run the following commands from the repository directory in MATLAB:

```matlab
main_pcac_quadrotor
verify_paper_results
```

The default workflow runs the six reported simulations. It does not rerun the Monte Carlo study. QP timing is excluded from strict comparison because it depends on the execution platform.

## Verification status

The six deterministic simulations reproduce the paper’s Tables IV and V at the reported precision. The generated figures reproduce Figs. 1-4, and all runs complete without solver or constraint failures in the verified environment.
