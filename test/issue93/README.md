# Pinned ODE cone row regression

Related to XGC-Team/xgc2-harness#93. This is a source-correctness change in the
actual Gazebo/ODE source repository, not a fitted substitute vehicle model.
Base: Gazebo 11.15.1, b22c6e15e52299865b31093b8feebc9ca19e26e8.

## Defects and scope

PGS stores each contact as normal,tangent1,tangent2[,torsion], but its default
solve-order constructor fills positive-findex rows from the tail. A single
contact therefore solves in order [normal,tangent2,tangent1]. The old cone
function chose previous/next partners from solve-order adjacency, then used
original storage index +/-1 for the Jacobian. At tangent2 of the last contact,
it can read a full row beyond the allocated Jacobian. At tangent1 it can use
the normal row instead of the other tangent. Chunking or arbitrary reordering
makes solve-order adjacency an invalid criterion even when the row is in bounds.

The fixed function uses normal+1 and normal+2, and selects the component by
index-normal. It retains the source model's pre-step-velocity component weighting,
anisotropic per-row coefficient and existing zero-slip/zero-bound rule. It does
not claim to replace that rule with a textbook static Coulomb cone.

The old zero-slip branch did not set ERP outputs. In nonzero slip its ERP lower
bound was the negative of the non-ERP upper bound, rather than its own upper
bound. All four outputs are now initialized and each pair is symmetric.
ComputeRows now reads inline ERP pointers only when inline correction is enabled;
the separate threaded correction path passes null and does not access an
uninitialized inline solution.

## Equations, units and tests

For tangential Jacobian rows J1,J2, pre-step generalized velocity nu, set
u1=J1*nu, u2=J2*nu [m/s]. For speed above the retained 1e-18 threshold, bounds are

    abs(lambda_i) <= mu_i * abs(lambda_n) * abs(u_i)/sqrt(u1*u1+u2*u2).

lambda is force [N], not impulse. ERP uses its own normal force lambda_erp_n.
The change corrects row lookup/output initialization; it does not change this law.

Run `python3 test/issue93/test_cone_rows.py --output cone-receipt` after the
production edit. The runner extracts both functions directly from the pinned
Git source and current production .cpp and compiles them with minimal scalar/body
fixtures, C++17, -Wall -Wextra -Werror, AddressSanitizer and UBSan. It reproduces
original heap overflow, [-36,54] N asymmetric ERP bounds (expected [-54,54]), and
untouched ERP output sentinels at zero slip. The fixed suite covers 2162 cases:
all solve permutations of three/four-row contacts, singleton solve chunks,
nonzero normal-row offsets, one/two moving bodies, contact angular velocity,
positive/negative/zero components, unequal mu, distinct normal/ERP forces,
null inline ERP and zero slip. It is NOT a full ODE integration test or Gazebo run.

The one-time hash-guarded edit script is retained as a reproducibility entry.
The branch CI applies it only to the known original blob, tests the actual
production result, then commits/pushes ONLY that .cpp on the named work branch.
No default-branch mutation, merge, deployment or robot action occurs.

## Required local physical validation

Build this source using the same Gazebo dependency/toolchain and publish a new
isolated test image or library artifact; record commit, package versions and
binary SHA256. Do not label the existing 11.15.1 package as patched based on its
version string. Re-run the same frozen inputs and one parameter set with
cone and pyramid controls at 4/2/1 ms. Record wheel torque/speed, all body velocity
components and contact forces. A historical cone fit against the buggy binary
is not independent validation of the repaired implementation. Full library
build/ABI checks, actual source-package correspondence, open-loop holdout response
and four-car acceptance remain untested until explicit run receipts exist.
