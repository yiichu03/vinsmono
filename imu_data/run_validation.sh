#!/usr/bin/env bash
set -euo pipefail

# Run after building export_vins_preint_pack and test_imu_preintegration.
bin_dir=${1:?Usage: bash run_validation.sh BINARY_DIRECTORY [OUTPUT_DIRECTORY]}
data_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
output_dir=${2:-$(mktemp -d -t vins-preint-validation.XXXXXX)}
mkdir -p -- "$output_dir"

"$bin_dir/test_imu_preintegration" 2>&1 | tee "$output_dir/internal_cases.log"
"$bin_dir/export_vins_preint_pack" \
  --imu_txt "$data_dir/imu_data_Tangent_0.txt" \
  --config_yaml "$data_dir/cpc_config_Tangent_0.yaml" \
  --out_txt "$output_dir/vins_preint_core.txt" --check_jacobian_fd \
  2>&1 | tee "$output_dir/core.log"
echo "[ OK ] core regression suite: internal F/V/J/cov checks and GTSAM comparison passed."
echo "Results: $output_dir"
