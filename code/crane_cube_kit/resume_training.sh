#!/usr/bin/env bash
# Total steps, not additional steps. Refuse to reuse an output folder.
set -euo pipefail
cd "$(dirname "$0")"
training_config="${1:-outputs/act_varied_novae_6000/checkpoints/last/pretrained_model/train_config.json}"
training_steps="${2:-18000}"
training_output="${3:-outputs/act_varied_novae_18000}"
test -f "$training_config"
if test -e "$training_output"; then
  echo "出力先が既にあります。別のフォルダ名を指定してください: $training_output" >&2
  exit 1
fi
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-4}"
exec .venv-learning/bin/lerobot-train --config_path="$training_config" \
  --resume=true --steps="$training_steps" --output_dir="$training_output"
