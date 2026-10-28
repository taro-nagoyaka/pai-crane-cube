#!/usr/bin/env bash
set -euo pipefail
dataset_path="${1:-data/train}"
training_output="${2:-outputs/act_cpu_smoke}"
training_steps="${3:-200}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-4}"
lerobot-train \
  --dataset.repo_id=local/crane_cube_train \
  --dataset.root="$dataset_path" \
  --policy.type=act \
  --policy.device=cpu \
  --policy.push_to_hub=false \
  --policy.chunk_size=10 \
  --policy.n_action_steps=1 \
  --policy.dim_model=128 \
  --policy.n_heads=4 \
  --policy.dim_feedforward=512 \
  --policy.n_encoder_layers=2 \
  --policy.n_decoder_layers=1 \
  --policy.n_vae_encoder_layers=2 \
  --batch_size=2 \
  --num_workers=0 \
  --steps="$training_steps" \
  --log_freq=20 \
  --save_freq=200 \
  --eval_freq=0 \
  --wandb.enable=false \
  --output_dir="$training_output" \
  --job_name=crane_cube_cpu
