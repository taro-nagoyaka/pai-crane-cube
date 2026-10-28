# 10手実行が始動直後に停止した場合

提供されたdeploy_chunk10.csvは20行、0.003～1.900秒の記録でした。
最終実測joint3は-0.3775 rad、最終送信目標は-0.7200 radで、アームは動いています。
保存済み指令間の最大変化は0.1224 radで、0.15 radの制限内でした。
次の10手への切替付近の予測で停止した可能性がありますが、旧CSVには拒否した予測がないため確定できません。
また実測と目標の間にも遅れがあり、モデル・実行系を含む閉ループの問題として調査します。

## 今回の変更

- ACT標準のtemporal ensemblingを選べるようにしました。
- 毎回10手先まで予測し、過去の予測で「今の時刻」に対応する操作と重ね合わせます。
- 予測チャンクを丸ごと切り替える代わりに、その時刻の重み付き平均を指令します。
- モデルの重み・正規化・学習データは変更しません。
- 0.15 radの指令変化上限を維持します。超過すれば引き続き停止します。
- 拒否した予測も、直前の指令・対象関節・変化量とともにCSVへ保存します。

これは原因を絞る比較です。滑らかになっても把持できる保証はなく、停滞が残る場合もあります。
教師軌道を再生する処理や、学習していない動作を外から補う処理は入れていません。

## 手順

policy_server.pyをCtrl+Cで止め、最新ZIPを上書き展開してください。

```bash
cd /home/ubuntu
unzip -o crane_cube_cpu_kit.zip
```

Gazeboも終了・再起動して初期状態に戻します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

モデル用の別ターミナルで実行します。

```bash
cd /home/ubuntu/crane_cube_kit
source .venv-learning/bin/activate
python policy_server.py \
  --checkpoint outputs/act_branch_2000/checkpoints/last/pretrained_model \
  --temporal-ensemble-coeff 0.01
```

`n_action_steps=1`、`temporal_ensemble_coeff=0.01` の表示を確認してください。
前の `--action-steps 10` は付けません。

別のROSターミナルで実行します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /home/ubuntu/crane_cube_kit
/usr/bin/python3 robot.py deploy --seconds 35 --trace deploy_ensemble.csv
```

Space/qで停止できます。自動停止や停滞があればdeploy_ensemble.csvと最後のログを共有してください。
同名CSVがある場合は別名を指定します。既存ファイルを自動上書きしません。

## CSVの意味

- `decision=allowed`: 上限検査を通過。コントローラー側の実行成功の保証ではありません。
- `decision=rejected_jump`: 指令変化上限を超え、送信せず停止。
- `decision=rejected_latency`: 観測から時間が経過しすぎたため、送信せず停止。
- `previous_target_*`: 前回送信した目標角度。
- `prediction_*`: モデルの予測。ensemble有効時は重ね合わせ後の予測です。
- `target_*`: 今回提案された、関節可動範囲内の目標角度。拒否行は実行されていません。
- `policy_step`: サーバーがresetされてからの予測要求番号。

LeRobot 0.4.3の実装に合わせ、ACTTemporalEnsemblerを生成してresetし、n_action_steps=1で使います。
この環境では設定処理・構文の確認はできますが、実モデルのGazebo動作は未検証です。
