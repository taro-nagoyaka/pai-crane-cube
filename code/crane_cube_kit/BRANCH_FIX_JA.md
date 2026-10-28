# 教師動作の肘の曲げ方を揃える修正

## 調査結果

アップロードされた train_trace.csv / validation_trace.csv を比較しました。

| 実測関節角度 | 学習データ | 検証データ |
|---|---:|---:|
| joint2 | -1.160 ～ 0.000 rad | -1.160 ～ +1.501 rad |
| joint3 | -2.010 ～ 0.000 rad | -2.010 ～ +1.422 rad |

検証側の74/284フレーム（約26%）では、joint2またはjoint3が学習側の範囲から0.03 rad以上外れていました。
この区間だけで絶対誤差の合計の約69.7%を占めます。
範囲外の区間の平均誤差は0.892 rad、残りの区間は0.136 radでした。
したがって、教師の未学習の肘姿勢が大きい誤差と強く対応しています。
範囲内でも誤差が残るので、この修正だけで性能が十分になるとは判断できません。

URDFで順運動学を計算すると、ほぼ同じ接近位置・手先姿勢を次のように異なる関節角で実現していました。

| 項目 | 学習側 8.3秒 | 検証側 10.9秒 |
|---|---:|---:|
| joint2 | -0.032 rad | +1.501 rad |
| joint3 | -1.433 rad | +1.422 rad |
| 手首の半径（ベースからの水平距離） | 0.08949 m | 0.09019 m |
| 手首の高さ（ベース基準） | 0.17015 m | 0.17041 m |
| 手先ピッチ q2-q3-q4 | 1.57081 rad | 1.57111 rad |

位置は1 mm以内、ピッチもほぼ同じです。異なる逆運動学解・経路が選ばれたと解釈できます。
この結果だけでは、学習設定や操作指令の時刻対応に問題がないことまでは証明できません。

## 修正内容

最初の固定位置タスクでは、MoveItの経路制約で次の範囲に限定します。

- joint2: -1.80 ～ +0.02 rad
- joint3: -2.53 ～ +0.02 rad

いずれもユーザー環境のURDFの可動範囲内で、既存home・vertical姿勢を含みます。
負側の肘姿勢は実際に学習側の成功データで観測されています。
「関節角の符号を後から書き換える」「既存の正側データを負側へ反転する」処理は行いません。
計画できない場合は停止し、無理に角度を切り詰めて動かしません。
分岐は抑えますが、毎回完全に同じ経路を生成する保証はありません。

新しいC++変更のコンパイル・MoveIt計画・物理保持は、ユーザー環境での確認が必要です。

## 実行手順

1. 最新ZIPを `/home/ubuntu/crane_cube_cpu_kit.zip` に置き、上書き展開します。

```bash
cd /home/ubuntu
unzip -o crane_cube_cpu_kit.zip
```

2. 既存の操作プログラムやサンプルを終了し、Gazeboを再起動してキューブを元へ戻します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

3. 別のコンテナ内ターミナルで実行します。

```bash
cd /home/ubuntu/crane_cube_kit
bash collect_auto.sh --out raw_auto_branch
```

自動で再ビルドします。ログに `Teacher uses constrained negative elbow branch` と表示されます。
動作終了後、掴んだまま持ち上げて保持できていた場合だけyで保存します。
旧データ `raw_auto_episodes` とモデルは残し、新しい記録は別フォルダに保存します。

4. まず1回だけ成功するか確認し、次を実行します。

```bash
/usr/bin/python3 check_branch.py --raw raw_auto_branch
```

PASSは肘姿勢の範囲確認のみで、把持成功率やモデル精度の保証ではありません。
計画失敗やCHECKが出た場合は、追加収集せずログを共有してください。

## その後

最初の1回が成功・PASSになったら、同じ制約で成功記録を追加します。
まず合計5回程度を目安にし、毎回Gazeboを初期状態へ戻してください。
5回は最小の改善確認の目安であり、汎化性能を保証する件数ではありません。

```bash
source .venv-learning/bin/activate
python convert_dataset.py --raw raw_auto_branch --out data_branch
bash train_cpu.sh data_branch/train outputs/act_branch_2000 2000
```

旧の分割は変更せず、新しいデータだけで別の分割を作ります。
旧検証データは異なる肘姿勢への参考テストとして残せますが、
新しい制約付きタスクの検証結果と同じ条件として直接比較しないでください。

同梱の診断スクリプトで、学習・検証・現在角度基準を比較します。

```bash
python diagnose_model.py \
  --checkpoint outputs/act_branch_2000/checkpoints/last/pretrained_model \
  --data data_branch --out diagnostics_branch
```

誤差が改善しても、Gazeboでモデル自身を動かして把持・保持を確認するまでは完成とは扱いません。
