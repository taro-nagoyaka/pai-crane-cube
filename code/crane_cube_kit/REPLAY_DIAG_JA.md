# モデルが接近途中で停滞した場合の制御接続診断

deploy_ensemble.csvでは352指令すべてがallowedで、35.1秒まで実行していました。
最後の実測と指令の差は最大約0.0181 radです。上限超過による停止ではありません。
モデルが接近途中の姿勢付近の指令を出し続け、指定時間で終了しています。
Goal reachedは、その時の関節目標への到達を示し、把持タスクの成功ではありません。

モデルの予測精度に加え、教師データのactionと実行インターフェースの整合性を確認します。
公式教師はMoveItの軌道を実行する一方、モデルは0.1秒ごとに0.2秒の目標関節指令を送り直します。
今回の診断では、**保存済み教師actionを、そのモデル側インターフェースで再生**します。
学習モデルの成功とは扱いません。新たな学習や教師データの書換えも行いません。

## 手順

1. policy_server.pyをCtrl+Cで終了し、最新ZIPを上書き展開してください。

```bash
cd /home/ubuntu
unzip -o crane_cube_cpu_kit.zip
```

2. Gazeboを終了・再起動して、キューブとアームを初期位置に戻します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

3. 別のROSターミナルで、次を実行してください。モデルサーバーは起動しません。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /home/ubuntu/crane_cube_kit
/usr/bin/python3 replay_teacher.py --raw raw_auto_branch --trace replay_teacher_trace.csv
```

保存済み記録のうち、ファイル名順で最初の1件を読みます。
`--episode raw_auto_branch/episode_....npz`で明示的に選ぶこともできます。
239フレームなら約24秒のシミュレーション時間、0.25倍速では実時間で約96秒以上です。
q / Spaceで中断できます。0.15 radの指令変化上限と関節可動範囲を確認してから送信します。
同名ログがある場合は別の--trace名を指定してください。

## 結果の判断

- 掴んで持ち上げられる: 保存したactionと実行方法は、少なくともその1軌道ではタスクを再現できます。
  次はモデルの閉ループ予測のずれを優先して直します。
- 掴めない / 位置がずれる / 遅れる: 記録actionと実行方法の対応を先に調べます。
  モデル学習だけで解決できるとは判断せず、replay_teacher_trace.csvを確認します。
- エラー停止: 最後のエラーとCSVを共有してください。上限を上げて続行しないでください。

この診断が成功しても、モデルそのものが把持できたことにはなりません。
同じ初期配置の1軌道の検証であり、任意のキューブ位置に対する汎化の検証でもありません。

## 実装・検証範囲

画像・実測stateのコピーではなく、記録されたactionを順番に送信します。
関節順・FPS・成功ラベル・有限値・可動範囲・初期姿勢・指令変化量を検査します。
正常終了時は最後のグリッパー目標を維持し、途中中断時は現在角度の保持を指令します。
この配布環境ではデータ読込・構文を検証しています。ROS/Gazeboの物理再生はユーザー環境での確認が必要です。
