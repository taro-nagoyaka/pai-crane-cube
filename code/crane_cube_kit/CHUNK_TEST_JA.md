# 小さく動いた後に停滞する場合の比較テスト

今回のログでは、35秒終了時にjoint2約-0.343、joint3約-0.456、joint4約-0.154 radでほぼ一定でした。
単に遅いだけではなく、初期動作付近から進んでいないと判断できます。
原因が推論設定だけにあると確定したわけではありません。モデルの予測誤差が蓄積し、
教師データとは異なる状態へ移った可能性もあります。

配布済みの初期設定は、ACTが予測した10手のうち最初の1手だけを使い、毎回再予測していました。
この版は、**同じチェックポイントの10手を順番に実行してから再予測する比較**を追加します。
学習済み重み・正規化・教師データは変更しません。手動の教師軌道へ切り替える処理もありません。
これで把持できることを保証するものではなく、再学習の前に動作進行を切り分けるテストです。

## 1. 上書き展開

起動中のpolicy_server.pyをCtrl+Cで終了してから、最新ZIPを展開します。

```bash
cd /home/ubuntu
unzip -o crane_cube_cpu_kit.zip
```

## 2. Gazeboを初期状態へ戻す

GazeboのターミナルでCtrl+Cを押し、同じカメラ付きシーンを再起動します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

## 3. 10手実行設定でモデルを起動

学習venvのターミナルで実行します。

```bash
cd /home/ubuntu/crane_cube_kit
source .venv-learning/bin/activate
python policy_server.py \
  --checkpoint outputs/act_branch_2000/checkpoints/last/pretrained_model \
  --action-steps 10
```

次の表示を確認してください。

```text
ACT chunk_size=10, n_action_steps=10
CPU policy ready: ...
```

10手は、10 Hzのシミュレーションで1秒分です。その間もクライアントは毎回観測の鮮度を確認し、
関節制限・指令の飛び・推論遅延の検査を維持します。画像・状態に基づく新しい推論は10手ごとになります。
Space/qでの停止も維持します。

## 4. ログ付きで実行

別のROSターミナルで実行します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /home/ubuntu/crane_cube_kit
/usr/bin/python3 robot.py deploy --seconds 35 --trace deploy_chunk10.csv
```

実測角度qと、送信した指令角度targetを表示します。CSVにはモデルの未クリップ予測も記録します。
`deploy_chunk10.csv` が既にある場合は、`deploy_chunk10_retry.csv` など別名にしてください。
既存のログを自動上書きしません。

停止した場合は上限を上げずにエラーを共有してください。
最後まで進まなかった場合も、`deploy_chunk10.csv` を共有してください。

## 結果の見方

- qとtargetがともにほぼ一定：モデルが継続する動作を出せていない可能性。
- targetは変わるがqが追従しない：実行・追従側を優先して調査。
- 大きく進むが把持できない：モデル誤差、接近位置、グリッパー動作を調査。

参照したLeRobot 0.4.3の配布ソースでは、reset()がn_action_stepsに応じて予測キューを作り直し、
select_action()がそのキューから順に取り出します。変更後のキュー長はその仕組みを使っています。
設定検査・Python構文・キューの取り出し順は検査できますが、この環境にはROS/PyTorchがないため、
ユーザーの実モデルによるGazebo動作は未検証です。
