# CRANE+ / Gazebo → LeRobot CPU学習 → Gazebo推論

**姿勢を変えた教師データの自動収集を追加しました。**
`collect_batch.sh` がGazebo再起動・4種類の教師軌道・高さと保持の自動判定・
成功データの保存を繰り返します。最初は1件で確認してください。
詳しい手順は **BATCH_AUTO_JA.md** を参照してください。
従来の手動確認付き `collect_auto.sh` も引き続き使えます。

予測平均化でも接近途中で停滞する場合は、`REPLAY_DIAG_JA.md` の教師action再生診断を先に行ってください。
モデル側の制御インターフェースで保存済みactionがタスクを再現できるかを切り分けます。

10手実行の切替付近で指令変化上限により停止した場合は、`ENSEMBLE_TEST_JA.md` を参照してください。
ACTの予測を重ね合わせる比較と、拒否した指令も記録する診断ログを追加しています。

モデル実行時に少し動いて停滞する場合は、`CHUNK_TEST_JA.md` の10手実行比較を参照してください。
同じチェックポイントを使い、再学習せずに実行設定だけを比較できます。

**最新の教師動作は、肘の曲げ方のばらつきを抑える制約付きです。**
収集済みCSVの調査結果と新しい記録手順は `BRANCH_FIX_JA.md` を参照してください。
新しい記録先は `bash collect_auto.sh --out raw_auto_branch` で分けてください。

**手動操作が難しい場合は、追加の `AUTO_JA.md` を先に読んでください。**
公式サンプルを元にした自動持ち上げ・自動記録を `bash collect_auto.sh` で実行できます。
この自動版のC++ビルド・Gazeboでの保持動作はユーザー環境での確認が必要です。

ROS 2 Humble、WSL内の既存コンテナ、`crane_plus_gazebo crane_plus_with_table.launch.py`向けの初期実装です。
**学習済みモデル・成功済み把持軌道は含みません。** 教師操作で実際に掴めることを確認し、成功データを収集してから学習します。
こちらではROS/Gazeboを実行できないため、コンテナでの起動・描画・物理把持・学習・閉ループ推論は未検証です。

## 1. コンテナ内に配置する

ZIPを展開した `crane_cube_kit` フォルダを、既存コンテナの `/home/ubuntu/` に置いてください。
WSL側からコピーする場合の例です。コンテナIDは `docker ps` で確認してください。

```bash
docker cp crane_cube_kit CONTAINER_ID:/home/ubuntu/
```

以下は、既存コンテナ内の対話ターミナルで実行します。
フォルダ、収集データ、学習結果は、コンテナ削除後も残すためWSL側のbind mount配下に置くのがおすすめです。
別の場所でも動きますが、以降の `cd` をその場所へ変更してください。

必要な追加パッケージ（未導入の場合）：

```bash
sudo apt-get update
sudo apt-get install -y python3-numpy python3-pil python3-venv python3-pip ffmpeg
```

sudoがなければコンテナのrootターミナルでapt-getを実行してください。
既存ROS/Gazeboの再インストールは不要です。

## 2. カメラ付きシーンを生成して起動する

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /home/ubuntu/crane_cube_kit
/usr/bin/python3 prepare_scene.py
```

元のGazeboを起動しているターミナルでCtrl+Cを押し、完全に終了してから、次を実行します。
同じワールド・ロボットを二重起動しないでください。

```bash
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

生成する内容：

- インストール済み `table.sdf` とlaunchを元に、別のファイルを生成。元ファイルは変更しません。
- 元のwoodcubeモデルと配置 `(0, -0.1, 1.0)` を保持し、モデル名を `woodcube` に固定。
- 固定RGBカメラ：320×240、20 Hz。学習用には224×224へ変換し、10 Hzで記録。
- Sensorsプラグインと `/camera/front/image` のGazebo→ROS画像ブリッジを追加。
- シミュレーション速度の上限目標は実時間の0.25倍。CPU推論に余裕を持たせる初期値です。

別のROS環境をsourceしたターミナルで確認：

```bash
ros2 topic hz /camera/front/image
```

これは受信の確認用です。低速シミュレーションでは表示される実時間当たりのHzも低くなります。
画像が出なければ、Gazebo起動ログのSensors・rendering・bridgeエラーを確認してください。
WSLで既存Gazeboが表示できても、カメラ描画に使うOgre2/EGLが動くとは限りません。
既存のFuelモデルのキャッシュはそのまま利用します。キャッシュがなければモデルのダウンロードが必要です。

## 3. 手動で把持動作を作る

別ターミナルでROS環境をsourceし、シーンを起動したまま実行します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /home/ubuntu/crane_cube_kit
/usr/bin/python3 robot.py teach
```

**これはGazebo向けの直接関節操作です。MoveItの衝突回避計画は通りません。**
最初は `f` で0.01 rad刻みにし、キューブを押し飛ばさないように操作してください。

| キー | 操作 |
|---|---|
| `1`～`4` | 操作するアーム関節を選択 |
| `5` | グリッパーを選択 |
| `+` / `=` | 選択した関節の目標角度を増やす |
| `-` | 選択した関節の目標角度を減らす |
| `f` | 0.05 / 0.01 rad刻みを切替 |
| `r` | 現在姿勢を保持して記録開始 |
| `s` | 目視で成功を確認してエピソード保存 |
| `d` | 失敗したエピソードを破棄 |
| `v` | 学習に使う視点を `camera_preview.png` に保存 |
| Space | 現在角度を目標に保持し、記録を破棄 |
| `q` | 終了して現在角度を保持 |

数字キーは上段の数字を使ってください。操作はこのプログラムのターミナルにフォーカスして行います。
RVizや別の操作プログラムから同時に指令を送らないでください。記録するactionは本プログラムが送った指令です。

### 成功する動作を先に1回作る

1. `v` で画像を保存し、キューブとアーム全体が見えるか確認します。
2. `5` と `+` / `-` で開閉方向を確かめます。符号から「開く／閉じる」は決め打ちしていません。
3. アーム関節を少しずつ動かし、指の間にキューブが入る姿勢を作ります。
4. グリッパーを閉じ、少しだけ持ち上げます。物理的に保持できることを確認します。
5. 直立に近い高い位置へ持ち上げ、シミュレーション時間で2秒保持します。

URDFでは `crane_plus_link_endtip` はjoint4の位置であり、指の間の把持中心ではありません。
全関節0の直立姿勢で、このリンクはベースから約0.2479 mです。
Gazeboのロボット配置z=1.015を足すと約1.2629 mですが、キューブ中心や指先の高さではありません。
このキットは関節角で操作するため、Gazebo worldとROS TF worldの未確認のずれを把持座標へ使いません。

上まで持ち上がる前に滑る場合は、まず接触位置、グリッパー幅、キューブサイズ、摩擦、質量を確認します。
模倣学習だけでは、シミュレータ内で成立しない把持を成立させることはできません。
このキットは摩擦やモデル寸法を勝手に変更せず、キューブを手へ固定する処理も入れていません。

## 4. 成功データを収集する

毎回ほぼ同じアーム初期姿勢と、キューブの初期位置から始めます。
`r` → 接近 → 把持 → 持ち上げ → 2秒保持 → `s` で保存します。
失敗したら `d` で破棄します。保存先は `raw_episodes/episode_*.npz` です。

**`s` は操作者が成功と判定するキーです。接触・キューブ高度・保持時間の自動判定はありません。**
映像でキューブが追従し、落下していないことを確認してから保存してください。

初期段階は、先に2エピソードで変換・学習の処理確認をします。性能のあるモデルを作る段階では、
50～100エピソード程度を初期の目安にし、検証結果を見て追加してください。成功率を保証する件数ではありません。
手先の接近の仕方やキューブ位置を少し変えたデータも必要です。単なる同一軌道の複製は汎化の検証になりません。

シーンの戻し方：最初は `q` で操作プログラムを終了し、Gazeboのlaunchを終了・再起動すると確実です。
GUIでリセットする場合も、時刻が戻ったら操作プログラムを再起動してください。
グリッパーを開くだけではキューブの初期位置・速度は戻りません。

記録するデータ：関節名で並べた実測角度5個、指令目標角度5個、RGB画像、各センサーの時刻です。
受信順がjoint1,2,4,3,handでも、常にjoint1,2,3,4,handへ並べ替えます。
10 Hzのシミュレーション時刻を基準に近接時刻の観測を採用します。ハードウェア同期ではありません。
時間の飛び・画像重複・古い観測・状態と画像の時刻差が大きい記録は保存を拒否します。

## 5. LeRobotの学習環境を作る

**同じコンテナ内**で実行します。ROSノードは引き続き `/usr/bin/python3` を使い、学習だけ別venvを使います。
Python 3.10以上、CPU版PyTorch 2.7.1、torchvision 0.22.1、LeRobot 0.4.3に固定します。
最新版を追従する構成ではなく、配布APIを確認したバージョンに合わせています。

```bash
cd /home/ubuntu/crane_cube_kit
bash setup_learning.sh
```

この段階はPyPI/PyTorch配布サイトへのアクセスと、数GBの空き容量が必要です。
初回学習時には、ResNet18のImageNet学習済み重みもダウンロードします。
ROS用のシステムPythonに `pip install lerobot` はしないでください。

## 6. データ変換とCPU学習

```bash
cd /home/ubuntu/crane_cube_kit
source .venv-learning/bin/activate
python convert_dataset.py
bash train_cpu.sh data/train outputs/act_cpu_smoke 200
```

データはエピソード単位で80%学習用・20%検証用へ分割します。
LeRobotDatasetは画像形式を使い、動画のエンコード負荷を避けています。Hubへのアップロードは行いません。
同じ `data/train` が既にある場合は自動で上書きしません。データを追加したら `--out data_v2` など別名で再変換してください。

**200ステップは動作確認用です。把持を学習できたことを意味しません。**
学習ログの時間から所要時間を見積もり、データ量・検証結果に応じて本学習します。
例：10,000ステップで新しい出力先へ学習する場合：

```bash
bash train_cpu.sh data/train outputs/act_cpu_run01 10000
```

本学習のACTは、Transformer幅128、2エンコーダ層、チャンク10、バッチ2の小型構成です。
CPU時間を抑える初期案であり、標準ACTと同等の性能を保証するものではありません。
画像バックボーンにはResNet18を使用します。CPU時間はデータ量・機種によって変わります。

中断した学習を再開する場合はLeRobotのresumeを使います：

```bash
lerobot-train \
  --config_path=outputs/act_cpu_run01/checkpoints/last/pretrained_model/train_config.json \
  --resume=true
```

## 7. 検証データで操作誤差を測る

```bash
python evaluate.py \
  --checkpoint outputs/act_cpu_run01/checkpoints/last/pretrained_model \
  --data data/validation
```

出力は、未学習エピソードの正しい観測を与えた場合の、関節目標角度の平均絶対誤差（rad）です。
**これは把持成功率ではありません。** 閉ループでは小さな誤差が積み重なるため、次のGazebo実行も必須です。

## 8. 学習済みモデルをGazeboで動かす

カメラ付きGazeboを再起動して、教師データと同じアーム・キューブ初期状態にします。
推論前に手動操作が終わっていることを確認します。

学習venvのターミナル：

```bash
cd /home/ubuntu/crane_cube_kit
source .venv-learning/bin/activate
python policy_server.py \
  --checkpoint outputs/act_cpu_run01/checkpoints/last/pretrained_model
```

別のROSターミナル（venvをactivateしない）：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /home/ubuntu/crane_cube_kit
/usr/bin/python3 robot.py deploy --seconds 60
```

通信先は同じコンテナ内の `127.0.0.1:8766` です。ポート公開は不要です。
学習時に保存した正規化・逆正規化の処理もチェックポイントから読み込みます。
各試行の開始時にはACT内部の履歴をリセットします。

`q` / Spaceで停止し、現在角度を目標に保持します。
観測が古い、推論が0.1秒のシミュレーション時間を超える、目標角度が1回で0.15 radを超えて飛ぶ場合も停止します。
速度制限による指令の改変を黙って行うのではなく、停止して初期状態・学習品質を確認する方式です。

CPU推論が間に合わない場合は、Gazeboを終了し、例えば次のように低速化して再起動します。

```bash
/usr/bin/python3 prepare_scene.py --rtf 0.1
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

学習・実行は同じ10 Hzのシミュレーション時間を維持します。実時間の実行速度だけを遅くします。
実行速度が遅いことと、学習精度が低いことは別に評価してください。

最初は固定配置で試行し、続いて収集データにない小さな位置変更でも試します。
初期配置、把持成功、目標高さ、2秒保持、落下、所要時間を試行ごとに記録してください。
定量的な高さ判定にはGazeboのwoodcube実測poseが必要です。この版では自動成功判定は実装していません。

## 検証範囲

- この配布物のPython構文、シェル構文、関節順変換、角度範囲、画像の行パディング・BGR変換、シーン生成、
  launch変換、エピソード検証をローカルで確認しています。
- LeRobot 0.4.3の配布wheelを確認し、使用するAPIと設定名を照合しています。
- ROS/Gazeboの起動、Ogre2描画、物理把持、CPU学習、チェックポイント読込、閉ループ推論はユーザー環境での確認が必要です。
- 5cm木製キューブの内部SDF（質量、衝突形状、摩擦、モデル原点）はFuel側のモデルに依存し、この版では変更していません。

## 参考

- CRANE+公式: https://github.com/rt-net/crane_plus
- LeRobot ACT v0.4.3: https://huggingface.co/docs/lerobot/v0.4.3/act
- LeRobotDataset v3: https://huggingface.co/docs/lerobot/v0.4.3/lerobot-dataset-v3

`prepare_scene.py` が生成するlaunchは、ユーザー環境のRT Corporation製Apache-2.0のlaunchを変換したものです。
パッケージには元のメッシュやFuelモデルを同梱していません。
