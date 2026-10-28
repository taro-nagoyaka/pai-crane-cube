# CRANE+ キューブ把持学習 PAI最終課題

## 課題と到達点

GazeboのCRANE+で木製キューブを掴み、アームを直立付近まで上げて保持するタスク。
AIはLeRobotのACTを使用し、RGB画像と関節角から目標関節角を推論する。
教師データ収集、CPU学習、オフライン評価、Gazeboでの推論実行まで実装した。
教師は持ち上げ・保持まで成功したが、学習済み方策は把持付近で停滞し、
目標全体の達成は未確認・未達である。教師動作とAIの成功を混同しない。
最終状態は2026年10月26日時点。

説明資料は [PDF](report/crane_cube_report.pdf) と [編集用Word](report/crane_cube_report.docx)。
GitHubでは、このREADMEがある階層をリポジトリのルートとし、VisibilityはPublicに設定する。
提出時のPDF名は `説明資料_PAI最終課題_omnicampusアカウント名.pdf` にする。
アカウント名は実際のomnicampus登録名を用い、GitHubアカウント名と同一とは限らない。

## 内容

- report/crane_cube_report.docx：編集可能な簡単なレポート。
- report/crane_cube_report.pdf：印刷・閲覧用の同内容レポート。
- report/crane_cube_report.txt：同内容のプレーンテキスト。
- code/crane_cube_kit/：使用したPython・C++・シェル、生成スクリプト、テスト、操作資料。
- evidence/：受領した検証予測CSV、6000ステップの実行CSV、集計した評価結果。
- export_from_container.py：既存コンテナの重み・生成ファイル・ログを追加して提出用ZIPを作る。

ダウンロード用の基本ZIPに学習済み重み・生データ・変換済みデータセットは含まれない。
コンテナ内の実ファイルを下記手順で追加できる。export_from_container.pyで作成したfull版の
実際の同梱状況はsubmission_manifest.jsonのmodel_included / training_data_includedを参照する。
モデルの代用品や架空の重みは入れていない。
18000ステップの実行CSVは未受領のため含まず、最終動作結果は操作者の目視報告として記載した。
学習ログの要約値は evidence/evaluation_summary.json に出典区分とともに記録した。

## 動作環境

- Windows上のWSL2、既存Dockerコンテナ pai_ros2。
- ベースイメージ：airobotbook/ros2-desktop-ai-robot-book-humble:latest。
  使用時のイメージdigestは記録していないため、将来のlatestの一致は保証しない。
- Ubuntu / ROS 2 Humble / Ignition Gazebo、MoveIt 2、ros_gz_bridge、ign_ros2_control。
- 既存ROSワークスペース：/home/ubuntu/airobot_ws。
- 依存パッケージ：crane_plus_description、crane_plus_gazebo、crane_plus_control、
  crane_plus_moveit_config、crane_plus_examples。
- ROS用Python：/usr/bin/python3。学習用：Python 3.10の .venv-learning。
- LeRobot 0.4.3、CPU版PyTorch 2.7.1、torchvision 0.22.1。

ROSパッケージ、メッシュ、FuelのTable / Wood cube 5cm等のモデルは外部依存であり、
本ZIPはそれらを再配布しない。既存コンテナにインストールされたものを使用する。
インターネットからのFuelモデル取得が初回に必要になることがある。
生成launch・world・CMakeLists.txt・package.xmlはprepare_scene.py / prepare_teacher.pyで再生成する。
既存のCRANE+公式パッケージを直接変更しない。

## コンテナ内の学習済みファイルを追加する

このZIPを /home/ubuntu に展開し、既存の学習ワークスペースはそのまま残す。
次の標準ライブラリだけのコマンドで、別名の提出用ZIPを作る。

```bash
/usr/bin/python3 /home/ubuntu/crane_cube_submission/export_from_container.py \
  --workspace /home/ubuntu/crane_cube_kit
```

重み、正規化等の前処理・後処理、実際の生成launch、診断結果、実行CSV、
環境ロックファイルがあれば同梱する。元ファイルを移動・削除せず、出力が存在すれば停止する。
デフォルトでは学習データを同梱しない。再学習用データも提出する場合は末尾に
`--include-data` を付ける。大きなZIPになるため、提出先の容量条件を確認する。

出力例：/home/ubuntu/crane_cube_submission_full_YYYYMMDD_HHMMSS.zip。
ファイル一覧とSHA-256はsubmission_manifest.jsonへ保存する。
学習済み重みの取得先を変える場合は `--checkpoint /実際の/pretrained_model` を指定する。
出力内の推論用配置名はact_varied_novae_18000に固定なので、最終モデルを指定すること。

## 再実行手順

以下は展開した code/crane_cube_kit を /home/ubuntu/crane_cube_kit に配置した例。
既存の作業フォルダへの上書きは不要であり、提出物を試す場合は別のコンテナ／コピーを推奨する。
全て同じコンテナ内で実行する。各ROS用ターミナルでは次を読み込む。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
cd /home/ubuntu/crane_cube_kit
```

### 収集

```bash
bash collect_batch.sh --out raw_auto_varied --episodes 24 --max-attempts 48
/usr/bin/python3 check_branch.py --raw raw_auto_varied
```

Gazeboの起動・初期化と教師実行は収集側が行う。別のGazeboやモデルを同時実行しない。
自動選別は高さ・近接・安定保持・直立と記録品質による。接触力による把持の証明ではない。
記録できた除外データはbatch_logs内でsuccess=Falseとして保管し、学習フォルダには混ぜない。
姿勢やキューブ位置の変化、再開方法、判定式はcode/crane_cube_kit/BATCH_AUTO_JA.mdを参照。

### 変換とCPU学習

学習時には、既存のraw_auto_branchの7件と追加24件を統合した。

```bash
/usr/bin/python3 merge_raw.py --inputs raw_auto_branch raw_auto_varied --out raw_combined_v1
# 初回のみ：bash setup_learning.sh
source .venv-learning/bin/activate
python convert_dataset.py --raw raw_combined_v1 --out data_varied_v1
bash train_varied_cpu.sh data_varied_v1/train outputs/act_varied_novae_6000 6000
bash resume_training.sh
```

学習25エピソード6246フレーム、検証6エピソード1517フレーム。
train_varied_cpu.sh / resume_training.shは実行したコマンドを提出時にシェル化した補助ファイル。
モデル構造128次元・4ヘッド・2エンコーダ・1デコーダ、chunk_size=10、n_action_steps=1、
VAEなし、batch_size=2、CPU、学習6000ステップから合計18000ステップへ再開。
旧train_cpu.shは初期検討用のVAEあり設定なので、最終実験の再現には上記補助ファイルを使う。

### 評価

```bash
python diagnose_model.py \
  --checkpoint outputs/act_varied_novae_18000/checkpoints/last/pretrained_model \
  --data data_varied_v1 --out diagnostics_varied_novae18000
```

出力先が既にある場合は別名を指定する。オフライン評価は正しい教師観測を入力した
絶対目標関節角のMAEであり、Gazeboの把持成功率を表さない。
検証時の動作中フレームは、同一エピソードで隣接関節角の最大変化が0.005 radを超えるもの。

### AIによる推論

ターミナル1：ROS環境を読み込んでGazebo起動。

```bash
/usr/bin/python3 prepare_scene.py
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

ターミナル2：学習用仮想環境でモデルサーバーを起動したままにする。

```bash
cd /home/ubuntu/crane_cube_kit
source .venv-learning/bin/activate
python policy_server.py \
  --checkpoint outputs/act_varied_novae_18000/checkpoints/last/pretrained_model \
  --temporal-ensemble-coeff 0.01
```

ターミナル3：ROS環境を読み込み、実行・記録する。

```bash
cd /home/ubuntu/crane_cube_kit
/usr/bin/python3 robot.py deploy --seconds 45 \
  --trace "deploy_varied18000_$(date +%Y%m%d_%H%M%S).csv"
```

q / Spaceで停止・現在角度保持。試行前にGazeboを終了・再起動して初期状態へ戻す。
関節限界、観測の古さ、推論遅延、指令の急変を検査する。MoveItによる衝突回避は
教師側にのみあり、モデル出力の直接実行側には経路全体の衝突保証がない。
本提出はシミュレーション用で、実機へそのまま適用することを目的としていない。

## ファイルの役割

| 処理 | 主なコード |
| --- | --- |
| カメラ付きシーン生成 | prepare_scene.py |
| 教師パッケージ生成・教師動作 | prepare_teacher.py、lift_teacher.cpp |
| 自動収集と選別 | collect_batch.sh、collect_batch.py、batch_record.py、batch_support.py、batch_archive.py |
| 手動確認付き収集 | collect_auto.sh、auto_record.py、teacher_reference.py |
| 検査・統合・変換 | check_branch.py、merge_raw.py、convert_dataset.py |
| 学習環境・最終学習 | setup_learning.sh、train_varied_cpu.sh、resume_training.sh |
| AI推論・ロボット制御 | policy_server.py、robot.py、common.py |
| 診断・教師再生 | diagnose_model.py、evaluate.py、replay_teacher.py |
| ROS不要の単体テスト | test_core.py、test_batch.py |

## 確認範囲

ローカルの23件の単体テストで画像変換、関節順序、参照補間、成功選別、除外保存等を確認。
ROS/Gazeboでのビルド・収集・CPU学習・推論は操作者のコンテナで実施した。
提出時に追加した学習補助シェルと梱包スクリプトは構文／模擬ファイルで確認し、
ユーザー環境の再学習を提出準備のために再実行してはいない。

## 出典とライセンス

- CRANE+公式：https://github.com/rt-net/crane_plus
  教師C++およびlaunch生成は公式サンプルを基に改変。著作権表示とApache-2.0表記を保持。
  code/crane_cube_kit/LICENSE-CRANE.txtも参照。
- LeRobot：https://github.com/huggingface/lerobot
  利用バージョン0.4.3。依存コードはsetup_learning.shで取得し、本ZIPでは再配布しない。
- ACT：https://arxiv.org/abs/2304.13705
  画像・状態から動作列を予測する模倣学習手法として利用。

外部依存のコード・モデル・データはそれぞれの配布条件に従う。
本レポートは成果を提出するための整理であり、学習済み方策がタスクを完遂したとの主張ではない。
