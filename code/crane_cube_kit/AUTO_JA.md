# 自動で持ち上げて記録する版

**更新:** 教師動作の肘姿勢を揃える制約を追加しました。
最新の収集手順は `BRANCH_FIX_JA.md` を優先し、`--out raw_auto_branch` で新しいデータを分けてください。

手動の関節操作は不要です。既存の公式pick_and_placeサンプルで把持できた環境を対象に、
Humble版公式サンプルの接近・把持手順を使い、運搬・配置の代わりに直立姿勢へ移って保持します。
**この変更版のビルド・実行・最上部まで保持できるかは、お使いのROS環境で確認が必要です。**
こちらではROS/GazeboがなくC++のコンパイルや物理動作は確認できていません。

## 1. 改良版を上書き展開

新しいZIPを `/home/ubuntu/crane_cube_cpu_kit.zip` としてコンテナ内に置き、実行します。

```bash
cd /home/ubuntu
unzip -o crane_cube_cpu_kit.zip
```

ソースファイルだけを上書きします。既に収集したデータ・学習結果はZIPに含まれず、削除しません。

## 2. シーンを初期状態にする

`robot.py teach` を実行中なら `q` で終了します。公式pick_and_placeも終了させてください。
サンプルでキューブが移動しているので、今のGazeboをCtrl+Cで終了し、再起動します。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/airobot_ws/install/setup.bash
ros2 launch /home/ubuntu/crane_cube_kit/generated/cube_camera.launch.py
```

前の手順で生成したカメラ付きlaunchをそのまま使います。シーンの再生成は不要です。

## 3. 別ターミナルで自動実行・記録

同じコンテナの別ターミナルで実行してください。

```bash
cd /home/ubuntu/crane_cube_kit
bash collect_auto.sh
```

これだけで次を行います。

1. 小さな追加ROSパッケージを `teacher_ws` に作り、colconでビルド。
2. 既存CRANE+のMoveIt設定を読み込み。
3. 記録を開始し、直立 → home → 指を開く → 接近 → 下降 → 把持。
4. キューブを少し持ち上げ、homeを経由して直立姿勢へ移動。
5. グリッパーを閉じたまま、シミュレーション時間で2秒保持して終了。
6. 成功確認を表示。

最後に次の質問が出ます。

```text
キューブを掴んだまま上まで持ち上げ、2秒保持できましたか？ [y/N]:
```

**実際に保持できたら `y` とEnter**で保存します。落とした場合や空振りならEnterだけで破棄します。
途中のキー操作や、関節角度の調整は不要です。記録は `raw_auto_episodes/` に保存されます。
初回はC++のビルドに時間がかかります。既存のCRANE+ソース・インストールは変更しません。
以後のビルドは変更がなければ差分チェックのみです。

コンパイルエラー、計画失敗、実行失敗はログに表示して停止します。
Ctrl+Cで中断できます。途中の失敗記録を成功データとして保存しません。
手動で微調整して続行する必要はありません。エラーが出た場合は、そのエラーを共有してください。

## 何回収集するか

まずは1回、最後まで持ち上がるかを確認してください。
繰り返す際は、毎回Gazeboを再起動してキューブを初期位置へ戻してから `bash collect_auto.sh` を実行します。
1回の成功でモデルが完成するわけではありません。

この版は既定位置の自動教師です。キューブを自由に移動させて掴むものではありません。
同じ固定配置だけで集めたデータは、まず固定配置での模倣学習の動作確認に使います。
位置の変化にも対応させるには、自動教師側の把持座標をシーンと一緒に変える追加作業が必要です。

## 学習への接続

2回以上成功したら、最初の処理確認として以下を実行できます。
学習venvは前のREADMEの `bash setup_learning.sh` で作成しておきます。

```bash
cd /home/ubuntu/crane_cube_kit
source .venv-learning/bin/activate
python convert_dataset.py --raw raw_auto_episodes --out data_auto
bash train_cpu.sh data_auto/train outputs/act_auto_smoke 200
```

200ステップは処理確認用で、把持成功を保証しません。
学習用と検証用にエピソードで分割します。以降の本学習・推論はREADME_JA.mdと同じです。
データパスは `data_auto/train`、検証は `data_auto/validation` に読み替えてください。
追加データで再変換するときは `--out data_auto_v2` など新しい出力先を指定します。

## 記録する操作指令について

画像、実測関節状態、MoveItが計画した軌道の参照位置を記録します。
**実測関節状態をそのままactionとしてコピーする方法ではありません。**
アーム・グリッパーの計画軌道を時刻付きで受信し、各時刻の0.2秒先の参照位置を線形補間してactionにします。
これは配布済み推論クライアントの「絶対目標角度を0.2秒で指令する」形式に合わせた初期実装です。
計画の発行からコントローラー開始までの遅延は厳密には補正していません。
コントローラー内部の補間と一致する保証もないため、Gazeboでの閉ループ評価が必要です。
手動版と意味の異なるactionを混ぜないため、保存先も `raw_auto_episodes` に分けています。

成功ラベルは操作者がキューブを見て付けます。接触・キューブの高さを自動判定したものではありません。
MoveItの計画成功だけを物理把持成功とは扱いません。

### 配布前の確認

Python/シェルの構文、起動ファイルの変換、関節順、画像処理、記録データ検査、
教師軌道の参照位置補間など8項目のテストを確認しています。
C++のコンパイル、ROS起動、最上部への移動と物理保持は未検証です。

### 参考・ライセンス

教師の接近・把持手順はCRANE+ Humbleの公式例に基づきます。
https://github.com/rt-net/crane_plus/tree/humble/crane_plus_examples

Copyright 2020 RT Corporation / Apache-2.0。変更点はソース冒頭に明記しています。
LICENSE-CRANE.txtに原ライセンスを同梱しています。
