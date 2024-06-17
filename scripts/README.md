## フォルダ説明

| フォルダ名 | 内容 | 使用頻度 |
| --- | --- | --- |
| scripts/ | シミュレーションや実験に使用するコード | ☆☆☆ |
| scripts/archive | 開発途中に作成したコード。もう使用しないコード。 | ☆ |
| scripts/px4_sim_pkg | ライブラリ化したクラス群。 | ☆☆☆ |
| scripts/test | 開発中に使用したテストコード。簡単な動作の検証を行う。 | ☆☆ |

## ファイル説明

|  | ファイル名 | 内容 |
| --- | --- | --- |
| scripts/ | dmd_train_circle.py | XY, XZ平面の円飛行でA行列を学習。学習したA行列をcsv出力 |
| scripts/ | dmd_test_circlexy.py | dmd_train_circle.pyで学習したA行列を読み込んでcirclexyの状態予測をオンラインで実行 |
| scripts/ | dmd_test_eightxz.py | dmd_train_circle.pyで学習したA行列を読み込んでeightxzの状態予測をオンラインで実行 |
| scripts/ | dmd_test_rightleft.py | dmd_train_circle.pyで学習したA行列を読み込んでrightleftの状態予測をオンラインで実行 |
| scripts/ | dmd_test_updown.py | dmd_train_circle.pyで学習したA行列を読み込んでupdownの状態予測をオンラインで実行 |
| scripts/ | guided_circle_test.py | 指定された時間、円軌道を実行 |
| scripts/ | guided_hover_test.py | 指定された時間、ホバリングを実行 |
| scripts/ | pose_transformer.py | OptiTrack→ROSの座標変換を行う自作関数（要デバッグ）。Launchファイルでpose_transformer.pyを実行。その後、guided_hover_test.pyを実行することで、正常にホバリングが行われるかテスト。 |
| scripts/archive/guided_dmd | guided_dmd_*.py | 円軌道でdmdを実行しA行列を学習。その後*の軌道（circle, eightxy, rightleft, updown）で検証。 |
| scripts/px4_sim_pkg | DMD.py | DMDを行う関数。 |
| scripts/px4_sim_pkg | MavrosNode.py | 飛行に関するセットアップを行う関数。 |
| scripts/px4_sim_pkg | RTDMD.py | RTDMDを行うクラス。 |
| scripts/px4_sim_pkg | Trajectory.py | 軌道生成するクラス。 |
|  |  |  |
|  |  |  |
|  |  |  |
|  |  |  |