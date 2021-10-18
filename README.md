# 第３回自動運転AIチャレンジ
日本語 | [English](./README_EN.md)

本リポジトリでは[第３回自動運転AIチャレンジ](https://www.jsae.or.jp/jaaic/index.html)の参加者のための環境構築手順・大会ルール等、大会参加のために必要なデータを提供しています。

本大会の競技内容・ルールについては[RULE.md](./RULE.md)で確認お願いいたします。

本大会ではAutoware.Autoをベースとして自動走行の実装を行っていただきます。Autoware.Autoのチュートリアルを下記リンクに用意いたしましたのでご参考にしてください。
- [Introduction to Autoware.Auto for AI Challenge competitors (pdf)](doc/Introduction_to_Autoware_for_AI_Challenge_Competitors.pdf)
- [Introduction to Autoware.Auto for AI Challenge competitors (youtube)](https://www.youtube.com/watch?v=p8ay7KTOANM)

## 動作環境
本大会で使用していただくPCの動作環境として以下を推奨しております。
- OS: Ubuntu 20.04
- CPU: Intel Corei7(8コア)以上
- GPU:
  - NVIDIA Geforce RTX2080(GTX1080Ti)以上(GPU実装のあるROSノードを実装する場合)
  - NVIDIA Geforce GTX1080以上(GPU実装のあるROSノードを実装しない場合)
- メモリー: 32GB以上
- ストレージ	SSD 30GB 以上

上記のスペックのPCが用意できない場合、LGSVLシミュレータ動作PCとAutoware動作PCを分けて用意していただくことも可能です。環境はそれぞれ以下を推奨しております。

### LGSVLシミュレータ動作PC
- OS: Ubuntu 20.04もしくはWindows 10
- CPU: Intel Corei7(4コア)以上
- GPU: NVIDIA Geforce GTX 1080 以上
- メモリー: 16GB以上
- ストレージ	SSD 20GB 以上

### Autoware動作PC
- OS: Ubuntu 20.04
- CPU: Intel Corei7(4コア)以上
- GPU: NVIDIA Geforce GTX 1080 以上(GPU実装のあるROSノードを実装しないのであればGPUは不要です)
- メモリー: 16GB以上
- ストレージ	SSD 10GB 以上

## Autoware環境(ROS2環境)セットアップ
本大会でのROS2の実行環境はDockerコンテナ内に構築します。

事前に
- [Docker](https://docs.docker.jp/linux/index.html)
  - sudo無しで実行できるよう、インストール後に[こちらの手順](https://docs.docker.com/engine/install/linux-postinstall/)を行ってください。
  - この手順が正常に行われていない場合、以下のようなエラーがade実行時に発生します。
```
Got permission denied while trying to connect to the Docker daemon socket at ...
```
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#install-guide)
- [ADE](https://ade-cli.readthedocs.io/en/latest/)

のインストールを行ってください。

### 本レポジトリのclone
事前に[git lfs](https://packagecloud.io/github/git-lfs/install)をインストールしてください。

```
git clone https://github.com/AutomotiveAIChallenge/aichallenge2021
```

### ROS2+Autoware.Autoのインストール
Autoware.AutoはADEを使用してDocker環境でセットアップされることが推奨されています。(https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-ade.html)

公式のDockerファイルに加えてlgsvl_bridgeを導入する必要があるため、本リポジトリでADE向けのDockerファイルを提供しています。

```
bash setup_autoware.sh
```

でインストールしてください。
これによってAutoware.Auto(1.0.0)のDockerイメージが追加され、`autoware/adehome`がADEのホームディレクトリに設定されます。
ローカル環境で上記スクリプトを使用せずにAutoware.Autoをセットアップしていただいても構いませんが、Autoware.Autoのバージョンは1.0.0を使用してください。


### ADEコンテナ起動
```
# autoware/adehomeで実行

# ADEコンテナ起動
ade start --update
# ADEコンテナに入る
ade enter
```

## サンプルコード(ROS2パッケージ)セットアップ

### サンプルコードについて
参加者の皆様にはシナリオを遂行するROS2パッケージを作成していただきますが、本リポジトリ内でそのベースとなるサンプルコードとして`autoware/adehome/aichallenge_ws/src`に以下のROS2パッケージを提供しております。
- aichallenge_launch
  - 大元のlaunchファイル`aichallenge.launch.py`を含んでいます。すべてのROS2ノードはこのlaunchファイルから起動されます。
- aichallenge_eval
  - スコア算出用のパッケージです。
- aichallenge_msgs
  - メッセージ定義を含みます。
- vehicle_pose_publisher
    - 今回走行する点群地図の特徴量が少なく、NDTマッチングが外れやすいため、localizationの正解データを提供しております。
    - `/aichallenge/vehicle_pose`をsubscribeしていただくことで車両位置を取得していただけます。
- aichallenge_submit
  - このディレクトリの内容は自由に変更していただいて構いません。
  - 提出時にはこのディレクトリの内容のみ提出していただきますので、参加者の皆さまが実装されたROS2パッケージはすべてこのディレクトリ内に配置してください。配布段階で以下のパッケージを含んでいます。
  - aichallenge_submit_launch
    - `aichallenge_submit_launch.launch.py`が大元のlaunchファイル`aichallenge.launch.py`から呼び出されますので、このlaunchファイルを適宜改修して皆様が実装されたROS2ノードが起動されるように設定してください。
  - aichallenge_sample
    - サンプルの自動走行実装です。
    - 最大35kmでlaneletの中心に沿って進むように設定されています。車両回避は設定されておらず、前方に車両が近づいた場合はブレーキを踏む挙動をします。
  - sample_localizer
    - `/aichallenge/vehicle_pose`を元にlocalizationを行うパッケージです。

### サンプルコードビルド
```
# ADEコンテナ内で
source /opt/AutowareAuto/setup.bash
cd aichallenge_ws
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
```

皆様に作成していただいたROS2パッケージについても`aichallenge_ws/src/aichallenge_submit`以下に配置していただき、上記手順でビルドできるようにしてください。

### サンプルコード起動
```
# ADEコンテナ内で
source /opt/AutowareAuto/setup.bash
source ~/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.py
```
これによってrvizが起動しますが、完全な動作にはlgsvl_bridgeを使用してシミュレータと通信することが必要です。別のターミナルを開き、以下のコマンドでlgsvl_bridgeを起動させておいてください。
```
# ADEコンテナに入る
ade enter

# ADEコンテナ内で
source ~/aichallenge_ws/install/setup.bash
lgsvl_bridge
```
ここまででAutoware側の設定・実行は完了です。セットアップが正常に行われていれば、rvizには車両モデルとLaneletが表示されます。
![画面](/image/rviz.png)

## シミュレータ環境セットアップ
### シミュレータのバイナリダウンロード
下記リンクからシミュレータのバイナリをダウンロードします。

[simulator.zip](https://drive.google.com/file/d/1Q-MVYxtuSPMz8obmvA6FwbuaP6qJA0ul/view?usp=sharing)

上記リンクにあるzipファイルを解凍して頂くと  

- simulator_linux(linux環境用シミュレータ)
- simulator_windows(windows環境用シミュレータ)

と環境に応じて用意しております。実行環境に合わせて各ディレクトリ内のシミュレータをご使用下さい。

*本シミュレータに関する免責事項について
参加者のみなさまが本ファイルをダウンロードすることによって、みなさまのコンピュータ、またはネットワーク環境  
等に支障・障害が生じた場合、本大会運営事務局はいかなる理由によるものでも一切責任を負いません。  
また、これらの事象によって生じた損害等についても、本大会運営事務局は一切責任を負いません。*

### アカウント登録
https://wise.svlsimulator.com/sign-in

上記リンクからLGSVLのアカウントを登録し、ログインします。

### クラスタ登録
ダウンロードしたバイナリを起動します。
![画面](/image/initial.png)
LINK TO CLOUDをクリックするとブラウザが開きクラスタ登録画面が表示されるので、適当な名前を入力しCreate clusterを押します。
![画面](/image/cluster.png)

### 地図、車両、プラグイン登録
Store → Maps → IndianapolisMotorSpeedwayの+ボタンを押します。
![画面](/image/map.png)
Store → Vehicles → DallaraIL15の+ボタンを押します。
![画面](/image/vehicle.png)
Store → Plugins → Timer Sensorの+ボタンを押します。
![画面](/image/plugin.png)

### センサ設定の追加
先程追加したDallaraIL15の画面を開き、画像赤丸のボタンからSensor Configurationsを開きます。
![画面](/image/sensorconfig_open.png)
Add New Configurationを押します。
![画面](/image/sensorconfig_addnew.png)
名前をAutoware.Auto、BridgeをROS2に設定して、Applyを押します。
![画面](/image/sensorconfig_name.png)
画像赤丸のボタン(Upload sensor configuration)を押します。
![画面](/image/sensorconfig_upload.png)
本リポジトリのDallaraIL15_Autoware.Auto.jsonを選択します。
![画面](/image/sensorconfig_select.png)
Saveボタンを押します。
![画面](/image/sensorconfig_save.png)


### シミュレーション設定作成・開始
Simulations → Add Newをクリックした後、Select Clusterからクラスタを登録したものに設定します。
![画面](/image/setting1.png)
Nextを押し、Runtime TemplateにAPI Onlyを選択します。
![画面](/image/setting2.png)
Next → Next → Publishを押し、以下の画面が表示されたらRun Simulationを押します。これによってシミュレーションが開始されます。
![画面](/image/simulation_run.png)
シミュレータの画面にはAPI Ready!と表示されます。
![画面](/image/api_ready.png)

## シナリオ実行・タイム取得
シミュレータ内でのシナリオの実行(自車両追加・NPC車両の動き制御・地図読み込み等)は[pythonスクリプト](./scenario/scenario.train.py)で行います。

シナリオを実行する前に以下を確認してください。
- シミュレータ
  - API Readyの状態にあること
- ROS2
  - `aichallenge.launch.py`がlaunchされ、rvizが表示されていること
  - `lgsvl_bridge`が起動していること

### LGSVL Simulator Python APIのセットアップ
シミュレータのシナリオ実行にはLGが提供しているPythonAPI(2021.2)を使用します。シナリオを実行するPCはシミュレータ動作用PCを推奨しております。

```
git clone https://github.com/lgsvl/PythonAPI
cd PythonAPI
git checkout refs/tags/2021.2
```
READMEに従ってPythonAPIをインストールしてください。

### シナリオ実行
まず[センサ設定の追加](#センサ設定の追加)の項目で追加したセンサ設定(Autoware.Auto)のconfiguration idを取得します。画像の赤丸部分のボタンをクリックすることでクリップボードにコピーされます。取得した値はコマンド実行時の`vehicle_id`オプションで使用します。
![画面](/image/configuration_id.png)

以下のコマンドでシナリオを実行します。
```
cd scenario
# bridgeにはAutoware動作用PCのIPを、hostにはシミュレータ動作用PCのIPを入力してください。
python3 scenario.train.py --vehicle_id={取得したconfiguration id} --bridge=127.0.0.1 --host=127.0.0.1
```

### タイム取得
競技内容・ルールに関しては[RULE.md](./RULE.md)をご確認下さい。

タイムは`/aichallenge/score`トピックにpublishされます。
確認方法は下記の通りです。

シミュレータを開始した後にtopicをechoさせる事で確認を取る事が出来ます。
```
# ADEコンテナ内で

source ~/aichallenge_ws/install/setup.bash
ros2 topic echo /aichallenge/score
```


又、publishされるタイミングは下記の場合になります。
1. ゴール到達時
2. scenario.train.pyを実行し、5分間経過した場合

`/aichallenge/score`がpublishされた時点でタイムの算出となります。

`/aichallenge/score`の内容については下記の通りとなります
- time : ペナルティ込みの時間です。この値が競技における正式なタイムになります。
- rawTime : シナリオが開始されてからゴール到着までにかかった時間です。
- hasFinished : ゴールに到達したら1, それ以外の場合は0になります。
- contactPenalty : 接触によるペナルティです。(1回につき5秒タイムに上乗せ)
- trackLimitPenalty : コースアウトによるペナルティです。(コース外にいた時間がタイムに上乗せされます。)


# お問い合わせ

## 更新等の通知に関して
githubの更新などがある場合は、以下のURLのissueに新たにコメントします。

本issueをsubscribeいただければ、更新時に通知されます（通知をオンにしてください）。

https://github.com/AutomotiveAIChallenge/aichallenge2021/issues/1

## お問い合わせ受付に関して
競技内容、リポジトリ内容物等に関するお問い合わせについては、github上のissueにてお願いします。質問は日本語、英語どちらでも構いません。質問者様は質問が解決した際issueをcloseしてください。

質問内容は競技内容に直接関係あるものに限ります。ソフトウェアの使用方法に関するご質問については、公平性の観点から回答いたしかねます。

各issueでの質問については、基本的に2営業日以内に回答いたします。ただし、検討に時間を要する質問や質問数が多い場合等については、2営業日以上いただく可能性があることはご理解ください。
