# 第３回自動運転 AI チャレンジ

日本語 | [English](./README_EN.md)

本リポジトリでは[第３回自動運転 AI チャレンジ](https://www.jsae.or.jp/jaaic/index.html)の参加者のための環境構築手順・大会ルール等、大会参加のために必要なデータを提供しています。

本大会の競技内容・ルールについては[RULE.md](./RULE_JAP.md)で確認お願いいたします。

本大会では Autoware.Auto をベースとして自動走行の実装を行っていただきます。Autoware.Auto のチュートリアルを下記リンクに用意いたしましたのでご参考にしてください。

- [Introduction to Autoware.Auto for AI Challenge competitors (pdf)](doc/Introduction_to_Autoware_for_AI_Challenge_Competitors.pdf)
- [Introduction to Autoware.Auto for AI Challenge competitors (youtube)](https://www.youtube.com/watch?v=p8ay7KTOANM)

## 動作環境

本大会で使用していただく PC の動作環境として以下を推奨しております。

- OS: Ubuntu 20.04
- CPU: Intel Corei7(8 コア)以上
- GPU:
  - NVIDIA Geforce RTX2080(GTX1080Ti)以上(GPU 実装のある ROS ノードを実装する場合)
  - NVIDIA Geforce GTX1080 以上(GPU 実装のある ROS ノードを実装しない場合)
- メモリー: 32GB 以上
- ストレージ SSD 30GB 以上

上記のスペックの PC が用意できない場合、LGSVL シミュレータ動作 PC と Autoware 動作 PC を分けて用意していただくことも可能です。環境はそれぞれ以下を推奨しております。

### LGSVL シミュレータ動作 PC

- OS: Ubuntu 20.04 もしくは Windows 10
- CPU: Intel Corei7(4 コア)以上
- GPU: NVIDIA Geforce GTX 1080 以上
- メモリー: 16GB 以上
- ストレージ SSD 20GB 以上

### Autoware 動作 PC

- OS: Ubuntu 20.04
- CPU: Intel Corei7(4 コア)以上
- GPU: NVIDIA Geforce GTX 1080 以上(GPU 実装のある ROS ノードを実装しないのであれば GPU は不要です)
- メモリー: 16GB 以上
- ストレージ SSD 10GB 以上

## Autoware 環境(ROS2 環境)セットアップ

本大会での ROS2 の実行環境は Docker コンテナ内に構築します。

事前に

- [Docker](https://docs.docker.jp/linux/index.html)
  - sudo 無しで実行できるよう、インストール後に[こちらの手順](https://docs.docker.com/engine/install/linux-postinstall/)を行ってください。
  - この手順が正常に行われていない場合、以下のようなエラーが ade 実行時に発生します。

```
Got permission denied while trying to connect to the Docker daemon socket at ...
```

- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#install-guide)
- [ADE](https://ade-cli.readthedocs.io/en/latest/)

のインストールを行ってください。

### 本レポジトリの clone

事前に[git lfs](https://packagecloud.io/github/git-lfs/install)をインストールしてください。

```
sudo apt install git-lfs
git lfs install --skip-repo

git clone https://github.com/AutomotiveAIChallenge/aichallenge2021
```

PCD ファイルが LFS サーバーから正常にダウンロードされていることを確認してください。

```
ls -lh aichallenge2021/autoware/adehome/aichallenge_ws/src/aichallenge_launch/data/IndianapolisMotorSpeedway.pcd
```

正常にダウンロードされている場合、ファイルサイズは約 300MB になります。

### ROS2+Autoware.Auto のインストール

Autoware.Auto は ADE を使用して Docker 環境でセットアップされることが推奨されています。(https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-ade.html)

公式の Docker ファイルに加えて lgsvl_bridge を導入する必要があるため、本リポジトリで ADE 向けの Docker ファイルを提供しています。

```
bash setup_autoware.sh
```

でインストールしてください。
これによって Autoware.Auto(1.0.0)の Docker イメージが追加され、`autoware/adehome`が ADE のホームディレクトリに設定されます。
ローカル環境で上記スクリプトを使用せずに Autoware.Auto をセットアップしていただいても構いませんが、Autoware.Auto のバージョンは 1.0.0 を使用してください。

### ADE コンテナ起動

```
# autoware/adehomeで実行

# ADEコンテナ起動
ade start --update
# ADEコンテナに入る
ade enter
```

## サンプルコード(ROS2 パッケージ)セットアップ

### サンプルコードについて

参加者の皆様にはシナリオを遂行する ROS2 パッケージを作成していただきますが、本リポジトリ内でそのベースとなるサンプルコードとして`autoware/adehome/aichallenge_ws/src`に以下の ROS2 パッケージを提供しております。

- aichallenge_launch
  - 大元の launch ファイル`aichallenge.launch.py`を含んでいます。すべての ROS2 ノードはこの launch ファイルから起動されます。
- aichallenge_eval
  - スコア算出用のパッケージです。
- aichallenge_msgs
  - メッセージ定義を含みます。
- vehicle_pose_publisher
  - 今回走行する点群地図の特徴量が少なく、NDT マッチングが外れやすいため、localization の正解データを提供しております。
  - `/aichallenge/vehicle_pose`を subscribe していただくことで車両位置を取得していただけます。
- aichallenge_submit
  - このディレクトリの内容は自由に変更していただいて構いません。
  - 提出時にはこのディレクトリの内容のみ提出していただきますので、参加者の皆さまが実装された ROS2 パッケージはすべてこのディレクトリ内に配置してください。配布段階で以下のパッケージを含んでいます。
  - aichallenge_submit_launch
    - `aichallenge_submit_launch.launch.py`が大元の launch ファイル`aichallenge.launch.py`から呼び出されますので、この launch ファイルを適宜改修して皆様が実装された ROS2 ノードが起動されるように設定してください。
  - aichallenge_sample
    - サンプルの自動走行実装です。
    - 最大 35km で lanelet の中心に沿って進むように設定されています。車両回避は設定されておらず、前方に車両が近づいた場合はブレーキを踏む挙動をします。
  - sample_localizer
    - `/aichallenge/vehicle_pose`を元に localization を行うパッケージです。

### サンプルコードビルド

```
# ADEコンテナ内で
source /opt/AutowareAuto/setup.bash
cd aichallenge_ws
rosdep update
rosdep install -y -r -i --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build
```

皆様に作成していただいた ROS2 パッケージについても`aichallenge_ws/src/aichallenge_submit`以下に配置していただき、上記手順でビルドできるようにしてください。

### サンプルコード起動

```
# ADEコンテナ内で
source /opt/AutowareAuto/setup.bash
source ~/aichallenge_ws/install/setup.bash
ros2 launch aichallenge_launch aichallenge.launch.py
```

これによって rviz が起動しますが、完全な動作には lgsvl_bridge を使用してシミュレータと通信することが必要です。別のターミナルを開き、以下のコマンドで lgsvl_bridge を起動させておいてください。

```
# ADEコンテナに入る
ade enter

# ADEコンテナ内で
source ~/aichallenge_ws/install/setup.bash
lgsvl_bridge
```

ここまでで Autoware 側の設定・実行は完了です。セットアップが正常に行われていれば、rviz には車両モデルと Lanelet が表示されます。
![画面](/image/rviz.png)

## シミュレータ環境セットアップ

### シミュレータのバイナリダウンロード

下記リンクからシミュレータのバイナリをダウンロードします。

[simulator.zip](https://drive.google.com/file/d/1Q-MVYxtuSPMz8obmvA6FwbuaP6qJA0ul/view?usp=sharing)

上記リンクにある zip ファイルを解凍して頂くと

- simulator_linux(linux 環境用シミュレータ)
- simulator_windows(windows 環境用シミュレータ)

と環境に応じて用意しております。実行環境に合わせて各ディレクトリ内のシミュレータをご使用下さい。

_本シミュレータに関する免責事項について
参加者のみなさまが本ファイルをダウンロードすることによって、みなさまのコンピュータ、またはネットワーク環境  
等に支障・障害が生じた場合、本大会運営事務局はいかなる理由によるものでも一切責任を負いません。  
また、これらの事象によって生じた損害等についても、本大会運営事務局は一切責任を負いません。_

### アカウント登録

https://wise.svlsimulator.com/sign-in

上記リンクから LGSVL のアカウントを登録し、ログインします。

### クラスタ登録

ダウンロードしたバイナリを起動します。
![画面](/image/initial.png)
LINK TO CLOUD をクリックするとブラウザが開きクラスタ登録画面が表示されるので、適当な名前を入力し Create cluster を押します。
![画面](/image/cluster.png)

### 地図、車両、プラグイン登録

Store → Maps → IndianapolisMotorSpeedway の+ボタンを押します。
![画面](/image/map.png)
Store → Vehicles → DallaraIL15 の+ボタンを押します。
![画面](/image/vehicle.png)
Store → Plugins → Timer Sensor の+ボタンを押します。
![画面](/image/plugin.png)

### センサ設定の追加

先程追加した DallaraIL15 の画面を開き、画像赤丸のボタンから Sensor Configurations を開きます。
![画面](/image/sensorconfig_open.png)
Add New Configuration を押します。
![画面](/image/sensorconfig_addnew.png)
名前を Autoware.Auto、Bridge を ROS2 に設定して、Apply を押します。
![画面](/image/sensorconfig_name.png)
画像赤丸のボタン(Upload sensor configuration)を押します。
![画面](/image/sensorconfig_upload.png)
本リポジトリの DallaraIL15_Autoware.Auto.json を選択します。
![画面](/image/sensorconfig_select.png)
Save ボタンを押します。
![画面](/image/sensorconfig_save.png)

### シミュレーション設定作成・開始

Simulations → Add New をクリックした後、Select Cluster からクラスタを登録したものに設定します。
![画面](/image/setting1.png)
Next を押し、Runtime Template に API Only を選択します。
![画面](/image/setting2.png)
Next → Next → Publish を押し、以下の画面が表示されたら Run Simulation を押します。これによってシミュレーションが開始されます。
![画面](/image/simulation_run.png)
シミュレータの画面には API Ready!と表示されます。
![画面](/image/api_ready.png)

## シナリオ実行・タイム取得

シミュレータ内でのシナリオの実行(自車両追加・NPC 車両の動き制御・地図読み込み等)は[python スクリプト](./scenario/scenario.train.py)で行います。

シナリオを実行する前に以下を確認してください。

- シミュレータ
  - API Ready の状態にあること
- ROS2
  - `aichallenge.launch.py`が launch され、rviz が表示されていること
  - `lgsvl_bridge`が起動していること

### LGSVL Simulator Python API のセットアップ

シミュレータのシナリオ実行には LG が提供している PythonAPI(2021.2)を使用します。シナリオを実行する PC はシミュレータ動作用 PC を推奨しております。

```
git clone https://github.com/lgsvl/PythonAPI
cd PythonAPI
git checkout refs/tags/2021.2
```

README に従って PythonAPI をインストールしてください。

### シナリオ実行

まず[センサ設定の追加](#センサ設定の追加)の項目で追加したセンサ設定(Autoware.Auto)の configuration id を取得します。画像の赤丸部分のボタンをクリックすることでクリップボードにコピーされます。取得した値はコマンド実行時の`vehicle_id`オプションで使用します。
![画面](/image/configuration_id.png)

以下のコマンドでシナリオを実行します。

```
cd scenario
# bridgeにはAutoware動作用PCのIPを、hostにはシミュレータ動作用PCのIPを入力してください。
python3 scenario.train.py --vehicle_id={取得したconfiguration id} --bridge=127.0.0.1 --host=127.0.0.1
```

### タイム取得

競技内容・ルールに関しては[RULE.md](./RULE.md)をご確認下さい。

タイムは`/aichallenge/score`トピックに publish されます。
確認方法は下記の通りです。

シミュレータを開始した後に topic を echo させる事で確認を取る事が出来ます。

```
# ADEコンテナ内で

source ~/aichallenge_ws/install/setup.bash
ros2 topic echo /aichallenge/score
```

又、publish されるタイミングは下記の場合になります。

1. ゴール到達時
2. scenario.train.py を実行し、5 分間経過した場合

`/aichallenge/score`が publish された時点でタイムの算出となります。

`/aichallenge/score`の内容については下記の通りとなります

- time : ペナルティ込みの時間です。この値が競技における正式なタイムになります。
- rawTime : シナリオが開始されてからゴール到着までにかかった時間です。
- hasFinished : ゴールに到達したら 1, それ以外の場合は 0 になります。
- contactPenalty : 接触によるペナルティです。(1 回につき 5 秒タイムに上乗せ)
- trackLimitPenalty : コースアウトによるペナルティです。(コース外にいた時間がタイムに上乗せされます。)

# オンライン評価環境について

## 評価時のオンライン環境での実行フローの概略

スコアの算出にあたっては、オンライン評価環境の web ページよりパッケージ`aichallenge_submit`のみを提出していただき、自動採点を行います。
提出後、オンライン評価環境では`evaluation/`以下のスクリプトを使って下記の手順で評価されます。

### (1) aichallenge_submit の配置

アップロードしていただいた`aichallenge_submit.tar.gz`は`evaluation/`以下に配置されます。

### (2) docker build

`evaluation/build.sh`が実行され、`evaluation/Dockerfile`で定義される docker イメージが作成されます。このイメージの作成手順は下記の通りです。

1. `/opt/AutowareAuto`に`binary-foxy:1.0.0`で提供されているビルド済みの Autoware、`/opt/aichallenge_ws`に本リポジトリの`autoware/adehome/aichallenge_ws`で提供されているソースの配置
2. `ros-foxy-lgsvl-bridge`と`lgsvl/PythonAPI`のインストール
3. 提出いただいた`aichallenge_submit.tar.gz`を`/opt/aichallenge_ws/src/aichallenge_submit`へ展開
4. `rosdep install`と`colcon build`の実行

### (3) シミュレーション実行

オンライン評価環境で simulator が立ち上がり、API モードのシミュレーションが開始されます。

同一のマシンで`evaluation/run.sh`が実行され、docker コンテナが立ち上がり採点が行われます。コンテナ内では`evaluation/main.bash`の実行によって、以下が行われます。

1. rosbag の記録開始
2. lgsvl_bridge の立ち上げ
3. ROS2 ノード群の起動
4. シナリオの開始

実際の採点時の手順や実行されるコマンドも、simulator の自動起動/終了・score の取得・rosbag のアップロード等の手順が追加されていることを除いて`evaluation/main.bash`と同一です。

`evaluation/run.sh`で実行した場合、`evaluation/output`以下に rosbag と実行時ログ(ros2 launch の出力)が保存されます。

#### 実行されるシナリオについて

オンライン評価環境では下の 2 つのシナリオが実行されます。

- 配布シナリオ(`scenario.train.py`)

  オンライン評価環境での動作を検証できるよう、配布しているシナリオについても実行され、その時の rosbag と実行時ログが web ページから取得できます。このシナリオのタイムは順位には関係しません。

- 評価用シナリオ(非公開)

  配布していないタイム評価のためのシナリオです。このシナリオの実行時のタイムトピックの出力を記録し、順位を決めるタイムとします。rosbag や実行時ログはダウンロードできません。

## オンライン評価環境にソースコードを提出する際の手順

### (1) ソースコードを圧縮する

`autoware/adehome/aichallenge_ws/src/aichallenge_submit/create-tar-file.sh`を使用し`aichallenge_submit`内のソースコードを圧縮してください。

```sh
cd autoware/adehome/aichallenge_ws/src/aichallenge_submit/
./create-tar-file.sh
```

`autoware/adehome/aichallenge_ws/src/aichallenge_submit.tar.gz`に圧縮済みのファイルが生成されていることを確認してください。

### (2) `evaluation/` で docker 内での自動実行ができることを確認する

オンライン評価環境にアップロードする前に、ローカル環境を使いオンライン環境と同様の Docker コンテナ内でビルド・実行ができることを以下の手順で確認してください。

まず、(1)で作成した`aichallenge_submit.tar.gz`を`evaluation/`以下に配置してください。ファイル構成は下記のようになります。

```
evaluation/
|-- Dockerfile
|-- aichallenge_submit.tar.gz
|-- build.sh
|-- main.bash
`-- run.sh
```

- `evaluation/output/`には実行時の rosbag やスコアの記録が保存されます。以前の実行時に作成されたディレクトリがあれば削除してください。
- `evaluation/bringup/`にはこのリポジトリの内容(のうち必要な部分)が`./build.sh`を実行した時にコピーされます。

次に、作成いただいた`aichallenge_submit`を含む docker イメージをビルドしてください。

```sh
./build.sh
```

ビルドが完了したら、本 README の手順通りにシミュレーターを起動し、API モードのシミュレーションを開始してください。API ready!と表示されていることを確認したら、`run.sh`によって docker コンテナを立ち上げ採点のフローを実行してください。
`run.sh`内で使われている環境変数`LG_VEHICLE_ID`は本 README のセンサ設定時に取得した configuration id に設定してください。

```sh
export LG_VEHICLE_ID='{取得したconfiguration id}'
./run.sh
```

最後に、`evaluation/output/score.json`に出力されるスコアを確認してください。`evaluation/output/`以下には rosbag と実行時ログも出力されますので、デバッグ等の参考にしてください。

### (3) オンライン評価環境 web ページよりソースをアップロードする

[web ページ](https://aichallenge21.tier4.jp/)にログイン後画面の指示に従って(1)で作成した`aichallenge_submit.tar.gz`をアップロードしてください。

アップロードが終了すると、ソースのビルド・シミュレーションの実行が順番に行われます。

- 正常に終了した場合は`Scoring complete`と表示され、配布シナリオ・評価用シナリオそれぞれのタイムが表示されます。さらに配布シナリオについては`evaluation/output/`に出力されるものと同様の rosbag と実行時ログがタイム下に表示されるリンクよりダウンロードできます。最後にアップロードした評価シナリオのタイムが、ランキングにて最終タイムとして使われます。
- 正常にシナリオ実行が終了しても、launch に失敗した等でスコアが出力されていない場合は`No result`、チェックポイントを全て通過していない場合は`Checkpoint not passed`と表示され、いずれの場合も最終的なタイムとしては使われません。
- ビルドに失敗した場合は`Build error`が表示されます。(1),(2)の手順に従って Docker image のビルドができることを再度ご確認ください。
- シミュレーターの実行に失敗した場合は`Simulator error`と表示されます。この場合サーバーサイドで内部エラーが生じている可能性があるため再度アップロードお願いします。繰り返し表示されてしまう場合はお問合せください。

なお、採点実行中は新たなソースのアップロードはできません。またアップロードできるのは 1 日 3 回までで、日本時間 0 時にリセットされます。

# お問い合わせ

## 更新等の通知に関して

github の更新などがある場合は、以下の URL の issue に新たにコメントします。

本 issue を subscribe いただければ、更新時に通知されます（通知をオンにしてください）。

https://github.com/AutomotiveAIChallenge/aichallenge2021/issues/1

## お問い合わせ受付に関して

競技内容、リポジトリ内容物等に関するお問い合わせについては、github 上の issue にてお願いします。質問は日本語、英語どちらでも構いません。質問者様は質問が解決した際 issue を close してください。

オンラインシミュレータにログインできないなど、オンラインシミュレータのアカウントに関するお問い合わせはai-challenge@jsae.or.jp宛にお願いいたします。

質問内容は競技内容に直接関係あるものに限ります。ソフトウェアの使用方法に関するご質問については、公平性の観点から回答いたしかねます。

各 issue での質問については、基本的に 2 営業日以内に回答いたします。ただし、検討に時間を要する質問や質問数が多い場合等については、2 営業日以上いただく可能性があることはご理解ください。
