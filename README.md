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
sudo apt install git-lfs
git lfs install --skip-repo

git clone https://github.com/AutomotiveAIChallenge/aichallenge2021
```

PCDファイルがLFSサーバーから正常にダウンロードされていることを確認してください。
```
ls -lh aichallenge2021/autoware/adehome/aichallenge_ws/src/aichallenge_launch/data/IndianapolisMotorSpeedway.pcd
```
正常にダウンロードされている場合、ファイルサイズは約300MBになります。

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

# オンライン評価環境について
## 評価時のオンライン環境での実行フローの概略
スコアの算出にあたっては、オンライン評価環境のwebページよりパッケージ`aichallenge_submit`のみを提出していただき、自動採点を行います。
提出後、オンライン評価環境では`evaluation/`以下のスクリプトを使って下記の手順で評価されます。

### (1) aichallenge_submitの配置
アップロードしていただいた`aichallenge_submit.tar.gz`は`evaluation/`以下に配置されます。

### (2) docker build
`evaluation/build.sh`が実行され、`evaluation/Dockerfile`で定義されるdockerイメージが作成されます。このイメージの作成手順は下記の通りです。

1. `/opt/AutowareAuto`に`binary-foxy:1.0.0`で提供されているビルド済みのAutoware、`/opt/aichallenge_ws`に本リポジトリの`autoware/adehome/aichallenge_ws`で提供されているソースの配置
2. `ros-foxy-lgsvl-bridge`と`lgsvl/PythonAPI`のインストール
3. 提出いただいた`aichallenge_submit.tar.gz`を`/opt/aichallenge_ws/src/aichallenge_submit`へ展開
4. `rosdep install`と`colcon build`の実行

### (3) シミュレーション実行
オンライン評価環境でsimulatorが立ち上がり、APIモードのシミュレーションが開始されます。

同一のマシンで`evaluation/run.sh`が実行され、dockerコンテナが立ち上がり採点が行われます。コンテナ内では`evaluation/main.bash`の実行によって、以下が行われます。

1. rosbagの記録開始
2. lgsvl_bridgeの立ち上げ
3. ROS2ノード群の起動
4. シナリオの開始

実際の採点時の手順や実行されるコマンドも、simulatorの自動起動/終了・scoreの取得・rosbagのアップロード等の手順が追加されていることを除いて`evaluation/main.bash`と同一です。

`evaluation/run.sh`で実行した場合、`evaluation/output`以下にrosbagと実行時ログ(ros2 launchの出力)が保存されます。

#### 実行されるシナリオについて
オンライン評価環境では下の2つのシナリオが実行されます。

- 配布シナリオ(`scenario.train.py`)

    オンライン評価環境での動作を検証できるよう、配布しているシナリオについても実行され、その時のrosbagと実行時ログがwebページから取得できます。このシナリオのタイムは順位には関係しません。

- 評価用シナリオ(非公開)

    配布していないタイム評価のためのシナリオです。このシナリオの実行時のタイムトピックの出力を記録し、順位を決めるタイムとします。rosbagや実行時ログはダウンロードできません。
    

## オンライン評価環境にソースコードを提出する際の手順
### (1) ソースコードを圧縮する
`autoware/adehome/aichallenge_ws/src/aichallenge_submit/create-tar-file.sh`を使用し`aichallenge_submit`内のソースコードを圧縮してください。

```sh
cd autoware/adehome/aichallenge_ws/src/aichallenge_submit/
./create-tar-file.sh
```

`autoware/adehome/aichallenge_ws/src/aichallenge_submit.tar.gz`に圧縮済みのファイルが生成されていることを確認してください。

### (2) `evaluation/` でdocker内での自動実行ができることを確認する
オンライン評価環境にアップロードする前に、ローカル環境を使いオンライン環境と同様のDockerコンテナ内でビルド・実行ができることを以下の手順で確認してください。

まず、(1)で作成した`aichallenge_submit.tar.gz`を`evaluation/`以下に配置してください。ファイル構成は下記のようになります。
```
evaluation/
|-- Dockerfile
|-- aichallenge_submit.tar.gz
|-- build.sh
|-- main.bash
`-- run.sh
```

- `evaluation/output/`には実行時のrosbagやスコアの記録が保存されます。以前の実行時に作成されたディレクトリがあれば削除してください。
- `evaluation/bringup/`にはこのリポジトリの内容(のうち必要な部分)が`./build.sh`を実行した時にコピーされます。

次に、作成いただいた`aichallenge_submit`を含むdockerイメージをビルドしてください。
```sh
./build.sh
```

ビルドが完了したら、本READMEの手順通りにシミュレーターを起動し、APIモードのシミュレーションを開始してください。API ready!と表示されていることを確認したら、`run.sh`によってdockerコンテナを立ち上げ採点のフローを実行してください。
`run.sh`内で使われている環境変数`LG_VEHICLE_ID`は本READMEのセンサ設定時に取得したconfiguration idに設定してください。
```sh
export LG_VEHICLE_ID='{取得したconfiguration id}'
./run.sh
```

最後に、`evaluation/output/score.json`に出力されるスコアを確認してください。`evaluation/output/`以下にはrosbagと実行時ログも出力されますので、デバッグ等の参考にしてください。

### (3) オンライン評価環境webページよりソースをアップロードする

[webページ](https://aichallenge21.tier4.jp/)にログイン後画面の指示に従って(1)で作成した`aichallenge_submit.tar.gz`をアップロードしてください。

アップロードが終了すると、ソースのビルド・シミュレーションの実行が順番に行われます。

- 正常に終了した場合は`Scoring complete`と表示され、配布シナリオ・評価用シナリオそれぞれのタイムが表示されます。さらに配布シナリオについては`evaluation/output/`に出力されるものと同様のrosbagと実行時ログがタイム下に表示されるリンクよりダウンロードできます。最後にアップロードした評価シナリオのタイムが、ランキングにて最終タイムとして使われます。
- 正常にシナリオ実行が終了しても、launchに失敗した等でスコアが出力されていない場合は`No result`、チェックポイントを全て通過していない場合は`Checkpoint not passed`と表示され、いずれの場合も最終的なタイムとしては使われません。
- ビルドに失敗した場合は`Build error`が表示されます。(1),(2)の手順に従ってDocker imageのビルドができることを再度ご確認ください。
- シミュレーターの実行に失敗した場合は`Simulator error`と表示されます。この場合サーバーサイドで内部エラーが生じている可能性があるため再度アップロードお願いします。繰り返し表示されてしまう場合はお問合せください。

なお、採点実行中は新たなソースのアップロードはできません。またアップロードできるのは1日3回までで、日本時間0時にリセットされます。

# お問い合わせ

## 更新等の通知に関して
githubの更新などがある場合は、以下のURLのissueに新たにコメントします。

本issueをsubscribeいただければ、更新時に通知されます（通知をオンにしてください）。

https://github.com/AutomotiveAIChallenge/aichallenge2021/issues/1

## お問い合わせ受付に関して
競技内容、リポジトリ内容物等に関するお問い合わせについては、github上のissueにてお願いします。質問は日本語、英語どちらでも構いません。質問者様は質問が解決した際issueをcloseしてください。

質問内容は競技内容に直接関係あるものに限ります。ソフトウェアの使用方法に関するご質問については、公平性の観点から回答いたしかねます。

各issueでの質問については、基本的に2営業日以内に回答いたします。ただし、検討に時間を要する質問や質問数が多い場合等については、2営業日以上いただく可能性があることはご理解ください。
