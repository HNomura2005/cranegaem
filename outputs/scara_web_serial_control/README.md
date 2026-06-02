# クレーンまるばつロボ

Arduino Mega 2560とブラウザ画面で、SCARA型ロボットを動かすスターターです。
クレーンゲームのように自分でロボットを操作して、ピンポン玉を3x3のマスに入れます。
入ったマスを画面で記録し、入らなかったときも記録できます。

## どう動くか

- `うで1`: 1つ目の関節モーター。Arduinoコマンドでは`J1`。
- `うで2`: 2つ目の関節モーター。Arduinoコマンドでは`J2`。
- `上下`: 先端を上げ下げするモーター。Arduinoコマンドでは`Z`。
- `電磁石`: 先端で玉をつかむ/はなす。

前の版にあった`R`は、先端を回転させる追加モーター用でした。
今回のクレーンゲームでは不要なので、UIとArduinoコードから外しました。

## ファイル

- `index.html`: Chrome/Edgeで開く操作画面
- `ArduinoMegaScaraControl.ino`: Arduino Mega 2560に書き込むスケッチ

## 必要なもの

- Arduino Mega 2560
- STEP/DIR入力のステッピングモータードライバ 3個
- ステッピングモーター 3個
- 電磁石
- 電磁石用MOSFETモジュール、またはリレーモジュール
- モーター用の別電源
- 電磁石用の別電源
- 物理的な非常停止スイッチ
- 3x3のマス板
- 玉を置くスタート場所
- ピンポン玉

モーターや電磁石をArduinoのピンに直接つながないでください。
Arduino、モータードライバ、MOSFET/リレーはGNDを共通にします。

## ピン

| 役目 | Arduino Mega 2560 |
| --- | --- |
| うで1 STEP / DIR | 22 / 23 |
| うで2 STEP / DIR | 24 / 25 |
| 上下 STEP / DIR | 26 / 27 |
| ドライバENABLE | 30 |
| 電磁石ON/OFF | 31 |

## 使い方

1. Arduino IDEで`AccelStepper`ライブラリを入れます。
2. `ArduinoMegaScaraControl.ino`を開きます。
3. ボードを`Arduino Mega or Mega 2560`にして書き込みます。
4. ChromeまたはEdgeで`index.html`を開きます。
5. `ロボにつなぐ`を押してArduinoのポートを選びます。
6. `玉をつかむ`で電磁石をONにします。
7. `じぶんでそうさ`のボタンで玉を運びます。
8. `玉をはなす`で電磁石をOFFにします。
9. 玉が入ったら、そのマスを画面で押して記録します。
10. 入らなかったら、`入らなかった`を押します。

## 実機で必ず調整するところ

`ArduinoMegaScaraControl.ino`のこの値を、実際のロボに合わせて変えてください。

```cpp
constexpr long SAFE_Z = 0;
constexpr long HOME_J1 = 0;
constexpr long HOME_J2 = 0;
```

まずはモーター電源を弱め、玉なしでゆっくり試してください。
ゲームのマス位置はロボが自動で決めません。子どもが操作して、入った結果だけを画面に記録します。

## コマンド

- `JOG J1 200`: うで1を少し動かす
- `JOG J2 -200`: うで2を少し動かす
- `JOG Z 100`: 上下を少し動かす
- `MAG 1`: 電磁石ON
- `MAG 0`: 電磁石OFF
- `HOME`: おうちの位置へ戻る
- `STOP`: 停止して電磁石OFF
- `ZERO`: 今の位置をゼロにする
- `STATUS`: 今の位置を見る

## GitHubで管理するとき

このフォルダをGitHubリポジトリに入れると管理しやすいです。

```powershell
git init
git add outputs/scara_web_serial_control
git commit -m "Add pingpong scara web controller"
git branch -M main
git remote add origin https://github.com/YOUR_NAME/YOUR_REPO.git
git push -u origin main
```

`gh`コマンドがある場合は、GitHub CLIからリポジトリ作成もできます。

## 安全メモ

ブラウザの`ぜんぶ止める`ボタンは便利ですが、安全装置ではありません。
必ず、モーター電源と電磁石電源を物理的に切れる非常停止スイッチを入れてください。
