# Spresense CXD5602PWBIMU Localizer

## 概要

このファームウェアは、Sony Spresenseボードに搭載されたCXD5602PWBIMUセンサからデータを取得し、ボード上で姿勢（クォータニオン）と位置を推定するものです。
推定結果は、ROS 2ノードで受信するためにシリアルポート経由でバイナリ形式で送信されます。

## 機能

- IMU（加速度計・ジャイロスコープ）データの読み取りとフィルタリング
- Madgwickフィルタを応用したアルゴリズムによるリアルタイムな姿勢推定
- 重力加速度を除去した加速度の積分による速度と位置の推定
- ゼロ速度更新（ZUPT）によるドリフト補正
- シリアル通信によるROS 2ノードへのデータ送信

## 通信プロトコル

データは以下のバイナリ形式でシリアルポートから送信されます。

- **ヘッダ:** `0xAA 0xBB 0xCC 0xDD`
- **ペイロード:** `IMUData`構造体
- **チェックサム:** ペイロードの各バイトのXOR

### `IMUData` 構造体
```c
struct __attribute__((packed)) IMUData {
  uint32_t timestamp;
  float temp;
  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;
  float linear_acceleration_x;
  float linear_acceleration_y;
  float linear_acceleration_z;
  float raw_linear_acceleration_x;
  float raw_linear_acceleration_y;
  float raw_linear_acceleration_z;
  float quat_w;
  float quat_x;
  float quat_y;
  float quat_z;
  float velocity_x;
  float velocity_y;
  float velocity_z;
  float position_x;
  float position_y;
  float position_z;
};
```

## 姿勢リセット機能

このファームウェアは、シリアルポート経由でリセットコマンドを受信する機能を持ちます。
`REST`という4バイトの文字列を受信すると、推定されている姿勢、位置、速度などの内部状態がすべてリセットされ、初期化シーケンスが再実行されます。

## ビルドと書き込み

1. [Spresense Arduino Library](https://github.com/sonydevworld/spresense-arduino-lib) のセットアップ手順に従い、Arduino IDEを準備します。
2. このスケッチ (`cxd5602pwbimu_localizer_arduino.ino`) をArduino IDEで開きます。
3. ターゲットボードとして「Spresense」を選択します。
4. スケッチをコンパイルし、Spresenseボードに書き込みます。