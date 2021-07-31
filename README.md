# pico_synth_ex v0.0.0 (2021-07-31)

## 概要

Raspberry Pi Pico用のポリフォニック・シンセサイザです．

『Interface 2021年8月号』で解説されているモノフォニック・シンセサイザ（プログラム9）を拡張して，

- 4和音演奏の実現
- 2個目のオシレータ，LFO（低周波発振器），EG（包絡線発生器）の追加
- オシレータのピッチ微調整対応

といった「本格的なシンセサイザに近づけるための対応」を試みました．


### 参考文献

- 石垣 良；リアルタイム処理のために軽量化！シンセサイザの製作，Interface，2021年8月号，CQ出版社，pp.142-153．  
  https://interface.cqpub.co.jp/magazine/202108/


## 仕様

- 部品構成：（2オシレータ ＋ 1フィルタ ＋ 1アンプ ＋ 1 LFO ＋ 1 EG）× 4セット
- 制御方法：UART通信（115200bps）　※MIDIやUSB MIDIには未対応
- オーディオ出力：PWM（GPIO28，PWMキャリア周波数：48kHz，サンプリング周波数：48kHz）
  - 市販品のPimoroni Pico VGA Demo Base基板を使用


## プログラム開発環境

このプログラムは，

- GNU Arm Embedded Toolchain Version 9-2019-q4-major（gcc-arm-none-eabi version 9.2.1）
- Raspberry Pi Pico C/C++ SDK version 1.1.2
- RP2040 bootrom B0 version（ブートROMバージョン1）

の組み合わせで動作確認しています．
