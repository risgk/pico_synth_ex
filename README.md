# pico_synth_ex v0.1.0 (2021-09-02)

## 概要

Raspberry Pi Pico用のポリフォニック・シンセサイザです．

『Interface 2021年8月号』で解説されているモノフォニック・シンセサイザ（プログラム9）を拡張して，

- 4和音演奏の実現
- 2個目のオシレータ，LFO（低周波発振器），EG（包絡線発生器）の追加
- オシレータのピッチ微調整対応

といった「本格的なシンセサイザに近づけるための対応」を試みました．


## 動作環境

このプログラムは，

- Pimoroni Pico（ブートROMバージョン1（RP2040 bootrom B0 version））
- Pimoroni Pico VGA Demo Base基板（PWMオーディオ出力に使用，市販品）
- Raspberry Pi Pico C/C++ SDK version 1.1.2
- GNU Arm Embedded Toolchain Version 9-2019-q4-major（gcc-arm-none-eabi version 9.2.1）

の組み合わせで動作確認しています．


## ビルド方法

1. pico_synth_exディレクトリにpico-sdk/external/pico_sdk_import.cmakeのコピーを置く．
2. 環境変数PICO_SDK_PATHにpico-sdkディレクトリへのパスをセットする．
2. buildディレクトリを作成し，pico-examplesと同様にcmakeやnmake（make）コマンドでビルドを行う．
4. pico_synth.uf2ファイルが出力される．


## 仕様

- 部品構成：（2オシレータ ＋ 1フィルタ ＋ 1アンプ ＋ 1 LFO ＋ 1 EG）× 4セット
- 制御方法：UART通信（115200bps）　※MIDIやUSB MIDIには未対応
- オーディオ出力：PWM（GPIO28，PWMキャリア周波数：48kHz，サンプリング周波数：48kHz）
  - 市販品のPimoroni Pico VGA Demo Base基板を使用


## コントロール方法

このプログラムは，UART通信で特定の文字（コマンド）を送信することでコントロールできます．

- 'q'，'w'，'e'，'r'，'t'，'y'，'u'，'i'：ド，レ，ミ，ファ，ソ，ラ，シ，ドの音を鳴らす（音域：中央のドから1オクターブ上のドまで）
- '2'，'3'，'5'，'6'，'7'：ド＃，レ＃，ファ＃，ソ＃，ラ＃の音を鳴らす
- '1'／'9'：キーのオクターブシフト量を1下げる／上げる（-5～+4）
- '0'：全ての音を止める
- 'A'／'a'：オシレータの波形設定値を1下げる／上げる（0：下降ノコギリ波，1：矩形波）
- 'S'／'s'：オシレータ2粗ピッチ設定値を1下げる／上げる（+0～+24）
- 'D'／'d'：オシレータ2微ピッチ設定値を1下げる／上げる（+0～+32）
- 'F'／'f'：オシレータのミックス設定値を1下げる／上げる（0～64，オシレータ1と2のミックス・バランス）
- 'G'／'g'：フィルタのカットオフ設定値を1下げる／上げる（0～120，カットオフ周波数が約19Hz～約20kHzに変化）
- 'H'／'h'：フィルタのレゾナンス設定値を1下げる／上げる（0～5，Q値が約0.7～4.0に変化）
- 'J'／'j'：フィルタのEGからのカットオフ変調量設定値を1下げる／上げる（+0～+60）
- 'X'／'x'：EGのディケイ・タイム設定値を1下げる／上げる（0～64）
- 'C'／'c'：EGのサスティン・レベル設定値を1下げる／上げる（0～64）
- 'B'／'b'：LFOの深さ設定値を1下げる／上げる（0～64，ピッチの変調量）
- 'N'／'n'：LFOの速さ設定値を1下げる／上げる（0～64，周波数が約0.2Hz～約20Hzに変化）


## 状態プリント例

    Pitch             : [  60,  64,  67,  71 ]
    Gate              : [   1,   1,   1,   1 ]
    Octave Shift      :  +0 (1/9)
    Osc Waveform      :   0 (A/a)
    Osc 2 Coarse Pitch:  +0 (S/s)
    Osc 2 Fine Pitch  :  +4 (D/d)
    Osc 1/2 Mix       :  16 (F/f)
    Filter Cutoff     :  60 (G/g)
    Filter Resonance  :   3 (H/h)
    Filter EG Amount  : +60 (J/j)
    EG Decay Time     :  40 (X/x)
    EG Sustain Level  :   0 (C/c)
    LFO Depth         :  16 (B/b)
    LFO Rate          :  48 (N/n)
    Start Time        :   44/  47
    Processing Time   : 2238/2242


## 参考文献

- 石垣 良；リアルタイム処理のために軽量化！シンセサイザの製作，Interface，2021年8月号，CQ出版社，pp.142-153．  
  https://interface.cqpub.co.jp/magazine/202108/
