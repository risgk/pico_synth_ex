////////////////////////////////////////////////////////////
//////// pico_synth_ex v0.0.0 (2021-07-31) /////////////////
////////////////////////////////////////////////////////////
#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/float.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"

typedef int32_t Q28; // 小数部28ビットの符号付き固定小数点数
typedef int16_t Q14; // 小数部14ビットの符号付き固定小数点数

#define ONE_Q28 ((Q28) (1 << 28)) // Q28型の1.0
#define ONE_Q14 ((Q14) (1 << 14)) // Q14型の1.0
#define PI      ((float) M_PI)    // float型の円周率
#define FCLKSYS (120000000) // システムクロック周波数（Hz）
#define FS      (48000)     // サンプリング周波数（Hz）
#define FA      (440.0F)    // 基準周波数（Hz）

//////// オシレータ群 ////////////////////////////
static uint32_t Osc_freq_table[122];         // 周波数テーブル
static Q14      Osc_tune_table[256];         // 周波数調整テーブル
static Q14      Osc_wave_tables[2][31][512]; // 波形テーブル群
static Q14      Osc_mix_table[65];           // ミックス用テーブル

static volatile uint8_t Osc_waveform       = 0; // 波形設定値
static volatile int8_t  Osc_2_coarse_pitch = 0; // オシレータ2粗ピッチ設定値
static volatile int8_t  Osc_2_fine_pitch   = 4; // オシレータ2微ピッチ設定値
static volatile uint8_t Osc_1_2_mix        = 0; // オシレータ1／2ミックス設定値

static void Osc_init() {
  for (uint8_t pitch = 0; pitch < 122; ++pitch) {
    uint32_t freq = (FA * powf(2, (pitch - 69.0F) / 12)) * (1LL << 32) / FS;
    freq = ((freq >> 4) << 4) + 1; // 少し半端な値にする
    Osc_freq_table[pitch] = freq;
  }

  for (uint16_t tune = 0; tune < 256; ++tune) {
    Osc_tune_table[tune] =
        (powf(2, (tune - 128.0F) / (12 * 256)) * ONE_Q14) - ONE_Q14;
  }

  // TODO: 4半音毎にテーブルを持っているが、2または3半音毎のほうが良いかも
  // TODO: 参照テーブルを追加して、Osc_wave_tablesの重複データを無くしたい
  for (uint8_t pitch_div_4 = 0; pitch_div_4 < 31; ++pitch_div_4) {
    // TODO: ((FS - 0.1F) / 2) 以下という制約は不自然，改善の余地アリ
    uint16_t harm_max = // 最大倍音次数
        (((FS - 1.0F) / 2) * (1LL << 32) / FS) /
        Osc_freq_table[(pitch_div_4 << 2) + 1];
    if (harm_max > 127) { harm_max = 127; }

    // 下降ノコギリ波，矩形波を生成
    for (uint16_t i = 0; i < 512; ++i) {
      float sum_saw    = 0.0F;
      float sum_square = 0.0F;
      for (uint16_t k = 1; k <= harm_max; ++k) {
        sum_saw += (2 / PI) * (sinf(2 * PI * k * i / 512) / k);
        if ((k % 2) == 1) {
          sum_square += (4 / PI) * (sinf(2 * PI * k * i / 512) / k);
        }
      }
      sum_saw    *= 0.5F; // Osc_process() でオーバ・フローしないように必要
      sum_square *= 0.5F;
      Osc_wave_tables[0][pitch_div_4][i] = float2fix(sum_saw,    14);
      Osc_wave_tables[1][pitch_div_4][i] = float2fix(sum_square, 14);
    }
  }

  for (uint8_t i = 0; i < 65; ++i) {
    Osc_mix_table[i] = sqrtf((64 - i) / 64.0F) * ONE_Q14 * 0.5F;
  }
}

static inline Q28 Osc_phase_to_audio(uint32_t phase, uint8_t pitch) {
  Q14* wave_table = Osc_wave_tables[Osc_waveform][(pitch + 3) >> 2];
  uint16_t curr_index = phase >> 23;
  uint16_t next_index = (curr_index + 1) & 0x000001FF;
  Q14 curr_sample = wave_table[curr_index];
  Q14 next_sample = wave_table[next_index];
  Q14 next_weight = (phase >> 9) & 0x3FFF;
  return (curr_sample << 14) + ((next_sample - curr_sample) * next_weight);
}

static inline Q28 Osc_process(uint8_t id,
                              uint16_t full_pitch, Q14 pitch_mod_in) {
  static uint32_t phase_1[4]; // オシレータ1の位相
  int32_t full_pitch_1 = full_pitch + ((256 * pitch_mod_in) >> 14);
  full_pitch_1 += (full_pitch_1 < 0)          * (0 - full_pitch_1);
  full_pitch_1 -= (full_pitch_1 > (120 << 8)) * (full_pitch_1 - (120 << 8));
  uint8_t pitch_1 = (full_pitch_1 + 128) >> 8;
  uint8_t tune_1  = (full_pitch_1 + 128) & 0xFF;
  uint32_t freq_1 = Osc_freq_table[pitch_1];
  phase_1[id] += freq_1 + ((id - 1) << 8); // ボイス毎にずらす
  phase_1[id] += ((int32_t) (freq_1 >> 8) * Osc_tune_table[tune_1]) >> 6;

  static uint32_t phase_2[4]; // オシレータ2の位相
  int32_t full_pitch_2 =
      full_pitch_1 + (Osc_2_coarse_pitch << 8) + (Osc_2_fine_pitch << 2);
  full_pitch_2 += (full_pitch_2 < 0)          * (0 - full_pitch_2);
  full_pitch_2 -= (full_pitch_2 > (120 << 8)) * (full_pitch_2 - (120 << 8));
  uint8_t pitch_2 = (full_pitch_2 + 128) >> 8;
  uint8_t tune_2  = (full_pitch_2 + 128) & 0xFF;
  uint32_t freq_2 = Osc_freq_table[pitch_2];
  phase_2[id] += freq_2 + ((id - 1) << 8); // ボイス毎にずらす
  phase_2[id] += ((int32_t) (freq_2 >> 8) * Osc_tune_table[tune_2]) >> 6;

  // TODO: wave_table切替えをスムーズにしたい（周期の頭で切替えるのが良い？）
  return ((Osc_phase_to_audio(phase_1[id], pitch_1) >> 14) *
                              Osc_mix_table[Osc_1_2_mix - 0]) +
         ((Osc_phase_to_audio(phase_2[id], pitch_2) >> 14) *
                              Osc_mix_table[64 - Osc_1_2_mix]);
}

//////// フィルタ ////////////////////////////////
struct FILTER_COEFS { Q28 b0_a0, a1_a0, a2_a0; }; // フィルタ係数群

static struct FILTER_COEFS Filter_coefs_table[6][481]; // フィルタ係数群テーブル

static volatile uint8_t Filter_cutoff     = 120; // カットオフ設定値
static volatile uint8_t Filter_resonance  = 0;   // レゾナンス設定値
static volatile int8_t  Filter_mod_amount = 0;   // カットオフ変調量設定値

static void Filter_init() {
  for (uint8_t resonance = 0; resonance < 6; ++resonance) {
    for (uint16_t cutoff = 0; cutoff < 481; ++cutoff) {
      float f0    = FA * powf(2, ((cutoff / 4.0F) - 54) / 12);
      float w0    = 2 * PI * f0 / FS;
      float q     = powf(sqrtf(2), resonance - 1.0F);
      float alpha = sinf(w0) / (2 * q);
      float b0    = (1 - cosf(w0)) / 2;
      float a0    =  1 + alpha;
      float a1    = -2 * cosf(w0);
      float a2    =  1 - alpha;
      Filter_coefs_table[resonance][cutoff].b0_a0 = float2fix(b0 / a0, 28);
      Filter_coefs_table[resonance][cutoff].a1_a0 = float2fix(a1 / a0, 28);
      Filter_coefs_table[resonance][cutoff].a2_a0 = float2fix(a2 / a0, 28);
    }
  }
}

static inline int32_t mul_s32_s32_h32(int32_t x, int32_t y) {
  // 符号付き32ビット同士の乗算結果の上位32ビット
  int32_t x1 = x >> 16; uint32_t x0 = x & 0xFFFF;
  int32_t y1 = y >> 16; uint32_t y0 = y & 0xFFFF;
  int32_t x0_y1 = x0 * y1;
  int32_t z = ((x0 * y0) >> 16) + (x1 * y0) + (x0_y1 & 0xFFFF);
  return (z >> 16) + (x0_y1 >> 16) + (x1 * y1);
}

static inline Q28 Filter_process(uint8_t id, Q28 audio_in, Q14 cutoff_mod_in) {
  static uint16_t curr_cutoff[4];           // カットオフ現在値
  int32_t targ_cutoff = Filter_cutoff << 2; // カットオフ目標値
  targ_cutoff += (Filter_mod_amount * cutoff_mod_in) >> (14 - 2);
  targ_cutoff += (targ_cutoff < 0)   * (0 - targ_cutoff);
  targ_cutoff -= (targ_cutoff > 480) * (targ_cutoff - 480);
  curr_cutoff[id] += (curr_cutoff[id] < targ_cutoff);
  curr_cutoff[id] -= (curr_cutoff[id] > targ_cutoff);
  struct FILTER_COEFS* coefs_ptr =
      &Filter_coefs_table[Filter_resonance][curr_cutoff[id]];

  static Q28 x1[4], x2[4], y1[4], y2[4];
  Q28 x0 = audio_in;
  Q28 x3 = x0 + (x1[id] << 1) + x2[id];
  Q28 y0 = mul_s32_s32_h32(coefs_ptr->b0_a0, x3)     << 4;
  y0    -= mul_s32_s32_h32(coefs_ptr->a1_a0, y1[id]) << 4;
  y0    -= mul_s32_s32_h32(coefs_ptr->a2_a0, y2[id]) << 4;
  x2[id] = x1[id]; y2[id] = y1[id]; x1[id] = x0; y1[id] = y0;
  return y0;
}

//////// アンプ //////////////////////////////////
static inline Q28 Amp_process(uint8_t id, Q28 audio_in, Q14 gain_in) {
  return (audio_in >> 14) * gain_in; // 計算を簡略化
}

//////// EG（Envelope Generator） ////////////////
static uint32_t EG_exponential_table[65]; // 指数関数テーブル

static volatile uint8_t EG_attack_time   = 0;  // アタック・タイム設定値
static volatile uint8_t EG_decay_time    = 0;  // ディケイ・タイム設定値
static volatile uint8_t EG_sustain_level = 64; // サスティン・レベル設定値

static inline void EG_init() {
  for (uint8_t index = 0; index < 65; ++index) {
    EG_exponential_table[index] = 24000 * powf(10, (index - 32.0F) / 16);
  }
}

static inline Q14 EG_process(uint8_t id, uint8_t gate_in) {
  static Q14     curr_level[4];        // EG出力レベル現在値
  static uint8_t curr_gate[4];         // ゲート入力レベル現在値
  static uint8_t curr_attack_phase[4]; // 現在アタック・フェーズかどうか

#if 1
  curr_attack_phase[id] |= (curr_gate[id] == 0) & gate_in;
  curr_attack_phase[id] &= (curr_level[id] < ONE_Q14) & gate_in;
  curr_gate[id]          =  gate_in;

  Q14 attack_targ_level = ONE_Q14 + (ONE_Q14 >> 1);
  curr_level[id] += curr_attack_phase[id] *
                    ((attack_targ_level - curr_level[id]) >> 5);

  Q14 decay_targ_level = (EG_sustain_level << 8) * curr_gate[id];
  int32_t decay = (curr_attack_phase[id] == 0) &
                  (curr_level[id] > decay_targ_level);
  curr_level[id] += decay * ((decay_targ_level - curr_level[id]) >> 5);
#endif

  return curr_level[id];
}

//////// LFO（Low Frequency Oscillator） /////////
static uint32_t LFO_freq_table[65]; // 周波数テーブル

static volatile uint8_t LFO_depth = 0;  // 深さ設定値
static volatile uint8_t LFO_rate  = 48; // 速さ設定値

static void LFO_init() {
  for (uint8_t rate = 0; rate < 65; ++rate) {
    LFO_freq_table[rate] =
        2 * powf(10, (rate - 32.0F) / 32) * (1LL << 32) / FS;
  }
}

static inline Q14 LFO_process(uint8_t id) {
  static uint32_t phase[4]; // 位相
  phase[id] += LFO_freq_table[LFO_rate] + ((id - 1) << 8); // ボイス毎にずらす

  // 三角波を生成
  uint16_t phase_h16 = phase[id] >> 16;
  uint16_t out = phase_h16;
  out += (phase_h16 >= 32768) * (65536 - (phase_h16 << 1));
  return ((out - 16384) * LFO_depth) >> 7;
}

//////// PWMオーディオ出力部 /////////////////////
#define PWMA_L_GPIO  (28)           // PWM出力するGPIO番号（左チャンネル）
#define PWMA_L_SLICE (6)            // PWMスライス番号（左チャンネル）
#define PWMA_L_CHAN  (PWM_CHAN_A)   // PWMチャンネル（左チャンネル）
#define PWMA_R_GPIO  (27)           // PWM出力するGPIO番号（右チャンネル）
#define PWMA_R_SLICE (5)            // PWMスライス番号（右チャンネル）
#define PWMA_R_CHAN  (PWM_CHAN_B)   // PWMチャンネル（右チャンネル）
#define PWMA_CYCLE   (FCLKSYS / FS) // PWM周期

static void pwm_irq_handler();

static void PWMA_init() {
  gpio_set_function(PWMA_R_GPIO, GPIO_FUNC_PWM);
  gpio_set_function(PWMA_L_GPIO, GPIO_FUNC_PWM);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_set_irq_enabled(PWMA_L_SLICE, true);
  pwm_set_wrap(PWMA_R_SLICE, PWMA_CYCLE - 1);
  pwm_set_wrap(PWMA_L_SLICE, PWMA_CYCLE - 1);
  pwm_set_chan_level(PWMA_R_SLICE, PWMA_R_CHAN, PWMA_CYCLE / 2);
  pwm_set_chan_level(PWMA_L_SLICE, PWMA_L_CHAN, PWMA_CYCLE / 2);
  pwm_set_enabled(PWMA_R_SLICE, true);
  pwm_set_enabled(PWMA_L_SLICE, true);
}

static inline void PWMA_process(Q28 audio_in) {
  int32_t level_int32 = (audio_in >> 18) + (PWMA_CYCLE / 2);
  uint16_t level = (level_int32 > 0) * level_int32;
  pwm_set_chan_level(PWMA_R_SLICE, PWMA_R_CHAN, level);
  pwm_set_chan_level(PWMA_L_SLICE, PWMA_L_CHAN, level);
}

//////// 割り込みハンドラとメイン関数 ////////////
static volatile uint16_t start_time     = 0; // 開始時間
static volatile uint16_t max_start_time = 0; // 最大開始時間
static volatile uint16_t proc_time      = 0; // 処理時間
static volatile uint16_t max_proc_time  = 0; // 最大処理時間

static volatile uint8_t gate_voice[4];  // ゲート制御値（ボイス毎）
static volatile uint8_t pitch_voice[4]; // ピッチ制御値（ボイス毎）
static volatile int8_t  octave_shift;   // キーのオクターブシフト量

static inline Q28 process_voice(uint8_t id) {
  Q14 lfo_out    = LFO_process(id);
  Q14 eg_out     = EG_process(id, gate_voice[id]);
  Q28 osc_out    = Osc_process(id, pitch_voice[id] << 8, lfo_out);
  Q28 filter_out = Filter_process(id, osc_out, eg_out);
  Q28 amp_out    = Amp_process(id, filter_out, eg_out);
  return amp_out;
}

static void pwm_irq_handler() {
  pwm_clear_irq(PWMA_L_SLICE);
  start_time = pwm_get_counter(PWMA_L_SLICE);

  Q28 voice_out[4];
  voice_out[0] = process_voice(0);
  voice_out[1] = process_voice(1);
  voice_out[2] = process_voice(2);
  voice_out[3] = process_voice(3);
  PWMA_process((voice_out[0] + voice_out[1] +
                voice_out[2] + voice_out[3]) >> 2);

  uint16_t end_time = pwm_get_counter(PWMA_L_SLICE);
  proc_time = end_time - start_time; // 計算を簡略化
  max_start_time +=
      (start_time > max_start_time) * (start_time - max_start_time);
  max_proc_time +=
      (proc_time > max_proc_time) * (proc_time - max_proc_time);
}

static inline void note_on_off(uint8_t key)
{
  uint8_t pitch = key + (octave_shift * 12);
  if      (pitch_voice[0] == pitch) { gate_voice[0] = (gate_voice[0] == 0); }
  else if (pitch_voice[1] == pitch) { gate_voice[1] = (gate_voice[1] == 0); }
  else if (pitch_voice[2] == pitch) { gate_voice[2] = (gate_voice[2] == 0); }
  else if (pitch_voice[3] == pitch) { gate_voice[3] = (gate_voice[3] == 0); }
  else if (gate_voice[0] == 0) { pitch_voice[0] = pitch; gate_voice[0] = 1; }
  else if (gate_voice[1] == 0) { pitch_voice[1] = pitch; gate_voice[1] = 1; }
  else if (gate_voice[2] == 0) { pitch_voice[2] = pitch; gate_voice[2] = 1; }
  else                         { pitch_voice[3] = pitch; gate_voice[3] = 1; }
}

static inline void all_notes_off()
{
  for (uint8_t id = 0; id < 4; ++id) { gate_voice[id] = 0; }
}

int main() {
  for (uint8_t id = 0; id < 4; ++id) { pitch_voice[id] = 60; }
#if 1
  note_on_off(60); note_on_off(64); note_on_off(67); note_on_off(71);
#endif

  set_sys_clock_khz(FCLKSYS / 1000, true);
  stdio_init_all();
  LFO_init(); EG_init(); Osc_init(); Filter_init(); PWMA_init();

  while (true) {
    switch (getchar_timeout_us(0)) {
    case 'q': note_on_off(60); break; // ド
    case '2': note_on_off(61); break; // ド＃
    case 'w': note_on_off(62); break; // レ
    case '3': note_on_off(63); break; // レ＃
    case 'e': note_on_off(64); break; // ミ
    case 'r': note_on_off(65); break; // ファ
    case '5': note_on_off(66); break; // ファ＃
    case 't': note_on_off(67); break; // ソ
    case '6': note_on_off(68); break; // ソ＃
    case 'y': note_on_off(69); break; // ラ
    case '7': note_on_off(70); break; // ラ＃
    case 'u': note_on_off(71); break; // シ
    case 'i': note_on_off(72); break; // ド

    case '1': if (octave_shift       > -5)  { --octave_shift;       } break;
    case '9': if (octave_shift       < +4)  { ++octave_shift;       } break;
    case '0': all_notes_off(); break;

    case 'A': if (Osc_waveform       > 0)   { --Osc_waveform;       } break;
    case 'a': if (Osc_waveform       < 1)   { ++Osc_waveform;       } break;
    case 'S': if (Osc_2_coarse_pitch > +0)  { --Osc_2_coarse_pitch; } break;
    case 's': if (Osc_2_coarse_pitch < +24) { ++Osc_2_coarse_pitch; } break;
    case 'D': if (Osc_2_fine_pitch   > +0)  { --Osc_2_fine_pitch;   } break;
    case 'd': if (Osc_2_fine_pitch   < +32) { ++Osc_2_fine_pitch;   } break;
    case 'F': if (Osc_1_2_mix        > 0)   { --Osc_1_2_mix;        } break;
    case 'f': if (Osc_1_2_mix        < 64)  { ++Osc_1_2_mix;        } break;

    case 'G': if (Filter_cutoff      > 0)   { --Filter_cutoff;      } break;
    case 'g': if (Filter_cutoff      < 120) { ++Filter_cutoff;      } break;
    case 'H': if (Filter_resonance   > 0)   { --Filter_resonance;   } break;
    case 'h': if (Filter_resonance   < 5)   { ++Filter_resonance;   } break;
    case 'J': if (Filter_mod_amount  > +0)  { --Filter_mod_amount;  } break;
    case 'j': if (Filter_mod_amount  < +60) { ++Filter_mod_amount;  } break;

    case 'Z': if (EG_attack_time     > 0)   { --EG_attack_time;     } break;
    case 'z': if (EG_attack_time     < 64)  { ++EG_attack_time;     } break;
    case 'X': if (EG_decay_time      > 0)   { --EG_decay_time;      } break;
    case 'x': if (EG_decay_time      < 64)  { ++EG_decay_time;      } break;
    case 'C': if (EG_sustain_level   > 0)   { --EG_sustain_level;   } break;
    case 'c': if (EG_sustain_level   < 64)  { ++EG_sustain_level;   } break;

    case 'B': if (LFO_depth          > 0)   { --LFO_depth;          } break;
    case 'b': if (LFO_depth          < 64)  { ++LFO_depth;          } break;
    case 'N': if (LFO_rate           > 0)   { --LFO_rate;           } break;
    case 'n': if (LFO_rate           < 64)  { ++LFO_rate;           } break;
    }
    static uint32_t loop_counter = 0; // ループ回数
    if ((++loop_counter & 0xFFFFF) == 0) {
      printf("Pitch             : [ %3hhu, %3hhu, %3hhu, %3hhu ]\n",
          pitch_voice[0], pitch_voice[1], pitch_voice[2], pitch_voice[3]);
      printf("Gate              : [ %3hhu, %3hhu, %3hhu, %3hhu ]\n",
          gate_voice[0], gate_voice[1], gate_voice[2], gate_voice[3]);
      printf("Octave Shift      : %+3hd (1/9)\n", octave_shift);
      printf("Osc Waveform      : %3hhu (A/a)\n", Osc_waveform);
      printf("Osc 2 Coarse Pitch: %+3hd (S/s)\n", Osc_2_coarse_pitch);
      printf("Osc 2 Fine Pitch  : %+3hd (D/d)\n", Osc_2_fine_pitch);
      printf("Osc 1/2 Mix       : %3hhu (F/f)\n", Osc_1_2_mix);
      printf("Filter Cutoff     : %3hhu (G/g)\n", Filter_cutoff);
      printf("Filter Resonance  : %3hhu (H/h)\n", Filter_resonance);
      printf("Filter EG Amount  : %+3hd (J/j)\n", Filter_mod_amount);
      printf("EG Attack Time    : %3hhu (Z/z)\n", EG_attack_time);
      printf("EG Decay Time     : %3hhu (X/x)\n", EG_decay_time);
      printf("EG Sustain Level  : %3hhu (C/c)\n", EG_sustain_level);
      printf("LFO Depth         : %3hhu (B/b)\n", LFO_depth);
      printf("LFO Rate          : %3hhu (N/n)\n", LFO_rate);
      printf("Start Time        : %4hu/%4hu\n",   start_time, max_start_time);
      printf("Processing Time   : %4hu/%4hu\n\n", proc_time, max_proc_time);
    }
  }
}
