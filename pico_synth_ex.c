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

//////// オシレータ //////////////////////////////
static uint32_t Osc_freq_table[121];      // 周波数テーブル
static Q14      Osc_wave_tables[31][512]; // 波形テーブル群

static volatile int32_t Osc_wav = 0; // 出力波形設定値
static volatile int32_t Osc_2co = 0; // オシレータ2の粗ピッチ設定値
static volatile int32_t Osc_2fi = 0; // オシレータ2の微ピッチ設定値
static volatile int32_t Osc_mix = 0; // オシレータ1／2のミックス設定値

static void Osc_init() {
  for (uint32_t pitch = 0; pitch < 121; ++pitch) {
    uint32_t freq = (FA * powf(2, (pitch - 69.0F) / 12)) * (1LL << 32) / FS;
    freq = ((freq >> 4) << 4) + 7; // 少し半端な値にする
    Osc_freq_table[pitch] = freq;
  }

  // TODO: 参照テーブルを追加して、Osc_wave_tablesの重複データを無くしたい
  for (uint32_t pitch4 = 0; pitch4 < 31; ++pitch4) {
    uint32_t harm_max = // 最大倍音次数
        (((FS - 1.0F) / 2) * (1LL << 32) / FS) / Osc_freq_table[pitch4 << 2];
    if (harm_max > 127) { harm_max = 127; }

    for (uint32_t i = 0; i < 512; ++i) {
      float sum = 0.0F;
      for (uint32_t k = 1; k <= harm_max; ++k) {
        sum += (2 / PI) * (sinf(2 * PI * k * i / 512) / k);
      }
      sum *= 0.25F;
      Osc_wave_tables[pitch4][i] = float2fix(sum, 14);
    }
  }
}

static inline Q28 Osc_process(uint32_t id, uint32_t pitch, Q14 pitch_mod) {
  static uint32_t phase[4]; // 位相
  phase[id] += Osc_freq_table[pitch] + (id * 256); // 周波数を少しずらす

  uint32_t pitch4 = (pitch + 3) >> 2;
  Q14* wave_table = Osc_wave_tables[pitch4];
  uint32_t curr_index = phase[id] >> 23;
  uint32_t next_index = (curr_index + 1) & 0x000001FF;
  Q14 curr_sample = wave_table[curr_index];
  Q14 next_sample = wave_table[next_index];
  Q14 next_weight = (phase[id] >> 9) & 0x3FFF;
  return (curr_sample << 14) +
         ((next_sample - curr_sample) * next_weight);
}

//////// フィルタ ////////////////////////////////
struct F_COEFS { Q28 b0_a0, a1_a0, a2_a0; };
struct F_COEFS Fil_table[6][481]; // フィルタ係数群テーブル

static volatile int32_t Fil_cut = 120; // カットオフ設定値
static volatile int32_t Fil_res = 0;   // レゾナンス設定値
static volatile int32_t Fil_mod = 0;   // カットオフ変調量設定値

static void Fil_init() {
  for (uint32_t res = 0; res < 6; ++res) {
    for (uint32_t cut = 0; cut < 481; ++cut) {
      float f0    = FA * powf(2, ((cut / 4.0F) - 54) / 12);
      float w0    = 2 * PI * f0 / FS;
      float q     = powf(sqrtf(2), res - 1.0F);
      float alpha = sinf(w0) / (2 * q);
      float b0    = (1 - cosf(w0)) / 2;
      float a0    =  1 + alpha;
      float a1    = -2 * cosf(w0);
      float a2    =  1 - alpha;
      Fil_table[res][cut].b0_a0 = float2fix(b0 / a0, 28);
      Fil_table[res][cut].a1_a0 = float2fix(a1 / a0, 28);
      Fil_table[res][cut].a2_a0 = float2fix(a2 / a0, 28);
    }
  }
}

static inline int32_t mul_32_32_h32(int32_t x, int32_t y) {
  // 32ビット同士の乗算結果の上位32ビット
  int32_t x1 = x >> 16; uint32_t x0 = x & 0xFFFF;
  int32_t y1 = y >> 16; uint32_t y0 = y & 0xFFFF;
  int32_t x0_y1 = x0 * y1;
  int32_t z = ((x0 * y0) >> 16) + (x1 * y0) + (x0_y1 & 0xFFFF);
  return (z >> 16) + (x0_y1 >> 16) + (x1 * y1);
}

static inline Q28 Fil_process(uint32_t id, Q28 x0, Q14 cut_mod) {
  static uint32_t f_counter[4];    // フィルタ処理回数
  static uint32_t curr_cut[4];     // カットオフ現在値
  int32_t targ_cut = Fil_cut << 2; // カットオフ目標値（設定値の4倍）
  targ_cut += (Fil_mod * cut_mod) >> (14 - 2);
  targ_cut += (targ_cut < 0)   * (0 - targ_cut);
  targ_cut -= (targ_cut > 480) * (targ_cut - 480);
#if 1
  curr_cut[id] += (curr_cut[id] < targ_cut);
  curr_cut[id] -= (curr_cut[id] > targ_cut);
#else
  uint32_t delta = ((++f_counter[id] & 0x3) == 0);
  curr_cut[id] += (curr_cut[id] < targ_cut) * delta;
  curr_cut[id] -= (curr_cut[id] > targ_cut) * delta;
#endif
  struct F_COEFS* coefs = &Fil_table[Fil_res][curr_cut[id]];

  static Q28 x1[4], x2[4], y1[4], y2[4];
  Q28 x3 = x0 + (x1[id] << 1) + x2[id];
  Q28 y0 = mul_32_32_h32(coefs->b0_a0, x3)        << 4;
  y0    -= mul_32_32_h32(coefs->a1_a0, y1[id]) << 4;
  y0    -= mul_32_32_h32(coefs->a2_a0, y2[id]) << 4;
  x2[id] = x1[id]; y2[id] = y1[id];
  x1[id] = x0;        y1[id] = y0;
  return y0;
}

//////// アンプ //////////////////////////////////
static inline int32_t mul_32_16_h32(int32_t x, int16_t y) {
  // 32ビットと16ビットの乗算結果の上位32ビット
  int32_t x1 = x >> 16; uint32_t x0 = x & 0xFFFF;
  return ((x0 * y) >> 16) + (x1 * y);
}

static inline Q28 Amp_process(uint32_t id, Q28 in, Q14 gain) {
  return mul_32_16_h32(in, gain) << 2;
}

//////// EG（Envelope Generator） ////////////////
static volatile int32_t EG_att = 0;  // アタック・タイム設定値
static volatile int32_t EG_dec = 0;  // ディケイ・タイム設定値
static volatile int32_t EG_sus = 64; // サスティン・レベル設定値

static inline Q14 EG_process(uint32_t id, int32_t gate) {
  static Q14 curr_level[4];           // レベル現在値
  Q14        targ_level = gate << 14; // レベル目標値
#if 1
  curr_level[id] =
      targ_level - (((targ_level - curr_level[id]) * 16368) / 16384);
#else
  curr_level[id] =
      targ_level - (((targ_level - curr_level[id]) * 255) / 256);
#endif
  return curr_level[id];
}

//////// LFO（Low Frequency Oscillator） /////////
static volatile int32_t LFO_dep = 0;  // 深さ設定値
static volatile int32_t LFO_rat = 32; // 速さ設定値

static inline Q14 LFO_process(uint32_t id) {
  return 0; // TODO
}

//////// PWMオーディオ出力部 /////////////////////
#define PWMA_GPIO  (28)           // PWM出力するGPIO番号
#define PWMA_SLICE (6)            // PWMスライス番号
#define PWMA_CHAN  (PWM_CHAN_A)   // PWMチャンネル
#define PWMA_CYCLE (FCLKSYS / FS) // PWM周期

static void pwm_irq_handler();

static void PWMA_init() {
  gpio_set_function(PWMA_GPIO, GPIO_FUNC_PWM);
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_set_irq_enabled(PWMA_SLICE, true);
  pwm_set_wrap(PWMA_SLICE, PWMA_CYCLE - 1);
  pwm_set_chan_level(PWMA_SLICE, PWMA_CHAN, PWMA_CYCLE / 2);
  pwm_set_enabled(PWMA_SLICE, true);
}

static inline void PWMA_process(Q28 in) {
  int32_t level_int32 = (in >> 18) + (PWMA_CYCLE / 2);
  uint16_t level = (level_int32 > 0) * level_int32;
  pwm_set_chan_level(PWMA_SLICE, PWMA_CHAN, level);
}

//////// 割り込みハンドラとメイン関数 ////////////
static volatile uint16_t s_time     = 0; // 開始時間
static volatile uint16_t max_s_time = 0; // 最大開始時間
static volatile uint16_t p_time     = 0; // 処理時間
static volatile uint16_t max_p_time = 0; // 最大処理時間

static volatile int32_t  gate_voice[4];  // ゲート制御値（ボイス毎）
static volatile uint32_t pitch_voice[4]; // ピッチ制御値（ボイス毎）
static volatile int32_t  octave_shift;   // キーのオクターブシフト量

static void pwm_irq_handler() {
  pwm_clear_irq(PWMA_SLICE);
  s_time = pwm_get_counter(PWMA_SLICE);

  Q28 voice_out[4];
  for (uint32_t id = 0; id < 4; ++id) {
    Q14 eg_out = EG_process(id, gate_voice[id]);
    Q28 osc_out = Osc_process(id, pitch_voice[id], ONE_Q14);
    Q28 fil_out = Fil_process(id, osc_out, eg_out);
    Q28 amp_out = Amp_process(id, fil_out, eg_out);
    voice_out[id] = amp_out;
  }
  PWMA_process((voice_out[0] + voice_out[1] +
                voice_out[2] + voice_out[3]) >> 2);

  uint16_t end_time = pwm_get_counter(PWMA_SLICE);
  p_time = end_time - s_time; // 簡略化
  max_s_time += (s_time > max_s_time) * (s_time - max_s_time);
  max_p_time += (p_time > max_p_time) * (p_time - max_p_time);
}

static inline void note_on_off(uint32_t key)
{
  uint32_t pitch = key + (octave_shift * 12);
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
  for (uint32_t id = 0; id < 4; ++id) { gate_voice[id] = 0; }
}

int main() {
  for (uint32_t id = 0; id < 4; ++id) { pitch_voice[id] = 60; }
#if 1
  note_on_off(60); note_on_off(64); note_on_off(67); note_on_off(71);
#endif

  set_sys_clock_khz(FCLKSYS / 1000, true);
  stdio_init_all();
  Osc_init(); Fil_init(); PWMA_init();

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

    case '1': if (octave_shift > -5) { --octave_shift; } break;
    case '9': if (octave_shift < +4) { ++octave_shift; } break;
    case '0': all_notes_off(); break;

    case 'A': if (Osc_wav > 0)   { --Osc_wav; } break;
    case 'a': if (Osc_wav < 1)   { ++Osc_wav; } break;
    case 'S': if (Osc_2co > 0)   { --Osc_2co; } break;
    case 's': if (Osc_2co < +24) { ++Osc_2co; } break;
    case 'D': if (Osc_2fi > 0)   { --Osc_2fi; } break;
    case 'd': if (Osc_2fi < +32) { ++Osc_2fi; } break;
    case 'F': if (Osc_mix > 0)   { --Osc_mix; } break;
    case 'f': if (Osc_mix < 64)  { ++Osc_mix; } break;

    case 'G': if (Fil_cut > 0)   { --Fil_cut; } break;
    case 'g': if (Fil_cut < 120) { ++Fil_cut; } break;
    case 'H': if (Fil_res > 0)   { --Fil_res; } break;
    case 'h': if (Fil_res < 5)   { ++Fil_res; } break;
    case 'J': if (Fil_mod > 0)   { --Fil_mod; } break;
    case 'j': if (Fil_mod < +60) { ++Fil_mod; } break;

    case 'Z': if (EG_att  > 0)   { --EG_att;  } break;
    case 'z': if (EG_att  < 64)  { ++EG_att;  } break;
    case 'X': if (EG_dec  > 0)   { --EG_dec;  } break;
    case 'x': if (EG_dec  < 64)  { ++EG_dec;  } break;
    case 'C': if (EG_sus  > 0)   { --EG_sus;  } break;
    case 'c': if (EG_sus  < 64)  { ++EG_sus;  } break;

    case 'B': if (LFO_rat > 0)   { --LFO_rat; } break;
    case 'b': if (LFO_rat < 64)  { ++LFO_rat; } break;
    case 'N': if (LFO_dep > 0)   { --LFO_dep; } break;
    case 'n': if (LFO_dep < 64)  { ++LFO_dep; } break;
    }
    static uint32_t loop_counter = 0; // ループ回数
    if ((++loop_counter & 0xFFFFF) == 0) {
      printf("Pitch          : [ %3lu, %3lu, %3lu, %3lu ]\n",
          pitch_voice[0], pitch_voice[1], pitch_voice[2], pitch_voice[3]);
      printf("Gate           : [   %ld,   %ld,   %ld,   %ld ]\n",
          gate_voice[0], gate_voice[1], gate_voice[2], gate_voice[3]);
      printf("Octave Shift   :  %+1ld (1/9)\n",  octave_shift);
      printf("Osc Wave       :   %1ld (A/a)\n",  Osc_wav);
      printf("Osc 2 Coarse   : %+3ld (S/s)\n",   Osc_2co);
      printf("Osc 2 Fine     : %+3ld (D/d)\n",   Osc_2fi);
      printf("Osc 1/2 Mix    :  %2ld (F/f)\n",   Osc_mix);
      printf("Fil Cutoff     : %3ld (G/g)\n",    Fil_cut);
      printf("Fil Resonance  :   %1ld (H/h)\n",  Fil_res);
      printf("Fil EG Amount  : %+3ld (J/j)\n",   Fil_mod);
      printf("EG Attack      :  %2ld (Z/z)\n",   EG_att);
      printf("EG Decay       :  %2ld (X/x)\n",   EG_dec);
      printf("EG Sustain     :  %2ld (C/c)\n",   EG_sus);
      printf("LFO Depth      :  %2ld (B/b)\n",   LFO_dep);
      printf("LFO Rate       :  %2ld (N/n)\n",   LFO_rat);
      printf("Start Time     : %4u/%4u\n",   s_time, max_s_time);
      printf("Processing Time: %4u/%4u\n\n", p_time, max_p_time);
    }
  }
}
