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
static uint32_t          Osc_freq_table[121];      // 周波数テーブル
static Q14               Osc_wave_tables[31][512]; // 波形テーブル群

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

static inline Q28 Osc_process(uint32_t voice, uint32_t pitch) {
  static uint32_t phase[4]; // 位相
  phase[voice] += Osc_freq_table[pitch] + (voice * 256); // 周波数を少しずらす

  uint32_t pitch4 = (pitch + 3) >> 2;
  Q14* wave_table = Osc_wave_tables[pitch4];
  uint32_t curr_index = phase[voice] >> 23;
  uint32_t next_index = (curr_index + 1) & 0x000001FF;
  Q14 curr_sample = wave_table[curr_index];
  Q14 next_sample = wave_table[next_index];
  Q14 next_weight = (phase[voice] >> 9) & 0x3FFF;
  return (curr_sample << 14) +
         ((next_sample - curr_sample) * next_weight);
}

//////// フィルタ ////////////////////////////////
struct F_COEFS { Q28 b0_a0, a1_a0, a2_a0; };
struct F_COEFS Fil_table[8][121]; // フィルタ係数群テーブル

static volatile uint32_t Fil_cut = 120; // カットオフ設定値
static volatile uint32_t Fil_res = 0;   // レゾナンス設定値

static void Fil_init() {
  for (uint32_t res = 0; res < 8; ++res) {
    for (uint32_t cut = 0; cut < 121; ++cut) {
      float f0    = FA * powf(2, (cut - 54.0F) / 12);
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

static inline Q28 Fil_process(uint32_t voice, Q28 x0) {
  static uint32_t f_counter[4];       // フィルタ処理回数
  static uint32_t curr_cut[4];        // カットオフ現在値
  uint32_t        targ_cut = Fil_cut; // カットオフ目標値
  uint32_t delta = ((++f_counter[voice] & 0xF) == 0);
  curr_cut[voice] += (curr_cut[voice] < targ_cut) * delta;
  curr_cut[voice] -= (curr_cut[voice] > targ_cut) * delta;
  struct F_COEFS* coefs = &Fil_table[Fil_res][curr_cut[voice]];

  static Q28 x1[4], x2[4], y1[4], y2[4];
  Q28 x3 = x0 + (x1[voice] << 1) + x2[voice];
  Q28 y0 = mul_32_32_h32(coefs->b0_a0, x3)        << 4;
  y0    -= mul_32_32_h32(coefs->a1_a0, y1[voice]) << 4;
  y0    -= mul_32_32_h32(coefs->a2_a0, y2[voice]) << 4;
  x2[voice] = x1[voice]; y2[voice] = y1[voice];
  x1[voice] = x0;        y1[voice] = y0;
  return y0;
}

//////// アンプ //////////////////////////////////
static inline int32_t mul_32_16_h32(int32_t x, int16_t y) {
  // 32ビットと16ビットの乗算結果の上位32ビット
  int32_t x1 = x >> 16; uint32_t x0 = x & 0xFFFF;
  return ((x0 * y) >> 16) + (x1 * y);
}

static inline Q28 Amp_process(uint32_t voice, Q28 in, Q14 gain_in) {
  return mul_32_16_h32(in, gain_in) << 2;
}

//////// LFO（Low Frequency Oscillator） /////////
// TODO

//////// EG（Envelope Generator） ////////////////
static inline Q14 EG_process(uint32_t voice, int32_t gate) {
  static Q14 curr_level[4];           // レベル現在値
  Q14        targ_level = gate << 14; // レベル目標値
  curr_level[voice] =
      targ_level - (((targ_level - curr_level[voice]) * 255) / 256);
  return curr_level[voice];
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

static volatile int32_t  voice_gate[4];  // ゲート制御値（ボイス毎）
static volatile uint32_t voice_pitch[4]; // ピッチ制御値（ボイス毎）
static volatile int32_t  octave_shift;   // オクターブシフト量

static void pwm_irq_handler() {
  pwm_clear_irq(PWMA_SLICE);
  s_time = pwm_get_counter(PWMA_SLICE);

  Q28 voice_level[4];
  for (uint32_t voice = 0; voice < 4; ++voice) {
    Q14 eg_out = EG_process(voice, voice_gate[voice]);
    Q28 osc_out = Osc_process(voice, voice_pitch[voice]);
    Q28 fil_out = Fil_process(voice, osc_out);
    Q28 amp_out = Amp_process(voice, fil_out, eg_out);
    voice_level[voice] = amp_out;
  }
  PWMA_process((voice_level[0] + voice_level[1] +
                voice_level[2] + voice_level[3]) >> 2);

  uint16_t end_time = pwm_get_counter(PWMA_SLICE);
  p_time = end_time - s_time; // 簡略化
  max_s_time += (s_time > max_s_time) * (s_time - max_s_time);
  max_p_time += (p_time > max_p_time) * (p_time - max_p_time);
}

static inline void note_on_off(uint32_t key)
{
  uint32_t pitch = key + (octave_shift * 12);
  if      (voice_pitch[0] == pitch) { voice_gate[0] = (voice_gate[0] == 0); }
  else if (voice_pitch[1] == pitch) { voice_gate[1] = (voice_gate[1] == 0); }
  else if (voice_pitch[2] == pitch) { voice_gate[2] = (voice_gate[2] == 0); }
  else if (voice_pitch[3] == pitch) { voice_gate[3] = (voice_gate[3] == 0); }
  else if (voice_gate[0] == 0) { voice_pitch[0] = pitch; voice_gate[0] = 1; }
  else if (voice_gate[1] == 0) { voice_pitch[1] = pitch; voice_gate[1] = 1; }
  else if (voice_gate[2] == 0) { voice_pitch[2] = pitch; voice_gate[2] = 1; }
  else                         { voice_pitch[3] = pitch; voice_gate[3] = 1; }
}

static inline void all_notes_off()
{
  for (uint32_t voice = 0; voice < 4; ++voice) { voice_gate[voice] = 0; }
}

int main() {
  for (uint32_t voice = 0; voice < 4; ++voice) { voice_pitch[voice] = 60; }
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
    case '9': if (octave_shift < 4)  { ++octave_shift; } break;
    case '0': all_notes_off(); break;
    case 'z': if (Fil_cut > 0)   { --Fil_cut; } break;
    case 'x': if (Fil_cut < 120) { ++Fil_cut; } break;
    case 'n': if (Fil_res > 0)   { --Fil_res; } break;
    case 'm': if (Fil_res < 7)   { ++Fil_res; } break;
    }
    static uint32_t loop_counter = 0; // ループ回数
    if ((++loop_counter & 0xFFFFF) == 0) {
      printf("pitch:[%3lu,%3lu,%3lu,%3lu], gate:[%ld,%ld,%ld,%ld], oct:%+ld, ",
          voice_pitch[0], voice_pitch[1], voice_pitch[2], voice_pitch[3],
          voice_gate[0], voice_gate[1], voice_gate[2], voice_gate[3],
          octave_shift);
      printf("cut:%3lu, res:%lu, ",
          Fil_cut, Fil_res);
      printf("start:%4u/%4u, processing:%4u/%4u\n",
          s_time, max_s_time, p_time, max_p_time);
    }
  }
}
