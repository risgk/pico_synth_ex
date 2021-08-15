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
//////// オシレータ //////////////////////////////
static uint32_t Osc_freq_table[8] = {   // 周波数テーブル
  261.6F * (1LL << 32) / FS,
  293.7F * (1LL << 32) / FS,
  329.6F * (1LL << 32) / FS,
  349.2F * (1LL << 32) / FS,
  392.0F * (1LL << 32) / FS,
  440.0F * (1LL << 32) / FS,
  493.9F * (1LL << 32) / FS,
  523.3F * (1LL << 32) / FS };
static Q14 Osc_wave_tables[8][512];    // 波形テーブル群
static volatile uint32_t Osc_pitch[4]; // ピッチ設定値

static void Osc_init() {
  for (uint32_t pitch = 0; pitch < 8; ++pitch) {
    uint32_t harm_max = // 最大倍音次数
      (23000.0F * (1LL << 32) / FS)
                          / Osc_freq_table[pitch];
    if (harm_max > 127) { harm_max = 127; }

    for (uint32_t i = 0; i < 512; ++i) {
      float sum = 0.0F;
      for (uint32_t k = 1; k <= harm_max; ++k) {
        sum += (2 / PI)
               * (sinf(2 * PI * k * i / 512) / k);
      }
      sum *= 0.25F;
      Osc_wave_tables[pitch][i] = float2fix(sum,
                                            14);
    }
  }
}
static inline Q28 Osc_process(uint32_t voice) {
  static uint32_t phase[4]; // 位相
  phase[voice] += Osc_freq_table[Osc_pitch[voice]];

  Q14* wave_table = Osc_wave_tables[Osc_pitch[voice]];
  uint32_t curr_index = phase[voice] >> 23;
  uint32_t next_index = (curr_index + 1) &
                                       0x000001FF;
  Q14 curr_sample = wave_table[curr_index];
  Q14 next_sample = wave_table[next_index];
  Q14 next_weight = (phase[voice] >> 9) & 0x3FFF;
  return (curr_sample << 14) +
      ((next_sample - curr_sample) * next_weight);
}
//////// フィルタ ////////////////////////////////
struct F_COEFS { Q28 b0_a0, a1_a0, a2_a0; };
struct F_COEFS Fil_table[6][121]; // フィルタ係数群テーブル
static volatile uint32_t Fil_cut = 120; // カットオフ設定値
static volatile uint32_t Fil_res = 0;   // レゾナンス設定値

static void Fil_init() {
  for (uint32_t res = 0; res < 8; ++res) {
    for (uint32_t cut = 0; cut < 121; ++cut) {
      float f0    = 19912.1F
                   * powf(2, (cut - 120.0F) / 12);
      float w0    = 2 * PI * f0 / FS;
      float q     = powf(sqrtf(2), res - 1.0F);
      float alpha = sinf(w0) / (2 * q);
      float b0    = (1 - cosf(w0)) / 2;
      float a0    =  1 + alpha;
      float a1    = -2 * cosf(w0);
      float a2    =  1 - alpha;
      Fil_table[res][cut].b0_a0 =
                           float2fix(b0 / a0, 28);
      Fil_table[res][cut].a1_a0 =
                           float2fix(a1 / a0, 28);
      Fil_table[res][cut].a2_a0 =
                           float2fix(a2 / a0, 28);
    }
  }
}
static inline int32_t mul_32_32_h32(int32_t x,
                                    int32_t y) {
  // 32ビット同士の乗算結果の上位32ビット
  int32_t x1 = x >> 16; uint32_t x0 = x & 0xFFFF;
  int32_t y1 = y >> 16; uint32_t y0 = y & 0xFFFF;
  int32_t x0_y1 = x0 * y1;
  int32_t z = ((x0 * y0) >> 16) +
               (x1 * y0) + (x0_y1 & 0xFFFF);
  return (z >> 16) + (x0_y1 >> 16) + (x1 * y1);
}
static inline Q28 Fil_process(uint32_t voice, Q28 x0) {
  static uint32_t f_counter[4];       // フィルタ処理回数
  static uint32_t curr_cut[4];        // カットオフ現在値
  uint32_t        targ_cut = Fil_cut; // カットオフ目標値
  uint32_t delta = ((++f_counter[voice] & 0xF) == 0);
  curr_cut[voice] += (curr_cut[voice] < targ_cut) * delta;
  curr_cut[voice] -= (curr_cut[voice] > targ_cut) * delta;
  struct F_COEFS* coefs =
    &Fil_table[Fil_res][curr_cut[voice]];

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
static inline int32_t mul_32_16_h32(int32_t x,
                                    int16_t y) {
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
static volatile int32_t EG_gate_on[4]; // EGゲート設定値

static inline Q14 EG_process(uint32_t voice) {
  static Q14 curr_level[4];                        // レベル現在値
  Q14        targ_level = EG_gate_on[voice] << 14; // レベル目標値
  curr_level[voice] = targ_level -
                      (((targ_level - curr_level[voice]) * 255) / 256);
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
  irq_set_exclusive_handler(PWM_IRQ_WRAP,
                            pwm_irq_handler);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_set_irq_enabled(PWMA_SLICE, true);
  pwm_set_wrap(PWMA_SLICE, PWMA_CYCLE - 1);
  pwm_set_chan_level(PWMA_SLICE,
                     PWMA_CHAN, PWMA_CYCLE / 2);
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

static void pwm_irq_handler() {
  pwm_clear_irq(PWMA_SLICE);
  s_time = pwm_get_counter(PWMA_SLICE);

  Q28 voice_level[4];
  for (uint32_t voice = 0; voice < 4; ++voice) {
    Q14 eg_out  = EG_process(voice);
    Q28 osc_out = Osc_process(voice);
    Q28 fil_out = Fil_process(voice, osc_out);
    Q28 amp_out = Amp_process(voice, fil_out, eg_out);
    voice_level[voice] = amp_out;
  }
  PWMA_process((voice_level[0] + voice_level[1] +
                voice_level[2] + voice_level[3]) >> 2);

  uint16_t end_time = pwm_get_counter(PWMA_SLICE);
  p_time = ((end_time - s_time) + PWMA_CYCLE)
                                         % PWMA_CYCLE;
  max_s_time += (s_time > max_s_time) *
                (s_time - max_s_time);
  max_p_time += (p_time > max_p_time) *
                (p_time - max_p_time);
}
int main() {
  set_sys_clock_khz(FCLKSYS / 1000, true);
  stdio_init_all();
  Osc_init(); Fil_init(); PWMA_init();
#if 1
  Osc_pitch[0] = 0; EG_gate_on[0] = 1; 
  Osc_pitch[1] = 2; EG_gate_on[1] = 1; 
  Osc_pitch[2] = 4; EG_gate_on[2] = 1; 
  Osc_pitch[3] = 6; EG_gate_on[3] = 1; 
#endif
  while (true) {
    switch (getchar_timeout_us(0)) {
#if 0
    case '1': Osc_pitch = 0; Amp_on = 1; break; // ド
    case '2': Osc_pitch = 1; Amp_on = 1; break; // レ
    case '3': Osc_pitch = 2; Amp_on = 1; break; // ミ
    case '4': Osc_pitch = 3; Amp_on = 1; break; // ファ
    case '5': Osc_pitch = 4; Amp_on = 1; break; // ソ
    case '6': Osc_pitch = 5; Amp_on = 1; break; // ラ
    case '7': Osc_pitch = 6; Amp_on = 1; break; // シ
    case '8': Osc_pitch = 7; Amp_on = 1; break; // ド
    case '0':                Amp_on = 0; break;
#endif
    case 'z': if (Fil_cut > 0)   { --Fil_cut; } break;
    case 'x': if (Fil_cut < 120) { ++Fil_cut; } break;
    case 'n': if (Fil_res > 0)   { --Fil_res; } break;
    case 'm': if (Fil_res < 7)   { ++Fil_res; } break;
    }
    static uint32_t loop_counter = 0; // ループ回数
    if ((++loop_counter & 0xFFFFF) == 0) {
      printf("p:[%lu,%lu,%lu,%lu], g:[%ld,%ld,%ld,%ld], c:%3lu, r:%lu, ",
        Osc_pitch[0], Osc_pitch[1], Osc_pitch[2], Osc_pitch[3],
        EG_gate_on[0], EG_gate_on[1], EG_gate_on[2], EG_gate_on[3],
        Fil_cut, Fil_res);
      printf("start:%4u/%4u, processing:%4u/%4u\n",
        s_time, max_s_time, p_time, max_p_time);
    }
  }
}
