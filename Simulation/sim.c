/*
 * sim.c : simulation de la maquette sur PC
 *
 * Le vrai code du firmware (Firmware/LM_V2/Core, fichiers LM*) est compile
 * pour PC avec un main.h de remplacement (stub/main.h) et tourne sur un
 * modele de la maquette, periode par periode a 5 kHz :
 *   - 2 axes instables : p'' = w0^2.p + beta.u + perturbation ;
 *   - 4 bobines R-L pilotees par L293D en PWM sur EN (impulsion centree,
 *     chute de tension du pont, roue libre par les diodes, conduction
 *     discontinue autour de 0 A) ;
 *   - ADC 12 bits avec offsets et bruit sur les courants et les capteurs Hall.
 * La maquette simulee (w0 = 25 rad/s, beta = 70000) est volontairement
 * differente du reglage provisoire de LM_config.h (30 rad/s, 90000).
 *
 * Lancement : python simuler.py (voir README.md).
 */
#include "LM.h"
#include "LM_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

GPIO_TypeDef gA, gB, gC; ADC_TypeDef adc1_inst;
static TIM_TypeDef tim1 = { 0, TIM_CR1_CMS, 1, 8000, {0,0,0,0} };
ADC_HandleTypeDef hadc1 = { &adc1_inst, { LM_NB_ADC } };
DMA_HandleTypeDef hdma_adc1; TIM_HandleTypeDef htim1 = { &tim1 }; UART_HandleTypeDef huart2;
static volatile uint16_t *adc_buf; static uint32_t tick_ms; static int quiet_data = 1;
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h, uint32_t m) { (void)h; (void)m; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *b, uint32_t n) { (void)h; (void)n; adc_buf = (volatile uint16_t *)b; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h; (void)c; return HAL_OK; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return 80000000u; }
uint32_t HAL_GetTick(void) { return tick_ms; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)t; int c = 0; for (int i = 0; i < n; i++) c += p[i] == ','; if (!(quiet_data && p[0] != '#' && c != 4)) fwrite(p, 1, n, stdout); return HAL_OK; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s) { (void)p; (void)pin; (void)s; }
void Error_Handler(void) { printf("ERROR_HANDLER\n"); exit(1); }

/* ---- maquette ---- */
typedef struct { double w0, beta, d; double p, v; } axis_t;
static axis_t ax_[2];
static double icoil[4], Rt = 11.0, Lt = 0.025, Vbus = 12.0, Vdrop = 2.6, noise_mT = 0.05, ioff[4] = {0.003,-0.002,0.004,-0.001};
static double tsim; static int frozen = 1; static int ideal_cur = 0; static int no_bias = 0;
static double gauss(void) { double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0); return sqrt(-2 * log(u)) * cos(2 * M_PI * v); }
static uint16_t q12(double v) { double r = floor(v / 3.3 * 4095.0 + 0.5); if (r < 0) r = 0; if (r > 4095) r = 4095; return (uint16_t)r; }

static void period(void)
{
  const int NS = 100; const double dt = LM_TS / NS; double iavg[4] = {0,0,0,0};
  if (ideal_cur) { for (int k = 0; k < 4; k++) { icoil[k] = lm.i_ref[k]; iavg[k] = icoil[k]; } } else
  for (int k = 0; k < 4; k++) {
    const double sd = lm.duty[k], s = (sd > 0) ? 1 : (sd < 0 ? -1 : 0), d = fabs(sd);
    for (int j = 0; j < NS; j++) {
      const double ph = (j + 0.5) / NS; const int en = (fabs(ph - 0.5) < d / 2) && s != 0;   /* impulsion centree */
      double V;
      double i = icoil[k];
      if (en) V = s * Vbus - Vdrop * ((i > 0) - (i < 0));
      else V = (i > 1e-9) ? -Vbus : (i < -1e-9 ? Vbus : 0);   /* roue libre par les diodes */
      double ni = i + (V - Rt * i) / Lt * dt;
      if (!en && ((i > 0 && ni < 0) || (i < 0 && ni > 0))) ni = 0;   /* conduction discontinue */
      icoil[k] = ni; iavg[k] += ni / NS;
    }
  }
  if (!frozen) for (int a = 0; a < 2; a++) {
    const double u = (a == 0) ? (iavg[0] - iavg[1]) / 2 : (iavg[2] - iavg[3]) / 2;
    const double acc = ax_[a].w0 * ax_[a].w0 * ax_[a].p + ax_[a].beta * u + ax_[a].d;
    ax_[a].v += acc * LM_TS; ax_[a].p += ax_[a].v * LM_TS;
  }
  const double mid = 3.3 / 2;
  for (int k = 0; k < 4; k++) adc_buf[LM_ADC_I_X1 + k] = q12(mid + (iavg[k] + ioff[k]) * LM_I_V_PER_A + 0.0005 * gauss());
  adc_buf[LM_ADC_HALL1] = q12(mid + (ax_[0].p + noise_mT * gauss()) * LM_HALL_V_PER_MT + 0.02);
  adc_buf[LM_ADC_HALL2] = q12(mid + (ax_[1].p + noise_mT * gauss()) * LM_HALL_V_PER_MT - 0.01);
  adc_buf[LM_ADC_HALL3] = q12(mid + (noise_mT * gauss()) * LM_HALL_V_PER_MT + 0.015);
  adc_buf[LM_ADC_VBUS]  = q12(Vbus / LM_VBUS_RATIO);
  HAL_ADC_ConvCpltCallback(&hadc1);
  tsim += LM_TS; tick_ms = (uint32_t)(tsim * 1000);
  if (((uint32_t)(tsim * LM_F_LOOP_HZ + 0.5)) % 10u == 0u) LM_Background();
}
static void run(double s) { long n = lround(s * LM_F_LOOP_HZ); for (long j = 0; j < n; j++) period(); }
static void start(double w0t, double betat)
{
  memset(ax_, 0, sizeof(ax_)); memset(icoil, 0, sizeof(icoil)); tsim = 0; frozen = 1;
  for (int a = 0; a < 2; a++) { ax_[a].w0 = w0t; ax_[a].beta = betat; }
  LM_Init(); if (no_bias || getenv("NOBIAS")) lm.i_bias = 0; LM_Start(); run(0.6);          /* calibration des zeros, objet retire */
  frozen = 0; ax_[0].p = 0.5; ax_[1].p = -0.3; lm.mode = LM_MODE_LEVITATION;
}
/* metriques sur une fenetre */
typedef struct { double pmax, pmin, pend, imax; } win_t;
static win_t watch(double s, int axis)
{
  win_t w = { -1e9, 1e9, 0, 0 }; long n = lround(s * LM_F_LOOP_HZ);
  for (long j = 0; j < n; j++) { period(); double p = ax_[axis].p; if (p > w.pmax) w.pmax = p; if (p < w.pmin) w.pmin = p;
    for (int k = 0; k < 4; k++) if (fabs(icoil[k]) > w.imax) w.imax = fabs(icoil[k]); }
  w.pend = ax_[axis].p; return w;
}
static void curloop(double bias, double amp)
{
  start(25, 70000); lm.mode = LM_MODE_CURRENT; frozen = 1; ax_[0].p = 0; ax_[1].p = 0;
  const double fs[] = {5, 12, 25, 50, 100};
  for (int q = 0; q < 5; q++) { double f = fs[q], ar = 0, ai = 0, cr = 0, ci = 0; long n = lround(LM_F_LOOP_HZ / f) * 10;
    for (long j = 0; j < n; j++) { double th = 2 * M_PI * f * j * LM_TS, r = amp * sin(th); lm.i_manual[0] = (float)(bias + r); period();
      if (j >= n / 2) { double y = icoil[0] - bias; ar += r * cos(th); ai -= r * sin(th); cr += y * cos(th); ci -= y * sin(th); } }
    double g = hypot(cr, ci) / hypot(ar, ai), ph = atan2(ci * ar - cr * ai, cr * ar + ci * ai) * 180 / M_PI;
    printf("  %5.0f Hz : gain %.2f, phase %6.1f deg\n", f, g, ph); }
}
int main(int argc, char **argv)
{
  if (argc > 1 && argv[1][0] == 'L') {   /* journal complet pour tester l'outil PC */
    quiet_data = 0; start(25, 70000); frozen = 1; ax_[0].p = 0; ax_[1].p = 0; lm.mode = LM_MODE_OFF; run(0.05);
    lm.mode = LM_MODE_VOLTAGE; lm.step_coil = 0; lm.step_value = 4.0f; run(0.02); lm.cmd = LM_CMD_STEP; run(0.4);
    lm.v_manual[0] = 0; lm.mode = LM_MODE_OFF; run(0.05);
    ax_[0].p = 0.3; ax_[0].v = 0; ax_[1].p = -0.2; ax_[1].v = 0; frozen = 0; lm.mode = LM_MODE_LEVITATION; run(1.0); lm.id_axis = 0; lm.cmd = LM_CMD_SWEEP; run(8.0);
    return 0; }
  if (argc > 1 && argv[1][0] == 'c') { printf("boucle de courant, sinus 0,1 A autour de 0 A :\n"); curloop(0, 0.1); printf("autour de 0,2 A :\n"); curloop(0.2, 0.1); return 0; }
  if (argc > 1 && argv[1][0] == 'i') ideal_cur = 1;
  if (argc > 1 && argv[1][0] == 'n') { no_bias = 1; printf("(sans courant de biais)\n"); }
  const double W0T = 25, BETAT = 70000;   /* maquette "vraie", differente du reglage provisoire */
  srand(1);
  printf("== S1 : maintien, puis perturbation constante sur X (2000 mT/s2) ==\n");
  for (int pid = 1; pid >= 0; pid--) {
    start(W0T, BETAT); if (!pid) { lm.ax.ki = 0; lm.ay.ki = 0; }
    win_t a = watch(1.0, 0); ax_[0].d = 2000; win_t b = watch(2.0, 0);
    printf("%s : apres 1 s |p|max=%.3f mT, fin=%.3f mT | avec perturbation : fin=%.3f mT (theorie PD %.3f), mode=%d defaut=%d, imax=%.3f A\n",
           pid ? "PID" : "PD ", fmax(fabs(a.pmax), fabs(a.pmin)), a.pend, b.pend, 2000.0 / (BETAT * lm.ax.kp - W0T * W0T), lm.mode, lm.fault, b.imax);
  }
  printf("== S2 : echelon de consigne +1 mT sur X, b = 1 puis b = 0,5 ==\n");
  for (int t = 0; t < 2; t++) {
    start(W0T, BETAT); lm.ax.b = t ? 0.5f : 1.0f; run(1.5);
    double r0 = lm.ax.ref; lm.ax.ref = (float)(r0 + 1.0);
    long n = lround(1.5 * LM_F_LOOP_HZ); double pk = -1e9, t5 = 0, y0 = ax_[0].p;
    double *hist = malloc(sizeof(double) * n);
    for (long j = 0; j < n; j++) { period(); hist[j] = ax_[0].p; if (hist[j] > pk) pk = hist[j]; }
    { double f = y0; for (long j = 0; j < n; j++) { f += (hist[j] - f) * LM_TS / 0.005; hist[j] = f; } }
    double yinf = 0; for (long j = n - 500; j < n; j++) yinf += hist[j] / 500; for (long j = 0; j < n; j++) if (fabs(hist[j] - yinf) > 0.05 * fabs(yinf - y0)) t5 = (j + 1) * LM_TS;
    printf("b=%.1f : depassement %.0f %%, temps de reponse a 5 %% %.0f ms, erreur finale %.3f mT, defaut=%d\n",
           lm.ax.b, 100 * (pk - yinf) / (yinf - y0), t5 * 1000, (r0 + 1.0) - yinf, lm.fault);
    free(hist);
  }
  printf("== S3 : identification sur X puis application ==\n");
  start(W0T, BETAT); run(1.0); lm.id_axis = 0; lm.cmd = LM_CMD_SWEEP;
  run(9.0);
  printf("vrai : w0=%.1f rad/s beta=%.0f | identifie : w0=%.2f beta=%.0f qualite=%.3f ok=%d | mode=%d defaut=%d\n",
         W0T, BETAT, lm.id_w0[0], lm.id_beta[0], lm.id_quality[0], lm.id_ok[0], lm.mode, lm.fault);
  lm.cmd = LM_CMD_APPLY_ID; run(0.1);
  printf("gains X apres application : kp=%.4f ki=%.4f kd=%.6f tf=%.2f ms routh=%.2f\n", lm.ax.kp, lm.ax.ki, lm.ax.kd, lm.ax.tf_d * 1000, lm.routh[0]);
  { double r0 = lm.ax.ref; lm.ax.ref = (float)(r0 + 1.0); long n = lround(1.5 * LM_F_LOOP_HZ); double pk = -1e9, y0 = ax_[0].p, t5 = 0; double *h = malloc(sizeof(double) * n);
    for (long j = 0; j < n; j++) { period(); h[j] = ax_[0].p; if (h[j] > pk) pk = h[j]; }
    { double f = y0; for (long j = 0; j < n; j++) { f += (h[j] - f) * LM_TS / 0.005; h[j] = f; } }
    double yinf = 0; for (long j = n - 500; j < n; j++) yinf += h[j] / 500; for (long j = 0; j < n; j++) if (fabs(h[j] - yinf) > 0.05 * fabs(yinf - y0)) t5 = (j + 1) * LM_TS;
    printf("echelon +1 mT avec le modele identifie : depassement %.0f %%, t5 %.0f ms, erreur %.3f mT\n", 100 * (pk - yinf) / (yinf - y0), t5 * 1000, (r0 + 1) - yinf); free(h); }
  printf("== S4 : reglage incoherent (beta < 0) ==\n");
  start(W0T, BETAT); lm.mode = LM_MODE_OFF; run(0.01); lm.beta[0] = -1.0f; lm.cmd = LM_CMD_RETUNE; run(0.01);
  lm.mode = LM_MODE_LEVITATION; run(0.01);
  printf("cfg_ok=%d mode=%d defaut=%d (attendu : defaut 5)\n", lm.cfg_ok, lm.mode, lm.fault);
  return 0;
}
