/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Wynik jednego przechwycenia - wszystko, co trzeba, zeby powiedziec CO
 * poszlo nie tak, bez osobnych narzedzi diagnostycznych.                 */
typedef struct {
	HAL_StatusTypeDef status; /* HAL_OK = przyszedl koniec klatki (VSYNC)   */
	uint32_t bytes; /* ile bajtow DMA realnie odebralo z DCMI     */
	uint32_t vsyncs; /* ile impulsow VSYNC DCMI zobaczyl w oknie   */
	uint32_t lines; /* ile linii (HREF) DCMI zobaczyl w oknie     */
	uint32_t elapsed_ms; /* czas od uzbrojenia do konca/timeoutu       */
	uint32_t frame_period_ms; /* odstep miedzy dwoma VSYNC (0 = nieznany)   */
	uint32_t stride; /* bajtow na linie wg DMA (bytes / HEIGHT)    */
	uint32_t dcmi_sr; /* stan linii HSYNC/VSYNC/FIFO w chwili konca */
	uint32_t dcmi_cr; /* DCMI CR (ENABLE/CAPTURE/tryb) w chwili konca*/
	uint32_t dma_cr; /* DMA SxCR (EN, kanal, kierunek...)          */
	uint32_t dma_fcr; /* DMA SxFCR (poziom FIFO)                    */
	uint32_t dcmi_err; /* hdcmi.ErrorCode                            */
	uint32_t dma_err; /* hdma_dcmi.ErrorCode                        */
} capture_result_t;

/* pin sondowany przez ProbePins() */
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	const char *name;
} probe_pin_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- SCCB ---------------------------------------------------------------
 * Datasheet: 0x42 = zapis, 0x43 = odczyt. HAL bierze adres 8-bitowy.     */
#define OV7670_I2C_ADDR       0x42u
#define OV7670_REG_PID        0x0Au   /* = 0x76 */
#define OV7670_REG_VER        0x0Bu   /* = 0x73 */

/* --- format obrazu: RGB565, QVGA 320x240 lub QQVGA 160x120 -------------- */
#define OV7670_QVGA           1       /* 1 = 320x240 (DCW /2), 0 = 160x120 (DCW /4) */

#if OV7670_QVGA
#define OV7670_WIDTH          320u
#define OV7670_HEIGHT         240u
#else
#define OV7670_WIDTH          160u
#define OV7670_HEIGHT         120u
#endif
#define OV7670_LINE_BYTES     (OV7670_WIDTH * 2u)
#define OV7670_FRAME_BYTES    (OV7670_LINE_BYTES * OV7670_HEIGHT) /* 153600 / 38400 */

/* Bufor przechwytywania ma zapas ponad klatke i DMA uzbrajamy na CALY bufor.
 * Koniec klatki wykrywamy przerwaniem FRAME (VSYNC), a nie koncem transferu
 * DMA - HAL sam z siebie wlacza DCMI_IT_FRAME dopiero, gdy DMA przeniesie
 * ostatnie slowo (DCMI_DMAXferCplt). Gdyby kamera wysylala mniej bajtow niz
 * uzbrojono, konczylo sie to cichym timeoutem, a gdy wiecej - przepelnieniem
 * FIFO. Z licznika NDTR wiemy, ILE bajtow kamera naprawde wyslala miedzy
 * dwoma VSYNC. Limit: NDTR DMA to 65535 slow = 262140 B, RAM to 320 KB.   */
#define CAPTURE_BUF_BYTES     (OV7670_FRAME_BYTES + 16u * OV7670_LINE_BYTES)
#define CAPTURE_BUF_WORDS     (CAPTURE_BUF_BYTES / 4u)
#define CAPTURE_TIMEOUT_MS    2000u

/* Klatke wysylamy takze wtedy, gdy kamera daje inna liczbe bajtow na linie
 * niz WIDTH*2 (np. 317 zamiast 320 - skaler DCW obcina piksele na brzegu),
 * byle liczba bajtow dzielila sie przez liczbe linii. Naglowek niesie
 * rzeczywisty stride, a Python sam wycina obraz.                          */
#define OV7670_MIN_LINE_BYTES (OV7670_LINE_BYTES * 3u / 4u)
#define OV7670_MAX_LINE_BYTES (CAPTURE_BUF_BYTES / OV7670_HEIGHT)

#define OV7670_WARMUP_FRAMES  10u     /* datasheet: AEC/AWB stabilizuje sie ~10 klatek */
#define TEST_COLOR_BAR        0       /* 1 = wbudowane 8 paskow zamiast obrazu z matrycy */

/* --- "jakosc" obrazu: wartosci startowe (zmienialne na zywo z konsoli) --
 * Nasycenie (s) skaluje macierz kolorow 0x4F..0x54 tak jak V4L2 w sterowniku
 * Linux; 100 = macierz domyslna. Powyzej ~110 najwiekszy wspolczynnik (0xE4)
 * obcina sie na 255 i proporcje kolorow sie psuja, wiec skalowanie jest
 * ograniczone tak, by NIC sie nie obcinalo. Wlasciwe nasycenie robi DSP:
 * COM13[6] (0x3D) = auto-zmniejszanie nasycenia UV przy duzym wzmocnieniu
 * (slabe swiatlo!), a SATCTR[7:4] (0xC9) to dolny limit tego algorytmu -
 * sterownik Linux daje 0x6/15 (~40%), stad mdle kolory w pokoju. Tu: 0xF.
 * Jasnosc: BRIGHT 0x55 (0x00 neutralnie, bit7 = ujemna: 0x18 jasniej, 0x98
 * ciemniej). Kontrast: CONTRAS 0x56 (0x40 neutralnie, wiecej = ostrzej).   */
#define OV7670_SATURATION_PCT 110u
#define OV7670_BRIGHTNESS     0x00u
#define OV7670_CONTRAST       0x48u
#define OV7670_SATCTR         0xF0u   /* UV saturation min = 15/15 */

/* --- filtr obrazu (na STM32, przed wysylka) ------------------------------
 * FILTER_NONE  : surowy RGB565 z kamery.
 * FILTER_CANNY : detekcja krawedzi Canny'ego - wynik jako RGB565 (biale
 *                krawedzie na czarnym), wiec Python nie wymaga zmian.
 * Progi histerezy dzialaja na |Gx|+|Gy| / 4 (0..255). Zmiana na zywo z
 * konsoli: 'f 0' / 'f 1', 't LOW HIGH'.                                   */
#define FILTER_NONE           0u
#define FILTER_CANNY          1u
#define OV7670_FILTER         FILTER_CANNY
#define CANNY_LOW_DEFAULT     20u
#define CANNY_HIGH_DEFAULT    50u
#define CANNY_HYST_PASSES     4u      /* przebiegi propagacji slabych krawedzi */

/* naglowek ramki rozpoznawany przez Python/ov7670_receiver.py */
#define FRAME_MAGIC_0         0xA5u
#define FRAME_MAGIC_1         0x5Au
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* D-Cache jest wylaczony (brak SCB_EnableDCache), ale wyrownanie 32 B nic nie
 * kosztuje i pozwala bezpiecznie wlaczyc cache w przyszlosci.            */
static uint8_t frame_buffer[CAPTURE_BUF_BYTES] __attribute__((aligned(32)));
static volatile uint8_t frame_ready;
static volatile uint8_t capture_error;
static volatile uint32_t vsync_count;
static volatile uint32_t line_count;
static volatile uint32_t vsync_tick_first; /* HAL_GetTick() 1. i ostatniego */
static volatile uint32_t vsync_tick_last; /* VSYNC w oknie przechwytywania */

extern DMA_HandleTypeDef hdma_dcmi; /* definicja w dcmi.c (CubeMX) */

/* Linie interfejsu kamery + XCLK. IDR odzwierciedla stan pinu takze w
 * trybie AF, wiec mozna je podgladac "pod" DCMI bez rekonfiguracji.       */
static const probe_pin_t probe_pins[] =
		{ { GPIOA, GPIO_PIN_8, "XCLK  PA8 " },
				{ GPIOA, GPIO_PIN_6, "PCLK  PA6 " }, { GPIOA, GPIO_PIN_4,
						"HREF  PA4 " }, { GPIOG, GPIO_PIN_9, "VSYNC PG9 " }, {
						GPIOC, GPIO_PIN_6, "D0    PC6 " }, { GPIOC, GPIO_PIN_7,
						"D1    PC7 " }, { GPIOC, GPIO_PIN_8, "D2    PC8 " }, {
						GPIOC, GPIO_PIN_9, "D3    PC9 " }, { GPIOE, GPIO_PIN_4,
						"D4    PE4 " }, { GPIOD, GPIO_PIN_3, "D5    PD3 " }, {
						GPIOE, GPIO_PIN_5, "D6    PE5 " }, { GPIOE, GPIO_PIN_6,
						"D7    PE6 " }, };
#define PROBE_PIN_COUNT (sizeof(probe_pins) / sizeof(probe_pins[0]))

/* ==========================================================================
 * Tablica rejestrow: QQVGA (160x120) RGB565, XCLK = 16 MHz (MCO1 = HSI).
 *
 * Baza: sterownik Linux drivers/media/i2c/ov7670.c (rejestry "zarezerwowane"
 * to sprawdzone magic numbers), skalowanie do QQVGA: OmniVision "OV7670
 * Implementation Guide", tabela QQVGA.
 *
 * Zegary (datasheet, "VGA Frame Timing"): t_LINE = 784 t_P, t_P = 2 t_PCLK
 * dla RGB. CLKRC=0x01 -> PCLK = 16/2 = 8 MHz -> linia 196 us, klatka
 * 510 linii = 100 ms (10 fps). QQVGA to 320 B na linie; przy PCLK/4 = 2 MHz
 * zajmuja one 160 us < 196 us, wiec DCW /4 + PCLK /4 sa spojne.
 * ========================================================================== */
static const uint8_t ov7670_regs[][2] = {
/* ---- zegar / format bazowy ---------------------------------------- */
{ 0x11, 0x01 }, /* CLKRC : F_wew = XCLK / 2                              */
{ 0x3A, 0x04 }, /* TSLB                                                   */
{ 0x12, 0x04 }, /* COM7  : RGB, baza VGA                                  */

/* ---- okno sensora (640x480 z licznika 784 taktow/linie) ------------
 * Okno VGA z Linuksa to hstart=158, hstop=14 (0x13/0x01/0xB6). W trybach
 * skalowanych DSP ma opoznienie potokowe i pierwsze kolumny wychodza
 * jako smieci (zielonkawe paski na lewym brzegu). Sterownik Linux dla
 * QVGA przesuwa okno o +10 px (hstart=168, hstop=24, vstart=12,
 * vstop=492); dla QQVGA sprawdzone konfiguracje uzywaja hstart=180,
 * hstop=36. Kodowanie: HSTART=hstart>>3, HSTOP=hstop>>3,
 * HREF[5:3]=hstop&7, HREF[2:0]=hstart&7 (HREF[7:6] zostaje 10).
 * Strojenie na zywo: 'w 17 xx' / 'w 18 xx' / 'w 32 xx'.               */
#if OV7670_QVGA
		{ 0x17, 0x15 }, /* HSTART : 168>>3                                         */
		{ 0x18, 0x03 }, /* HSTOP  : 24>>3                                          */
		{ 0x32, 0x80 }, /* HREF   : 10 | (24&7)<<3 | (168&7)                       */
		{ 0x19, 0x03 }, /* VSTRT  : 12>>2                                          */
		{ 0x1A, 0x7B }, /* VSTOP  : 492>>2                                         */
		{ 0x03, 0x00 }, /* VREF   : (492&3)<<2 | (12&3)                            */
#else
	{ 0x17, 0x16 }, /* HSTART : 180>>3                                         */
	{ 0x18, 0x04 }, /* HSTOP  : 36>>3                                          */
	{ 0x32, 0xA4 }, /* HREF   : 10 | (36&7)<<3 | (180&7)                       */
	{ 0x19, 0x02 }, /* VSTRT  : 10>>2                                          */
	{ 0x1A, 0x7A }, /* VSTOP  : 490>>2                                         */
	{ 0x03, 0x0A }, /* VREF   : (490&3)<<2 | (10&3)                            */
#endif
		{ 0x0C, 0x00 }, /* COM3   */
		{ 0x3E, 0x00 }, /* COM14  */
		{ 0x15, 0x00 }, /* COM10 : VSYNC aktywny stanem wysokim, HREF aktywny
		 *         stanem wysokim, PCLK wolnobiezny. Odpowiada
		 *         temu dcmi.c: VS=HIGH, HS=LOW, PCK=RISING.     */

		/* ---- gamma --------------------------------------------------------- */
		{ 0x7A, 0x20 }, { 0x7B, 0x10 }, { 0x7C, 0x1E }, { 0x7D, 0x35 }, { 0x7E,
				0x5A }, { 0x7F, 0x69 }, { 0x80, 0x76 }, { 0x81, 0x80 }, { 0x82,
				0x88 }, { 0x83, 0x8F }, { 0x84, 0x96 }, { 0x85, 0xA3 }, { 0x86,
				0xAF }, { 0x87, 0xC4 }, { 0x88, 0xD7 }, { 0x89, 0xE8 },

		/* ---- AGC / AEC ----------------------------------------------------- */
		{ 0x13, 0xE0 }, /* COM8  : AGC/AWB/AEC wylaczone na czas konfiguracji     */
		{ 0x00, 0x00 }, /* GAIN  */
		{ 0x10, 0x00 }, /* AECH  */
		{ 0x0D, 0x40 }, /* COM4  : okno AEC 1/4 (musi = COM17[7:6])              */
		{ 0x42, 0x80 }, /* COM17 : okno AEC 1/4                                   */
		{ 0x14, 0x18 }, /* COM9  : max wzmocnienie 4x                             */
		{ 0xA5, 0x05 }, { 0xAB, 0x07 }, /* BD50MAX / BD60MAX      */
		{ 0x24, 0x95 }, { 0x25, 0x33 }, { 0x26, 0xE3 }, /* AEW / AEB / VPT        */
		{ 0x9F, 0x78 }, { 0xA0, 0x68 }, { 0xA1, 0x03 }, { 0xA6, 0xD8 }, { 0xA7,
				0xD8 }, { 0xA8, 0xF0 }, { 0xA9, 0x90 }, { 0xAA, 0x94 }, { 0x13,
				0xE5 }, /* COM8  : AEC + szybki algorytm                          */

		/* ---- tor analogowy (magic numbers) --------------------------------- */
		{ 0x0E, 0x61 }, { 0x0F, 0x4B }, { 0x16, 0x02 }, { 0x1E, 0x07 }, { 0x21,
				0x02 }, { 0x22, 0x91 }, { 0x29, 0x07 }, { 0x33, 0x0B }, { 0x35,
				0x0B }, { 0x37, 0x1D }, { 0x38, 0x71 }, { 0x39, 0x2A }, { 0x3C,
				0x78 }, /* COM12 */
		{ 0x4D, 0x40 }, { 0x4E, 0x20 }, { 0x69, 0x00 }, { 0x6B, 0x0A }, /* DBLV  : PLL bypass, wewnetrzny regulator ON            */
		{ 0x74, 0x10 }, { 0x8D, 0x4F }, { 0x8E, 0x00 }, { 0x8F, 0x00 }, { 0x90,
				0x00 }, { 0x91, 0x00 }, { 0x96, 0x00 }, { 0x9A, 0x00 }, { 0xB0,
				0x84 }, { 0xB1, 0x0C }, { 0xB2, 0x0E }, { 0xB3, 0x82 }, { 0xB8,
				0x0A },

		/* ---- AWB ----------------------------------------------------------- */
		{ 0x43, 0x0A }, { 0x44, 0xF0 }, { 0x45, 0x34 }, { 0x46, 0x58 }, { 0x47,
				0x28 }, { 0x48, 0x3A }, { 0x59, 0x88 }, { 0x5A, 0x88 }, { 0x5B,
				0x44 }, { 0x5C, 0x67 }, { 0x5D, 0x49 }, { 0x5E, 0x0E }, { 0x6C,
				0x0A }, { 0x6D, 0x55 }, { 0x6E, 0x11 }, { 0x6F, 0x9F }, { 0x6A,
				0x40 }, { 0x01, 0x40 }, { 0x02, 0x60 }, /* GGAIN / BLUE / RED */
		{ 0x13, 0xE7 }, /* COM8  : AGC + AWB + AEC ON                             */

		/* ---- macierz kolorow, ostrosc, odszumianie ------------------------- */
		{ 0x4F, 0xB3 }, { 0x50, 0xB3 }, { 0x51, 0x00 }, { 0x52, 0x3D }, { 0x53,
				0xA7 }, { 0x54, 0xE4 }, { 0x58, 0x9E }, { 0x41, 0x08 }, /* COM16 */
		{ 0x3F, 0x00 }, /* EDGE  */
		{ 0x75, 0x05 }, { 0x76, 0xE1 }, { 0x4C, 0x00 }, { 0x77, 0x01 }, { 0x3D,
				0xC0 }, /* COM13 : gamma ON + auto-nasycenie UV                   */
		{ 0x4B, 0x09 }, { 0xC9, OV7670_SATCTR }, /* REG4B / SATCTR (patrz wyzej) */
		{ 0x41, 0x38 }, /* COM16 : auto edge + auto de-noise + AWB gain           */
		{ 0x56, 0x40 }, /* CONTRAS */
		{ 0x34, 0x11 }, { 0x3B, 0x12 }, /* COM11 : auto-detekcja 50/60 Hz                         */
		{ 0xA4, 0x88 }, { 0x96, 0x00 }, { 0x97, 0x30 }, { 0x98, 0x20 }, { 0x99,
				0x30 }, { 0x9A, 0x84 }, { 0x9B, 0x29 }, { 0x9C, 0x03 }, { 0x9D,
				0x4C }, { 0x9E, 0x3F }, { 0x78, 0x04 },

		/* ---- format wyjscia: RGB565 ---------------------------------------- */
		{ 0x12, 0x04 }, /* COM7   : RGB                                           */
		{ 0x8C, 0x00 }, /* RGB444 : wylaczone                                     */
		{ 0x04, 0x00 }, /* COM1   */
		{ 0x40, 0xD0 }, /* COM15  : pelny zakres 00..FF + RGB565                  */
		{ 0x14, 0x6A }, /* COM9   : limit AGC dla RGB                             */

		/* ---- skalowanie DCW (tabele QVGA / QQVGA z Implementation Guide) --- */
		{ 0x0C, 0x04 }, /* COM3             : DCW enable                          */
		{ 0x70, 0x3A }, /* SCALING_XSC      : wsp. poziomy (domyslny)             */
		{ 0x71, 0x35 }, /* SCALING_YSC      : wsp. pionowy (domyslny)             */
		{ 0xA2, 0x02 }, /* SCALING_PCLK_DELAY */
#if OV7670_QVGA
		{ 0x3E, 0x19 }, /* COM14            : bit4 DCW/scaling PCLK, bit3 reczne
		 *                    skalowanie, [2:0]=001 -> PCLK /2    */
		{ 0x72, 0x11 }, /* SCALING_DCWCTR   : downsample /2 poziomo i pionowo     */
		{ 0x73, 0xF1 }, /* SCALING_PCLK_DIV : /2 (spojne z COM14[2:0])            */
#else
	{ 0x3E, 0x1A }, /* COM14            : PCLK /4                             */
	{ 0x72, 0x22 }, /* SCALING_DCWCTR   : downsample /4 poziomo i pionowo     */
	{ 0x73, 0xF2 }, /* SCALING_PCLK_DIV : /4                                  */
#endif
		};
#define OV7670_REGS_COUNT (sizeof(ov7670_regs) / sizeof(ov7670_regs[0]))

/* Rejestry odczytywane po konfiguracji - wykrywa zapisy, ktore "przeszly"
 * po I2C, ale kamera ich nie przyjela.                                    */
static const uint8_t ov7670_verify[][2] = { { 0x12, 0x04 }, /* COM7  */
{ 0x40, 0xD0 }, /* COM15 */
{ 0x15, 0x00 }, /* COM10 */
{ 0x0C, 0x04 }, /* COM3  */
#if OV7670_QVGA
		{ 0x3E, 0x19 }, { 0x72, 0x11 }, { 0x73, 0xF1 },
#else
	{ 0x3E, 0x1A }, { 0x72, 0x22 }, { 0x73, 0xF2 },
#endif
		};

/* Macierz kolorow RGB dla nasycenia 100% (Linux ov7670.c, format RGB565):
 * 0x4F..0x54 = |wspolczynnik|, znaki w MTXS 0x58 (zostaja bez zmian, bo
 * skalujemy tylko wartosci bezwzgledne).                                  */
static const uint8_t ov7670_cmatrix_base[6] = { 0xB3, 0xB3, 0x00, 0x3D, 0xA7,
		0xE4 };

/* biezace ustawienia obrazu - startowo z #define, potem z konsoli */
static uint32_t tune_sat_pct = OV7670_SATURATION_PCT;
static uint8_t tune_brightness = OV7670_BRIGHTNESS;
static uint8_t tune_contrast = OV7670_CONTRAST;

/* filtr: tryb i progi Canny'ego (zmienialne z konsoli) */
static uint32_t filter_mode = OV7670_FILTER;
static uint8_t canny_low = CANNY_LOW_DEFAULT;
static uint8_t canny_high = CANNY_HIGH_DEFAULT;

/* Bufor roboczy filtru (1 B/piksel). Reszta etapow Canny'ego mieszka w
 * frame_buffer, ktory po konwersji do szarosci jest wolny - patrz opis
 * rozkladu pamieci przy Filter_Canny().                                   */
static uint8_t gray_buf[OV7670_WIDTH * OV7670_HEIGHT];
#define OV7670_VERIFY_COUNT (sizeof(ov7670_verify) / sizeof(ov7670_verify[0]))
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void LOG(const char *fmt, ...);

/* sterownik OV7670 */
static HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t val);
static HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *val);
static void OV7670_HardReset(void);
static HAL_StatusTypeDef OV7670_Init(void);
static uint8_t OV7670_VerifyRegs(void);
static HAL_StatusTypeDef OV7670_SetColorBar(uint8_t enable) __attribute__((unused));
static HAL_StatusTypeDef OV7670_SetImageTuning(uint32_t sat_pct,
		uint8_t brightness, uint8_t contrast);

/* przechwytywanie i wysylka */
static void Camera_Capture(capture_result_t *r);
static void Camera_ReportFailure(const capture_result_t *r);
static void Camera_SnapshotState(capture_result_t *r);
static void Camera_Reinit(void);
static void SendFrame(uint32_t stride);

/* proste narzedzia diagnostyczne */
static void ProbePins(uint32_t duration_ms);
static void DumpBuffer(uint32_t len, uint32_t stride);

/* konsola strojenia po UART (komendy wpisywane w terminalu Pythona) */
static void Console_Poll(void);
static void Console_Exec(char *line);
static void Console_DumpRegs(void);

/* filtry obrazu */
static uint32_t Filter_Apply(uint32_t stride, uint32_t height);
static uint32_t Filter_Canny(uint32_t width, uint32_t height,
		uint32_t in_stride);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	uint8_t pid = 0, ver = 0;
	uint32_t frame_count = 0, fail_count = 0, fail_streak = 0;
	capture_result_t cap;
	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART3_UART_Init();
	MX_DCMI_Init();
	/* USER CODE BEGIN 2 */
	LOG("\r\n\r\n=== OV7670 - NUCLEO-F756ZG ===\r\n");
	LOG("Format    : %ux%u RGB565 (%u B/klatke)\r\n", OV7670_WIDTH,
	OV7670_HEIGHT, (unsigned) OV7670_FRAME_BYTES);

	/* XCLK (MCO1 na PA8) biegnie od SystemClock_Config(); kamera potrzebuje
	 * go juz w trakcie resetu.                                             */
	HAL_Delay(50);
	OV7670_HardReset();

	if (OV7670_ReadReg(OV7670_REG_PID, &pid) != HAL_OK
			|| OV7670_ReadReg(OV7670_REG_VER, &ver) != HAL_OK) {
		LOG(
				"[BLAD] Kamera nie odpowiada na SCCB (0x%02X). Sprawdz 3.3V, GND,\r\n"
						"       pull-upy na SDA/SCL, XCLK na PA8, PWDN/RESET#.\r\n",
				OV7670_I2C_ADDR);
		Error_Handler();
	}
	LOG("PID/VER   : 0x%02X / 0x%02X (oczekiwane 0x76 / 0x73)\r\n", pid, ver);

	if (OV7670_Init() != HAL_OK) {
		LOG("[BLAD] Zapis rejestrow nie powiodl sie.\r\n");
		Error_Handler();
	}
	if (OV7670_VerifyRegs()) {
		LOG("Rejestry  : OK\r\n");
	}
	OV7670_SetImageTuning(tune_sat_pct, tune_brightness, tune_contrast);
	LOG(
			"Obraz     : nasycenie %lu%%, jasnosc 0x%02X, kontrast 0x%02X, SATCTR 0x%02X\r\n",
			(unsigned long) tune_sat_pct, tune_brightness, tune_contrast,
			(unsigned) OV7670_SATCTR);
	LOG("Filtr     : %s (Canny low=%u high=%u)\r\n",
			(filter_mode == FILTER_CANNY) ? "Canny" : "brak", canny_low,
			canny_high);
	LOG(
			"Konsola   : wpisz '?' w terminalu, zeby zobaczyc komendy strojenia.\r\n");
#if TEST_COLOR_BAR
	OV7670_SetColorBar(1);
	LOG("Tryb      : PASKI TESTOWE\r\n");
#endif

	/* Czy na pinach w ogole cos zyje? (kamera wysyla juz po konfiguracji) */
	HAL_Delay(100);
	ProbePins(200);

	/* Rozgrzewka: lapiemy i odrzucamy klatki, az AEC/AWB sie ustabilizuje.
	 * Pierwsza nieudana klatka przerywa rozgrzewke - petla glowna i tak
	 * bedzie raportowac problem.                                          */
	for (uint32_t i = 0; i < OV7670_WARMUP_FRAMES; i++) {
		Camera_Capture(&cap);
		if (cap.status != HAL_OK || cap.stride == 0u) {
			break;
		}
	}
	LOG("Start przechwytywania...\r\n\r\n");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* LD1 (zielona) miga przy kazdej probie, LD3 (czerwona) = blad. */
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		Camera_Capture(&cap);

		/* stride == 0 -> liczba bajtow nie sklada sie w HEIGHT rownych linii */
		if (cap.status != HAL_OK || cap.stride == 0u) {
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			fail_count++;
			fail_streak++;
			Camera_ReportFailure(&cap);
			if ((fail_count % 10u) == 1u) {
				ProbePins(200);
			}
			/* Dwa bledy z rzedu = DCMI/DMA moglo utknac (widziane: FRAME
			 * przychodzi, a DMA dostaje 0 B). Pelna reinicjalizacja obu. */
			if (fail_streak >= 2u) {
				Camera_Reinit();
				fail_streak = 0;
			}
			HAL_Delay(500);
			continue;
		}
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		fail_streak = 0;

		if (SCB->CCR & SCB_CCR_DC_Msk) {
			SCB_InvalidateDCache_by_Addr((uint32_t*) frame_buffer,
					(int32_t) cap.bytes);
		}

		frame_count++;
		if (frame_count == 1u || (frame_count % 50u) == 0u) {
			LOG(
					"[OK] Klatka #%lu: %lu B = %u linii x %lu B (oczekiwano %u B/linie),"
							" okres klatki %lu ms\r\n",
					(unsigned long) frame_count, (unsigned long) cap.bytes,
					OV7670_HEIGHT, (unsigned long) cap.stride,
					(unsigned) OV7670_LINE_BYTES,
					(unsigned long) cap.frame_period_ms);
			if (frame_count == 1u) {
				DumpBuffer(cap.bytes, cap.stride);
			}
		}
		/* Filtr przetwarza klatke w miejscu i zwraca stride wyniku (moze
		 * byc inny niz z kamery, np. 317 B -> 316 B po obcieciu do pelnych
		 * pikseli).                                                       */
		SendFrame(Filter_Apply(cap.stride, OV7670_HEIGHT));
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
/* ==========================================================================
 * Log przez UART3 (ST-LINK VCP)
 * ========================================================================== */
static void LOG(const char *fmt, ...) {
	char buf[192];
	va_list args;
	int len;

	va_start(args, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len > 0) {
		if ((size_t) len > sizeof(buf) - 1u) {
			len = (int) (sizeof(buf) - 1u); /* vsnprintf zwraca dlugosc "chciana" */
		}
		HAL_UART_Transmit(&huart3, (uint8_t*) buf, (uint16_t) len, 200);
	}
}

/* ==========================================================================
 * Sterownik OV7670 (SCCB)
 *
 * SCCB to nie pelne I2C: odczyt rejestru to DWIE transakcje zakonczone
 * STOP-em (zapis adresu, potem odczyt bajtu). HAL_I2C_Mem_Read (repeated
 * START) z ta kamera nie dziala.
 * ========================================================================== */
static HAL_StatusTypeDef OV7670_WriteReg(uint8_t reg, uint8_t val) {
	uint8_t buf[2] = { reg, val };
	HAL_StatusTypeDef st;

	st = HAL_I2C_Master_Transmit(&hi2c1, OV7670_I2C_ADDR, buf, 2, 100);
	HAL_Delay(1);
	return st;
}

static HAL_StatusTypeDef OV7670_ReadReg(uint8_t reg, uint8_t *val) {
	if (HAL_I2C_Master_Transmit(&hi2c1, OV7670_I2C_ADDR, &reg, 1, 100)
			!= HAL_OK) {
		return HAL_ERROR;
	}
	HAL_Delay(1);
	return HAL_I2C_Master_Receive(&hi2c1, OV7670_I2C_ADDR, val, 1, 100);
}

static void OV7670_HardReset(void) {
	HAL_GPIO_WritePin(PWDN_GPIO_Port, PWDN_Pin, GPIO_PIN_RESET); /* praca  */
	HAL_Delay(10);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET); /* reset  */
	HAL_Delay(20);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(100); /* datasheet: >1 ms po resecie */
}

static HAL_StatusTypeDef OV7670_Init(void) {
	/* reset programowy: COM7[7] = 1 */
	if (OV7670_WriteReg(0x12, 0x80) != HAL_OK) {
		return HAL_ERROR;
	}
	HAL_Delay(100);

	for (uint32_t i = 0; i < OV7670_REGS_COUNT; i++) {
		if (OV7670_WriteReg(ov7670_regs[i][0], ov7670_regs[i][1]) != HAL_OK) {
			return HAL_ERROR;
		}
	}
	HAL_Delay(100);
	return HAL_OK;
}

/* Zwraca 1, gdy wszystkie kluczowe rejestry maja oczekiwana wartosc;
 * loguje tylko niezgodnosci.                                              */
static uint8_t OV7670_VerifyRegs(void) {
	uint8_t ok = 1, val;

	for (uint32_t i = 0; i < OV7670_VERIFY_COUNT; i++) {
		if (OV7670_ReadReg(ov7670_verify[i][0], &val) != HAL_OK) {
			LOG("[BLAD] Rejestr 0x%02X: brak odpowiedzi SCCB\r\n",
					ov7670_verify[i][0]);
			ok = 0;
		} else if (val != ov7670_verify[i][1]) {
			LOG("[BLAD] Rejestr 0x%02X = 0x%02X, oczekiwano 0x%02X\r\n",
					ov7670_verify[i][0], val, ov7670_verify[i][1]);
			ok = 0;
		}
	}
	return ok;
}

/* Datasheet (SCALING_XSC/YSC): test_pattern = (YSC[7], XSC[7]):
 *   00 brak, 01 "shifting 1", 10 osiem paskow kolorowych, 11 paski + szarosc.
 * Osiem paskow = YSC[7]=1, XSC[7]=0 (tak samo robi sterownik Linux).       */
static HAL_StatusTypeDef OV7670_SetColorBar(uint8_t enable) {
	uint8_t xsc, ysc;

	if (OV7670_ReadReg(0x70, &xsc) != HAL_OK
			|| OV7670_ReadReg(0x71, &ysc) != HAL_OK) {
		return HAL_ERROR;
	}
	xsc &= (uint8_t) ~0x80u;
	ysc = enable ? (uint8_t) (ysc | 0x80u) : (uint8_t) (ysc & ~0x80u);

	if (OV7670_WriteReg(0x70, xsc) != HAL_OK) {
		return HAL_ERROR;
	}
	return OV7670_WriteReg(0x71, ysc);
}

/* Nasycenie = skalowanie macierzy kolorow (jak V4L2_CID_SATURATION w Linux),
 * jasnosc/kontrast = rejestry BRIGHT/CONTRAS. Wolane po tablicy rejestrow,
 * bo ona zapisuje domyslna macierz i kontrast.                            */
static HAL_StatusTypeDef OV7670_SetImageTuning(uint32_t sat_pct,
		uint8_t brightness, uint8_t contrast) {
	/* Najwiekszy wspolczynnik (0xE4 = 228) nie moze przekroczyc 255, a
	 * skalowac trzeba wszystkie jednakowo - inaczej rozjezdza sie balans
	 * barw. Stad twardy limit 111%.                                      */
	if (sat_pct > 111u) {
		sat_pct = 111u;
	}
	for (uint32_t i = 0; i < 6u; i++) {
		uint32_t v = (uint32_t) ov7670_cmatrix_base[i] * sat_pct / 100u;
		if (OV7670_WriteReg((uint8_t) (0x4Fu + i), (uint8_t) v) != HAL_OK) {
			return HAL_ERROR;
		}
	}
	if (OV7670_WriteReg(0x55, brightness) != HAL_OK) {
		return HAL_ERROR;
	}
	return OV7670_WriteReg(0x56, contrast);
}

/* ==========================================================================
 * Przechwytywanie: DCMI SNAPSHOT + DMA NORMAL na caly (nadmiarowy) bufor.
 *
 * Sekwencja: Start_DMA uzbraja DMA i ustawia CAPTURE; DCMI czeka na VSYNC,
 * zbiera jedna klatke i przy kolejnym VSYNC zglasza FRAME (sprzet sam
 * czysci CAPTURE). Przerwanie FRAME wlaczamy recznie od razu, bo HAL zrobilby
 * to dopiero po zakonczeniu DMA (patrz komentarz przy CAPTURE_BUF_BYTES).
 * OVR/ERR tez wlaczamy recznie - HAL wylacza je po pierwszym snapshocie.
 * ========================================================================== */
static void Camera_Capture(capture_result_t *r) {
	uint32_t t0;

	frame_ready = 0;
	capture_error = 0;
	vsync_count = 0;
	line_count = 0;
	vsync_tick_first = 0;
	vsync_tick_last = 0;
	hdcmi.ErrorCode = HAL_DCMI_ERROR_NONE;
	hdma_dcmi.ErrorCode = HAL_DMA_ERROR_NONE;
	__HAL_DCMI_CLEAR_FLAG(&hdcmi,
			DCMI_FLAG_FRAMERI | DCMI_FLAG_OVRRI | DCMI_FLAG_ERRRI | DCMI_FLAG_VSYNCRI | DCMI_FLAG_LINERI);
	*r = (capture_result_t ) { 0 };

	t0 = HAL_GetTick();
	r->status = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT,
			(uint32_t) frame_buffer, CAPTURE_BUF_WORDS);
	if (r->status != HAL_OK) {
		Camera_SnapshotState(r);
		return;
	}
	/* VSYNC i LINE tylko do liczenia impulsow - to najprostszy sposob, zeby
	 * zobaczyc, czy DCMI w ogole widzi synchronizacje i ile linii ma klatka. */
	__HAL_DCMI_ENABLE_IT(&hdcmi,
			DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR | DCMI_IT_VSYNC | DCMI_IT_LINE);

	while (!frame_ready && !capture_error
			&& (HAL_GetTick() - t0) < CAPTURE_TIMEOUT_MS) {
		Console_Poll(); /* CPU i tak czeka - obsluz komendy z terminala */
	}

	/* Stan odczytujemy PRZED Stop() - Stop() resetuje DMA i czysci CAPTURE. */
	r->elapsed_ms = HAL_GetTick() - t0;
	r->bytes = (CAPTURE_BUF_WORDS - hdma_dcmi.Instance->NDTR) * 4u;
	r->vsyncs = vsync_count;
	r->lines = line_count;
	Camera_SnapshotState(r);
	HAL_DCMI_Stop(&hdcmi);

	if (vsync_count >= 2u) {
		r->frame_period_ms = (vsync_tick_last - vsync_tick_first)
				/ (vsync_count - 1u);
	}

	if (frame_ready) {
		r->status = HAL_OK;
		/* Klatka jest "do wyslania", gdy sklada sie z HEIGHT rownych linii
		 * o sensownej dlugosci - niekoniecznie 320 B.                   */
		if (r->bytes >= OV7670_MIN_LINE_BYTES * OV7670_HEIGHT
				&& r->bytes <= OV7670_MAX_LINE_BYTES * OV7670_HEIGHT
				&& (r->bytes % OV7670_HEIGHT) == 0u) {
			r->stride = r->bytes / OV7670_HEIGHT;
		}
	} else if (capture_error) {
		r->status = HAL_ERROR;
	} else {
		r->status = HAL_TIMEOUT;
	}
}

/* Migawka stanu DCMI/DMA - wolana PRZED HAL_DCMI_Stop(), ktory to kasuje. */
static void Camera_SnapshotState(capture_result_t *r) {
	r->dcmi_sr = hdcmi.Instance->SR;
	r->dcmi_cr = hdcmi.Instance->CR;
	r->dma_cr = hdma_dcmi.Instance->CR;
	r->dma_fcr = hdma_dcmi.Instance->FCR;
	r->dcmi_err = hdcmi.ErrorCode;
	r->dma_err = hdma_dcmi.ErrorCode;
}

/* Pelna reinicjalizacja DCMI + DMA2_Stream1: HAL_DCMI_DeInit -> MspDeInit
 * robi HAL_DMA_DeInit (reset rejestrow strumienia), MX_DCMI_Init odtwarza
 * wszystko z konfiguracji CubeMX.                                         */
static void Camera_Reinit(void) {
	LOG("[INFO] Reinicjalizacja DCMI + DMA...\r\n");
	HAL_DCMI_DeInit(&hdcmi);
	MX_DCMI_Init();
}

static void Camera_ReportFailure(const capture_result_t *r) {
	unsigned hs = (r->dcmi_sr & DCMI_SR_HSYNC) ? 1u : 0u;
	unsigned vs = (r->dcmi_sr & DCMI_SR_VSYNC) ? 1u : 0u;
	unsigned fne = (r->dcmi_sr & DCMI_SR_FNE) ? 1u : 0u;

	/* Jedna linia z twardymi liczbami - z niej wynika prawie wszystko:
	 *   VSYNC=0            -> DCMI nie widzi ramek (PG9 / VSPolarity)
	 *   linii=0, VSYNC>0   -> jest ramka, ale brak HREF/PCLK (PA4 / PA6)
	 *   linii/VSYNC        -> realna wysokosc obrazu (120 = QQVGA, 240 = QVGA)
	 *   bajty/linie        -> realna szerokosc linii w bajtach (320 = QQVGA)   */
	LOG("[BLAD] status=%s  czas=%lu ms  bajty=%lu  VSYNC=%lu  linii=%lu"
			"  okres klatki=%lu ms  SR: HSYNC=%u VSYNC=%u FIFO=%u\r\n",
			(r->status == HAL_OK) ? "OK" :
			(r->status == HAL_TIMEOUT) ? "TIMEOUT" : "ERROR",
			(unsigned long) r->elapsed_ms, (unsigned long) r->bytes,
			(unsigned long) r->vsyncs, (unsigned long) r->lines,
			(unsigned long) r->frame_period_ms, hs, vs, fne);
	/* Surowe rejestry z chwili konca - do rozstrzygniecia, czy utknal DCMI
	 * (CR: ENABLE=bit0, CAPTURE=bit1) czy DMA (SxCR EN=bit0, FCR FS=[5:3]). */
	LOG(
			"       DCMI CR=0x%04lX err=0x%02lX | DMA CR=0x%08lX FCR=0x%02lX err=0x%02lX"
					" | stan HAL: dcmi=%u dma=%u\r\n",
			(unsigned long) r->dcmi_cr, (unsigned long) r->dcmi_err,
			(unsigned long) r->dma_cr, (unsigned long) r->dma_fcr,
			(unsigned long) r->dma_err, (unsigned) hdcmi.State,
			(unsigned) hdma_dcmi.State);
	/* Licznik linii biegnie takze w oczekiwaniu na poczatek klatki, wiec
	 * linie/klatke liczymy z czestotliwosci HREF i okresu klatki.         */
	if (r->frame_period_ms > 0u && r->elapsed_ms > 0u) {
		LOG(
				"       -> HREF: %lu/s -> ok. %lu linii/klatke; bajty/%u linii = %lu\r\n",
				(unsigned long) (r->lines * 1000u / r->elapsed_ms),
				(unsigned long) (r->lines * r->frame_period_ms / r->elapsed_ms),
				OV7670_HEIGHT, (unsigned long) (r->bytes / OV7670_HEIGHT));
	}

	switch (r->status) {
	case HAL_OK:
		LOG(
				"       Klatka zamknieta VSYNC-iem, ale %lu B nie sklada sie w %u\r\n"
						"       rownych linii po %lu..%lu B -> sprawdz COM14/0x72/0x73\r\n"
						"       (38400 = QQVGA, 153600 = QVGA, 614400 = VGA) albo HSPolarity.\r\n",
				(unsigned long) r->bytes, OV7670_HEIGHT,
				(unsigned long) OV7670_MIN_LINE_BYTES,
				(unsigned long) OV7670_MAX_LINE_BYTES);
		break;

	case HAL_TIMEOUT:
		if (r->vsyncs == 0u && r->lines == 0u && r->bytes == 0u) {
			LOG(
					"       -> DCMI nie widzi NICZEGO: sprawdz XCLK (PA8), zasilanie,\r\n"
							"          PWDN/RESET#, przewody PCLK/HREF/VSYNC (patrz sonda pinow).\r\n");
		} else if (r->vsyncs == 0u) {
			LOG(
					"       -> sa linie/dane, ale zero VSYNC: przewod PG9 lub VSPolarity.\r\n");
		} else if (r->bytes == 0u) {
			LOG(
					"       -> sa ramki, ale zero danych: sprawdz PCLK (PA6) i HREF (PA4).\r\n");
		} else {
			LOG(
					"       -> dane i VSYNC sa, ale FRAME nie przyszedl - sprawdz\r\n"
							"          VSPolarity/HSPolarity w dcmi.c.\r\n");
		}
		break;

	default:
		if (r->dcmi_err & HAL_DCMI_ERROR_OVR) {
			LOG(
					"       OVR: przepelnienie FIFO DCMI - kamera wyslala wiecej niz\r\n"
							"       %lu B bez VSYNC (bufor pelny) albo DMA nie nadaza.\r\n",
					(unsigned long) CAPTURE_BUF_BYTES);
		}
		if (r->dcmi_err & HAL_DCMI_ERROR_SYNC) {
			LOG("       SYNC: blad synchronizacji (polaryzacje).\r\n");
		}
		break;
	}

	if (r->bytes > 0u) {
		DumpBuffer(r->bytes, r->bytes / OV7670_HEIGHT);
	}
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *phdcmi) {
	(void) phdcmi;
	frame_ready = 1;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *phdcmi) {
	(void) phdcmi;
	capture_error = 1;
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *phdcmi) {
	(void) phdcmi;
	vsync_tick_last = HAL_GetTick();
	if (vsync_count == 0u) {
		vsync_tick_first = vsync_tick_last;
	}
	vsync_count++;
}

void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *phdcmi) {
	(void) phdcmi;
	line_count++;
}

/* ==========================================================================
 * Konsola strojenia po UART
 *
 * Linie tekstu (zakonczone \n) przychodza z terminala Pythona. Odbior jest
 * odpytywany (bez przerwan) w petli oczekiwania na klatke, wiec komenda
 * wyslana zaraz po odebraniu ramki trafia bez strat; bajty przychodzace w
 * trakcie HAL_UART_Transmit (wysylka klatki) moga zginac - Python wysyla
 * komendy tylko w przerwie miedzy ramkami.
 * ========================================================================== */
static void Console_Poll(void) {
	static char line[32];
	static uint32_t n;
	uint8_t ch;

	__HAL_UART_CLEAR_OREFLAG(&huart3);
	while (HAL_UART_Receive(&huart3, &ch, 1, 0) == HAL_OK) {
		if (ch == '\r' || ch == '\n') {
			if (n > 0u) {
				line[n] = '\0';
				n = 0;
				Console_Exec(line);
			}
		} else if (n < sizeof(line) - 1u) {
			line[n++] = (char) ch;
		}
	}
}

static void Console_Exec(char *line) {
	unsigned a = 0, b = 0;
	uint8_t val;

	switch (line[0]) {
	case 'w': /* w RR VV - zapis rejestru (hex) */
		if (sscanf(line + 1, "%x %x", &a, &b) == 2 && a < 256u && b < 256u) {
			HAL_StatusTypeDef st = OV7670_WriteReg((uint8_t) a, (uint8_t) b);
			OV7670_ReadReg((uint8_t) a, &val);
			LOG("[konsola] w 0x%02X = 0x%02X -> %s, odczyt 0x%02X\r\n", a, b,
					(st == HAL_OK) ? "OK" : "BLAD", val);
		} else {
			LOG("[konsola] skladnia: w RR VV (hex)\r\n");
		}
		break;

	case 'r': /* r RR - odczyt rejestru */
		if (sscanf(line + 1, "%x", &a) == 1 && a < 256u
				&& OV7670_ReadReg((uint8_t) a, &val) == HAL_OK) {
			LOG("[konsola] r 0x%02X = 0x%02X\r\n", a, val);
		} else {
			LOG("[konsola] skladnia: r RR (hex)\r\n");
		}
		break;

	case 's': /* s NNN - nasycenie macierzy w % (dec, max 111)   */
	case 'b': /* b VV  - jasnosc BRIGHT (hex, bit7 = ujemna)     */
	case 'c': /* c VV  - kontrast CONTRAS (hex, 0x40 neutralnie) */
		if (sscanf(line + 1, (line[0] == 's') ? "%u" : "%x", &a) != 1
				|| (line[0] != 's' && a > 255u)) {
			LOG(
					"[konsola] skladnia: s NNN (dec) | b VV (hex) | c VV (hex)\r\n");
			break;
		}
		if (line[0] == 's') {
			tune_sat_pct = (a > 111u) ? 111u : a;
		} else if (line[0] == 'b') {
			tune_brightness = (uint8_t) a;
		} else {
			tune_contrast = (uint8_t) a;
		}
		OV7670_SetImageTuning(tune_sat_pct, tune_brightness, tune_contrast);
		LOG("[konsola] nasycenie %lu%%, jasnosc 0x%02X, kontrast 0x%02X\r\n",
				(unsigned long) tune_sat_pct, tune_brightness, tune_contrast);
		break;

	case 'd':
		Console_DumpRegs();
		break;

	case 'f': /* f N - filtr: 0 = brak, 1 = Canny */
		if (sscanf(line + 1, "%u", &a) == 1 && a <= FILTER_CANNY) {
			filter_mode = a;
			LOG("[konsola] filtr = %s\r\n",
					(filter_mode == FILTER_CANNY) ? "Canny" : "brak");
		} else {
			LOG("[konsola] skladnia: f 0 | f 1\r\n");
		}
		break;

	case 't': /* t LOW HIGH - progi histerezy Canny'ego (0..255, dec) */
		if (sscanf(line + 1, "%u %u", &a, &b) == 2 && a < 256u && b < 256u
				&& a <= b) {
			canny_low = (uint8_t) a;
			canny_high = (uint8_t) b;
			LOG("[konsola] Canny: low=%u high=%u\r\n", a, b);
		} else {
			LOG("[konsola] skladnia: t LOW HIGH (dec, LOW <= HIGH)\r\n");
		}
		break;

	default:
		LOG(
				"[konsola] komendy:\r\n"
						"  w RR VV  zapis rejestru (hex), np. 'w 3d 80' = COM13 bez auto-UV\r\n"
						"  r RR     odczyt rejestru\r\n"
						"  s NNN    nasycenie macierzy %% (100 = domyslne, max 111)\r\n"
						"  b VV     jasnosc BRIGHT (hex: 00 neutr., 18 jasniej, 98 ciemniej)\r\n"
						"  c VV     kontrast CONTRAS (hex: 40 neutr., 60 mocniej)\r\n"
						"  d        zrzut rejestrow koloru/ekspozycji\r\n"
						"  f N      filtr: 0 = surowy obraz, 1 = krawedzie Canny\r\n"
						"  t L H    progi Canny (dec 0..255), np. 't 30 80'\r\n"
						"  przyklady: 'w c9 f0' (SATCTR min=15), 'w c9 60' (jak Linux),\r\n"
						"             'w 3d 80' (auto-UV off), 'w 3b 92' (night mode)\r\n");
		break;
	}
}

/* Zrzut rejestrow, ktore decyduja o kolorze i ekspozycji - warto porownac
 * odczyt w "mdlym" i w "dobrym" swietle.                                  */
static void Console_DumpRegs(void) {
	static const struct {
		uint8_t reg;
		const char *name;
	} regs[] = { { 0x3D, "COM13 (gamma/auto-UV)" }, { 0xC9,
			"SATCTR (UV sat min/res)" }, { 0x13, "COM8  (AGC/AWB/AEC)" }, {
			0x00, "GAIN" }, { 0x10, "AECH  (ekspozycja hi)" }, { 0x04,
			"COM1  (ekspozycja lo)" }, { 0x01, "BLUE  (AWB gain B)" }, { 0x02,
			"RED   (AWB gain R)" }, { 0x6A, "GGAIN (AWB gain G)" }, { 0x55,
			"BRIGHT" }, { 0x56, "CONTRAS" }, { 0x58, "MTXS" }, { 0x4F, "MTX1" },
			{ 0x50, "MTX2" }, { 0x51, "MTX3" }, { 0x52, "MTX4" },
			{ 0x53, "MTX5" }, { 0x54, "MTX6" }, { 0x3B, "COM11 (night mode)" },
			{ 0x41, "COM16" }, };
	uint8_t val;

	LOG("---- rejestry ----\r\n");
	for (uint32_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
		if (OV7670_ReadReg(regs[i].reg, &val) == HAL_OK) {
			LOG("  0x%02X %-24s = 0x%02X\r\n", regs[i].reg, regs[i].name, val);
		}
	}
	LOG("------------------\r\n");
}

/* ==========================================================================
 * Proste narzedzia diagnostyczne
 * ========================================================================== */

/* Przez duration_ms probkuje w petli stan kazdej linii kamery i liczy zbocza
 * narastajace + procent czasu w stanie wysokim. Linia z 0 zboczy jest
 * martwa (przewod, zly pin, kamera w power-down). Oczekiwane przy 10 fps
 * QQVGA i 200 ms: XCLK/PCLK tysiace zboczy (aliasing), HREF ~240, VSYNC ~2,
 * D0..D7 > 0 (paski testowe zmieniaja dane).                              */
static void ProbePins(uint32_t duration_ms) {
	uint32_t edges[PROBE_PIN_COUNT] = { 0 };
	uint32_t highs[PROBE_PIN_COUNT] = { 0 };
	uint8_t prev[PROBE_PIN_COUNT];
	uint32_t samples = 0, t0;

	for (uint32_t i = 0; i < PROBE_PIN_COUNT; i++) {
		prev[i] = (probe_pins[i].port->IDR & probe_pins[i].pin) ? 1u : 0u;
	}
	t0 = HAL_GetTick();
	while ((HAL_GetTick() - t0) < duration_ms) {
		for (uint32_t i = 0; i < PROBE_PIN_COUNT; i++) {
			uint8_t v = (probe_pins[i].port->IDR & probe_pins[i].pin) ? 1u : 0u;
			highs[i] += v;
			edges[i] += (v && !prev[i]) ? 1u : 0u;
			prev[i] = v;
		}
		samples++;
	}

	LOG("---- Sonda pinow (%lu ms) ----\r\n", (unsigned long) duration_ms);
	for (uint32_t i = 0; i < PROBE_PIN_COUNT; i++) {
		uint32_t pct = samples ? (highs[i] * 100u / samples) : 0u;
		LOG("  %s zbocza=%-7lu wysoki=%3lu%%  %s\r\n", probe_pins[i].name,
				(unsigned long) edges[i], (unsigned long) pct,
				edges[i] ?
						"ok" :
						(pct ? "STALE 1 - martwa linia" : "STALE 0 - martwa linia"));
	}
	LOG("------------------------------\r\n");
}

static void DumpHex(const char *label, uint32_t from, uint32_t count) {
	char line[3 * 24 + 1];
	uint32_t n = 0;

	if (from + count > CAPTURE_BUF_BYTES) {
		count = CAPTURE_BUF_BYTES - from;
	}
	for (uint32_t i = 0; i < count && n + 3 < sizeof(line); i++) {
		n += (uint32_t) snprintf(line + n, sizeof(line) - n, "%02X ",
				frame_buffer[from + i]);
	}
	LOG("  %s [%5lu..]: %s\r\n", label, (unsigned long) from, line);
}

/* Poczatek bufora, okolice granicy 1. linii (wg stride) i udzial zer/0xFF.
 * Paski testowe: 8 paskow x 40 B - bialy F7FF.. na starcie KAZDEJ linii,
 * czarny 0000.. na koncu. Gdzie po czerni wraca biel, tam konczy sie
 * linia - to bezposredni pomiar rzeczywistej dlugosci linii.            */
static void DumpBuffer(uint32_t len, uint32_t stride) {
	uint32_t zeros = 0, ffs = 0;

	if (len > CAPTURE_BUF_BYTES) {
		len = CAPTURE_BUF_BYTES;
	}
	for (uint32_t i = 0; i < len; i++) {
		zeros += (frame_buffer[i] == 0x00u) ? 1u : 0u;
		ffs += (frame_buffer[i] == 0xFFu) ? 1u : 0u;
	}
	DumpHex("start      ", 0, 16);
	if (stride >= 8u && stride + 16u <= len) {
		DumpHex("granica L0 ", stride - 8u, 24);
	}
	if (OV7670_LINE_BYTES != stride && OV7670_LINE_BYTES + 16u <= len) {
		DumpHex("linia nom. ", OV7670_LINE_BYTES - 8u, 24); /* granica wg WIDTH*2 */
	}
	LOG("  z %lu B: zer=%lu%%, 0xFF=%lu%%\r\n", (unsigned long) len,
			(unsigned long) (len ? zeros * 100u / len : 0u),
			(unsigned long) (len ? ffs * 100u / len : 0u));
}

/* ==========================================================================
 * Filtry obrazu
 * ========================================================================== */

/* Wybiera filtr wg filter_mode. Zwraca stride danych do wyslania. */
static uint32_t Filter_Apply(uint32_t stride, uint32_t height) {
	switch (filter_mode) {
	case FILTER_CANNY:
		return Filter_Canny(stride / 2u, height, stride);
	default:
		return stride;
	}
}

/* Kwantyzacja kierunku gradientu do 4 sektorow (bez atan2):
 *   0 = poziomy (|gy| < tan22.5*|gx|)  -> sasiedzi lewo/prawo
 *   1 = 45 st. (gx i gy tego samego znaku) -> (x-1,y-1) i (x+1,y+1)
 *   2 = pionowy                        -> gora/dol
 *   3 = 135 st.                        -> (x+1,y-1) i (x-1,y+1)
 * tan(22.5 st.) = 0.414 ~ 53/128.                                          */
static inline uint8_t Canny_Dir(int32_t gx, int32_t gy) {
	int32_t ax = (gx < 0) ? -gx : gx;
	int32_t ay = (gy < 0) ? -gy : gy;
	if (ay * 128 < ax * 53) {
		return 0;
	}
	if (ax * 128 < ay * 53) {
		return 2;
	}
	return ((gx < 0) == (gy < 0)) ? 1 : 3;
}

/* Etap 1: RGB565 (MSB first, in_stride B/linie) -> szarosc, 1 B/piksel.
 * Y = 0.30 R + 0.59 G + 0.11 B w arytmetyce calkowitej (/256).            */
static void Canny_Gray(const uint8_t *rgb, uint32_t in_stride, uint8_t *gray,
		uint32_t w, uint32_t h) {
	for (uint32_t y = 0; y < h; y++) {
		const uint8_t *src = rgb + y * in_stride;
		uint8_t *dst = gray + y * w;
		for (uint32_t x = 0; x < w; x++) {
			uint32_t px = ((uint32_t) src[2u * x] << 8) | src[2u * x + 1u];
			uint32_t r = (px >> 8) & 0xF8u; /* 5 bitow -> 8 */
			uint32_t g = (px >> 3) & 0xFCu; /* 6 bitow -> 8 */
			uint32_t b = (px << 3) & 0xF8u;
			dst[x] = (uint8_t) ((77u * r + 151u * g + 28u * b) >> 8);
		}
	}
}

/* Etap 2: Gauss 5x5 = jadro 1 4 6 4 1 rozdzielone na przebieg poziomy
 * (img -> tmp) i pionowy (tmp -> img). Brzegi: indeks obcinany (replikacja). */
static void Canny_Blur(uint8_t *img, uint8_t *tmp, uint32_t w, uint32_t h) {
	for (uint32_t y = 0; y < h; y++) {
		const uint8_t *s = img + y * w;
		uint8_t *d = tmp + y * w;
		for (uint32_t x = 0; x < w; x++) {
			uint32_t xm2 = (x >= 2u) ? x - 2u : 0u;
			uint32_t xm1 = (x >= 1u) ? x - 1u : 0u;
			uint32_t xp1 = (x + 1u < w) ? x + 1u : w - 1u;
			uint32_t xp2 = (x + 2u < w) ? x + 2u : w - 1u;
			d[x] = (uint8_t) ((s[xm2] + 4u * s[xm1] + 6u * s[x] + 4u * s[xp1]
					+ s[xp2] + 8u) >> 4);
		}
	}
	for (uint32_t y = 0; y < h; y++) {
		uint32_t ym2 = (y >= 2u) ? y - 2u : 0u;
		uint32_t ym1 = (y >= 1u) ? y - 1u : 0u;
		uint32_t yp1 = (y + 1u < h) ? y + 1u : h - 1u;
		uint32_t yp2 = (y + 2u < h) ? y + 2u : h - 1u;
		const uint8_t *r0 = tmp + ym2 * w, *r1 = tmp + ym1 * w;
		const uint8_t *r2 = tmp + y * w, *r3 = tmp + yp1 * w;
		const uint8_t *r4 = tmp + yp2 * w;
		uint8_t *d = img + y * w;
		for (uint32_t x = 0; x < w; x++) {
			d[x] = (uint8_t) ((r0[x] + 4u * r1[x] + 6u * r2[x] + 4u * r3[x]
					+ r4[x] + 8u) >> 4);
		}
	}
}

/* Etap 3: Sobel 3x3 -> |G| = (|Gx|+|Gy|)/4 obciete do 255 oraz kierunek
 * 0..3. Brzeg 1 px dostaje |G| = 0.                                        */
static void Canny_Sobel(const uint8_t *img, uint8_t *mag, uint8_t *dir,
		uint32_t w, uint32_t h) {
	for (uint32_t x = 0; x < w; x++) {
		mag[x] = 0;
		mag[(h - 1u) * w + x] = 0;
	}
	for (uint32_t y = 1; y + 1u < h; y++) {
		const uint8_t *up = img + (y - 1u) * w;
		const uint8_t *mid = img + y * w;
		const uint8_t *dn = img + (y + 1u) * w;
		uint8_t *m = mag + y * w;
		uint8_t *dd = dir + y * w;
		m[0] = 0;
		m[w - 1u] = 0;
		for (uint32_t x = 1; x + 1u < w; x++) {
			int32_t gx = (int32_t) up[x + 1u] + 2 * (int32_t) mid[x + 1u]
					+ (int32_t) dn[x + 1u] - (int32_t) up[x - 1u]
					- 2 * (int32_t) mid[x - 1u] - (int32_t) dn[x - 1u];
			int32_t gy = (int32_t) dn[x - 1u] + 2 * (int32_t) dn[x]
					+ (int32_t) dn[x + 1u] - (int32_t) up[x - 1u]
					- 2 * (int32_t) up[x] - (int32_t) up[x + 1u];
			int32_t ax = (gx < 0) ? -gx : gx;
			int32_t ay = (gy < 0) ? -gy : gy;
			int32_t s = (ax + ay) >> 2;
			m[x] = (uint8_t) ((s > 255) ? 255 : s);
			dd[x] = Canny_Dir(gx, gy);
		}
	}
}

/* Etap 4: tlumienie niemaksymalne w miejscu - piksel zostaje tylko, gdy
 * jest >= obu sasiadow wzdluz kierunku gradientu. Wiersz y-1 jest juz
 * nadpisany, wiec jego oryginal trzymamy w buforze liniowym.              */
static void Canny_Nms(uint8_t *mag, const uint8_t *dir, uint32_t w, uint32_t h) {
	static uint8_t row_prev[OV7670_WIDTH], row_cur[OV7670_WIDTH];

	memcpy(row_prev, mag, w);
	for (uint32_t y = 1; y + 1u < h; y++) {
		uint8_t *m = mag + y * w;
		const uint8_t *below = m + w;
		const uint8_t *dd = dir + y * w;
		memcpy(row_cur, m, w);
		for (uint32_t x = 1; x + 1u < w; x++) {
			uint32_t v = row_cur[x], a, b;
			switch (dd[x]) {
			case 0:
				a = row_cur[x - 1u];
				b = row_cur[x + 1u];
				break;
			case 1:
				a = row_prev[x - 1u];
				b = below[x + 1u];
				break;
			case 2:
				a = row_prev[x];
				b = below[x];
				break;
			default:
				a = row_prev[x + 1u];
				b = below[x - 1u];
				break;
			}
			if (v < a || v < b) {
				m[x] = 0;
			}
		}
		memcpy(row_prev, row_cur, w);
	}
}

/* Jeden przebieg propagacji histerezy (w przod lub w tyl): slaba krawedz
 * (128) dotykajaca mocnej (255) w 8-sasiedztwie staje sie mocna. Zwraca
 * liczbe zmian.                                                            */
static uint32_t Canny_HystPass(uint8_t *mag, uint32_t w, uint32_t h,
		int reverse) {
	uint32_t changed = 0;

	for (uint32_t i = 1; i + 1u < h; i++) {
		uint32_t y = reverse ? (h - 1u - i) : i;
		uint8_t *m = mag + y * w;
		const uint8_t *up = m - w, *dn = m + w;
		for (uint32_t j = 1; j + 1u < w; j++) {
			uint32_t x = reverse ? (w - 1u - j) : j;
			if (m[x] == 128u
					&& (m[x - 1u] == 255u || m[x + 1u] == 255u
							|| up[x - 1u] == 255u || up[x] == 255u
							|| up[x + 1u] == 255u || dn[x - 1u] == 255u
							|| dn[x] == 255u || dn[x + 1u] == 255u)) {
				m[x] = 255u;
				changed++;
			}
		}
	}
	return changed;
}

/* Etap 5: progowanie z histereza: >= high -> 255 (mocna), >= low -> 128
 * (slaba), reszta 0; potem kilka przebiegow propagacji w przod i w tyl
 * zamiast rekurencji (brak RAM na stos). Slabe bez kontaktu zostaja 128 -
 * etap wyjsciowy traktuje je jak tlo.                                     */
static void Canny_Hysteresis(uint8_t *mag, uint32_t w, uint32_t h, uint8_t low,
		uint8_t high) {
	for (uint32_t i = 0; i < w * h; i++) {
		mag[i] = (mag[i] >= high) ? 255u : (mag[i] >= low) ? 128u : 0u;
	}
	for (uint32_t pass = 0; pass < CANNY_HYST_PASSES; pass++) {
		uint32_t changed = Canny_HystPass(mag, w, h, 0);
		changed += Canny_HystPass(mag, w, h, 1);
		if (changed == 0u) {
			break;
		}
	}
}

/*
 * Operator Canny'ego na klatce RGB565 lezacej w frame_buffer (in_stride
 * bajtow na linie). Wynik: RGB565 (0xFFFF krawedz / 0x0000 tlo) w
 * frame_buffer, stride = width*2. Zwraca ten stride.
 *
 * Rozklad pamieci (N = width*height, 2N <= CAPTURE_BUF_BYTES):
 *   gray_buf            : szarosc -> po rozmyciu: obraz rozmyty
 *   frame_buffer[0..N)  : bufor tymczasowy rozmycia -> potem KIERUNEK
 *   frame_buffer[N..2N) : |gradient| -> po NMS/histerezie: mapa krawedzi
 *   frame_buffer[0..2N) : wynik RGB565 - zapis "do przodu" nadpisuje tylko
 *                         bajty juz odczytane (piksel i czyta [N+i], pisze
 *                         [2i],[2i+1]; 2i+1 <= N+i dla i < N).
 */
static uint32_t Filter_Canny(uint32_t width, uint32_t height,
		uint32_t in_stride) {
	const uint32_t n = width * height;
	uint8_t *tmp_dir = frame_buffer; /* [0..n)  */
	uint8_t *mag = frame_buffer + n; /* [n..2n) */

	if (width < 5u|| height < 5u || width > OV7670_WIDTH
	|| n > sizeof(gray_buf) || 2u * n > CAPTURE_BUF_BYTES) {
		return in_stride; /* nie zmiesci sie - wyslij surowe */
	}

	Canny_Gray(frame_buffer, in_stride, gray_buf, width, height);
	Canny_Blur(gray_buf, tmp_dir, width, height);
	Canny_Sobel(gray_buf, mag, tmp_dir, width, height);
	Canny_Nms(mag, tmp_dir, width, height);
	Canny_Hysteresis(mag, width, height, canny_low, canny_high);

	for (uint32_t i = 0; i < n; i++) {
		uint8_t v = (mag[i] == 255u) ? 0xFFu : 0x00u;
		frame_buffer[2u * i] = v;
		frame_buffer[2u * i + 1u] = v;
	}
	return width * 2u;
}

/* ==========================================================================
 * Ramka na UART:
 *   [0xA5][0x5A]['O']['V'][W_lo][W_hi][H_lo][H_hi][S_lo][S_hi][dane...]
 * W = piksele w linii (= stride/2), H = linie, S = stride = bajtow na linie
 * tak, jak je dostal DMA. Dane: H x S bajtow, RGB565 MSB first.
 * ========================================================================== */
static void SendFrame(uint32_t stride) {
	uint32_t width = stride / 2u;
	uint32_t total = stride * OV7670_HEIGHT;
	uint8_t hdr[10] = { FRAME_MAGIC_0, FRAME_MAGIC_1, 'O', 'V', (uint8_t) (width
			& 0xFFu), (uint8_t) (width >> 8), (uint8_t) (OV7670_HEIGHT & 0xFFu),
			(uint8_t) (OV7670_HEIGHT >> 8), (uint8_t) (stride & 0xFFu),
			(uint8_t) (stride >> 8) };

	HAL_UART_Transmit(&huart3, hdr, sizeof(hdr), 200);
	/* HAL_UART_Transmit bierze rozmiar 16-bitowy - QVGA (153600 B) idzie w
	 * kawalkach. 32 KB przy 921600 baud = ~360 ms.                          */
	for (uint32_t off = 0; off < total; off += 32768u) {
		uint32_t n = total - off;
		if (n > 32768u) {
			n = 32768u;
		}
		HAL_UART_Transmit(&huart3, frame_buffer + off, (uint16_t) n, 2000);
	}
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
