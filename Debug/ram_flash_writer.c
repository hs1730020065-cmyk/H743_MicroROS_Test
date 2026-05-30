typedef unsigned int uint32_t;

#define FLASH_BASE_ADDR 0x52002000UL
#define FLASH_KEYR1     (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x04UL))
#define FLASH_CR1       (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x0CUL))
#define FLASH_SR1       (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x10UL))
#define FLASH_CCR1      (*(volatile uint32_t *)(FLASH_BASE_ADDR + 0x14UL))

#define FLASH_KEY1      0x45670123UL
#define FLASH_KEY2      0xCDEF89ABUL
#define FLASH_CR_LOCK   (1UL << 0)
#define FLASH_CR_PG     (1UL << 1)
#define FLASH_CR_SER    (1UL << 2)
#define FLASH_CR_PSIZE  (3UL << 4)
#define FLASH_CR_START  (1UL << 7)
#define FLASH_CR_SNB    (7UL << 8)

#define FLASH_SR_BSY    (1UL << 0)
#define FLASH_SR_QW     (1UL << 2)
#define FLASH_SR_ERRS   ((1UL << 17) | (1UL << 18) | (1UL << 19) | \
                         (1UL << 21) | (1UL << 22) | (1UL << 23) | \
                         (1UL << 24) | (1UL << 25) | (1UL << 26) | \
                         (1UL << 28))
#define FLASH_SR_EOP    (1UL << 16)

#define SRC_ADDR        0x24000000UL
#define DST_ADDR        0x08000000UL
#define IMAGE_LEN       114432UL
#define RESULT_ADDR     0x2401FFF0UL

static void wait_flash(void)
{
  while ((FLASH_SR1 & (FLASH_SR_BSY | FLASH_SR_QW)) != 0UL) {
  }
}

void _start(void)
{
  volatile uint32_t *result = (volatile uint32_t *)RESULT_ADDR;
  volatile uint32_t *src = (volatile uint32_t *)SRC_ADDR;
  volatile uint32_t *dst = (volatile uint32_t *)DST_ADDR;

  *result = 0x11111111UL;

  if ((FLASH_CR1 & FLASH_CR_LOCK) != 0UL) {
    FLASH_KEYR1 = FLASH_KEY1;
    FLASH_KEYR1 = FLASH_KEY2;
  }

  wait_flash();
  FLASH_CCR1 = FLASH_SR_ERRS | FLASH_SR_EOP;

  FLASH_CR1 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
  FLASH_CR1 |= FLASH_CR_SER | FLASH_CR_PSIZE | FLASH_CR_START;
  wait_flash();
  FLASH_CR1 &= ~(FLASH_CR_SER | FLASH_CR_SNB);
  FLASH_CCR1 = FLASH_SR_ERRS | FLASH_SR_EOP;

  for (uint32_t offset = 0; offset < IMAGE_LEN; offset += 32UL) {
    FLASH_CR1 |= FLASH_CR_PG;
    __asm volatile ("isb");
    __asm volatile ("dsb");

    for (uint32_t i = 0; i < 8UL; i++) {
      *dst++ = *src++;
    }

    __asm volatile ("dsb");
    wait_flash();
    FLASH_CR1 &= ~FLASH_CR_PG;

    if ((FLASH_SR1 & FLASH_SR_ERRS) != 0UL) {
      *result = FLASH_SR1;
      break;
    }
  }

  if (*result == 0x11111111UL) {
    *result = 0x12345678UL;
  }

  for (;;) {
    __asm volatile ("bkpt #0");
  }
}
