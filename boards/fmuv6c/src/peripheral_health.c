/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <nuttx/irq.h>
#include <arch/board/peripheral_health.h>

static struct board_sensor_health_s g_health[BOARD_HEALTH_COUNT];

int board_sensor_health(unsigned index, struct board_sensor_health_s *out)
{
  irqstate_t flags;
  if (index >= BOARD_HEALTH_COUNT || out == NULL) return -EINVAL;
  flags = enter_critical_section();
  *out = g_health[index];
  leave_critical_section(flags);
  return 0;
}

void board_sensor_health_publish(unsigned index,
                                const struct board_sensor_health_s *status)
{
  irqstate_t flags;
  if (index >= BOARD_HEALTH_COUNT || status == NULL) return;
  flags = enter_critical_section();
  g_health[index] = *status;
  leave_critical_section(flags);
}
