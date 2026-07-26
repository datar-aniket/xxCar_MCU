/* Host unit test for the calibration protocol (apps/cal/cal_proto.c).
 *
 * The protocol is deliberately text-in / JSON-out so a human can drive a
 * calibration from a plain terminal when the GUI misbehaves. That property is
 * only real if the parser tolerates what a human types - trailing CR from a
 * terminal, extra spaces, mixed case - and if the emitted JSON is actually
 * well formed. Both are checked here, on the host.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "cal_proto.h"

static int g_fail;

static void expect_cmd(const char *line, enum cal_cmd_e want)
{
  struct cal_cmd_s c;

  cal_proto_parse(line, &c);

  if (c.cmd != want)
    {
      printf("FAIL parse(\"%s\"): got %d want %d\n", line, (int)c.cmd,
             (int)want);
      g_fail++;
    }
}

static void expect_contains(const char *what, const char *hay, const char *needle)
{
  if (strstr(hay, needle) == NULL)
    {
      printf("FAIL %s: \"%s\" does not contain \"%s\"\n", what, hay, needle);
      g_fail++;
    }
}

static void test_basic_commands(void)
{
  expect_cmd("hello",      CAL_CMD_HELLO);
  expect_cmd("capture",    CAL_CMD_CAPTURE);
  expect_cmd("commit",     CAL_CMD_COMMIT);
  expect_cmd("abort",      CAL_CMD_ABORT);
  expect_cmd("quit",       CAL_CMD_QUIT);
  expect_cmd("nonsense",   CAL_CMD_UNKNOWN);
  expect_cmd("",           CAL_CMD_NONE);
}

/* A terminal sends CRLF, a GUI sends LF, and people type trailing spaces.
 * None of those are protocol errors and all of them have to parse.
 */

static void test_tolerates_human_input(void)
{
  expect_cmd("hello\r",      CAL_CMD_HELLO);
  expect_cmd("  hello  ",    CAL_CMD_HELLO);
  expect_cmd("HELLO",        CAL_CMD_HELLO);
  expect_cmd("hello\r\n",    CAL_CMD_HELLO);
}

static void test_set_parses_name_and_value(void)
{
  struct cal_cmd_s c;

  cal_proto_parse("set CAL_ACC0_BX -0.125", &c);

  if (c.cmd != CAL_CMD_SET)
    {
      printf("FAIL set: cmd %d\n", (int)c.cmd);
      g_fail++;
      return;
    }

  if (strcmp(c.name, "CAL_ACC0_BX") != 0)
    {
      printf("FAIL set: name \"%s\"\n", c.name);
      g_fail++;
    }

  if (fabsf(c.fval - (-0.125f)) > 1e-6f)
    {
      printf("FAIL set: value %f\n", c.fval);
      g_fail++;
    }
}

/* A name longer than the parameter store allows must be rejected, not
 * truncated into a different valid parameter's name.
 */

static void test_overlong_name_rejected(void)
{
  struct cal_cmd_s c;

  cal_proto_parse("get CAL_THIS_NAME_IS_FAR_TOO_LONG", &c);

  if (c.cmd != CAL_CMD_UNKNOWN)
    {
      printf("FAIL overlong name: cmd %d, name \"%s\"\n", (int)c.cmd, c.name);
      g_fail++;
    }
}

/* A value that fails to parse must not silently become 0.0 - once SET writes
 * to the parameter store (Task 5), that would persist a wrong calibration
 * value indistinguishable from someone deliberately choosing zero.
 */

static void test_set_rejects_garbage_value(void)
{
  struct cal_cmd_s c;

  cal_proto_parse("set CAL_ACC0_BX abc", &c);

  if (c.cmd != CAL_CMD_UNKNOWN)
    {
      printf("FAIL set garbage value: cmd %d, fval %f\n", (int)c.cmd, c.fval);
      g_fail++;
    }
}

/* strtof() happily parses the numeric prefix of "1.5xyz" and stops at the
 * 'x', returning 1.5 - it is *endptr, not the return value, that says trailing
 * junk was left over. Dropping that check would silently store 1.5 for a
 * value the host never actually sent (a typo, a stray unit suffix), so it
 * must be rejected exactly like pure garbage.
 */

static void test_set_rejects_trailing_garbage(void)
{
  struct cal_cmd_s c;

  cal_proto_parse("set CAL_ACC0_BX 1.5xyz", &c);

  if (c.cmd != CAL_CMD_UNKNOWN)
    {
      printf("FAIL set trailing garbage: cmd %d, fval %f\n", (int)c.cmd,
             (double)c.fval);
      g_fail++;
    }
}

/* A deliberate zero must still parse as a legitimate SET, not be confused
 * with the garbage-value rejection above.
 */

static void test_set_accepts_zero_value(void)
{
  struct cal_cmd_s c;

  cal_proto_parse("set CAL_ACC0_BX 0", &c);

  if (c.cmd != CAL_CMD_SET)
    {
      printf("FAIL set zero value: cmd %d\n", (int)c.cmd);
      g_fail++;
      return;
    }

  if (c.fval != 0.0f)
    {
      printf("FAIL set zero value: fval %f\n", c.fval);
      g_fail++;
    }
}

static void test_hello_is_json(void)
{
  char buf[256];
  int n = cal_proto_hello(buf, sizeof(buf));

  if (n <= 0)
    {
      printf("FAIL hello: returned %d\n", n);
      g_fail++;
      return;
    }

  expect_contains("hello", buf, "\"evt\":\"hello\"");
  expect_contains("hello", buf, "\"proto\":1");

  if (buf[n - 1] != '\n')
    {
      printf("FAIL hello: not newline terminated\n");
      g_fail++;
    }
}

static void test_captured_carries_vectors(void)
{
  char buf[256];
  const float acc[3] = { 0.01f, -0.02f, 9.79f };
  const float gyr[3] = { 0.001f, 0.002f, -0.003f };
  int n = cal_proto_captured(buf, sizeof(buf), 7, acc, gyr, 41.25f);

  if (n <= 0)
    {
      printf("FAIL captured: returned %d\n", n);
      g_fail++;
      return;
    }

  expect_contains("captured", buf, "\"evt\":\"captured\"");
  expect_contains("captured", buf, "\"n\":7");
  expect_contains("captured", buf, "9.79");
  expect_contains("captured", buf, "41.25");
}

/* An error message is built from text we do not control. A quote or backslash
 * in it must not be able to break out of the JSON string.
 */

static void test_error_escapes_json(void)
{
  char buf[256];
  int n = cal_proto_error(buf, sizeof(buf), "bad \"quoted\" \\ thing");

  if (n <= 0)
    {
      printf("FAIL error: returned %d\n", n);
      g_fail++;
      return;
    }

  expect_contains("error", buf, "\\\"quoted\\\"");
  expect_contains("error", buf, "\\\\");
}

/* Emitters must never run off the end of a short buffer. */

static void test_truncation_is_safe(void)
{
  char small[8];
  int n = cal_proto_hello(small, sizeof(small));

  if (n != -1)
    {
      printf("FAIL truncation: returned %d, expected -1\n", n);
      g_fail++;
    }
}

int main(void)
{
  test_basic_commands();
  test_tolerates_human_input();
  test_set_parses_name_and_value();
  test_overlong_name_rejected();
  test_set_rejects_garbage_value();
  test_set_rejects_trailing_garbage();
  test_set_accepts_zero_value();
  test_hello_is_json();
  test_captured_carries_vectors();
  test_error_escapes_json();
  test_truncation_is_safe();

  if (g_fail != 0)
    {
      printf("cal_proto: %d failure(s)\n", g_fail);
      return 1;
    }

  printf("cal_proto: parser and emitters verified - OK\n");
  return 0;
}
