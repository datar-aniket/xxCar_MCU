/****************************************************************************
 * apps/clear/clear_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * `clear` - clear the terminal.
 *
 * Writes the VT100 sequences to erase the screen and put the cursor back in
 * the top-left corner. Both are needed: erasing alone leaves the cursor wherever
 * it was, and the shell would then draw its next prompt half way down a blank
 * screen.
 *
 * Written to stdout, so it clears whichever terminal the shell is on - the
 * console, a TELEM port, or the USB shell.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>

#include <nuttx/vt100.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  static const char clearscreen[] = VT100_CLEARSCREEN;
  static const char cursorhome[]  = VT100_CURSORHOME;

  /* Straight to fd 1 rather than through printf: these are escape sequences,
   * not text, and there is no reason to run them through a formatter.
   */

  write(STDOUT_FILENO, clearscreen, sizeof(clearscreen));
  write(STDOUT_FILENO, cursorhome, sizeof(cursorhome));

  return 0;
}
