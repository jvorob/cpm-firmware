// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Gabriel Marcano, 2023

#ifndef MORSE_H_
#define MORSE_H_

/** This is a placeholder example header file */

#include <stdbool.h>
#include <stdint.h>

struct morse_state {
  char *p_str;

  // This is the pointer to the next thing to render
  // which phase of which symbol of which char
  uint32_t char_offset;
  uint32_t symbol_offset;  // which dot or dash of this char are we on
  int symbol_phases_left;  // for . and -,  1 is dit, 0 is gap
                            // spaces just get 0
                            

  // // What to render
  // bool render_state; // led on or off
  // uint32_t render_time; // how long to stay here
};

struct morse_output {
    int duration_ms; // duration of this dot/dash/gap
    bool out_high;   // is the output (led, beep, CW) high for the duration
    bool valid;      // will be set to 0 on EOF
};

/** Initializes morse code structures, must be called once before use **/
void init_morse_table();

/** Gets next unit (high or low + duration) of morse sequence 
 * 
 **/
struct morse_output morseAdvance(struct morse_state *state);



#endif//MORSE_H_
