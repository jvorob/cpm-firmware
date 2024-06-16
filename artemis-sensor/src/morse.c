// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Janet vorobyeva, 2024

/** Example morse code lib **/

#include <morse.h>

#include <stdbool.h>

const int DOT_LENGTH = 120;
const int DASH_LENGTH = 300;
const int DOT_GAP = 80; // gap between dots
const int DASH_GAP = 80; // gap between dots
const int LETTER_GAP = 320; // gap between letters ('l')
const int WORD_GAP = 300; // gap between words ('w')

const int MULTIPLIER = 120; // 100 is default speed, <100 is faster


// ====================== MORSE BYTECODE TABLES ========================


// Morse bytecode:
// . or - means send a dot or dash (DOT_LENGTH or DASH_LENGTH,
//        followed by a DOTGAP)
// l means rest for one letter gap
// ; terminates a symbol

// All the alphabetic characters
char MORSE_ALPHAS[] = ".-l;-...l;-.-.l;-..l;.l;..-.l;--.l;....l;..l;.---l;-.-l;.-..l;--l;-.l;---l;.--.l;--.-l;.-.l;...l;-l;..-l;...-l;.--l;-..-l;-.--l;--..l;";

// symbols
char MORSE_SYMBOLS[] = 
  " |lll;"
  "\n|lll;"
  ".|.-.-.-lll;"
  "0|-----l;"
  "1|.----l;"
  "2|..---l;"
  "3|...--l;"
  "4|....-l;"
  "5|.....l;"
  "6|-....l;"
  "7|--...l;"
  "8|---..l;"
  "9|----.l;"
  "\0\0\0\0\0";


// For each ascii char, points into MORSE_ALPHAS or MORSE_SYMBOLS at the right place
char *morse_table[256] = {};

/* I'm lazy, so i've defined constants in a shitty bytecode that encode each character's
 * morse code representation. This function builds a table for each ascii character that
 * points into the correct part of the bytecode strings above */
void init_morse_table() {
  for (int i = 0; i < 256; i++) {
    morse_table[i] = ";";
  }

  //fill in alphabet chars
  char *next_seq = &MORSE_ALPHAS[0];
  for (int i = 0; i < 26; i++) {
    morse_table['a' + i] = next_seq;
    morse_table['A' + i] = next_seq;

    //find end of curr letter
    while (*next_seq != ';' && *next_seq != '\0')
      { next_seq++; }
    
    // skip over the semicolon
    next_seq++;
  }

  //fill in other chars
  char *p = &MORSE_SYMBOLS[0];
  while(*p != 0) {
    char curr_char = *p;
    p += 2; //skip char, skip |
    morse_table[(int)curr_char] = p;
    // walk p forward until after the next ;
    while(*p != ';' && *p != '\0') { p++; }
    if(*p == ';') { p++; }
  }

  //TODO: fill in the other chars
  morse_table[' '] = "lll;";
}


// ====================== MORSE STATE MACHINE ========================

// FLOW:
// Sending a morse string is a seq of chars
// Each char is a sequence of symbols (dit, dah, rest)
// Each symbol can have an on and an off


// Returns true if successful, false if EOF
struct morse_output morseAdvance(struct morse_state *state) {
  
  char curr_char = state->p_str[state->char_offset];
  char curr_sym = morse_table[(int)curr_char][state->symbol_offset];

  // Need to advance phase
  if(state->symbol_phases_left == 0){ // Curr symbol done
    // Need to get new symbol
    if(curr_sym != ';') { // if not at end, try to advance symbol
      state->symbol_offset++;
      curr_sym = morse_table[(int)curr_char][state->symbol_offset];
    }

    if(curr_sym == ';' || curr_sym == '\0') { // Curr char done
      //Need to get new char
      state->char_offset++;
      curr_char = state->p_str[state->char_offset];
      if(curr_char == '\0') {
        // finished the whole string
        struct morse_output retval = {0}; // .valid=false
        return retval;
      }

      //New Char gotten
      //init new symbol
      state->symbol_offset = 0;
      curr_sym = morse_table[(int)curr_char][state->symbol_offset];
    } else {
      //Already advanced smybol
      //state->symbol_offset++;
      //curr_sym = morse_table[curr_char][state->symbol_offset];
    }
    // New symbol gotten

    //Init phase:
    switch(curr_sym) {
      case '.':
      case '-':
        state->symbol_phases_left = 1; // 2 phsaes total
        break;
      default:
        state->symbol_phases_left = 0;
        break;
    }
  } else { //just go to next phase
    state->symbol_phases_left--;
  }

  // Now: we're on the right character
  // Need to render it
  
  // This will cease working if we need a special char for on, but its fine
  bool is_on = state->symbol_phases_left > 0;
  int duration;
  switch(curr_sym) {
    case '.':
      if (is_on) { duration = DOT_LENGTH; }
      else       { duration = DOT_GAP; }
      break;
    case '-':
      if (is_on) { duration = DASH_LENGTH; }
      else       { duration = DASH_GAP; }
      break;
    case 'l':
    default: //TODO: some sort of error handling for other cases?
      duration = LETTER_GAP;
      break;
  }

  struct morse_output retval = {
    .duration_ms = duration * MULTIPLIER / 100,
    .out_high = is_on,
    .valid = true,
  };
  return retval;
  
  //set_leds(is_on);
  //am_util_delay_ms(duration *  MULTIPLIER / 100);
}
