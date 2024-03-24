#ifndef __ENCODERS_h
#define __ENCODERS_h

uint32_t left_position = 0;
uint32_t left_prev_position = 0;
int32_t left_delta_position = 0;

uint32_t right_position = 0;
uint32_t right_prev_position = 0;
int32_t right_delta_position = 0;

uint32_t limit_min = -2147483648;
uint32_t limit_max = 2147483647;
uint32_t encoder_max = 4294967296;

void update_left_wheel(uint32_t position) {

  left_position = position;
  uint32_t increment = left_position - left_prev_position;

  if(increment < limit_min) {
      increment = (encoder_max - left_prev_position) + left_position;
  } else if(increment > limit_max) {
      increment = ((encoder_max - left_position) + left_prev_position) * -1;
  }
  left_delta_position += increment;
  left_prev_position = left_position;
}

void update_right_wheel(uint32_t position) {

    right_position = position;
    uint32_t increment = right_position - right_prev_position;

    if(increment < limit_min) {
        increment = (encoder_max - right_prev_position) + right_position;
    } else if(increment > limit_max) {
        increment = ((encoder_max - right_position) + right_prev_position) * -1;
    }
    right_delta_position += increment;
    right_prev_position = right_position;
}

// we expect delta_position to be negative
int32_t get_left_delta(void) {
    int32_t delta_position = left_delta_position;
    left_delta_position = 0;
    return delta_position;
}

int32_t get_right_delta(void) {
    int32_t delta_position = right_delta_position;
    right_delta_position = 0;
    return delta_position;
}

#endif