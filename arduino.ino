// Author: Riccardo Paolo Bestetti <pbl@bestov.io>
// Copyright 2021 Riccardo Paolo Bestetti
// License: you can use this however you want, provided that:
//          - you release any derived work under this same license
//          - and you give credits to the author (me)

#include <TimerOne.h>

// debugging?
#define DEBUG
#ifdef DEBUG
  char debug_buffer[128];
  #define PRINT(...) { sprintf(debug_buffer, __VA_ARGS__); Serial.print(debug_buffer); }
#else
  #define PRINT(...)
#endif

#ifndef DEBUG_SPEED
  // input polling frequency
  #define POLLING_HZ 100

  // how many samples to debounce over
  #define DEBOUNCE_SAMPLES 16

  // how many times units (1/POLLING_HZ seconds long) can elapse between
  // short presses to be considered part of the same sequence
  #define SEQUENCE_UNITS 50
#else
  #define POLLING_HZ 2
  #define DEBOUNCE_SAMPLES 3
  #define SEQUENCE_UNITS 5
#endif

// input and output pins - numbered from the top button going down
const int out[] = { 7, 8, 9 };
const int in[] = { 12, 11, 10 };
const int n_buttons = sizeof out / sizeof out[0];

void isr();

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif

  // configure port registers
  pinMode(13, OUTPUT);

  int i;
  for (i = 0; i < n_buttons; i++) {
    pinMode(in[i], INPUT_PULLUP);
    pinMode(out[i], OUTPUT);
    digitalWrite(out[i], LOW);
  }

  // set up timer interrupt
  Timer1.initialize(1000000 / POLLING_HZ);
  Timer1.attachInterrupt(isr);

  PRINT("hello!\n");
}

//////////
// DATA //
//////////

// object representing the latest debounced state of a button, and the latest
// DEBOUNCE_SAMPLES samples from the corresponding input port
struct s_debounced_button {
  int state;                      // the latest debounced state of the button
  int samples[DEBOUNCE_SAMPLES];  // the latest samples from the input port (circular)
  int next_sample;                // the position for the next sample
  int sum;                        // the sum over samples[]
};

// the tresholds for changing state
const int tres_lo = 4 * DEBOUNCE_SAMPLES / 9;
const int tres_hi = (int)(.5f + (5 * DEBOUNCE_SAMPLES / 9));

// the debounced buttons
s_debounced_button debounced_buttons[n_buttons] = { 0 };

// timer remaining time units, where one unit is 1/POLLING_HZ seconds
// the value "-1" means that the timer is not running
int timer_remaining_units = -1;

// short press remaining time units, where one unit is 1/POLLING_HZ seconds
// used to terminate short press sequences
int shortpress_remaining_units = -1;

// types of event
enum event {
  NullEvent,  // acts as a terminator for the queue
  BtnDown,    // button pressed
  BtnUp,      // button depressed
  Sequence    // short press sequence is over
};

// object representing an event
struct s_event {
  enum event type;  // the type of event
  int button;       // for button events, the button of the event; otherwise undefined
};

// queue implemented as an array - should be good enough
s_event queue[16] = { 0 };  // 16 queued events should be more than enough for anything
const int queue_size = sizeof queue / sizeof queue[0];


///////////////
// FUNCTIONS //
///////////////

// enqueue an event
void enqueue_event(s_event *event);

// dequeue the first n events, performs no bound checking
void dequeue_n(size_t howmany);

// timer event generation
inline void expire_timer();

// called when the timer has expired
inline void timer_expired();

// debouncing logic and button event generation
inline void read_buttons();

// process event queue
inline void process_events();

// opens all relays
inline void stop_all();

// called when there is an event of the form "N presses, short"
inline void short_event(size_t n, int btn);

// called when there is an event of the form "N presses, long"
inline void long_event(size_t n, int btn);

// enqueue an event
void enqueue_event(s_event *event) {
  int i;
  for (i = 0; i < queue_size; i++)
    if (queue[i].type == NullEvent) {  // found free slot
      PRINT("new event %d with type %d\n", i, event->type);
      memcpy(&queue[i], event, sizeof queue[0]);
      return;
    }
}

// dequeue the first n events, performs no bound checking
void dequeue_n(size_t howmany) {
  PRINT("dequeueing %d events\n", howmany);
  memmove(&queue[0], &queue[howmany], howmany * sizeof queue[0]);
  if (howmany != 0)  // avoid bof
    queue[queue_size - howmany].type = NullEvent;
}

// executed at frequency POLLING_HZ
void isr() {
  expire_timer();
  read_buttons();
  process_events();
}

// timer event generation
inline void expire_timer() {
  if (timer_remaining_units >= 0 && timer_remaining_units-- == 0)  // decrement and check timer expired
    timer_expired();
}

// timer expired event - doesn't get queued, it is always executed immediately
inline void timer_expired() {
  PRINT("timer expired\n");
  stop_all();  // I'm lazy!
}

// opens all relays
inline void stop_all() {
  int i;
  for (i = 0; i < n_buttons; i++)
    digitalWrite(out[i], LOW);
}

// debouncing logic and button event generation
inline void read_buttons() {
  // button event prototypes
  static s_event button_down_event = { .type = BtnDown };
  static s_event button_up_event   = { .type = BtnUp };
  static s_event sequence_event    = { .type = Sequence };
  
  int i;
  for (i = 0; i < n_buttons; i++) {
    s_debounced_button *btn;
    int diff;

    btn = &debounced_buttons[i];
    diff = !digitalRead(in[i]) - btn->samples[btn->next_sample];

    btn->samples[btn->next_sample] += diff;
    btn->sum += diff;

    btn->next_sample = (btn->next_sample + 1) % DEBOUNCE_SAMPLES;

    if (diff == 1)  // reset the timer before debouncing to avoid user frustration :)
      shortpress_remaining_units = SEQUENCE_UNITS;

    if (btn->sum > tres_hi && btn->state == 0) {
      btn->state = 1;
      button_down_event.button = i;
      enqueue_event(&button_down_event);
    }
    else if (btn->sum < tres_lo && btn->state == 1) {
      btn->state = 0;
      button_up_event.button = i;
      enqueue_event(&button_up_event);
    }
  }

  // only process the timer if it hasn't expired already
  if (shortpress_remaining_units >= 0 && shortpress_remaining_units-- == 0)
    enqueue_event(&sequence_event);
}

// process event queue
inline void process_events() {
  // we are interested in the following kinds of sequences:
  // down, up, down, up, ..., up, sequence => N presses, short event
  // down, up, down, up, ..., down, sequence => N presses, long event
  // timer => timer expired event
  // parsing strategy:
  // - if you find up, sequence or null, consume it, stop the water, and return
  // - if you find down, go on until it keeps alternating (checking that's always
  //   the same button). if you find a null, ignore everything and retry later. if
  //   you find sequence and the second-last was an up, you have a N presses, short
  //   event; if it was a down, you have an N presses, long event (where N is the
  //   amount of down events in the sequence). if the alternating pattern is not
  //   satisfied, stop everything.

  s_event *event = &queue[0];

  if (event->type != BtnDown) {
    if (event->type != NullEvent) {
      PRINT("first event is %d\n", event->type);
      stop_all();
      dequeue_n(1);
    }
    return;
  }

  PRINT("first event is BtnDown(%d), queue:\n", event->button);

  int i;
  int n_downs = 1;
  int btn = event->button;
  for (i = 1; i < queue_size; i++) {
    event = &queue[i];
    PRINT("   [%d] %d(%d)\n", i, event->type, event->button);
    switch (event->type) {
      case BtnUp:  // we should have the same button, and i should be odd
        if (event->button != btn || i % 2 != 1)
          goto panic_exit;
        break;
      case BtnDown:  // we should have the same button, and i should be even
        if (event->button != btn || i % 2 != 0)
          goto panic_exit;
        n_downs++;
        break;
      case Sequence:  // if even, second-last event was a up, else, it was a down
        if (i % 2 == 0)
          short_event(n_downs, btn);
        else
          long_event(n_downs, btn);
        goto good_exit;
      case NullEvent:
        goto neutral_exit;
    }
  }
panic_exit:
  stop_all();
good_exit:
  dequeue_n(i + 1);
neutral_exit:
  return;  // the label wants a statement
}

// called when there is an event of the form "N presses, short"
inline void short_event(size_t n, int btn) {
  if (timer_remaining_units >= 0) {  // stop the water
    timer_remaining_units = 0;
    return;
  }
  
  PRINT("action: short, %d times, btn %d\n", n, btn);
  stop_all();
  timer_remaining_units = 6000;
  digitalWrite(out[btn], HIGH);
}

// called when there is an event of the form "N presses, long"
inline void long_event(size_t n, int btn) {
  PRINT("action: long, %d times, btn %d\n", n, btn);
  stop_all();
  digitalWrite(out[btn], HIGH);
}

void loop() {}
