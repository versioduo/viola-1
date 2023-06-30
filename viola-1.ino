// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

#include "MIDISong.h"
#include <V2Base.h>
#include <V2Buttons.h>
#include <V2Color.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2Music.h>
#include <V2PowerSupply.h>
#include <V2Stepper.h>

V2DEVICE_METADATA("com.versioduo.viola-1", 32, "versioduo:samd:step");

static constexpr uint8_t notesMax{20};
static constexpr uint8_t nSteppers{4};
static V2LED::WS2812 LED(nSteppers + 2, PIN_LED_WS2812, &sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
static V2Link::Port Plug(&SerialPlug);
static V2Link::Port Socket(&SerialSocket);
static V2Base::Timer::Periodic Timer(2, 200000);
static V2Base::Analog::ADC ADC(V2Base::Analog::ADC::getID(PIN_VOLTAGE_SENSE));

// The button switches the state with a multi-click long-press.
static class Manual {
public:
  enum class Mode { Notes, Song, Test, Tune, Turn } mode{};
  Mode getMode() const {
    return _mode;
  }

  void setMode(Mode mode = Mode::Notes) {
    _mode = mode;

    switch (_mode) {
      case Mode::Notes:
        LED.reset();
        LED.setHSV(nSteppers + 0, V2Color::Orange, 1, 0.25);
        LED.setHSV(nSteppers + 1, V2Color::Orange, 1, 0.25);
        break;

      case Mode::Song:
        LED.reset();
        LED.setBrightness(nSteppers + 0, 0.25);
        LED.setBrightness(nSteppers + 1, 0.25);
        break;

      case Mode::Test:
        LED.reset();
        LED.rainbow(1, 3, 0.4);
        break;

      case Mode::Tune:
        LED.reset();
        LED.setHSV(nSteppers + 0, V2Color::Magenta, 1, 0.25);
        LED.setHSV(nSteppers + 1, V2Color::Magenta, 1, 0.25);
        break;

      case Mode::Turn:
        LED.reset();
        LED.setHSV(nSteppers + 0, V2Color::Cyan, 1, 0.25);
        LED.setHSV(nSteppers + 1, V2Color::Cyan, 1, 0.25);
        break;
    }
  }

private:
  Mode _mode{};
} Manual;

static class Stepper : public V2Stepper::Motor {
public:
  enum { Bow, BowPressure, Finger, FingerPressure };

  constexpr Stepper(const Motor::Config conf, uint8_t index) :
    Motor(conf, &Timer, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
    _index(index) {}

private:
  const uint8_t _index;

  void handleMovement(Move move) override {
    switch (move) {
      case Move::Forward:
        LED.setHSV(_index, V2Color::Cyan, 1, 0.4);
        break;

      case Move::Reverse:
        LED.setHSV(_index, V2Color::Orange, 1, 0.4);
        break;

      case Move::Stop:
        LED.setHSV(_index, V2Color::Green, 1, 0.15);
        break;
    }
  }
} Steppers[nSteppers]{
  Stepper(
    {
      .ampere{0.75},
      .microstepsShift{4},
      .speed{.min{20}, .max{700}, .accel{1000}},
    },
    Stepper::Bow),
  Stepper(
    {
      .ampere{0.5},
      .microstepsShift{4},
      .inverse{true},
      .home{.speed{500}, .stall{0.04}},
      .speed{.min{5}, .max{4000}, .accel{8000}},
    },
    Stepper::BowPressure),
  Stepper(
    {
      .ampere{0.6},
      .microstepsShift{4},
      .home{.speed{750}, .stall{0.04}},
      .speed{.min{50}, .max{2400}, .accel{24000}},
    },
    Stepper::Finger),
  Stepper(
    {
      .ampere{0.6},
      .microstepsShift{4},
      .inverse{true},
      .home{.speed{200}, .stall{0.07}},
      .speed{.min{50}, .max{150}, .accel{16000}},
    },
    Stepper::FingerPressure),
};

static class Power : public V2PowerSupply {
public:
  constexpr Power() : V2PowerSupply({.min{6}, .max{26}}) {}

  void begin() {
    pinMode(PIN_DRIVER_ENABLE, OUTPUT);
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

private:
  float handleMeasurement() override {
    // A voltage 10/100k divider.
    return 36.f * ADC.readChannel(V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE));
  }

  void handleOn() override {
    digitalWrite(PIN_DRIVER_ENABLE, LOW);
  }

  void handleOff() override {
    digitalWrite(PIN_DRIVER_ENABLE, HIGH);
  }

  void handleNotify(float voltage) override {
    // Power interruption, or commands without a power connection show yellow LEDs.
    if (voltage < config.min) {
      LED.splashHSV(0.5, V2Color::Yellow, 1, 0.5);
      return;
    }

    // Over-voltage shows red LEDs.
    if (voltage > config.max) {
      LED.splashHSV(0.5, V2Color::Red, 1, 1);
      return;
    }

    // The number of green LEDs shows the voltage.
    float fraction = voltage / (float)config.max;
    uint8_t n      = ceil((float)nSteppers * fraction);
    LED.splashHSV(0.5, n, V2Color::Green, 1, 0.5);
  }
} Power;

// Config, written to EEPROM.
static constexpr struct Configuration {
  struct {
    // The middle C, MIDI note 60, in this mapping is C3.
    uint8_t start{V2MIDI::C(2)};
    uint8_t count{20};
  } notes;

  struct {
    // Offsets in millimeters.
    float home{5};
    float min{4};
    float max{28};
  } bow;

  struct {
    // Overall string length in millimeters.
    float length{568};

    // Offset in millimeters from the home position to the first note.
    float home{5};
  } string;

  struct {
    // Additional steps of finger pressure past the detected home postion.
    uint8_t pressure{8};
  } finger;
} ConfigurationDefault;

static struct Configuration Config {
  ConfigurationDefault
};

// Calculate the effective velocity depending on the note velocity, aftertouch /
// pressure, and the volume controller.
static class {
public:
  operator bool() const {
    return _velocity > 0;
  }

  void reset() {
    _volume     = 100;
    _velocity   = 0;
    _aftertouch = 0;
    _fraction   = 0;
  }

  float getFraction() const {
    return _fraction;
  }

  uint8_t getVelocity() const {
    return _velocity;
  }

  void setVelocity(uint8_t velocity) {
    _velocity = velocity;
    if (_velocity == 0)
      _aftertouch = 0;
    update();
  }

  uint8_t getAftertouch() const {
    return _aftertouch;
  }

  void setAftertouch(uint8_t pressure) {
    _aftertouch = pressure;
    update();
  }

  uint8_t getVolume() const {
    return _volume;
  }

  void setVolume(uint8_t volume) {
    _volume = volume;
    update();
  }

private:
  uint8_t _volume{100};
  uint8_t _velocity{};
  uint8_t _aftertouch{};
  float _fraction{};

  void update() {
    const uint8_t velocity = _aftertouch > 0 ? _aftertouch : _velocity;
    _fraction              = adjustVolume((float)velocity / 127.f);
  }

  float adjustVolume(float fraction) {
    if (_volume < 100) {
      const float range = (float)_volume / 100.f;
      return fraction * range;
    }

    const float range = (float)(_volume - 100) / 27.f;
    return powf(fraction, 1 - (0.5f * range));
  }
} Velocity;

// Track the initialization of the bow pressure, the finger pressure, and position.
static class Home {
public:
  operator bool() const {
    return _bow && _finger;
  }

  void reset() {
    _bow    = false;
    _finger = false;
  }

  bool isBow() const {
    return _bow;
  }

  void setBow(bool ready) {
    _bow = ready;

    if (_bow && _finger)
      handler();
  }

  bool isFinger() const {
    return _finger;
  }

  void setFinger(bool ready) {
    _finger = ready;

    if (_bow && _finger)
      handler();
  }

private:
  bool _bow{};
  bool _finger{};

  void handler();
} Home;

static class {
public:
  float pressureMax{1};
  float rotationMax{1};
  bool reverse{};
  bool turn{};
  bool hold{};

  void stop() {
    pressureMax = 1;
    rotationMax = 1;
    reverse     = false;
    turn        = false;
    hold        = false;

    Steppers[Stepper::Bow].stop();
    if (Home)
      Steppers[Stepper::BowPressure].position(0);
  }

  void reset() {
    stop();
  }

  void update() {
    if (!Home.isBow())
      return;

    if (turn) {
      Velocity.reset();
      Steppers[Stepper::Bow].rotate(reverse ? -1.f : 1.f);
      return;
    }

    if (!Velocity) {
      Steppers[Stepper::Bow].stop();
      Steppers[Stepper::BowPressure].position(0);
      return;
    }

    if (hold) {
      Steppers[Stepper::Bow].stop();
      return;
    }

    const float speedRange    = 0.2f + (Velocity.getFraction() * 0.8f);
    const float speedAdjusted = powf(speedRange, 1.5);
    const float speed         = speedAdjusted * rotationMax;
    Steppers[Stepper::Bow].rotate(speed * (reverse ? -1.f : 1.f));

    const float pressureRange = Config.bow.max - Config.bow.min;
    float pressure            = Config.bow.min + (Velocity.getFraction() * pressureRange * pressureMax);

    // Limit the bow pressure to the fraction of the current speed target; avoid getting a
    // still too slow moving bow stuck against the string.
    const float pressureLimit = Steppers[Stepper::Bow].getSpeedTarget() / Steppers[Stepper::Bow].getSpeed();
    if (pressureLimit < 0.9f)
      pressure *= pressureLimit;

    Steppers[Stepper::BowPressure].position(pressure / 8.f * 200.f, 0.5);
  }

  void home() {
    Home.setBow(false);
    Steppers[Stepper::BowPressure].home(1000, 8 + Config.bow.home / 8.f * 200.f, []() { Home.setBow(true); });
    Steppers[Stepper::BowPressure].hold();
  }
} Bow;

static class {
public:
  uint8_t noteIndex{};
  float pitchbend{};
  struct {
    float rate{};
    float depth{0.5};
  } vibrato;
  float speedMax{1};
  float pressureMax{1};
  bool hold{};

  void stop() {
    noteIndex     = 0;
    pitchbend     = 0;
    vibrato.rate  = 0;
    vibrato.depth = 0.5;
    _vibrato      = {};
    speedMax      = 1;
    pressureMax   = 1;
    hold          = false;
    Steppers[Stepper::Finger].stop();
    release();
  }

  void reset() {
    stop();
    Home.setFinger(false);
  }

  void loop() {
    if (Velocity.getVelocity() == 0)
      return;

    if (noteIndex == 0)
      return;

    if (vibrato.rate <= 0)
      return;

    // A typical string vibrato is 5-8 Hz, 0.2-0.4 semitones.
    const float hz = 5 + (3 * vibrato.rate);
    if (V2Base::getUsecSince(_vibrato.usec) < (1000.f * 1000.f) / hz)
      return;

    _vibrato.high = !_vibrato.high;
    _vibrato.usec = V2Base::getUsec();
    update();
  }

  void touch() {
    if (!Home.isFinger())
      return;

    Steppers[Stepper::FingerPressure].position(60.f * (1.f - pressureMax));
  }

  void release() {
    if (!Home.isFinger())
      return;

    Steppers[Stepper::FingerPressure].position(60);
  }

  void update() {
    if (!Home.isFinger())
      return;

    // The base note is the open string.
    if (noteIndex > 0) {
      float steps         = getNotePosition(noteIndex);
      float stepsTwoNotes = 0;
      if (pitchbend < 0) {
        const uint8_t twoNotes = max((int8_t)noteIndex - 2, 0);
        stepsTwoNotes          = steps - getNotePosition(twoNotes);

      } else {
        const uint8_t twoNotes = min(noteIndex + 2, Config.notes.count - 1);
        stepsTwoNotes          = getNotePosition(twoNotes) - steps;
      }
      steps += stepsTwoNotes * pitchbend;
      _target = steps;

      // A typical string vibrato is 5-8 Hz, 0.2-0.4 semitones.
      if (vibrato.rate > 0) {
        const float oneNote  = getNotePosition(noteIndex + 1) - getNotePosition(noteIndex);
        const float fraction = 0.01f + (0.2f * powf(vibrato.depth, 1.5));
        steps += oneNote * fraction * (_vibrato.high ? 1.f : -1.f);
      }

      {
        // Adjust the pitch depending on the velocity. The increased bow pressure of higher velocities
        // result in higher pitches, because the tension of the string increases.
        //
        // Velocity ~80 (0.6) is the center / tuned velocity, because the open string cannot be corrected.
        // 0   -0.2
        // 0.4 -0.15
        // 0.6  0
        // 0.8  0.8
        // 1    1
        const float adjustVelocity = (1.3f * powf(Velocity.getFraction(), 3.5)) - 0.3f;

        // The adjustment is between 30 and 55 cent, depending on the pitch / actual string length.
        const float notePositionFraction = (float)noteIndex / (notesMax - 1);
        const float adjustPitchFraction  = 0.3f + (0.25f * (1.f - notePositionFraction));

        const float oneNoteSteps = getNotePosition(noteIndex + 1) - getNotePosition(noteIndex);
        steps -= adjustVelocity * adjustPitchFraction * oneNoteSteps;
      }

      Steppers[Stepper::Finger].position(steps, speedMax);
      if (!hold)
        touch();

    } else
      release();
  }

  bool inPosition() const {
    if (noteIndex == 0)
      return true;

    const float distance = Steppers[Stepper::Finger].getPosition() - _target;
    return fabs(distance) < 400.f;
  }

  void home() {
    Home.setFinger(false);

    static const auto fingerRelease = []() {
      // Move past the detected home position to increase the finger pressure when positioning to 0.
      Steppers[Stepper::FingerPressure].initializePosition(Config.finger.pressure);
      Steppers[Stepper::FingerPressure].position(60);

      Home.setFinger(true);
    };

    static const auto fingerHome = []() { Steppers[Stepper::FingerPressure].home(200, 0, fingerRelease); };

    // Move a few stepes before calling home(). We do not move any steps back after
    // the stall detection in home(), from this position we cannot reliably detect a
    // stall again.
    static const auto fingerBack = []() { Steppers[Stepper::FingerPressure].position(32, 0.5, fingerHome); };

    // Setup the finger after the rail has moved home; to avoid bending the screw
    // while the finger is in the middle of it.
    Steppers[Stepper::Finger].home(10000, Config.string.home / 8.f * 200.f, fingerBack);

    Steppers[Stepper::Finger].hold();
    Steppers[Stepper::FingerPressure].hold();

    noteIndex = 0;
    _target   = 0;
  }

private:
  float _target{};
  struct {
    unsigned long usec{};
    bool high;
  } _vibrato{};

  float getNotePosition(uint8_t index) {
    // The number of steps to shorten the string by, to play the n-th note above the
    // base note. The first note is played with the open string. The 'length' is the
    // overall string length in meters. This string length is 57 cm, one turn is 8 mm.
    const float distance = V2Music::String::getNoteDistance(index, Config.string.length);
    const float turns    = (distance - V2Music::String::getNoteDistance(1, Config.string.length)) / 8.f;
    return turns * 200.f;
  }
} Finger;

void Home::handler() {
  Bow.update();
  Finger.update();
};

static class Device : public V2Device {
public:
  constexpr Device() : V2Device() {
    metadata.vendor      = "Versio Duo";
    metadata.product     = "V2 viola-1";
    metadata.description = "1 String Circular Bow";
    metadata.home        = "https://versioduo.com/#viola-1";

    system.download  = "https://versioduo.com/download";
    system.configure = "https://versioduo.com/configure";

    // https://github.com/versioduo/arduino-board-package/blob/main/boards.txt
    usb.pid            = 0xe9a0;
    usb.ports.standard = 8;

    configuration = {.size{sizeof(Config)}, .data{&Config}};
  }

  enum class CC {
    Volume         = V2MIDI::CC::ChannelVolume,
    FingerSpeed    = V2MIDI::CC::Controller3,
    FingerPressure = V2MIDI::CC::Controller9,
    VibratoRate    = V2MIDI::CC::SoundController7,
    VibratoDepth   = V2MIDI::CC::SoundController8,
    BowSpeed       = V2MIDI::CC::ModulationWheel,
    BowPressure    = V2MIDI::CC::SoundController5,
    BowReverse     = V2MIDI::CC::Controller14,
    BowTurn        = V2MIDI::CC::Controller15,
  };

  void allNotesOff(bool home = false) {
    Manual.setMode();
    _playing.reset();
    Velocity.reset();

    if (!power())
      return;

    Bow.stop();
    Finger.stop();

    if (!Home || home || _force.trigger()) {
      Bow.home();
      Finger.home();
    }

    _timeoutUsec = V2Base::getUsec();
  }

  void play(uint8_t note, uint8_t velocity) {
    if (note < Config.notes.start || note >= Config.notes.start + Config.notes.count)
      return;

    led.flash(0.03, 0.3);

    if (!power())
      return;

    if (!Home)
      allNotesOff();

    _playing.update(note, velocity);

    // Restore previous note.
    if (velocity == 0) {
      uint8_t n;
      uint8_t v;
      if (_playing.getLast(n, v)) {
        note     = n;
        velocity = v;
      }
    }

    Velocity.setVelocity(velocity);

    if (velocity > 0) {
      Finger.noteIndex = note - Config.notes.start;
      Finger.update();

      // If we are not currently playing a note, delay the bow movement until we positioned the finger.
      if (!Velocity && !Finger.inPosition()) {
        Bow.hold    = true;
        Finger.hold = true;
        Finger.release();
      }
    }

    Velocity.setVelocity(velocity);
    Bow.update();
  }

  // Velocity 80 for tuning, the open string cannot be pitch corrected.
  void tune(uint8_t note) {
    allNotesOff();
    play(note, 80);
  }

  // Turn bow to apply rosin.
  void turn() {
    allNotesOff();
    Bow.turn = true;
    Bow.update();
  }

private:
  unsigned long _timeoutUsec{};
  V2Music::ForcedStop _force;
  V2Music::Playing<notesMax> _playing;

  void handleLoop() override {
    if (_timeoutUsec > 0 && V2Base::getUsecSince(_timeoutUsec) > 900 * 1000 * 1000)
      reset();

    if (Bow.hold && Finger.inPosition()) {
      Bow.hold    = false;
      Finger.hold = false;
      Bow.update();
      Finger.update();
    }

    Finger.loop();
  }

  void handleReset() override {
    _timeoutUsec = 0;
    _force.reset();
    _playing.reset();
    Manual.setMode();
    Home.reset();
    Bow.reset();
    Finger.reset();
    Power.off();

    for (uint8_t i = 0; i < nSteppers; i++)
      Steppers[i].reset();
  }

  bool power() {
    bool continuous;

    if (!Power.on(continuous))
      return false;

    if (!continuous) {
      for (uint8_t i = 0; i < nSteppers; i++)
        Steppers[i].reset();
    }

    return true;
  }

  void handleNote(uint8_t channel, uint8_t note, uint8_t velocity) override {
    _timeoutUsec = V2Base::getUsec();

    play(note, velocity);
  }

  void handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) override {
    _timeoutUsec = V2Base::getUsec();

    play(note, 0);
  }

  void handleAftertouchChannel(uint8_t channel, uint8_t pressure) override {
    _timeoutUsec = V2Base::getUsec();

    Velocity.setAftertouch(pressure);
    Bow.update();
    Finger.update();
  }

  void handleAftertouch(uint8_t channel, uint8_t note, uint8_t pressure) override {
    if (note - Config.notes.start != Finger.noteIndex)
      return;

    handleAftertouchChannel(channel, pressure);
  }

  void handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) override {
    if (channel != 0)
      return;

    _timeoutUsec = V2Base::getUsec();

    switch (controller) {
      case (uint8_t)CC::Volume:
        Velocity.setVolume(value);
        Bow.update();
        break;

      case (uint8_t)CC::FingerSpeed:
        Finger.speedMax = (float)(value + 1) / 128.f;
        Finger.update();
        break;

      case (uint8_t)CC::FingerPressure:
        Finger.pressureMax = (float)(value + 1) / 128.f;
        Finger.update();
        break;

      case (uint8_t)CC::VibratoRate:
        Finger.vibrato.rate = (float)value / 127.f;
        Finger.update();
        break;

      case (uint8_t)CC::VibratoDepth:
        Finger.vibrato.depth = (float)value / 127.f;
        Finger.update();
        break;

      case (uint8_t)CC::BowSpeed:
        Bow.rotationMax = (float)(value + 1) / 128.f;
        Bow.update();
        break;

      case (uint8_t)CC::BowPressure:
        Bow.pressureMax = (float)(value + 1) / 128.f;
        Bow.update();
        break;

      case (uint8_t)CC::BowReverse:
        Bow.reverse = value > 63;
        Bow.update();
        break;

      case (uint8_t)CC::BowTurn:
        if (value > 63)
          turn();
        else
          allNotesOff();
        break;

      case V2MIDI::CC::AllSoundOff:
      case V2MIDI::CC::AllNotesOff:
        allNotesOff();
        break;
    }
  }

  void handlePitchBend(uint8_t channel, int16_t value) override {
    _timeoutUsec = V2Base::getUsec();

    Finger.pitchbend = (float)value / (value < 0 ? 8192.f : 8191.f);
    Finger.update();
  }

  void handleSystemReset() override {
    reset();
  }

  void exportInput(JsonObject json) override {
    {
      JsonObject jsonPitchbend = json.createNestedObject("pitchbend");
      jsonPitchbend["value"]   = (int16_t)(Finger.pitchbend * (Finger.pitchbend < 0 ? 8192.f : 8191.f));
    }

    {
      JsonObject jsonChromatic = json.createNestedObject("chromatic");
      jsonChromatic["start"]   = Config.notes.start;
      jsonChromatic["count"]   = Config.notes.count;
    }

    {
      JsonObject jsonAftertouch = json.createNestedObject("aftertouch");
      jsonAftertouch["value"]   = Velocity.getAftertouch();
    }

    JsonArray jsonControllers = json.createNestedArray("controllers");
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Volume";
      jsonController["number"]  = (uint8_t)CC::Volume;
      jsonController["value"]   = Velocity.getVolume();
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Finger Speed";
      jsonController["number"]  = (uint8_t)CC::FingerSpeed;
      jsonController["value"]   = (uint8_t)(Finger.speedMax * 127.f);
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Finger Pressure";
      jsonController["number"]  = (uint8_t)CC::FingerPressure;
      jsonController["value"]   = (uint8_t)(Finger.pressureMax * 127.f);
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Vibrato Rate";
      jsonController["number"]  = (uint8_t)CC::VibratoRate;
      jsonController["value"]   = (uint8_t)(Finger.vibrato.rate * 127.f);
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Vibrato Depth";
      jsonController["number"]  = (uint8_t)CC::VibratoDepth;
      jsonController["value"]   = (uint8_t)(Finger.vibrato.depth * 127.f);
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Bow Speed";
      jsonController["number"]  = (uint8_t)CC::BowSpeed;
      jsonController["value"]   = (uint8_t)(Bow.rotationMax * 127.f);
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Bow Pressure";
      jsonController["number"]  = (uint8_t)CC::BowPressure;
      jsonController["value"]   = (uint8_t)(Bow.pressureMax * 127.f);
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Bow Reverse";
      jsonController["type"]    = "toggle";
      jsonController["number"]  = (uint8_t)CC::BowReverse;
      jsonController["value"]   = (uint8_t)(Bow.reverse ? 127 : 0);
    }
    {
      JsonObject jsonController = jsonControllers.createNestedObject();
      jsonController["name"]    = "Bow Turn";
      jsonController["type"]    = "toggle";
      jsonController["number"]  = (uint8_t)CC::BowTurn;
      jsonController["value"]   = (uint8_t)(Bow.turn ? 127 : 0);
    }
  }

  void exportSettings(JsonArray json) override {
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "note";
      setting["title"]   = "Notes";
      setting["label"]   = "Start";
      setting["default"] = ConfigurationDefault.notes.start;
      setting["path"]    = "notes/start";
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";
      setting["label"]   = "Count";
      setting["min"]     = 1;
      setting["max"]     = notesMax;
      setting["default"] = ConfigurationDefault.notes.count;
      setting["path"]    = "notes/count";
    }

    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";

      setting["title"]   = "Bow";
      setting["label"]   = "Home";
      setting["min"]     = 0;
      setting["max"]     = 50;
      setting["step"]    = 0.1;
      setting["default"] = ConfigurationDefault.bow.home;
      setting["path"]    = "bow/home";
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";

      setting["label"]   = "Minimum";
      setting["min"]     = 0;
      setting["max"]     = 50;
      setting["step"]    = 0.1;
      setting["default"] = ConfigurationDefault.bow.min;
      setting["path"]    = "bow/min";
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";

      setting["label"]   = "Maximum";
      setting["min"]     = 0;
      setting["max"]     = 50;
      setting["step"]    = 0.1;
      setting["default"] = ConfigurationDefault.bow.max;
      setting["path"]    = "bow/max";
    }

    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";

      setting["title"]   = "String";
      setting["label"]   = "Length";
      setting["min"]     = 1;
      setting["max"]     = 2000;
      setting["step"]    = 0.1;
      setting["default"] = ConfigurationDefault.string.length;
      setting["path"]    = "string/length";
    }
    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";

      setting["label"]   = "Home";
      setting["min"]     = 0;
      setting["max"]     = 50;
      setting["step"]    = 0.1;
      setting["default"] = ConfigurationDefault.string.home;
      setting["path"]    = "string/home";
    }

    {
      JsonObject setting = json.createNestedObject();
      setting["type"]    = "number";

      setting["title"]   = "Finger";
      setting["label"]   = "Pressure";
      setting["max"]     = 20;
      setting["default"] = ConfigurationDefault.finger.pressure;
      setting["path"]    = "finger/pressure";
    }
  }

  void exportConfiguration(JsonObject json) override {
    JsonObject jsonNotes = json.createNestedObject("notes");
    jsonNotes["#start"]  = "First note";
    jsonNotes["start"]   = Config.notes.start;
    jsonNotes["#count"]  = "Total number of notes ";
    jsonNotes["count"]   = Config.notes.count;

    JsonObject jsonBow = json.createNestedObject("bow");
    jsonBow["#home"]   = "Offset of home position in millimeters";
    jsonBow["home"]    = serialized(String(Config.bow.home, 1));
    jsonBow["#min"]    = "Offset for velocity 1 in millimeters";
    jsonBow["min"]     = serialized(String(Config.bow.min, 1));
    jsonBow["#max"]    = "Offset for velocity 127 in millimeters";
    jsonBow["max"]     = serialized(String(Config.bow.max, 1));

    JsonObject jsonString = json.createNestedObject("string");
    jsonString["#length"] = "Total string length in millimeters";
    jsonString["length"]  = serialized(String(Config.string.length, 1));
    jsonString["#home"]   = "Offset of first note in millimeters";
    jsonString["home"]    = serialized(String(Config.string.home, 1));

    JsonObject jsonFinger   = json.createNestedObject("finger");
    jsonFinger["#pressure"] = "Additional steps to increase the finger pressure";
    jsonFinger["pressure"]  = Config.finger.pressure;
  }

  void importConfiguration(JsonObject json) override {
    JsonObject jsonNotes = json["notes"];
    if (jsonNotes) {
      if (!jsonNotes["start"].isNull()) {
        uint8_t start = jsonNotes["start"];
        if (start > 127)
          start = 127;

        Config.notes.start = start;
      }

      if (!jsonNotes["count"].isNull()) {
        uint8_t count = jsonNotes["count"];
        if (count < 1)
          count = 1;

        else if (count > notesMax)
          count = notesMax;

        if (count > 128 - Config.notes.start)
          count = 128 - Config.notes.start;

        Config.notes.count = count;
      }
    }

    JsonObject jsonBow = json["bow"];
    if (jsonBow) {
      if (!jsonBow["home"].isNull()) {
        float home = jsonBow["home"];
        if (home < 0.f)
          home = 0;

        else if (home > 50.f)
          home = 50;

        Config.bow.home = home;
      }

      if (!jsonBow["min"].isNull()) {
        float min = jsonBow["min"];
        if (min < 0.f)
          min = 0;

        else if (min > 50.f)
          min = 50;

        Config.bow.min = min;
      }

      if (!jsonBow["max"].isNull()) {
        float max = jsonBow["max"];
        if (max < 0.f)
          max = 0;

        else if (max > 50.f)
          max = 50;

        Config.bow.max = max;
      }
    }

    JsonObject jsonString = json["string"];
    if (jsonString) {
      if (!jsonString["length"].isNull()) {
        float length = jsonString["length"];
        if (length < 1.f)
          length = 1;

        else if (length > 2000.f)
          length = 2000;

        Config.string.length = length;
      }

      if (!jsonString["home"].isNull()) {
        float home = jsonString["home"];
        if (home < 0.f)
          home = 0;

        else if (home > 50.f)
          home = 50;

        Config.string.home = home;
      }
    }

    JsonObject jsonFinger = json["finger"];
    if (jsonFinger) {
      if (!jsonFinger["pressure"].isNull()) {
        uint8_t pressure = jsonFinger["pressure"];
        if (pressure > 20)
          pressure = 20;

        Config.finger.pressure = pressure;
      }
    }
  }

  virtual void exportSystemMIDIFile(JsonObject json);

  void exportSystem(JsonObject json) override {
    {
      JsonObject jsonPower       = json.createNestedObject("power");
      jsonPower["voltage"]       = serialized(String(Power.getVoltage(), 1));
      jsonPower["interruptions"] = Power.getInterruptions();
    }
    {
      JsonObject jsonBow        = json.createNestedObject("bow");
      JsonObject jsonPressure   = jsonBow.createNestedObject("pressure");
      jsonPressure["posititon"] = serialized(String(Steppers[Stepper::BowPressure].getPosition() / 200.f * 8.f, 1));
    }
    {
      JsonObject jsonFinger   = json.createNestedObject("finger");
      jsonFinger["posititon"] = serialized(String(Steppers[Stepper::Finger].getPosition() / 200.f * 8.f, 1));

      JsonObject jsonPressure   = jsonFinger.createNestedObject("pressure");
      jsonPressure["posititon"] = (uint32_t)Steppers[Stepper::FingerPressure].getPosition();
    }

    exportSystemMIDIFile(json);
  }
} Device;

// Dispatch MIDI packets
static class MIDI {
public:
  void loop() {
    if (!Device.usb.midi.receive(&_midi))
      return;

    if (_midi.getPort() == 0) {
      Device.dispatch(&Device.usb.midi, &_midi);

    } else {
      _midi.setPort(_midi.getPort() - 1);
      Socket.send(&_midi);
    }
  }

private:
  V2MIDI::Packet _midi{};
} MIDI;

// Dispatch Link packets
static class Link : public V2Link {
public:
  Link() : V2Link(&Plug, &Socket) {
    Device.link = this;
  }

private:
  V2MIDI::Packet _midi{};

  // Receive a host event from our parent device
  void receivePlug(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      packet->receive(&_midi);
      Device.dispatch(&Plug, &_midi);
    }
  }

  // Forward children device events to the host
  void receiveSocket(V2Link::Packet *packet) override {
    if (packet->getType() == V2Link::Packet::Type::MIDI) {
      uint8_t address = packet->getAddress();
      if (address == 0x0f)
        return;

      if (Device.usb.midi.connected()) {
        packet->receive(&_midi);
        _midi.setPort(address + 1);
        Device.usb.midi.send(&_midi);
      }
    }
  }
} Link;

static class MIDIFile : public V2MIDI::File::Tracks {
public:
  constexpr MIDIFile() : V2MIDI::File::Tracks(MIDISong) {}

private:
  bool handleSend(uint16_t track, V2MIDI::Packet *packet) override {
    switch (track) {
      case 1:
        Device.dispatch(&Device.usb.midi, packet);
        break;

      case 2 ... 8:
        packet->setPort(track - 2);
        Socket.send(packet);
        break;
    }

    return true;
  }

  void handleStateChange(V2MIDI::File::Tracks::State state) override {
    switch (state) {
      case V2MIDI::File::Tracks::State::Stop:
        Device.allNotesOff();
        for (uint8_t i = 0; i < 8; i++) {
          V2MIDI::Packet midi;

          midi.setPort(i);
          Socket.send(midi.setControlChange(0, V2MIDI::CC::AllNotesOff, 0));
        }
        break;
    }
  }
} MIDIFile;

void Device::exportSystemMIDIFile(JsonObject json) {
  JsonObject jsonTrack = json.createNestedObject("track");
  char s[128];
  if (MIDIFile.copyTag(V2MIDI::File::Event::Meta::Title, s, sizeof(s)) > 0)
    jsonTrack["title"] = s;

  if (MIDIFile.copyTag(V2MIDI::File::Event::Meta::Copyright, s, sizeof(s)) > 0)
    jsonTrack["creator"] = s;
}

static class {
public:
  void stop() {
    if (!_enabled)
      Device.allNotesOff(true);

    _enabled = false;
  }

  void play() {
    Device.allNotesOff();
    _enabled  = true;
    _velocity = 20;
    _play     = {};
  }

  void loop() {
    if (!_enabled)
      return;

    playNote();
  }

private:
  void playNote() {
    if (V2Base::getUsecSince(_play.usec) < 1200 * 1000)
      return;

    _play.usec = V2Base::getUsec();

    if (_play.note == 0) {
      _play.note = Config.notes.start;
      Device.play(_play.note, _velocity);

    } else if (_play.note < Config.notes.start + Config.notes.count - 1) {
      Device.play(_play.note, 0);
      _play.note++;
      Device.play(_play.note, _velocity);

    } else {
      Device.play(_play.note, 0);
      _play.note = 0;

      _velocity += 25;
      if (_velocity > 127) {
        Device.reset();
        _enabled = false;
      }

      // Immediately start to move the finger back to the first note.
      Device.play(Config.notes.start + 1, 1);
      Device.play(Config.notes.start + 1, 0);
    }
  }

  bool _enabled{};
  uint8_t _velocity{};
  struct {
    uint8_t note;
    unsigned long usec;
  } _play{};
} TestMode;

static class Button : public V2Buttons::Button {
public:
  constexpr Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

private:
  const V2Buttons::Config _config{.clickUsec{200 * 1000}, .holdUsec{500 * 1000}};

  void handleClick(uint8_t count) override {
    switch (count) {
      case 0:
        MIDIFile.stop();
        TestMode.stop();
        Device.allNotesOff(true);
        break;

      case 1:
        MIDIFile.stop();
        TestMode.stop();
        Device.reset();
        break;
    }
  }

  void handleHold(uint8_t count) override {
    switch (count) {
      case 0:
        Manual.setMode(Manual::Mode::Song);
        MIDIFile.play();
        break;

      case 1:
        Manual.setMode(Manual::Mode::Test);
        TestMode.play();
        break;

      case 2:
        Manual.setMode(Manual::Mode::Tune);
        Device.tune(Config.notes.start);
        break;

      case 3:
        Manual.setMode(Manual::Mode::Tune);
        Device.tune(Config.notes.start + Config.notes.count - 1);
        break;

      case 4:
        Manual.setMode(Manual::Mode::Turn);
        Device.turn();
        break;
    }
  }
} Button;

void setup() {
  Serial.begin(9600);
  SPI.begin();

  LED.begin();
  LED.setMaxBrightness(0.5);

  Link.begin();

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 2);

  Power.begin();
  for (uint8_t i = 0; i < nSteppers; i++)
    Steppers[i].begin();

  // The priority needs to be lower than the SERCOM priorities.
  Timer.begin([]() {
    for (uint8_t i = 0; i < nSteppers; i++)
      Steppers[i].tick();
  });
  Timer.setPriority(3);

  ADC.begin();
  ADC.addChannel(V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE));

  Device.begin();
  Button.begin();
  Device.reset();
}

void loop() {
  for (uint8_t i = 0; i < nSteppers; i++)
    Steppers[i].loop();

  LED.loop();
  MIDI.loop();
  Link.loop();
  V2Buttons::loop();
  Power.loop();
  Device.loop();

  switch (Manual.getMode()) {
    case Manual::Mode::Song:
      MIDIFile.loop();
      break;

    case Manual::Mode::Test:
      TestMode.loop();
      break;
  }

  if (Link.idle() && Device.idle())
    Device.sleep();
}
