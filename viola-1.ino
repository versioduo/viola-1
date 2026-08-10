#include "MIDISong.h"
#include <V2Base.h>
#include <V2Buttons.h>
#include <V2Device.h>
#include <V2LED.h>
#include <V2Link.h>
#include <V2MIDI.h>
#include <V2Music.h>
#include <V2PowerSupply.h>
#include <V2Stepper.h>

V2DEVICE_METADATA("com.versioduo.viola-1", 63, "versioduo:samd:step");

namespace {
  constexpr uint8_t            notesMax{20};
  constexpr uint8_t            nSteppers{4};
  V2LED::WS2812<nSteppers + 2> LED(PIN_LED_WS2812, sercom2, SPI_PAD_0_SCK_1, PIO_SERCOM);
  V2Link::Port                 Plug(&SerialPlug, PIN_SERIAL_PLUG_TX_ENABLE);
  V2Link::Port                 Socket(&SerialSocket, PIN_SERIAL_SOCKET_TX_ENABLE);
  V2Base::Timer::Periodic      Timer(2, 200000);
  V2Base::Analog::ADC          ADC(V2Base::Analog::ADC::getID(PIN_VOLTAGE_SENSE));

  // The button switches the state with a multi-click long-press.
  class Manual {
  public:
    enum class Mode { Notes, Song, Test, Tune, Turn } mode{};
    Mode getMode() const {
      return _mode;
    }

    auto setMode(Mode mode = Mode::Notes) {
      _mode = mode;

      switch (_mode) {
        case Mode::Notes:
          LED.reset();
          LED.hsv({V2Colour::Orange, 1, 0.25}, nSteppers + 0, 2);
          break;

        case Mode::Song:
          LED.reset();
          LED.brightness(0.25, nSteppers + 0, 2);
          break;

        case Mode::Test:
          LED.reset();
          LED.rainbow(1, 3, 0.4);
          break;

        case Mode::Tune:
          LED.reset();
          LED.hsv({V2Colour::Magenta, 1, 0.25}, nSteppers + 0, 2);
          break;

        case Mode::Turn:
          LED.reset();
          LED.hsv({V2Colour::Cyan, 1, 0.25}, nSteppers + 0, 2);
          break;
      }
    }

  private:
    Mode _mode{};
  } Manual;

  class Stepper : public V2Stepper::Motor {
  public:
    enum { Bow, Pressure, Finger, FingerPressure };

    constexpr Stepper(const Motor::Config conf, uint8_t index) :
      Motor(conf, &Timer, &SPI, PIN_DRIVER_SELECT + index, PIN_DRIVER_STEP + index),
      _index(index) {}

  private:
    const uint8_t _index;

    auto handleMovement(Move move) -> void override {
      switch (move) {
        case Move::Forward:
          LED.hsv({V2Colour::Cyan, 1, 0.4}, _index);
          break;

        case Move::Reverse:
          LED.hsv({V2Colour::Orange, 1, 0.4}, _index);
          break;

        case Move::Stop:
          LED.hsv({V2Colour::Green, 1, 0.15}, _index);
          break;
      }
    }
  } Steppers[nSteppers]{
    Stepper(
      {
        .ampere{0.8},
        .microstepsShift{4},
        .speed{.min{20}, .max{700}, .accel{1000}},
      },
      Stepper::Bow),
    Stepper(
      {
        .ampere{0.7},
        .microstepsShift{4},
        .inverse{true},
        .home{.speed{500}, .stall{0.04}},
        .speed{.min{5}, .max{3000}, .accel{6000}},
      },
      Stepper::Pressure),
    Stepper(
      {
        .ampere{0.7},
        .microstepsShift{4},
        .home{.speed{750}, .stall{0.04}},
        .speed{.min{50}, .max{2400}, .accel{8000}},
      },
      Stepper::Finger),
    Stepper(
      {
        .ampere{0.7},
        .microstepsShift{4},
        .inverse{true},
        .home{.speed{200}, .stall{0.07}},
        .speed{.min{50}, .max{150}, .accel{16000}},
      },
      Stepper::FingerPressure),
  };

  class Power : public V2PowerSupply {
  public:
    constexpr Power() : V2PowerSupply({.min{12}, .max{30}}) {}

    auto begin() {
      pinMode(PIN_DRIVER_ENABLE, OUTPUT);
      digitalWrite(PIN_DRIVER_ENABLE, HIGH);
    }

  private:
    auto handleMeasurement() -> float override {
      // A voltage 10/100k divider.
      return 36.f * ADC.readChannel(V2Base::Analog::ADC::getChannel(PIN_VOLTAGE_SENSE));
    }

    auto handleOn() -> void override {
      digitalWrite(PIN_DRIVER_ENABLE, LOW);
    }

    auto handleOff() -> void override {
      digitalWrite(PIN_DRIVER_ENABLE, HIGH);
    }

    auto handleNotify(float voltage) -> void override {
      // Power interruption, or commands without a power connection show yellow LEDs.
      if (voltage < config.min) {
        LED.flash({V2Colour::Yellow, 1, 0.5}, 0.5);
        return;
      }

      // Over-voltage shows red LEDs.
      if (voltage > config.max) {
        LED.flash({V2Colour::Red, 1, 1}, 0.5);
        return;
      }

      // The number of green LEDs shows the voltage.
      float   fraction = voltage / (float)config.max;
      uint8_t n        = ceil((float)nSteppers * fraction);
      LED.flash({V2Colour::Green, 1, 0.5}, 0.5, 0, n);
    }
  } Power;

  // Config, written to EEPROM.
  constexpr struct Configuration {
    struct {
      // The middle C, MIDI note 60, in this mapping is C3.
      uint8_t start{V2MIDI::C(2)};
      uint8_t count{19};
    } notes;

    struct {
      // Offsets in millimeters.
      float home{6};
      float min{8};
      float max{16};
    } bow;

    struct {
      // Overall string length in millimeters.
      float length{587};

      // Offset in millimeters from the home position to the first note.
      float home{1};
    } string;

    struct {
      // Additional steps of finger pressure past the detected home postion.
      uint8_t pressure{8};
    } finger;
  } ConfigurationDefault;

  auto Config{ConfigurationDefault};

  // Calculate the effective velocity depending on the note velocity, aftertouch /
  // pressure, and the volume controller.
  class {
  public:
    operator bool() const {
      return _velocity > 0;
    }

    auto reset() {
      _volume     = 100;
      _velocity   = 0;
      _aftertouch = 0;
      _fraction   = 0;
    }

    auto fraction() -> float const {
      return _fraction;
    }

    auto set(uint8_t velocity) {
      _velocity   = velocity;
      _aftertouch = 0;
      update();
    }

    auto aftertouch() -> uint8_t const {
      return _aftertouch;
    }

    auto setAftertouch(uint8_t pressure) -> void {
      if (_velocity == 0)
        return;

      _aftertouch = pressure;
      update();
    }

    auto volume() -> uint8_t const {
      return _volume;
    }

    auto setVolume(uint8_t volume) {
      _volume = volume;
      update();
    }

  private:
    uint8_t _volume{100};
    uint8_t _velocity{};
    uint8_t _aftertouch{};
    float   _fraction{};

    auto update() -> void {
      uint8_t velocity{_aftertouch > 0 ? _aftertouch : _velocity};
      _fraction = adjustVolume(float(velocity) / 127.f);
    }

    auto adjustVolume(float fraction) -> float {
      if (_volume < 100) {
        float range{float(_volume) / 100.f};
        return fraction * range;
      }

      float range{float(_volume - 100) / 27.f};
      return powf(fraction, 1 - (0.5f * range));
    }
  } Velocity;

  // Track the initialization of the bow pressure, the finger pressure, and position.
  class Home {
  public:
    operator bool() const {
      return _bow && _finger;
    }

    auto reset() {
      _bow    = false;
      _finger = false;
    }

    auto isBow() -> bool const {
      return _bow;
    }

    auto setBow(bool ready) {
      _bow = ready;

      if (_bow && _finger)
        handler();
    }

    auto isFinger() -> bool const {
      return _finger;
    }

    auto setFinger(bool ready) {
      _finger = ready;

      if (_bow && _finger)
        handler();
    }

  private:
    bool _bow{};
    bool _finger{};

    void handler();
  } Home;

  class {
  public:
    float release{0.5};
    float pressureMax{1};
    float pressureSpeedMax{1};
    float rotationMax{1};
    bool  turn{};
    bool  reverse{};
    bool  hold{};

    auto stop() {
      release          = 0.5;
      pressureMax      = 1;
      pressureSpeedMax = 1;
      rotationMax      = 1;
      turn             = false;
      reverse          = false;
      hold             = false;

      Steppers[Stepper::Bow].stop();
      if (Home)
        Steppers[Stepper::Pressure].setPosition(0);
    }

    auto reset() {
      _usec  = 0;
      _speed = 0;
      stop();
    }

    auto update() {
      if (!Home.isBow())
        return;

      if (turn) {
        Velocity.reset();
        Steppers[Stepper::Bow].rotate(reverse ? -1.f : 1.f);
        return;
      }

      if (!Velocity) {
        _usec = V2Base::getUsec();
        Steppers[Stepper::Pressure].setPosition(0, pressureSpeedMax);
        return;

      } else {
        _usec = 0;
      }

      if (hold) {
        Steppers[Stepper::Bow].stop();
        return;
      }

      {
        float speedRange{0.3f + (Velocity.fraction() * 0.7f)};
        float speedAdjusted{powf(speedRange, 1.5)};
        _speed = speedAdjusted * rotationMax;
        Steppers[Stepper::Bow].rotate(_speed * (reverse ? -1.f : 1.f));
      }

      float pressureRange{Config.bow.max - Config.bow.min};
      float pressure{Config.bow.min + (Velocity.fraction() * pressureRange * pressureMax)};

      // Limit the bow pressure to the fraction of the current speed target; avoid getting a
      // still too slow moving bow stuck against the string.
      float pressureLimit{Steppers[Stepper::Bow].getSpeedTarget() / Steppers[Stepper::Bow].getSpeed()};
      if (pressureLimit < 0.9f)
        pressure *= pressureLimit;

      Steppers[Stepper::Pressure].setPosition(pressure / 8.f * 200.f, 0.5f * pressureSpeedMax);
    }

    auto loop() {
      if (_usec == 0 || V2Base::getUsecSince(_usec) < 40.f * 1000.f)
        return;

      _usec = V2Base::getUsec();

      float adjust{0.45f + (0.5f * release)};
      _speed *= adjust;
      if (_speed < 0.01f) {
        _usec  = 0;
        _speed = 0;
      }

      Steppers[Stepper::Bow].rotate(_speed * (reverse ? -1.f : 1.f));
    }

    auto home() {
      Home.setBow(false);
      Steppers[Stepper::Bow].freewheel();
      Steppers[Stepper::Pressure].home(1000, 8 + (Config.bow.home / 8.f * 200.f), []() { Home.setBow(true); });
      Steppers[Stepper::Pressure].hold();
    }

  private:
    float    _speed{};
    uint32_t _usec{};
  } Bow;

  class {
  public:
    uint8_t noteIndex{};
    float   pitchbend{};
    struct {
      float rate{};
      float depth{0.5};
    } vibrato;
    float speedMax{1};
    float pressureMax{1};
    bool  hold{};

    auto stop() {
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

    auto reset() {
      stop();
      Home.setFinger(false);
    }

    auto loop() {
      if (!Velocity)
        return;

      if (noteIndex == 0)
        return;

      if (vibrato.rate <= 0)
        return;

      // A typical string vibrato is 5-8 Hz, 0.2-0.4 semitones.
      float hz{5.f + (3.f * vibrato.rate)};
      if (V2Base::getUsecSince(_vibrato.usec) < (1000.f * 1000.f) / hz)
        return;

      _vibrato.high = !_vibrato.high;
      _vibrato.usec = V2Base::getUsec();
      update();
    }

    auto touch() {
      if (!Home.isFinger())
        return;

      Steppers[Stepper::FingerPressure].setPosition(60.f * (1.f - pressureMax));
    }

    auto release() -> void {
      if (!Home.isFinger())
        return;

      Steppers[Stepper::FingerPressure].setPosition(60);
    }

    auto update() -> void {
      if (!Home.isFinger())
        return;

      // The base note is the open string.
      if (noteIndex == 0) {
        release();
        return;
      }

      {
        float target{getNotePosition(noteIndex)};
        {
          float targetTwoNotes{};
          if (pitchbend < 0) {
            auto twoNotes{uint8_t(std::max((int8_t)noteIndex - 2, 0))};
            targetTwoNotes = target - getNotePosition(twoNotes);
          } else {
            auto twoNotes{uint8_t(std::min(noteIndex + 2, Config.notes.count - 1))};
            targetTwoNotes = getNotePosition(twoNotes) - target;
          }
          target += targetTwoNotes * pitchbend;
        }
        {
          // Adjust the pitch depending on the velocity. The increased bow pressure of higher velocities
          // result in higher pitches, because the tension of the string increases.
          float adjustVelocity{(1.f * powf(Velocity.fraction(), 2)) - 0.25f};

          // The adjustment is between 10 and 30 cent, depending on the pitch / actual string length.
          float notePositionFraction{float(noteIndex) / (notesMax - 1)};
          float adjustPitchFraction{0.1f + (0.2f * (1.f - notePositionFraction))};

          float oneNoteSteps{getNotePosition(noteIndex + 1) - getNotePosition(noteIndex)};
          target -= adjustVelocity * adjustPitchFraction * oneNoteSteps;
          if (target < 0.f)
            target = 0;
        }

        _target = target;
      }

      if (!hold)
        touch();

      // A typical string vibrato is 5-8 Hz, 0.2-0.4 semitones.
      if (vibrato.rate > 0.f) {
        float oneNoteSteps{getNotePosition(noteIndex + 1) - getNotePosition(noteIndex)};
        float fraction{0.01f + (0.2f * powf(vibrato.depth, 1.5))};
        float delta{oneNoteSteps * fraction * (_vibrato.high ? 1.f : -1.f)};
        float target{_target + delta};
        if (target < 0.f)
          target = 0;

        float distance{std::fabs(Steppers[Stepper::Finger].getPosition() - _target)};
        if (distance / 2.f <= std::fabs(delta)) {
          Steppers[Stepper::Finger].setPosition(target);
          return;
        }
      }

      Steppers[Stepper::Finger].setPosition(_target, speedMax);
    }

    auto inPosition() -> bool const {
      if (noteIndex == 0)
        return true;

      float distance{Steppers[Stepper::Finger].getPosition() - _target};
      return std::fabs(distance) < 400.f;
    }

    auto home() {
      Home.setFinger(false);

      static const auto fingerRelease = []() {
        // Move past the detected home position to increase the finger pressure when positioning to 0.
        Steppers[Stepper::FingerPressure].initializePosition(Config.finger.pressure);
        Steppers[Stepper::FingerPressure].setPosition(60);

        Home.setFinger(true);
      };

      static const auto fingerHome = []() {
        Steppers[Stepper::FingerPressure].home(200, 0, fingerRelease);
      };

      // Move a few stepes before calling home(). We do not move any steps back after
      // the stall detection in home(), from this position we cannot reliably detect a
      // stall again.
      static const auto fingerBack = []() {
        Steppers[Stepper::FingerPressure].setPosition(32, 0.5, fingerHome);
      };

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
      bool          high;
    } _vibrato{};

    auto getNotePosition(uint8_t index) -> float {
      // The number of steps to shorten the string by, to play the n-th note above the
      // base note. The first note is played with the open string. The 'length' is the
      // overall string length in meters. This string length is 57 cm, one turn is 8 mm.
      const float distance = V2Music::String::getNoteDistance(index, Config.string.length);
      const float turns    = (distance - V2Music::String::getNoteDistance(1, Config.string.length)) / 8.f;
      return turns * 200.f;
    }
  } Finger;

  auto Home::handler() -> void {
    Bow.update();
    Finger.update();
  }

  class Device : public V2Device {
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
      usb.ports.standard = 16;

      configuration = {.version{2}, .size{sizeof(Config)}, .data{&Config}};
    }

    enum class CC {
      Volume         = V2MIDI::CC::ChannelVolume,
      VibratoRate    = V2MIDI::CC::SoundController7,
      VibratoDepth   = V2MIDI::CC::SoundController8,
      FingerSpeed    = V2MIDI::CC::Controller3,
      FingerPressure = V2MIDI::CC::Controller9,
      FingerPosition = V2MIDI::CC::Controller85,
      BowSpeed       = V2MIDI::CC::ModulationWheel,
      Pressure       = V2MIDI::CC::SoundController5,
      PressureSpeed  = V2MIDI::CC::SoundController10,
      BowRelease     = V2MIDI::CC::SoundController6,
      Reverse        = V2MIDI::CC::Controller14,
      Turn           = V2MIDI::CC::Controller15,
    };

    auto allNotesOff(bool home = false) {
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

    auto play(uint8_t note, uint8_t velocity) {
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

      Velocity.set(velocity);

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

      Velocity.set(velocity);
      Bow.update();
    }

    // Velocity 80 for tuning, the open string cannot be pitch corrected.
    auto tune(uint8_t note) {
      allNotesOff();
      play(note, 80);
    }

    // Turn bow to apply rosin.
    auto turn() {
      allNotesOff();
      Bow.turn = true;
      Bow.update();
    }

  private:
    unsigned long              _timeoutUsec{};
    V2Music::ForcedStop        _force;
    V2Music::Playing<notesMax> _playing;

    auto handleLoop() -> void override {
      if (_timeoutUsec > 0 && V2Base::getUsecSince(_timeoutUsec) > 900 * 1000 * 1000)
        reset();

      if (Bow.hold && Finger.inPosition()) {
        Bow.hold    = false;
        Finger.hold = false;
        Bow.update();
        Finger.update();
      }

      Bow.loop();
      Finger.loop();
    }

    auto handleReset() -> void override {
      _timeoutUsec = 0;
      _force.reset();
      _playing.reset();
      Manual.setMode();
      Home.reset();
      Bow.reset();
      Finger.reset();
      Power.off();

      for (uint8_t i{}; i < nSteppers; i++)
        Steppers[i].reset();
    }

    auto power() -> bool {
      bool continuous;

      if (!Power.on(continuous))
        return false;

      if (!continuous)
        for (uint8_t i{}; i < nSteppers; i++)
          Steppers[i].reset();

      return true;
    }

    auto handleNote(uint8_t channel, uint8_t note, uint8_t velocity) -> void override {
      _timeoutUsec = V2Base::getUsec();

      play(note, velocity);
    }

    auto handleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) -> void override {
      _timeoutUsec = V2Base::getUsec();

      play(note, 0);
    }

    auto handleAftertouchChannel(uint8_t channel, uint8_t pressure) -> void override {
      _timeoutUsec = V2Base::getUsec();

      Velocity.setAftertouch(pressure);
      Bow.update();
      Finger.update();
    }

    auto handleAftertouch(uint8_t channel, uint8_t note, uint8_t pressure) -> void override {
      if (note - Config.notes.start != Finger.noteIndex)
        return;

      handleAftertouchChannel(channel, pressure);
    }

    auto handleControlChange(uint8_t channel, uint8_t controller, uint8_t value) -> void override {
      if (channel != 0)
        return;

      _timeoutUsec = V2Base::getUsec();

      switch (controller) {
        case uint8_t(CC::Volume):
          Velocity.setVolume(value);
          Bow.update();
          break;

        case uint8_t(CC::VibratoRate):
          Finger.vibrato.rate = (float)value / 127.f;
          Finger.update();
          break;

        case uint8_t(CC::VibratoDepth):
          Finger.vibrato.depth = (float)value / 127.f;
          Finger.update();
          break;

        case uint8_t(CC::FingerSpeed):
          Finger.speedMax = (float)(value + 1) / 128.f;
          Finger.update();
          break;

        case uint8_t(CC::FingerPressure):
          Finger.pressureMax = (float)(value + 1) / 128.f;
          Finger.update();
          break;

        case uint8_t(CC::FingerPosition):
          if (value < Config.notes.start || value >= Config.notes.start + Config.notes.count)
            break;

          Finger.noteIndex = value - Config.notes.start;
          Finger.update();
          break;

        case uint8_t(CC::BowSpeed):
          Bow.rotationMax = (float)(value + 1) / 128.f;
          Bow.update();
          break;

        case uint8_t(CC::Pressure):
          Bow.pressureMax = (float)(value + 1) / 128.f;
          Bow.update();
          break;

        case uint8_t(CC::PressureSpeed):
          Bow.pressureSpeedMax = (float)(value + 1) / 128.f;
          Bow.update();
          break;

        case uint8_t(CC::BowRelease):
          Bow.release = (float)(value + 1) / 128.f;
          break;

        case uint8_t(CC::Reverse):
          Bow.reverse = value > 63;
          Bow.update();
          break;

        case uint8_t(CC::Turn):
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

    auto handlePitchBend(uint8_t channel, int16_t value) -> void override {
      _timeoutUsec = V2Base::getUsec();

      Finger.pitchbend = (float)value / (value < 0 ? 8192.f : 8191.f);
      Finger.update();
    }

    auto handleSystemReset() -> void override {
      reset();
    }

    auto exportInput(JsonObject json) -> void override {
      {
        JsonObject jsonPitchbend = json["pitchbend"].to<JsonObject>();
        jsonPitchbend["value"]   = (int16_t)(Finger.pitchbend * (Finger.pitchbend < 0 ? 8192.f : 8191.f));
      }

      {
        JsonObject jsonChromatic = json["chromatic"].to<JsonObject>();
        jsonChromatic["start"]   = Config.notes.start;
        jsonChromatic["count"]   = Config.notes.count;
      }

      {
        JsonObject jsonAftertouch = json["aftertouch"].to<JsonObject>();
        jsonAftertouch["value"]   = Velocity.aftertouch();
      }

      JsonArray jsonControllers = json["controllers"].to<JsonArray>();
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Volume";
        jsonController["number"]  = uint8_t(CC::Volume);
        jsonController["value"]   = Velocity.volume();
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Vibrato Rate";
        jsonController["number"]  = uint8_t(CC::VibratoRate);
        jsonController["value"]   = (uint8_t)(Finger.vibrato.rate * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Vibrato Depth";
        jsonController["number"]  = uint8_t(CC::VibratoDepth);
        jsonController["value"]   = (uint8_t)(Finger.vibrato.depth * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Pitch Speed";
        jsonController["number"]  = uint8_t(CC::FingerSpeed);
        jsonController["value"]   = uint8_t(Finger.speedMax * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Finger Pressure";
        jsonController["number"]  = uint8_t(CC::FingerPressure);
        jsonController["value"]   = uint8_t(Finger.pressureMax * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Pitch Position";
        jsonController["number"]  = uint8_t(CC::FingerPosition);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Rotation Speed";
        jsonController["number"]  = uint8_t(CC::BowSpeed);
        jsonController["value"]   = uint8_t(Bow.rotationMax * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Pressure";
        jsonController["number"]  = uint8_t(CC::Pressure);
        jsonController["value"]   = uint8_t(Bow.pressureMax * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Pressure Speed";
        jsonController["number"]  = uint8_t(CC::PressureSpeed);
        jsonController["value"]   = uint8_t(Bow.pressureSpeedMax * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Rotation Release";
        jsonController["number"]  = uint8_t(CC::BowRelease);
        jsonController["value"]   = uint8_t(Bow.release * 127.f);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Reverse";
        jsonController["type"]    = "toggle";
        jsonController["number"]  = uint8_t(CC::Reverse);
        jsonController["value"]   = uint8_t(Bow.reverse ? 127 : 0);
      }
      {
        JsonObject jsonController = jsonControllers.add<JsonObject>();
        jsonController["name"]    = "Turn";
        jsonController["type"]    = "toggle";
        jsonController["number"]  = uint8_t(CC::Turn);
        jsonController["value"]   = uint8_t(Bow.turn ? 127 : 0);
      }
    }

    auto exportSettings(JsonArray json) -> void override {
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "title";
        setting["title"]   = "Notes";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "note";
        setting["label"]   = "Note";
        setting["default"] = ConfigurationDefault.notes.start;
        setting["path"]    = "notes/start";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "number";
        setting["label"]   = "Count";
        setting["text"]    = "Semitones";
        setting["min"]     = 1;
        setting["max"]     = notesMax;
        setting["default"] = ConfigurationDefault.notes.count;
        setting["path"]    = "notes/count";
      }

      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "title";
        setting["title"]   = "Bow";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "number";
        setting["label"]   = "Home";
        setting["text"]    = "Millimeter";
        setting["min"]     = 0;
        setting["max"]     = 50;
        setting["step"]    = 0.5;
        setting["default"] = ConfigurationDefault.bow.home;
        setting["path"]    = "bow/home";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "number";
        setting["label"]   = "Minimum";
        setting["text"]    = "Millimeter";
        setting["min"]     = 0;
        setting["max"]     = 50;
        setting["step"]    = 0.5;
        setting["default"] = ConfigurationDefault.bow.min;
        setting["path"]    = "bow/min";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "number";
        setting["label"]   = "Maximum";
        setting["text"]    = "Millimeter";
        setting["min"]     = 0;
        setting["max"]     = 50;
        setting["step"]    = 0.5;
        setting["default"] = ConfigurationDefault.bow.max;
        setting["path"]    = "bow/max";
      }

      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "title";
        setting["title"]   = "String";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "number";

        setting["label"]   = "Length";
        setting["text"]    = "Millimeter";
        setting["min"]     = 1;
        setting["max"]     = 2000;
        setting["default"] = ConfigurationDefault.string.length;
        setting["path"]    = "string/length";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "number";
        setting["label"]   = "Home";
        setting["text"]    = "Millimeter";
        setting["min"]     = 0;
        setting["max"]     = 50;
        setting["step"]    = 0.5;
        setting["default"] = ConfigurationDefault.string.home;
        setting["path"]    = "string/home";
      }

      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "title";
        setting["title"]   = "Finger";
      }
      {
        JsonObject setting = json.add<JsonObject>();
        setting["type"]    = "number";
        setting["label"]   = "Pressure";
        setting["text"]    = "Steps";
        setting["max"]     = 20;
        setting["default"] = ConfigurationDefault.finger.pressure;
        setting["path"]    = "finger/pressure";
      }
    }

    auto exportConfiguration(JsonObject json) -> void override {
      JsonObject jsonNotes = json["notes"].to<JsonObject>();
      jsonNotes["#start"]  = "First note";
      jsonNotes["start"]   = Config.notes.start;
      jsonNotes["#count"]  = "Total number of notes ";
      jsonNotes["count"]   = Config.notes.count;

      JsonObject jsonBow = json["bow"].to<JsonObject>();
      jsonBow["#home"]   = "Offset of home position in millimeters";
      jsonBow["home"]    = serialized(String(Config.bow.home, 1));
      jsonBow["#min"]    = "Offset for velocity 1 in millimeters";
      jsonBow["min"]     = serialized(String(Config.bow.min, 1));
      jsonBow["#max"]    = "Offset for velocity 127 in millimeters";
      jsonBow["max"]     = serialized(String(Config.bow.max, 1));

      JsonObject jsonString = json["string"].to<JsonObject>();
      jsonString["#length"] = "Total string length in millimeters";
      jsonString["length"]  = serialized(String(Config.string.length, 1));
      jsonString["#home"]   = "Offset of first note in millimeters";
      jsonString["home"]    = serialized(String(Config.string.home, 1));

      JsonObject jsonFinger   = json["finger"].to<JsonObject>();
      jsonFinger["#pressure"] = "Additional steps to increase the finger pressure";
      jsonFinger["pressure"]  = Config.finger.pressure;
    }

    auto importConfiguration(JsonObject json) -> void override {
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

    virtual auto exportSystemMIDIFile(JsonObject json) -> void;

    auto exportSystem(JsonObject json) -> void override {
      {
        JsonObject jsonPower       = json["power"].to<JsonObject>();
        jsonPower["voltage"]       = serialized(String(Power.getVoltage(), 1));
        jsonPower["interruptions"] = Power.getInterruptions();
      }
      {
        JsonObject jsonBow = json["bow"].to<JsonObject>();
        {
          auto j{jsonBow["pressure"].to<JsonObject>()};
          j["posititon"] = serialized(String(Steppers[Stepper::Pressure].getPosition() / 200.f * 8.f, 1));
        }
        jsonBow["speed"] = serialized(String(Steppers[Stepper::Pressure].getSpeed()));
      }
      {
        JsonObject jsonFinger   = json["finger"].to<JsonObject>();
        jsonFinger["posititon"] = serialized(String(Steppers[Stepper::Finger].getPosition() / 200.f * 8.f, 1));

        JsonObject jsonPressure   = jsonFinger["pressure"].to<JsonObject>();
        jsonPressure["posititon"] = (uint32_t)Steppers[Stepper::FingerPressure].getPosition();
      }

      exportSystemMIDIFile(json);
    }
  } Device;

  // Dispatch MIDI packets
  class MIDI {
  public:
    auto loop() {
      if (!Device.usb.midi.receive(_midi))
        return;

      if (_midi.port == 0) {
        Device.dispatch(&Device.usb.midi, &_midi);

      } else {
        V2Link::Packet p(_midi.port - 1, _midi);
        p.midi.port = 0;
        Socket.send(p);
      }
    }

  private:
    V2MIDI::Packet _midi{};
  } MIDI;

  // Dispatch Link packets
  class Link : public V2Link {
  public:
    Link() : V2Link(&Plug, &Socket) {
      Device.link = this;
    }

  private:
    // Receive a host event from our parent device
    auto receivePlug(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI)
        Device.dispatch(&Plug, &p.midi);
    }

    // Forward children device events to the host
    auto receiveSocket(V2Link::Packet& p) -> void override {
      if (p.type == V2Link::Packet::Type::MIDI) {
        p.midi.port = p.address;
        Device.usb.midi.send(p.midi);
      }
    }
  } Link;

  class MIDIFile : public V2MIDI::File::Tracks {
  public:
    constexpr MIDIFile() : V2MIDI::File::Tracks(MIDISong) {}

  private:
    auto handleSend(uint16_t track, V2MIDI::Packet* midi) -> bool override {
      switch (track) {
        case 1:
          Device.dispatch(&Device.usb.midi, midi);
          break;

        case 2 ... 8: {
          V2Link::Packet p(track - 2, *midi);
          Socket.send(p);
          break;
        }
      }

      return true;
    }

    auto handleStateChange(V2MIDI::File::Tracks::State state) -> void override {
      switch (state) {
        case V2MIDI::File::Tracks::State::Stop:
          Device.allNotesOff();
          for (uint8_t i = 0; i < 8; i++) {
            V2MIDI::Packet midi;
            midi.setControlChange(0, V2MIDI::CC::AllNotesOff, 0);
            Socket.send(V2Link::Packet(i, midi));
          }
          break;
      }
    }
  } MIDIFile;

  auto Device::exportSystemMIDIFile(JsonObject json) -> void {
    JsonObject jsonTrack{json["track"].to<JsonObject>()};
    char       s[128];
    if (MIDIFile.copyTag(V2MIDI::File::Event::Meta::Title, s, sizeof(s)) > 0)
      jsonTrack["title"] = s;

    if (MIDIFile.copyTag(V2MIDI::File::Event::Meta::Copyright, s, sizeof(s)) > 0)
      jsonTrack["creator"] = s;
  }

  class {
  public:
    auto stop() {
      if (!_enabled)
        Device.allNotesOff(true);

      _enabled = false;
    }

    auto play() {
      Device.allNotesOff();
      _enabled  = true;
      _velocity = 20;
      _play     = {};
    }

    auto loop() {
      if (!_enabled)
        return;

      playNote();
    }

  private:
    auto playNote() -> void {
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

    bool    _enabled{};
    uint8_t _velocity{};
    struct {
      uint8_t       note;
      unsigned long usec;
    } _play{};
  } TestMode;

  class Button : public V2Buttons::Button {
  public:
    constexpr Button() : V2Buttons::Button(&_config, PIN_BUTTON) {}

  private:
    const V2Buttons::Config _config{.clickUsec{200 * 1000}, .holdUsec{500 * 1000}};

    auto handleClick(uint8_t count) -> void override {
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

    auto handleHold(uint8_t count) -> void override {
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
          Device.tune(Config.notes.start + (Config.notes.count - 1));
          break;

        case 4:
          Manual.setMode(Manual::Mode::Turn);
          Device.turn();
          break;
      }
    }
  } Button;
}

auto setup() -> void {
  Serial.begin(9600);
  SPI.begin();

  LED.begin();
  LED.brightnessMax(0.5);

  Link.begin();

  // Set the SERCOM interrupt priority, it requires a stable ~300 kHz interrupt
  // frequency. This needs to be after begin().
  setSerialPriority(&SerialPlug, 2);
  setSerialPriority(&SerialSocket, 1);

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

auto loop() -> void {
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
