#include <Arduino.h>
#include <gpio.h>

Gpio::Gpio(uint8_t pinId) {
    this->pinId = pinId;
    pinMode(this->pinId, INPUT);
}

Gpio::~Gpio() {
    this->detachPin();
    pinMode(this->pinId, INPUT);
}

Gpio::Mode Gpio::getMode() {
    return this->mode;
}

void Gpio::setMode(Mode mode) {
    if (mode == this->mode) return;
    this->detachPin();
    this->mode = mode;
    if (this->mode == Mode::IN) {
        pinMode(this->pinId, this->isPullupEnabled ? INPUT_PULLUP : INPUT);
    } else {
        pinMode(this->pinId, OUTPUT);
        if (this->mode == Mode::PWM) {
#ifdef ARDUINO_ARCH_ESP32
            for (this->pwmChannel = 0; this->pwmChannel < this->PWM_CHANNEL_COUNT; this->pwmChannel++) {
                if (this->isPwmChannelUsed[this->pwmChannel]) continue;
                this->isPwmChannelUsed[this->pwmChannel] = true;
                constexpr uint16_t frequency = 1000;
                constexpr uint8_t resolution = 8;
                ledcSetup(this->pwmChannel, frequency, resolution);
                ledcAttachPin(this->pinId, this->pwmChannel);
                break;
            }
#endif
        } else if (this->mode == Gpio::Mode::DAC) {
#ifndef ARDUINO_ARCH_ESP32
            this->timerId = digitalPinToTimer(this->pinId);
            //TimerUtil::enableFastPwm(this->timerId, true);
#endif
        }
    }
}

bool Gpio::input() {
    this->setMode(Mode::IN);
    return digitalRead(this->pinId) == HIGH;
}

uint16_t Gpio::inputAnalog() {
    this->setMode(Mode::IN);
    return analogRead(this->pinId);
}

void Gpio::enablePullup(bool enabled) {
    this->isPullupEnabled = enabled;
    if (this->mode == Mode::IN) {
        pinMode(this->pinId, this->isPullupEnabled ? INPUT_PULLUP : INPUT);
    } else {
        this->setMode(Mode::IN);
    }
}

void Gpio::setInterrupt(void (*isr)(void), Edge edge) {
    this->setMode(Mode::IN);
    if (isr) {
#ifdef ARDUINO_ARCH_MEGAAVR
        const PinStatus edges[] = { PinStatus::CHANGE, PinStatus::FALLING, PinStatus::RISING };
#else
        const uint8_t edges[] = { CHANGE, FALLING, RISING };
#endif
        attachInterrupt(digitalPinToInterrupt(this->pinId), isr, edges[(uint8_t)edge]);
    } else {
        detachInterrupt(digitalPinToInterrupt(this->pinId));
    }
    this->isInterruptSet = isr;
}

void Gpio::output(bool value) {
    this->setMode(Mode::OUT);
    digitalWrite(this->pinId, value ? HIGH : LOW);
}

void Gpio::outputPwm(uint16_t value) {
    this->setMode(Mode::PWM);
#ifdef ARDUINO_ARCH_ESP32
    if (this->pwmChannel < this->PWM_CHANNEL_COUNT) {
        ledcWrite(this->pwmChannel, min(value, (uint16_t)256));
    }
#else
    analogWrite(this->pinId, min(value, 255));
#endif
}

void Gpio::outputAnalog(uint8_t value) {
    this->setMode(Mode::DAC);
#ifdef ARDUINO_ARCH_ESP32
    dacWrite(this->pinId, value);
#else
    //TimerUtil::setComparedValue(this->timerId, value);
#endif
}

void Gpio::detachPin() {
    if (this->mode == Mode::IN) {
        if (this->isInterruptSet) this->setInterrupt();
    } else if (this->mode == Mode::PWM) {
#ifdef ARDUINO_ARCH_ESP32
        if (this->pwmChannel < this->PWM_CHANNEL_COUNT) {
            ledcDetachPin(this->pinId);
            this->isPwmChannelUsed[this->pwmChannel] = false;
        }
#endif
    } else if (this->mode == Mode::DAC) {
#ifndef ARDUINO_ARCH_ESP32
        //TimerUtil::enableFastPwm(this->timerId, false);
#endif
    }
}

#ifdef ARDUINO_ARCH_ESP32
bool Gpio::isPwmChannelUsed[Gpio::PWM_CHANNEL_COUNT] = {};
#endif
