/*
BSD 2-Clause License

Copyright (c) 2020, Andrea Zoppi
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _CALFMOO_MIDI_HPP_
#define _CALFMOO_MIDI_HPP_

#include <stddef.h>
#include <stdint.h>

// ============================================================================

namespace calfmoo {
namespace midi {

#pragma pack(push)
#pragma pack()

// ============================================================================

enum {
    INT7_MIN        = -(1 << 6),
    INT7_MAX        = ((1 << 6) - 1),

    UINT7_CENTER    = (1 << 6),
    UINT7_MAX       = ((1 << 7) - 1),

    INT14_MIN       = -(1 << 13),
    INT14_MAX       = ((1 << 13) - 1),

    UINT14_CENTER   = (1 << 13),
    UINT14_MAX      = ((1 << 14) - 1),
};

enum MessageType {
    NONE                = 0x00,

    // Channel
    CHANNEL_BASE        = 0x80,
    NOTE_ON             = 0x80,
    NOTE_OFF            = 0x90,
    POLY_KEY            = 0xA0,
    CONTROL_CHANGE      = 0xB0,
    PROGRAM_CHANGE      = 0xC0,
    CHANNEL_PRESSURE    = 0xD0,
    PITCH_BEND          = 0xE0,

    // System
    SYSTEM_BASE         = 0xF0,
    SYSEX_START         = 0xF0,
    QUARTER_FRAME       = 0xF1,
    SONG_POSITION       = 0xF2,
    SONG_SELECT         = 0xF3,
    TUNE_REQUEST        = 0xF6,
    SYSEX_END           = 0xF7,

    // Real-time
    REALTIME_BASE       = 0xF8,
    TIMING_CLOCK        = 0xF8,
    START               = 0xFA,
    CONTINUE            = 0xFB,
    STOP                = 0xFC,
    ACTIVE_SENSING      = 0xFE,
    RESET               = 0xFF,
};

enum ControllerIndex {
    // MSB
    CC_BANK_SELECT          = 0x00,
    CC_MODULATION           = 0x01,
    CC_BREATH               = 0x02,
    CC_FOOT                 = 0x04,
    CC_PORTAMENTO_TIME      = 0x05,
    CC_DATA_ENTRY           = 0x06,
    CC_VOLUME               = 0x07,
    CC_BALANCE              = 0x08,
    CC_PAN                  = 0x0A,
    CC_EXPRESSION           = 0x0B,
    CC_EFFECT_1             = 0x0C,
    CC_EFFECT_2             = 0x0D,
    CC_GENERIC_1            = 0x10,
    CC_GENERIC_2            = 0x11,
    CC_GENERIC_3            = 0x12,
    CC_GENERIC_4            = 0x13,

    // LSB = MSB + 0x20
    CC_LSB_OFFSET           = 0x20,

    // MSB only
    CC_SUSTAIN              = 0x40,
    CC_PORTAMENTO_GLIDE     = 0x41,
    CC_SOSTENUTO            = 0x42,
    CC_SOFT_PEDAL           = 0x43,
    CC_LEGATO               = 0x44,
    CC_HOLD_2               = 0x45,
    CC_VARIATION            = 0x46,
    CC_RESONANCE            = 0x47,
    CC_RELEASE              = 0x48,
    CC_ATTACK               = 0x49,
    CC_BRIGHTNESS           = 0x4A,
    CC_CONTROLLER_6         = 0x4B,
    CC_CONTROLLER_7         = 0x4C,
    CC_CONTROLLER_8         = 0x4D,
    CC_CONTROLLER_9         = 0x4E,
    CC_CONTROLLER_10        = 0x4F,
    CC_GENERIC_5            = 0x50,
    CC_GENERIC_6            = 0x51,
    CC_GENERIC_7            = 0x52,
    CC_GENERIC_8            = 0x53,
    CC_PORTAMENTO_CONTROL   = 0x54,
    CC_EXTERNAL_DEPTH       = 0x5B,
    CC_TREMOLO_DEPTH        = 0x5C,
    CC_CHORUS_DEPTH         = 0x5D,
    CC_DETUNE_DEPTH         = 0x5E,
    CC_PHASER_DEPTH         = 0x5F,
    CC_DATA_INCREMENT       = 0x60,
    CC_DATA_DECREMENT       = 0x61,
    CC_NREG_PARAM_LSB       = 0x62,
    CC_NREG_PARAM_MSB       = 0x63,
    CC_REG_PARAM_LSB        = 0x64,
    CC_REG_PARAM_MSB        = 0x65,

    // Channel mode
    CC_CHANNEL_MODE_BASE    = 0x79,
    CC_RESET_ALL            = 0x79,
    CC_LOCAL_CONTROL        = 0x7A,
    CC_ALL_NOTES_OFF        = 0x7B,
    CC_OMNI_OFF             = 0x7C,
    CC_OMNI_ON              = 0x7D,
    CC_MONO_ON              = 0x7E,
    CC_POLY_ON              = 0x7F,
};

enum {
    HEADER_MASK         = 0x80,
    DATA_MASK           = 0x7F,
    SUBTYPE_MASK        = 0x0F,

    CHANNEL_COUNT       = 16,
    CHANNEL_MASK        = 0x0F,

    NOTE_COUNT          = 0x80,
};

struct ParserCallbacks;
class Parser;

// ============================================================================

inline uint16_t UnpackUint14(uint8_t const data[])
{
    uint16_t u = uint16_t(data[0]) | (uint16_t(data[1]) << 7);
    return u;
}

inline void PackUint14(uint8_t data[], uint16_t u)
{
    data[0] = uint8_t((u >> 0) & UINT7_MAX);
    data[1] = uint8_t((u >> 7) & UINT7_MAX);
}

inline int16_t UnpackInt14(uint8_t const data[])
{
    uint16_t u = UnpackUint14(data);
    int16_t i = int16_t(u) - UINT14_CENTER;
    return i;
}

inline void PackInt14(uint8_t data[], int16_t i)
{
    uint16_t u = uint16_t(i) + UINT14_CENTER;
    PackUint14(data, u);
}

// ============================================================================

struct ParserCallbacks
{
    ParserCallbacks();
    virtual ~ParserCallbacks();

    virtual bool OnUnhandled(uint8_t data);
    virtual void OnDiscard(Parser const& parser);

    virtual void OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
    virtual void OnNoteOff(uint8_t channel, uint8_t note, uint8_t velocity);
    virtual void OnPolyKey(uint8_t channel, uint8_t note, uint8_t pressure);
    virtual void OnControlChange(uint8_t channel, uint8_t control, uint8_t value);
    virtual void OnProgramChange(uint8_t channel, uint8_t program);
    virtual void OnChannelPressure(uint8_t channel, uint8_t pressure);
    virtual void OnPitchBend(uint8_t channel, int16_t bend);

    virtual void OnSysExStart();
    virtual void OnSysExData(uint8_t data);
    virtual void OnSysExEnd();
    virtual void OnQuarterFrame(uint8_t subtype, uint8_t value);
    virtual void OnSongPosition(uint16_t beat);
    virtual void OnSongSelect(uint8_t song);
    virtual void OnTuneRequest();

    virtual void OnTimingClock();
    virtual void OnStart();
    virtual void OnContinue();
    virtual void OnStop();
    virtual void OnActiveSensing();
    virtual void OnReset();
};

// ============================================================================

class Parser
{
public:
    enum {
        BUFFER_SIZE = 4,
    };

public:
    Parser(ParserCallbacks* callbacks = NULL);
    void Reset();
    void Parse(uint8_t data);

public:
    ParserCallbacks* callbacks_;
    bool running_status_;

protected:
    MessageType message_type_;
    uint8_t buffer_[BUFFER_SIZE];
    uint8_t tail_;
    bool more_unhandled_;
};

// ============================================================================

namespace messages {
#pragma pack(push, 1)

// ----------------------------------------------------------------------------

struct ChannelMessage
{
    uint8_t channel : 4;
    uint8_t message_type : 4;

protected:
    ChannelMessage(MessageType message_type, uint8_t channel = 0) :
        channel(channel),
        message_type(uint8_t(message_type & 0xF))
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct NoteOn : public ChannelMessage
{
    uint8_t note;
    uint8_t velocity;

    bool Validate() const
    {
        return (message_type == NOTE_ON &&
                note <= UINT7_MAX &&
                velocity <= UINT7_MAX);
    }

    NoteOn(uint8_t channel, uint8_t note, uint8_t velocity) :
        ChannelMessage(NOTE_ON, channel),
        note(note),
        velocity(velocity)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct NoteOff : public ChannelMessage
{
    uint8_t note;
    uint8_t velocity;

    bool Validate() const
    {
        return (message_type == NOTE_OFF &&
                note <= UINT7_MAX &&
                velocity <= UINT7_MAX);
    }

    NoteOff(uint8_t channel, uint8_t note, uint8_t velocity = 0) :
        ChannelMessage(NOTE_OFF, channel),
        note(note),
        velocity(velocity)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct PolyKey : public ChannelMessage
{
    uint8_t note;
    uint8_t pressure;

    bool Validate() const
    {
        return (message_type == POLY_KEY &&
                note <= UINT7_MAX &&
                pressure <= UINT7_MAX);
    }

    PolyKey(uint8_t channel, uint8_t note, uint8_t pressure) :
        ChannelMessage(POLY_KEY, channel),
        note(note),
        pressure(pressure)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct ControlChange : public ChannelMessage
{
    uint8_t control;
    uint8_t value;

    bool Validate() const
    {
        return (message_type == CONTROL_CHANGE &&
                control <= UINT7_MAX &&
                value <= UINT7_MAX);
    }

    ControlChange(uint8_t channel, uint8_t control, uint8_t value) :
        ChannelMessage(CONTROL_CHANGE, channel),
        control(control),
        value(value)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct ProgramChange : public ChannelMessage
{
    uint8_t program;

    bool Validate() const
    {
        return (message_type == PROGRAM_CHANGE &&
                program <= UINT7_MAX);
    }

    ProgramChange(uint8_t channel, uint8_t program) :
        ChannelMessage(PROGRAM_CHANGE, channel),
        program(program)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct ChannelPressure : public ChannelMessage
{
    uint8_t pressure;

    bool Validate() const
    {
        return (message_type == CHANNEL_PRESSURE &&
                pressure <= UINT7_MAX);
    }

    ChannelPressure(uint8_t channel, uint8_t pressure) :
        ChannelMessage(CHANNEL_PRESSURE, channel),
        pressure(pressure)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct PitchBend : public ChannelMessage
{
    uint8_t bend_l;
    uint8_t bend_h;

    int16_t bend() const
    {
        return UnpackInt14(&bend_l);
    }

    void bend(int16_t bend)
    {
        PackInt14(&bend_l, bend);
    }

    bool Validate() const
    {
        return (message_type == PITCH_BEND &&
                bend_l <= UINT7_MAX &&
                bend_h <= UINT7_MAX);
    }

    PitchBend(uint8_t channel, uint8_t bend_l, uint8_t bend_h) :
        ChannelMessage(PITCH_BEND, channel),
        bend_l(bend_l),
        bend_h(bend_h)
    {}

    PitchBend(uint8_t channel, int16_t bend) :
        ChannelMessage(PITCH_BEND, channel)
    {
        this->bend(bend);
    }
};

// ----------------------------------------------------------------------------

struct QuarterFrame
{
    uint8_t value   : 4;
    uint8_t subtype : 3;
    uint8_t _7_     : 1;

    bool Validate() const
    {
        return _7_ == 0;
    }

    QuarterFrame(uint8_t subtype, uint8_t value) :
        subtype(subtype),
        value(value),
        _7_(0)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

struct SongPosition
{
    uint8_t beat_l;
    uint8_t beat_h;

    uint16_t beat() const
    {
        return UnpackUint14(&beat_l);
    }

    void beat(uint16_t beat)
    {
        PackUint14(&beat_l, beat);
    }

    bool Validate() const
    {
        return (beat_l <= UINT7_MAX &&
                beat_h <= UINT7_MAX);
    }

    SongPosition(uint8_t beat_l, uint8_t beat_h) :
        beat_l(beat_l),
        beat_h(beat_h)
    {
        ;
    }

    SongPosition(uint16_t beat)
    {
        this->beat(beat);
    }
};

// ----------------------------------------------------------------------------

struct SongSelect
{
    uint8_t song;

    bool Validate() const
    {
        return song < UINT7_MAX;
    }

    SongSelect(uint8_t song) :
        song(song)
    {
        ;
    }
};

// ----------------------------------------------------------------------------

#pragma pack(pop)
}  // namespace messages

// ============================================================================

#pragma pack(pop)

}  // namespace midi
}  // namespace calfmoo

#endif  // _CALFMOO_MIDI_HPP_
