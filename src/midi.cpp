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

#include "calfmoo/midi.hpp"

namespace calfmoo {
namespace midi {

// ============================================================================

ParserCallbacks::ParserCallbacks()
{
    ;
}

// ----------------------------------------------------------------------------

ParserCallbacks::~ParserCallbacks()
{
    ;
}

// ----------------------------------------------------------------------------

bool ParserCallbacks::OnUnhandled(uint8_t data)
{
    (void)data;
    return false;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnDiscard(Parser const &parser)
{
    (void)parser;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
    (void)channel;
    (void)note;
    (void)velocity;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnNoteOff(uint8_t channel, uint8_t note, uint8_t velocity)
{
    (void)channel;
    (void)note;
    (void)velocity;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnPolyKey(uint8_t channel, uint8_t note, uint8_t pressure)
{
    (void)channel;
    (void)note;
    (void)pressure;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnControlChange(uint8_t channel, uint8_t control, uint8_t value)
{
    (void)channel;
    (void)control;
    (void)value;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnProgramChange(uint8_t channel, uint8_t program)
{
    (void)channel;
    (void)program;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnChannelPressure(uint8_t channel, uint8_t pressure)
{
    (void)channel;
    (void)pressure;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnPitchBend(uint8_t channel, int16_t bend)
{
    (void)channel;
    (void)bend;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnSysExStart()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnSysExData(uint8_t data)
{
    (void)data;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnSysExEnd()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnQuarterFrame(uint8_t subtype, uint8_t value)
{
    (void)subtype;
    (void)value;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnSongPosition(uint16_t beat)
{
    (void)beat;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnSongSelect(uint8_t song)
{
    (void)song;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnTuneRequest()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnTimingClock()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnStart()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnContinue()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnStop()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnActiveSensing()
{
    ;
}

// ----------------------------------------------------------------------------

void ParserCallbacks::OnReset()
{
    ;
}

// ============================================================================

Parser::Parser(ParserCallbacks *callbacks) :
    callbacks_(callbacks)
{
    Reset();
}

// ----------------------------------------------------------------------------

void Parser::Reset()
{
    message_type_ = NONE;
    tail_ = 0;
    more_unhandled_ = false;
}

// ----------------------------------------------------------------------------

void Parser::Parse(uint8_t data)
{
    if (more_unhandled_) {
        more_unhandled_ = callbacks_->OnUnhandled(data);
    }
    else if (data & HEADER_MASK) {
        if (data >= REALTIME_BASE) {
            switch (MessageType(data))
            {
                case TIMING_CLOCK: {
                    callbacks_->OnTimingClock();
                    break;
                }
                case START: {
                    callbacks_->OnStart();
                    break;
                }
                case CONTINUE: {
                    callbacks_->OnContinue();
                    break;
                }
                case STOP: {
                    callbacks_->OnStop();
                    break;
                }
                case ACTIVE_SENSING: {
                    callbacks_->OnActiveSensing();
                    break;
                }
                case RESET: {
                    callbacks_->OnReset();
                    Reset();
                    break;
                }
                default: {
                    more_unhandled_ = callbacks_->OnUnhandled(data);
                    break;
                }
            }
        }
        else if (data >= SYSTEM_BASE) {
            Reset();
            buffer_[tail_++] = data;
            message_type_ = MessageType(data);

            switch (message_type_)
            {
                case SYSEX_START: {
                    callbacks_->OnSysExStart();
                    break;
                }
                case TUNE_REQUEST: {
                    callbacks_->OnTuneRequest();
                    Reset();
                    break;
                }
                case SYSEX_END: {
                    callbacks_->OnSysExEnd();
                    Reset();
                    break;
                }
                case QUARTER_FRAME:
                case SONG_POSITION:
                case SONG_SELECT:
                {
                    break;
                }
                default: {
                    Reset();
                    more_unhandled_ = callbacks_->OnUnhandled(data);
                    break;
                }
            }
        }
        else if (data >= CHANNEL_BASE) {
            Reset();
            buffer_[tail_++] = data;
            message_type_ = MessageType(data & ~SUBTYPE_MASK);
        }
    }
    else {
        if (tail_ < BUFFER_SIZE) {
            buffer_[tail_++] = data;

            switch (message_type_)
            {
                case NOTE_ON: {
                    typedef messages::NoteOn Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnNoteOn(m.channel, m.note, m.velocity);
                        Reset();
                    }
                    break;
                }
                case NOTE_OFF: {
                    typedef messages::NoteOff Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnNoteOff(m.channel, m.note, m.velocity);
                        Reset();
                    }
                    break;
                }
                case POLY_KEY: {
                    typedef messages::PolyKey Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnPolyKey(m.channel, m.note, m.pressure);
                        Reset();
                    }
                    break;
                }
                case CONTROL_CHANGE: {
                    typedef messages::ControlChange Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnControlChange(m.channel, m.control, m.value);
                        Reset();
                    }
                    break;
                }
                case PROGRAM_CHANGE: {
                    typedef messages::ProgramChange Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnProgramChange(m.channel, m.program);
                        Reset();
                    }
                    break;
                }
                case CHANNEL_PRESSURE: {
                    typedef messages::ChannelPressure Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnChannelPressure(m.channel, m.pressure);
                        Reset();
                    }
                    break;
                }
                case PITCH_BEND: {
                    typedef messages::PitchBend Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnPitchBend(m.channel, m.bend());
                        Reset();
                    }
                    break;
                }

                case SYSEX_START: {
                    callbacks_->OnSysExData(data);
                    break;
                }
                case QUARTER_FRAME: {
                    typedef messages::QuarterFrame Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnQuarterFrame(m.subtype, m.value);
                        Reset();
                    }
                    break;
                }
                case SONG_POSITION: {
                    typedef messages::SongPosition Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnSongPosition(m.beat());
                        Reset();
                    }
                    break;
                }
                case SONG_SELECT: {
                    typedef messages::SongSelect Message;
                    if (tail_ == sizeof(Message)) {
                        Message const &m = reinterpret_cast<Message &>(buffer_);
                        callbacks_->OnSongSelect(m.song);
                        Reset();
                    }
                    break;
                }
                case TUNE_REQUEST:
                case SYSEX_END:
                {
                    break;
                }

                default: {
                    more_unhandled_ = callbacks_->OnUnhandled(data);
                    break;
                }
            }
        }
        else {
            callbacks_->OnDiscard(*this);
        }
    }
}

// ============================================================================

}  // namespace midi
}  // namespace calfmoo
