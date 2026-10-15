// SPDX-License-Identifier: GPL-2.0-or-later
#include "app/display_sdl.h"
#include "app/sdl_common.h"

#include <SDL3/SDL.h>

#include <algorithm>
#include <optional>
#include <vector>

#include "core/log.h"

namespace sg::app {

namespace {

struct KeyCodes {
    u8 set2;        // set 2 make code (0 = not mapped)
    bool extended;  // set 2 code is E0-prefixed
    u8 set3;        // set 3 make code
};

// PS/2 make codes for scan code sets 2 and 3.
KeyCodes key_codes(SDL_Scancode sc) {
    if (sc >= SDL_SCANCODE_A && sc <= SDL_SCANCODE_Z) {
        static constexpr u8 kLetters[26] = {0x1C, 0x32, 0x21, 0x23, 0x24, 0x2B, 0x34, 0x33, 0x43, 0x3B, 0x42, 0x4B, 0x3A,
                                            0x31, 0x44, 0x4D, 0x15, 0x2D, 0x1B, 0x2C, 0x3C, 0x2A, 0x1D, 0x22, 0x35, 0x1A};
        const u8 c = kLetters[sc - SDL_SCANCODE_A];
        return {c, false, c};
    }
    if (sc >= SDL_SCANCODE_1 && sc <= SDL_SCANCODE_0) {
        static constexpr u8 kDigits[10] = {0x16, 0x1E, 0x26, 0x25, 0x2E, 0x36, 0x3D, 0x3E, 0x46, 0x45};
        const u8 c = kDigits[sc - SDL_SCANCODE_1];
        return {c, false, c};
    }
    if (sc >= SDL_SCANCODE_F1 && sc <= SDL_SCANCODE_F12) {
        static constexpr u8 kF2[12] = {0x05, 0x06, 0x04, 0x0C, 0x03, 0x0B, 0x83, 0x0A, 0x01, 0x09, 0x78, 0x07};
        static constexpr u8 kF3[12] = {0x07, 0x0F, 0x17, 0x1F, 0x27, 0x2F, 0x37, 0x3F, 0x47, 0x4F, 0x56, 0x5E};
        return {kF2[sc - SDL_SCANCODE_F1], false, kF3[sc - SDL_SCANCODE_F1]};
    }
    switch (sc) {
        case SDL_SCANCODE_RETURN: return {0x5A, false, 0x5A};
        case SDL_SCANCODE_ESCAPE: return {0x76, false, 0x08};
        case SDL_SCANCODE_BACKSPACE: return {0x66, false, 0x66};
        case SDL_SCANCODE_TAB: return {0x0D, false, 0x0D};
        case SDL_SCANCODE_SPACE: return {0x29, false, 0x29};
        case SDL_SCANCODE_MINUS: return {0x4E, false, 0x4E};
        case SDL_SCANCODE_EQUALS: return {0x55, false, 0x55};
        case SDL_SCANCODE_LEFTBRACKET: return {0x54, false, 0x54};
        case SDL_SCANCODE_RIGHTBRACKET: return {0x5B, false, 0x5B};
        case SDL_SCANCODE_BACKSLASH: return {0x5D, false, 0x5C};
        case SDL_SCANCODE_SEMICOLON: return {0x4C, false, 0x4C};
        case SDL_SCANCODE_APOSTROPHE: return {0x52, false, 0x52};
        case SDL_SCANCODE_GRAVE: return {0x0E, false, 0x0E};
        case SDL_SCANCODE_COMMA: return {0x41, false, 0x41};
        case SDL_SCANCODE_PERIOD: return {0x49, false, 0x49};
        case SDL_SCANCODE_SLASH: return {0x4A, false, 0x4A};
        case SDL_SCANCODE_CAPSLOCK: return {0x58, false, 0x14};
        case SDL_SCANCODE_LSHIFT: return {0x12, false, 0x12};
        case SDL_SCANCODE_RSHIFT: return {0x59, false, 0x59};
        case SDL_SCANCODE_LCTRL: return {0x14, false, 0x11};
        case SDL_SCANCODE_RCTRL: return {0x14, true, 0x58};
        case SDL_SCANCODE_LALT: return {0x11, false, 0x19};
        case SDL_SCANCODE_RALT: return {0x11, true, 0x39};
        case SDL_SCANCODE_UP: return {0x75, true, 0x63};
        case SDL_SCANCODE_DOWN: return {0x72, true, 0x60};
        case SDL_SCANCODE_LEFT: return {0x6B, true, 0x61};
        case SDL_SCANCODE_RIGHT: return {0x74, true, 0x6A};
        case SDL_SCANCODE_INSERT: return {0x70, true, 0x67};
        case SDL_SCANCODE_DELETE: return {0x71, true, 0x64};
        case SDL_SCANCODE_HOME: return {0x6C, true, 0x6E};
        case SDL_SCANCODE_END: return {0x69, true, 0x65};
        case SDL_SCANCODE_PAGEUP: return {0x7D, true, 0x6F};
        case SDL_SCANCODE_PAGEDOWN: return {0x7A, true, 0x6D};
        // ISO keys: "#~" beside Enter and "\|" beside left Shift
        case SDL_SCANCODE_NONUSHASH: return {0x5D, false, 0x53};
        case SDL_SCANCODE_NONUSBACKSLASH: return {0x61, false, 0x13};
        case SDL_SCANCODE_SCROLLLOCK: return {0x7E, false, 0x5F};
        case SDL_SCANCODE_NUMLOCKCLEAR: return {0x77, false, 0x76};
        case SDL_SCANCODE_KP_DIVIDE: return {0x4A, true, 0x77};
        case SDL_SCANCODE_KP_MULTIPLY: return {0x7C, false, 0x7E};
        case SDL_SCANCODE_KP_MINUS: return {0x7B, false, 0x84};
        case SDL_SCANCODE_KP_PLUS: return {0x79, false, 0x7C};
        case SDL_SCANCODE_KP_ENTER: return {0x5A, true, 0x79};
        case SDL_SCANCODE_KP_PERIOD: return {0x71, false, 0x71};
        case SDL_SCANCODE_KP_0: return {0x70, false, 0x70};
        case SDL_SCANCODE_KP_1: return {0x69, false, 0x69};
        case SDL_SCANCODE_KP_2: return {0x72, false, 0x72};
        case SDL_SCANCODE_KP_3: return {0x7A, false, 0x7A};
        case SDL_SCANCODE_KP_4: return {0x6B, false, 0x6B};
        case SDL_SCANCODE_KP_5: return {0x73, false, 0x73};
        case SDL_SCANCODE_KP_6: return {0x74, false, 0x74};
        case SDL_SCANCODE_KP_7: return {0x6C, false, 0x6C};
        case SDL_SCANCODE_KP_8: return {0x75, false, 0x75};
        case SDL_SCANCODE_KP_9: return {0x7D, false, 0x7D};
        case SDL_SCANCODE_LGUI: return {0x1F, true, 0x8B};
        case SDL_SCANCODE_RGUI: return {0x27, true, 0x8C};
        case SDL_SCANCODE_APPLICATION: return {0x2F, true, 0x8D};
        default: return {0, false, 0};
    }
}

}  // namespace

void send_key(ip22::Indy& m, SDL_Scancode sc, bool down) {
    auto& kbc = m.ioc().kbc();
    // Print Screen and Pause are sequences in set 2 (Pause has no break).
    if (sc == SDL_SCANCODE_PRINTSCREEN || sc == SDL_SCANCODE_PAUSE) {
        const bool print = sc == SDL_SCANCODE_PRINTSCREEN;
        if (kbc.scan_set() == 3) {
            if (!down) kbc.key_input(0xF0);
            kbc.key_input(print ? 0x57 : 0x62);
        } else if (print) {
            static constexpr u8 kMake[] = {0xE0, 0x12, 0xE0, 0x7C};
            static constexpr u8 kBreak[] = {0xE0, 0xF0, 0x7C, 0xE0, 0xF0, 0x12};
            if (down) for (u8 b : kMake) kbc.key_input(b);
            else for (u8 b : kBreak) kbc.key_input(b);
        } else if (down) {
            for (u8 b : {0xE1, 0x14, 0x77, 0xE1, 0xF0, 0x14, 0xF0, 0x77}) kbc.key_input(b);
        }
        return;
    }
    const KeyCodes k = key_codes(sc);
    if (!k.set2) return;
    if (kbc.scan_set() == 3) {
        if (!down) kbc.key_input(0xF0);
        kbc.key_input(k.set3);
        return;
    }
    if (k.extended) kbc.key_input(0xE0);
    if (!down) kbc.key_input(0xF0);
    kbc.key_input(k.set2);
}

namespace {

// Plays HAL2 DAC output. The stream follows the guest's sample rate and
// voice count; when the guest runs ahead of real time (or the device stalls)
// more than a quarter of a second, new samples are dropped rather than
// letting the delay grow.
// One SDL audio stream in the guest's current format, reopened when that
// changes. A format the device refuses is not retried until it changes.
class AudioStream {
public:
    ~AudioStream() { close(); }
    SDL_AudioStream* get(SDL_AudioDeviceID device, unsigned voices, double rate, const char* what) {
        const int freq = static_cast<int>(rate + 0.5);
        if (freq == freq_ && static_cast<int>(voices) == voices_) return stream_;
        close();
        freq_ = freq;
        voices_ = static_cast<int>(voices);
        const SDL_AudioSpec spec{SDL_AUDIO_S16, voices_, freq_};
        stream_ = SDL_OpenAudioDeviceStream(device, &spec, nullptr, nullptr);
        if (!stream_) {
            SG_LOG(Warn, "audio", "cannot open audio %s: %s", what, SDL_GetError());
            return nullptr;
        }
        SDL_ResumeAudioStreamDevice(stream_);
        return stream_;
    }
    // A quarter of a second of audio in this format, in bytes.
    int quarter_second() const { return freq_ * voices_ * static_cast<int>(sizeof(s16)) / 4; }

private:
    void close() {
        if (stream_) SDL_DestroyAudioStream(stream_);
        stream_ = nullptr;
    }
    SDL_AudioStream* stream_ = nullptr;
    int freq_ = 0, voices_ = 0;
};

class AudioOut final : public AudioOutput {
public:
    AudioOut(ip22::Indy& machine, AudioOptions options) : machine_(machine) {
        if (!SDL_InitSubSystem(SDL_INIT_AUDIO)) {
            SG_LOG(Warn, "audio", "no audio: %s", SDL_GetError());
            return;
        }
        ok_ = true;
        if (options.output)
            machine_.hpc3().set_audio_sink([this](const s16* s, std::size_t frames, unsigned voices, double rate) {
                play(s, frames, voices, rate);
            });
        if (options.input)
            machine_.hpc3().set_audio_source([this](s16* s, std::size_t frames, unsigned voices, double rate) {
                record(s, frames, voices, rate);
            });
    }
    ~AudioOut() override {
        machine_.hpc3().set_audio_sink(nullptr);
        machine_.hpc3().set_audio_source(nullptr);
        playback_.reset();
        recording_.reset();
        if (ok_) SDL_QuitSubSystem(SDL_INIT_AUDIO);
    }
    AudioOut(const AudioOut&) = delete;
    AudioOut& operator=(const AudioOut&) = delete;

private:
    void play(const s16* samples, std::size_t frames, unsigned voices, double rate) {
        SDL_AudioStream* stream = playback_->get(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, voices, rate, "output");
        if (!stream) return;
        if (SDL_GetAudioStreamQueued(stream) > playback_->quarter_second()) return;
        SDL_PutAudioStreamData(stream, samples, static_cast<int>(frames * voices * sizeof(s16)));
    }

    // The recording device opens when the guest first records. What it has
    // not delivered yet is silence; a backlog beyond a quarter of a second
    // (the guest stopped recording for a while) is dropped.
    void record(s16* samples, std::size_t frames, unsigned voices, double rate) {
        const int bytes = static_cast<int>(frames * voices * sizeof(s16));
        int got = 0;
        if (SDL_AudioStream* stream = recording_->get(SDL_AUDIO_DEVICE_DEFAULT_RECORDING, voices, rate, "input")) {
            std::vector<u8> discard;
            for (int extra = SDL_GetAudioStreamAvailable(stream) - bytes - recording_->quarter_second(); extra > 0;
                 extra = SDL_GetAudioStreamAvailable(stream) - bytes - recording_->quarter_second()) {
                discard.resize(static_cast<std::size_t>(extra));
                if (SDL_GetAudioStreamData(stream, discard.data(), extra) <= 0) break;
            }
            got = std::max(0, SDL_GetAudioStreamData(stream, samples, bytes));
        }
        std::fill(reinterpret_cast<u8*>(samples) + got, reinterpret_cast<u8*>(samples) + bytes, u8{0});
    }

    ip22::Indy& machine_;
    bool ok_ = false;
    std::optional<AudioStream> playback_{std::in_place}, recording_{std::in_place};
};

}  // namespace

std::unique_ptr<AudioOutput> start_audio(ip22::Indy& machine, AudioOptions options) {
    return std::make_unique<AudioOut>(machine, options);
}

}  // namespace sg::app
