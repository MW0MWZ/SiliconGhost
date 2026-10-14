// SPDX-License-Identifier: GPL-2.0-or-later
#include "app/display_sdl.h"

#include <SDL3/SDL.h>

#include <algorithm>
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
        default: return {0, false, 0};
    }
}

void send_key(ip22::Indy& m, SDL_Scancode sc, bool down) {
    const KeyCodes k = key_codes(sc);
    if (!k.set2) return;
    auto& kbc = m.ioc().kbc();
    if (kbc.scan_set() == 3) {
        if (!down) kbc.key_input(0xF0);
        kbc.key_input(k.set3);
        return;
    }
    if (k.extended) kbc.key_input(0xE0);
    if (!down) kbc.key_input(0xF0);
    kbc.key_input(k.set2);
}

// PS/2 mouse fed from the window: a click captures the pointer (relative
// mode), Ctrl+Alt+G or losing focus releases it. Motion is gathered per
// frame and sent as one movement; button changes are sent at once.
class MouseCapture {
public:
    MouseCapture(SDL_Window* window, ip22::Indy& machine) : window_(window), machine_(machine) { update_title(); }

    // True if the event was consumed.
    bool handle(const SDL_Event& ev) {
        switch (ev.type) {
            case SDL_EVENT_MOUSE_BUTTON_DOWN:
            case SDL_EVENT_MOUSE_BUTTON_UP: {
                if (!captured_) {
                    if (ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN) set_captured(true);
                    return true;
                }
                const unsigned bit = ev.button.button == SDL_BUTTON_LEFT    ? 1u
                                     : ev.button.button == SDL_BUTTON_RIGHT ? 2u
                                     : ev.button.button == SDL_BUTTON_MIDDLE ? 4u
                                                                             : 0u;
                if (!bit) return true;
                buttons_ = ev.type == SDL_EVENT_MOUSE_BUTTON_DOWN ? (buttons_ | bit) : (buttons_ & ~bit);
                flush(true);
                return true;
            }
            case SDL_EVENT_MOUSE_MOTION:
                if (captured_) {
                    dx_ += ev.motion.xrel;
                    dy_ += ev.motion.yrel;
                }
                return true;
            case SDL_EVENT_KEY_DOWN:
                if (captured_ && ev.key.scancode == SDL_SCANCODE_G && (ev.key.mod & SDL_KMOD_CTRL) &&
                    (ev.key.mod & SDL_KMOD_ALT)) {
                    set_captured(false);
                    return true;
                }
                return false;
            case SDL_EVENT_WINDOW_FOCUS_LOST:
                set_captured(false);
                return false;
            default: return false;
        }
    }

    // Sends the motion gathered since the last call (or the buttons, if forced).
    void flush(bool force = false) {
        const int x = static_cast<int>(dx_), y = static_cast<int>(dy_);
        if (!force && x == 0 && y == 0) return;
        dx_ -= static_cast<float>(x);
        dy_ -= static_cast<float>(y);
        machine_.ioc().kbc().mouse_motion(x, -y, buttons_);  // PS/2 counts up as positive
    }

private:
    void set_captured(bool on) {
        if (on == captured_) return;
        captured_ = on;
        SDL_SetWindowRelativeMouseMode(window_, on);
        if (!on && buttons_) {
            buttons_ = 0;
            flush(true);
        }
        dx_ = dy_ = 0;
        update_title();
    }
    void update_title() {
        SDL_SetWindowTitle(window_, captured_ ? "SiliconGhost - Ctrl+Alt+G releases the mouse"
                                              : "SiliconGhost - click to capture the mouse");
    }

    SDL_Window* window_;
    ip22::Indy& machine_;
    bool captured_ = false;
    unsigned buttons_ = 0;
    float dx_ = 0, dy_ = 0;
};

}  // namespace

bool run_with_display(ip22::Indy& machine, Pacer& pacer, volatile std::sig_atomic_t& stop) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SG_LOG(Error, "display", "SDL_Init failed: %s", SDL_GetError());
        return false;
    }
    SDL_Window* window = SDL_CreateWindow("SiliconGhost", 1280, 1024, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = window ? SDL_CreateRenderer(window, nullptr) : nullptr;
    if (!renderer) {
        SG_LOG(Error, "display", "cannot create window: %s", SDL_GetError());
        SDL_Quit();
        return false;
    }
    SDL_SetRenderLogicalPresentation(renderer, 1280, 1024, SDL_LOGICAL_PRESENTATION_LETTERBOX);
    MouseCapture mouse(window, machine);
    SDL_Texture* texture = nullptr;
    unsigned tex_w = 0, tex_h = 0;
    std::vector<u32> pixels;

    constexpr u64 kFrameNs = 1000000000ull / 60;
    const u64 slice = static_cast<u64>(machine.cycles_per_second() / 1000);  // 1 ms of guest time
    auto& cpu = machine.cpu();
    pacer.update(cpu.cycles(), cpu.instructions());
    u64 next_frame_ns = SDL_GetTicksNS();

    while (!stop && !machine.powered_off()) {
        machine.run(slice);
        cpu.set_cycles_per_instruction(pacer.update(cpu.cycles(), cpu.instructions()));
        const u64 now_ns = SDL_GetTicksNS();
        if (now_ns < next_frame_ns) continue;
        next_frame_ns = std::max(next_frame_ns + kFrameNs, now_ns);

        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_EVENT_QUIT) stop = 1;
            else if (mouse.handle(ev)) continue;
            else if ((ev.type == SDL_EVENT_KEY_DOWN && !ev.key.repeat) || ev.type == SDL_EVENT_KEY_UP)
                send_key(machine, ev.key.scancode, ev.type == SDL_EVENT_KEY_DOWN);
        }
        mouse.flush();

        unsigned w = 0, h = 0;
        const bool signal = machine.newport() && machine.newport()->render(pixels, w, h);
        if (signal) {
            if (!texture || w != tex_w || h != tex_h) {
                if (texture) SDL_DestroyTexture(texture);
                texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                            static_cast<int>(w), static_cast<int>(h));
                tex_w = w;
                tex_h = h;
                SDL_SetRenderLogicalPresentation(renderer, static_cast<int>(w), static_cast<int>(h),
                                                 SDL_LOGICAL_PRESENTATION_LETTERBOX);
            }
            SDL_UpdateTexture(texture, nullptr, pixels.data(), static_cast<int>(w * sizeof(u32)));
        }
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        if (signal && texture) SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    if (texture) SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return true;
}

}  // namespace sg::app
