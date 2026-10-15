// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// A window of its own showing one serial port's terminal, with the keyboard
// typing into the port. Each window has its own SDL renderer and ImGui
// context; it can be closed and reopened while the terminal keeps receiving.
#pragma once

#include <SDL3/SDL.h>

#include <functional>
#include <string>

#include "app/terminal.h"

struct ImGuiContext;

namespace sg::app {

class TerminalWindow {
public:
    using Send = std::function<void(const std::string&)>;
    TerminalWindow(std::string title, Terminal& terminal, Send send, float ui_scale);
    ~TerminalWindow();
    TerminalWindow(const TerminalWindow&) = delete;
    TerminalWindow& operator=(const TerminalWindow&) = delete;

    bool is_open() const { return window_ != nullptr; }
    void open();
    void close();
    void raise();
    SDL_WindowID id() const { return window_ ? SDL_GetWindowID(window_) : 0; }

    // Takes an event for this window (by window ID); false if it is not ours.
    bool handle(const SDL_Event& ev);
    // Draws if anything changed (or the cursor blinked); call every frame.
    void frame(u64 now_ns);

private:
    void key_down(const SDL_KeyboardEvent& key);
    void type(const std::string& bytes);
    void paste();
    void copy();
    void resize_for_font();
    void draw();

    std::string title_;
    Terminal& terminal_;
    Send send_;
    float ui_scale_;
    float density_ = 1.0f;  // pixels per window coordinate
    int zoom_ = 1;
    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    ImGuiContext* ctx_ = nullptr;
    u64 shown_version_ = ~0ull;
    int pending_frames_ = 2;  // ImGui settles over a couple of frames after input
    int scroll_ = 0;          // lines scrolled back into history
    bool blink_on_ = true;
    u64 next_blink_ns_ = 0;
    bool focused_ = false;
};

}  // namespace sg::app
