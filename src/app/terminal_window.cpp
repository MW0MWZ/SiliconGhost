// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
#include "app/terminal_window.h"

#include <algorithm>
#include <cfloat>

#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "imgui.h"

namespace sg::app {

namespace {

// The terminal holds Latin-1; ImGui and the clipboard use UTF-8.
void append_utf8(std::string& out, char c) {
    const auto u = static_cast<unsigned char>(c);
    if (u < 0x80) {
        out += c;
    } else {
        out += static_cast<char>(0xC0 | (u >> 6));
        out += static_cast<char>(0x80 | (u & 0x3F));
    }
}

std::string utf8_to_latin1(const char* s) {
    std::string out;
    for (const auto* p = reinterpret_cast<const unsigned char*>(s); *p;) {
        if (*p < 0x80) {
            out += static_cast<char>(*p++);
        } else if ((*p & 0xE0) == 0xC0 && (p[1] & 0xC0) == 0x80) {
            const unsigned cp = ((p[0] & 0x1Fu) << 6) | (p[1] & 0x3Fu);
            if (cp <= 0xFF) out += static_cast<char>(cp);
            p += 2;
        } else {  // outside Latin-1: dropped
            ++p;
            while ((*p & 0xC0) == 0x80) ++p;
        }
    }
    return out;
}

constexpr float kBaseFont = 13.0f;  // ImGui's built-in font is drawn for 13 px

// RAII switch of the current ImGui context.
class ContextScope {
public:
    explicit ContextScope(ImGuiContext* ctx) : saved_(ImGui::GetCurrentContext()) { ImGui::SetCurrentContext(ctx); }
    ~ContextScope() { ImGui::SetCurrentContext(saved_); }
    ContextScope(const ContextScope&) = delete;
    ContextScope& operator=(const ContextScope&) = delete;

private:
    ImGuiContext* saved_;
};

}  // namespace

TerminalWindow::TerminalWindow(std::string title, Terminal& terminal, Send send, float ui_scale)
    : title_(std::move(title)), terminal_(terminal), send_(std::move(send)), ui_scale_(ui_scale) {}

TerminalWindow::~TerminalWindow() { close(); }

void TerminalWindow::open() {
    if (window_) {
        raise();
        return;
    }
    window_ = SDL_CreateWindow(title_.c_str(), 640, 400, SDL_WINDOW_HIGH_PIXEL_DENSITY);
    renderer_ = window_ ? SDL_CreateRenderer(window_, nullptr) : nullptr;
    if (!renderer_) {
        if (window_) SDL_DestroyWindow(window_);
        window_ = nullptr;
        return;
    }
    ctx_ = ImGui::CreateContext();
    {
        ContextScope scope(ctx_);
        ImGuiIO& io = ImGui::GetIO();
        io.IniFilename = nullptr;
        ImGui::StyleColorsDark();
        ImGui::GetStyle().ScaleAllSizes(ui_scale_);
        // Rasterised at the pixel density (Retina), drawn at coordinate size.
        density_ = std::max(1.0f, SDL_GetWindowPixelDensity(window_));
        for (int z = 1; z <= 2; ++z) {
            ImFontConfig cfg;
            cfg.SizePixels = kBaseFont * ui_scale_ * static_cast<float>(z) * density_;
            io.Fonts->AddFontDefault(&cfg);
        }
        io.Fonts->Build();
        io.FontGlobalScale = 1.0f / density_;
        ImGui_ImplSDL3_InitForSDLRenderer(window_, renderer_);
        ImGui_ImplSDLRenderer3_Init(renderer_);
    }
    SDL_StartTextInput(window_);
    resize_for_font();
    pending_frames_ = 3;
    shown_version_ = ~0ull;
}

void TerminalWindow::close() {
    if (!window_) return;
    {
        ContextScope scope(ctx_);
        ImGui_ImplSDLRenderer3_Shutdown();
        ImGui_ImplSDL3_Shutdown();
    }
    ImGui::DestroyContext(ctx_);
    ctx_ = nullptr;
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
    renderer_ = nullptr;
    window_ = nullptr;
    focused_ = false;
}

void TerminalWindow::raise() {
    if (window_) SDL_RaiseWindow(window_);
}

void TerminalWindow::resize_for_font() {
    ContextScope scope(ctx_);
    const ImGuiStyle& style = ImGui::GetStyle();
    ImFont* font = ImGui::GetIO().Fonts->Fonts[zoom_ - 1];
    const float size = font->FontSize / density_;  // in window coordinates
    const float cw = font->CalcTextSizeA(size, FLT_MAX, 0.0f, "M").x;
    const float lh = size;
    const float menu = ImGui::GetIO().Fonts->Fonts[0]->FontSize / density_ + style.FramePadding.y * 2;
    const int w = static_cast<int>(cw * Terminal::kCols + style.WindowPadding.x * 2 + 0.5f);
    const int h = static_cast<int>(lh * Terminal::kRows + menu + style.WindowPadding.y * 2 + 0.5f);
    SDL_SetWindowSize(window_, w, h);
    pending_frames_ = 3;
}

bool TerminalWindow::handle(const SDL_Event& ev) {
    if (!window_ || SDL_GetWindowFromEvent(&ev) != window_) return false;
    pending_frames_ = 6;
    switch (ev.type) {
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED: close(); return true;
        case SDL_EVENT_WINDOW_FOCUS_GAINED: focused_ = true; break;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            focused_ = false;
            blink_on_ = true;
            break;
        default: break;
    }
    {
        ContextScope scope(ctx_);
        ImGui_ImplSDL3_ProcessEvent(&ev);
    }
    if (ev.type == SDL_EVENT_TEXT_INPUT) type(utf8_to_latin1(ev.text.text));
    else if (ev.type == SDL_EVENT_KEY_DOWN) key_down(ev.key);
    return true;
}

void TerminalWindow::type(const std::string& bytes) {
    if (bytes.empty()) return;
    scroll_ = 0;
    blink_on_ = true;
    send_(bytes);
}

void TerminalWindow::key_down(const SDL_KeyboardEvent& key) {
    const bool ctrl = key.mod & SDL_KMOD_CTRL, shift = key.mod & SDL_KMOD_SHIFT, cmd = key.mod & SDL_KMOD_GUI;
    if ((ctrl && shift) || cmd) {  // copy and paste: Ctrl+Shift+C/V, or Cmd+C/V on a Mac
        if (key.key == SDLK_C) copy();
        else if (key.key == SDLK_V) paste();
        return;
    }
    if (shift && (key.key == SDLK_PAGEUP || key.key == SDLK_PAGEDOWN)) {
        const int page = Terminal::kRows - 1;
        scroll_ = std::clamp(scroll_ + (key.key == SDLK_PAGEUP ? page : -page), 0,
                             static_cast<int>(terminal_.scrollback().size()));
        pending_frames_ = 2;
        return;
    }
    switch (key.key) {
        case SDLK_RETURN: case SDLK_KP_ENTER: type("\r"); return;
        case SDLK_BACKSPACE: type("\x08"); return;
        case SDLK_TAB: type("\t"); return;
        case SDLK_ESCAPE: type("\x1B"); return;
        case SDLK_UP: type("\x1B[A"); return;
        case SDLK_DOWN: type("\x1B[B"); return;
        case SDLK_RIGHT: type("\x1B[C"); return;
        case SDLK_LEFT: type("\x1B[D"); return;
        case SDLK_HOME: type("\x1B[H"); return;
        case SDLK_DELETE: type("\x7F"); return;
        default: break;
    }
    if (ctrl) {
        if (key.key >= SDLK_A && key.key <= SDLK_Z) type(std::string(1, static_cast<char>(key.key - SDLK_A + 1)));
        else if (key.key == SDLK_LEFTBRACKET) type("\x1B");
        else if (key.key == SDLK_BACKSLASH) type("\x1C");
        else if (key.key == SDLK_RIGHTBRACKET) type("\x1D");
        else if (key.key == SDLK_SPACE || key.key == SDLK_2) type(std::string(1, '\0'));
    }
}

void TerminalWindow::copy() {
    std::string text;
    for (char c : terminal_.screen_text()) append_utf8(text, c);
    SDL_SetClipboardText(text.c_str());
}

void TerminalWindow::paste() {
    char* clip = SDL_GetClipboardText();
    if (!clip) return;
    std::string bytes = utf8_to_latin1(clip);
    SDL_free(clip);
    // Lines end in carriage returns, as typed.
    std::string out;
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        if (bytes[i] == '\r' && i + 1 < bytes.size() && bytes[i + 1] == '\n') continue;
        out += bytes[i] == '\n' ? '\r' : bytes[i];
    }
    type(out);
}

void TerminalWindow::frame(u64 now_ns) {
    if (!window_) return;
    if (focused_ && now_ns >= next_blink_ns_) {
        blink_on_ = !blink_on_;
        next_blink_ns_ = now_ns + 530000000ull;
        pending_frames_ = std::max(pending_frames_, 1);
    }
    if (terminal_.version() != shown_version_) pending_frames_ = std::max(pending_frames_, 1);
    if (pending_frames_ <= 0) return;
    --pending_frames_;
    shown_version_ = terminal_.version();
    ContextScope scope(ctx_);
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    draw();
    ImGui::Render();
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer_);
    SDL_RenderPresent(renderer_);
}

void TerminalWindow::draw() {
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(0, 0, 0, 255));
    ImGui::Begin("##terminal", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_MenuBar |
                     ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoSavedSettings);
    ImGui::PopStyleColor();
    const int history = static_cast<int>(terminal_.scrollback().size());
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("Edit")) {
#if defined(__APPLE__)
            const char* copy_keys = "Cmd+C";
            const char* paste_keys = "Cmd+V";
#else
            const char* copy_keys = "Ctrl+Shift+C";
            const char* paste_keys = "Ctrl+Shift+V";
#endif
            if (ImGui::MenuItem("Copy Screen", copy_keys)) copy();
            if (ImGui::MenuItem("Paste", paste_keys)) paste();
            ImGui::Separator();
            if (ImGui::MenuItem("Clear")) terminal_.reset();
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem("Normal Size", nullptr, zoom_ == 1)) zoom_ = 1, resize_for_font();
            if (ImGui::MenuItem("Double Size", nullptr, zoom_ == 2)) zoom_ = 2, resize_for_font();
            ImGui::Separator();
            if (ImGui::MenuItem("Scroll to Bottom", nullptr, false, scroll_ > 0)) scroll_ = 0;
            ImGui::EndMenu();
        }
        if (scroll_ > 0) ImGui::TextDisabled("  history: %d lines up (Shift+PgUp/PgDn, wheel)", scroll_);
        ImGui::EndMenuBar();
    }

    ImFont* font = io.Fonts->Fonts[zoom_ - 1];
    ImGui::PushFont(font);
    const float fs = ImGui::GetFontSize();  // the font's size in window coordinates
    const float cw = font->CalcTextSizeA(fs, FLT_MAX, 0.0f, "M").x;
    const float lh = fs;
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    if (ImGui::IsWindowHovered() && io.MouseWheel != 0.0f) {
        scroll_ = std::clamp(scroll_ + static_cast<int>(io.MouseWheel * 3), 0, history);
        pending_frames_ = 2;
    }
    scroll_ = std::min(scroll_, history);

    const ImU32 kFg = IM_COL32(204, 204, 204, 255), kBold = IM_COL32(255, 255, 255, 255), kBg = IM_COL32(0, 0, 0, 255);
    std::string run;
    for (int r = 0; r < Terminal::kRows; ++r) {
        const int index = history - scroll_ + r;  // into history followed by the screen
        const Terminal::Cell* cells;
        if (index < history) cells = terminal_.scrollback()[static_cast<std::size_t>(index)].data();
        else cells = &terminal_.at(index - history, 0);
        const float y = origin.y + static_cast<float>(r) * lh;
        for (int c = 0; c < Terminal::kCols;) {
            const u8 attr = cells[c].attr;
            int end = c;
            run.clear();
            while (end < Terminal::kCols && cells[end].attr == attr) append_utf8(run, cells[end++].ch);
            const float x0 = origin.x + static_cast<float>(c) * cw, x1 = origin.x + static_cast<float>(end) * cw;
            ImU32 fg = attr & Terminal::kBold ? kBold : kFg, bg = kBg;
            if (attr & Terminal::kReverse) std::swap(fg, bg);
            if (bg != kBg) dl->AddRectFilled(ImVec2(x0, y), ImVec2(x1, y + lh), bg);
            dl->AddText(font, fs, ImVec2(x0, y), fg, run.c_str());
            if (attr & Terminal::kUnderline) dl->AddLine(ImVec2(x0, y + lh - 1), ImVec2(x1, y + lh - 1), fg);
            c = end;
        }
    }
    // The cursor: a block that blinks while the window has the focus.
    const int crow = terminal_.cursor_row() + scroll_;
    if (terminal_.cursor_visible() && blink_on_ && crow < Terminal::kRows) {
        const float x = origin.x + static_cast<float>(terminal_.cursor_col()) * cw;
        const float y = origin.y + static_cast<float>(crow) * lh;
        if (focused_) {
            dl->AddRectFilled(ImVec2(x, y), ImVec2(x + cw, y + lh), kFg);
            std::string ch;
            append_utf8(ch, terminal_.at(terminal_.cursor_row(), terminal_.cursor_col()).ch);
            dl->AddText(font, fs, ImVec2(x, y), kBg, ch.c_str());
        } else {
            dl->AddRect(ImVec2(x, y), ImVec2(x + cw, y + lh), kFg);
        }
    }
    ImGui::Dummy(ImVec2(cw * Terminal::kCols, lh * Terminal::kRows));
    ImGui::PopFont();
    ImGui::End();
}

}  // namespace sg::app
