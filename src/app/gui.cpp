// SPDX-License-Identifier: GPL-2.0-or-later
// The main window: the Newport picture between a menu bar and a status bar,
// with the Settings dialog and the serial terminal windows.
#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "app/display_sdl.h"
#include "app/sdl_common.h"
#include "app/session.h"
#include "app/terminal.h"
#include "app/terminal_window.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "core/log.h"
#include "imgui.h"
#include "misc/cpp/imgui_stdlib.h"
#include "net/slirp_backend.h"
#include "net/tap_backend.h"

namespace sg::app {

namespace {

#if defined(__APPLE__)
constexpr const char* kAlt = "Option";
#else
constexpr const char* kAlt = "Alt";
#endif

std::string file_name(const std::string& path) {
    return path.empty() ? std::string() : std::filesystem::path(path).filename().string();
}

std::string folder_of(const std::string& path) {
    if (path.empty()) return {};
    std::error_code ec;
    const auto dir = std::filesystem::absolute(path, ec).parent_path();
    return ec ? std::string() : dir.string() + "/";
}

std::string default_settings_path() {
    char* dir = SDL_GetPrefPath("SiliconGhost", "SiliconGhost");
    if (!dir) return "siliconghost.ini";
    std::string path = std::string(dir) + "settings.ini";
    SDL_free(dir);
    return path;
}

// What a file chooser was opened for.
enum class Pick { Prom, Nvram, NewNvram, Disk, NewDisk, AddDisc, InsertDisc, OpenSettings, SaveSettings };

constexpr SDL_DialogFileFilter kPromFilters[] = {{"PROM images", "bin;rom;img"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kNvramFilters[] = {{"NVRAM images", "bin;nvram;nvr"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kDiskFilters[] = {{"Disk images", "img;raw;dsk;hd;hdd"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kDiscFilters[] = {{"CD-ROM images", "iso;img;bin;raw;cdr"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kSettingsFilters[] = {{"Settings", "ini;conf;cfg"}, {"All files", "*"}};

constexpr std::array<double, 6> kClockPresets = {100, 133, 150, 175, 200, 250};
constexpr std::array<u64, 7> kDiskSizesMiB = {512, 1024, 2048, 4096, 8192, 16384, 32768};

// An activity light: lit for a moment whenever its counter moves.
struct Light {
    u64 last = 0;
    u64 lit_until_ns = 0;
    void update(u64 value, u64 now_ns) {
        if (value != last) lit_until_ns = now_ns + 120000000ull;
        last = value;
    }
    bool lit(u64 now_ns) const { return now_ns < lit_until_ns; }
};

class Gui {
public:
    Gui(Settings settings, GuiOptions options, volatile std::sig_atomic_t& stop, volatile std::sig_atomic_t& next_disc)
        : session_(std::move(settings)), options_(std::move(options)), stop_(stop), next_disc_(next_disc) {}
    int run();

private:
    struct PickRequest {
        Gui* gui;
        Pick what;
        std::string location;
    };
    struct PickResult {
        Pick what;
        std::string path;
    };

    // Machine
    void power_on();
    void power_off();
    void press_power_button();
    void press_volume(ip22::Ioc2::PanelButton button);
    void eject();
    void status(std::string text) {
        status_text_ = std::move(text);
        status_until_ns_ = SDL_GetTicksNS() + 4000000000ull;
    }
    void message(std::string text) {
        message_ = std::move(text);
        open_message_ = true;
    }

    // Input
    void handle_event(const SDL_Event& ev);
    bool hotkey(const SDL_KeyboardEvent& key);
    void set_captured(bool on);
    void release_keys();
    void update_title();

    // Drawing
    void frame();
    void draw_menu();
    void draw_status_bar();
    void draw_settings();
    void draw_popups();
    void update_picture();
    void update_meters(u64 now_ns);

    // Files
    void choose(Pick what, const std::string& current);
    static void on_chosen(void* userdata, const char* const* files, int filter);
    void apply_choice(const PickResult& r);
    void open_settings();
    bool accept_settings();
    void save_settings_to(const std::string& path);

    Session session_;
    GuiOptions options_;
    volatile std::sig_atomic_t& stop_;
    volatile std::sig_atomic_t& next_disc_;
    std::string settings_path_;
    bool quit_ = false, quit_after_power_off_ = false;

    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    ImGuiContext* ctx_ = nullptr;
    float scale_ = 1.0f;
    SDL_Texture* texture_ = nullptr;
    unsigned tex_w_ = 0, tex_h_ = 0;
    std::vector<u32> pixels_;
    u64 shown_version_ = 0;
    bool picture_ = false, picture_stale_ = true;
    SDL_FRect dest_{};  // where the picture is drawn, in window coordinates
    bool fullscreen_ = false, show_status_ = true;
    int pending_frames_ = 3;

    bool captured_ = false;
    unsigned buttons_ = 0;
    float dx_ = 0, dy_ = 0;
    std::bitset<SDL_SCANCODE_COUNT> keys_down_;
    std::optional<ip22::Ioc2::PanelButton> volume_held_;
    u64 volume_release_ns_ = 0;

    std::unique_ptr<AudioOutput> audio_;
    std::array<Terminal, 2> terminals_;
    std::array<std::unique_ptr<TerminalWindow>, 2> terminal_windows_;

    // Status bar
    Light disk_light_, cd_light_, net_light_;
    ip22::Indy::Activity last_activity_{};
    u64 last_instructions_ = 0, meter_time_ns_ = 0, next_status_ns_ = 0;
    double cpu_load_ = 0, gfx_mpix_ = 0, speed_ = 0, mips_ = 0;
    std::string status_text_;
    u64 status_until_ns_ = 0;

    // Dialogs
    bool settings_open_ = false, about_open_ = false, keys_open_ = false;
    bool open_message_ = false, open_quit_ = false;
    std::string message_;
    Settings edit_;
    int net_mode_ = 0;  // user, tap, none
    std::string tap_name_, mac_text_, settings_error_;
    std::vector<std::string> forward_texts_;
    int disc_selected_ = -1;
    int new_disk_size_ = 3;  // index into kDiskSizesMiB
    std::mutex picks_mutex_;
    std::deque<PickResult> picks_;
};

int Gui::run() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SG_LOG(Error, "display", "SDL_Init failed: %s", SDL_GetError());
        return -1;
    }
    settings_path_ = options_.settings_file.empty() ? default_settings_path() : options_.settings_file;
    if (!options_.power_on && options_.settings_file.empty()) {
        Settings saved;
        std::string error;
        if (std::filesystem::exists(settings_path_) && load_settings(settings_path_, saved, error))
            session_.set_settings(saved);
    }

    const SDL_DisplayID display = SDL_GetPrimaryDisplay();
    scale_ = std::clamp(SDL_GetDisplayContentScale(display), 1.0f, 4.0f);
    // Room for a 1280x1024 picture with the bars, shrunk to fit the screen.
    const float bars = 2 * (13.0f + 6.0f) * scale_;
    float w = 1280, h = 1024 + bars;
    SDL_Rect usable;
    if (SDL_GetDisplayUsableBounds(display, &usable)) {
        const float fit = std::min({1.0f, usable.w * 0.95f / w, usable.h * 0.9f / h});
        w *= fit;
        h = (h - bars) * fit + bars;
    }
    window_ = SDL_CreateWindow("SiliconGhost", static_cast<int>(w), static_cast<int>(h), SDL_WINDOW_RESIZABLE);
    renderer_ = window_ ? SDL_CreateRenderer(window_, nullptr) : nullptr;
    if (!renderer_) {
        SG_LOG(Error, "display", "cannot create window: %s", SDL_GetError());
        if (window_) SDL_DestroyWindow(window_);
        SDL_Quit();
        return -1;
    }
    ctx_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx_);
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui::GetStyle().ScaleAllSizes(scale_);
    ImFontConfig font;
    font.SizePixels = 13.0f * scale_;
    io.Fonts->AddFontDefault(&font);
    ImGui_ImplSDL3_InitForSDLRenderer(window_, renderer_);
    ImGui_ImplSDLRenderer3_Init(renderer_);

    // Serial port 1 (the console, IRIX's ttyd1) is the SCC's channel B.
    static const char* const kPortNames[2] = {"Serial Port 1", "Serial Port 2"};
    for (unsigned port = 0; port < 2; ++port) {
        const unsigned ch = port == 0 ? 1 : 0;
        terminals_[port].set_reply([this, ch](const std::string& s) {
            if (auto* m = session_.machine())
                for (char c : s) m->serial_input(ch, static_cast<u8>(c));
        });
        terminal_windows_[port] = std::make_unique<TerminalWindow>(
            std::string("SiliconGhost - ") + kPortNames[port], terminals_[port],
            [this, ch](const std::string& s) {
                if (auto* m = session_.machine())
                    for (char c : s) m->serial_input(ch, static_cast<u8>(c));
            },
            scale_);
    }
    session_.set_serial_output([this](unsigned ch, u8 byte) {
        if (ch < 2) terminals_[ch == 1 ? 0 : 1].feed(byte);
    });

    if (options_.power_on) power_on();
    else if (session_.settings().prom.empty()) open_settings();
    update_title();

    constexpr u64 kFrameNs = 1000000000ull / 60;
    u64 next_frame_ns = SDL_GetTicksNS();
    u64 next_periodic_ns = next_frame_ns + 1000000000ull;
    while (!quit_) {
        if (stop_) break;
        if (next_disc_) {
            next_disc_ = 0;
            if (auto* m = session_.machine()) m->request_next_disc();
        }
        if (auto* m = session_.machine()) {
            const u64 slice = static_cast<u64>(m->cycles_per_second() / 1000);  // 1 ms of guest time
            while (SDL_GetTicksNS() < next_frame_ns && !m->powered_off()) session_.run(slice);
            if (m->powered_off()) {
                power_off();
                status("The Indy has powered itself off");
                if (quit_after_power_off_) quit_ = true;
            }
        } else {
            SDL_WaitEventTimeout(nullptr, 50);
        }
        const u64 now = SDL_GetTicksNS();
        if (now < next_frame_ns) continue;
        next_frame_ns = std::max(next_frame_ns + kFrameNs, now);
        if (now >= next_periodic_ns) {
            next_periodic_ns = now + 1000000000ull;
            session_.periodic();
        }
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) handle_event(ev);
        {
            std::deque<PickResult> picks;
            {
                std::lock_guard<std::mutex> lock(picks_mutex_);
                picks.swap(picks_);
            }
            for (const auto& p : picks) apply_choice(p);
            if (!picks.empty()) pending_frames_ = std::max(pending_frames_, 2);
        }
        if (volume_held_ && now >= volume_release_ns_) {
            if (auto* m = session_.machine()) m->ioc().panel_button(*volume_held_, false);
            volume_held_.reset();
        }
        if (captured_ && session_.on()) {
            const int x = static_cast<int>(dx_), y = static_cast<int>(dy_);
            if (x || y) {
                dx_ -= static_cast<float>(x);
                dy_ -= static_cast<float>(y);
                session_.machine()->ioc().kbc().mouse_motion(x, -y, buttons_);  // PS/2 counts up as positive
            }
        }
        frame();
        for (auto& t : terminal_windows_) t->frame(now);
    }

    if (!options_.screenshot.empty() && session_.on() && !session_.save_screenshot(options_.screenshot))
        std::fprintf(stderr, "screenshot: no video signal\n");
    power_off();
    if (!options_.power_on && options_.settings_file.empty()) save_settings_to(settings_path_);
    for (auto& t : terminal_windows_) t.reset();
    if (texture_) SDL_DestroyTexture(texture_);
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext(ctx_);
    SDL_DestroyRenderer(renderer_);
    SDL_DestroyWindow(window_);
    SDL_Quit();
    return 0;
}

// --- Machine ---------------------------------------------------------------

void Gui::power_on() {
    if (session_.on()) return;
    std::string error;
    if (!session_.power_on(error)) {
        message("The Indy cannot be powered on:\n\n" + error);
        return;
    }
    const Settings& s = session_.settings();
    if (s.audio || s.audio_in) audio_ = start_audio(*session_.machine(), {s.audio, s.audio_in});
    for (auto& t : terminals_) t.reset();
    picture_stale_ = true;
    meter_time_ns_ = 0;
    // Without a graphics board the console is the first serial port.
    if (!s.graphics) terminal_windows_[0]->open();
    update_title();
    pending_frames_ = 3;
}

void Gui::power_off() {
    if (!session_.on()) return;
    set_captured(false);
    release_keys();
    volume_held_.reset();
    audio_.reset();
    session_.power_off();
    picture_ = false;
    picture_stale_ = true;
    update_title();
    pending_frames_ = 3;
}

void Gui::press_power_button() {
    if (!session_.on()) {
        power_on();
        return;
    }
    session_.machine()->ioc().panel_button(ip22::Ioc2::PanelButton::Power, true);
    status("Power button pressed: the system shuts down and powers off");
}

void Gui::press_volume(ip22::Ioc2::PanelButton button) {
    auto* m = session_.machine();
    if (!m) return;
    if (volume_held_) m->ioc().panel_button(*volume_held_, false);
    m->ioc().panel_button(button, true);
    volume_held_ = button;
    volume_release_ns_ = SDL_GetTicksNS() + 150000000ull;
}

void Gui::eject() {
    auto* m = session_.machine();
    if (!m || !m->has_cdrom_drive()) return;
    if (m->disc_in_drive() < 0) status("The CD-ROM drive is empty");
    else if (!m->eject_disc()) status("The disc is locked in by the system; eject it from IRIX (eject command)");
    else status("Disc ejected");
}

// --- Input -------------------------------------------------------------------

void Gui::update_title() {
    std::string title = "SiliconGhost";
    if (!session_.on()) title += " - powered off";
    else if (captured_) title += std::string(" - Ctrl+") + kAlt + "+G releases the mouse";
    else if (picture_) title += " - click the screen to capture the mouse";
    SDL_SetWindowTitle(window_, title.c_str());
}

void Gui::set_captured(bool on) {
    if (on == captured_) return;
    captured_ = on;
    SDL_SetWindowRelativeMouseMode(window_, on);
    if (!on && buttons_ && session_.on()) {
        buttons_ = 0;
        session_.machine()->ioc().kbc().mouse_motion(0, 0, 0);
    }
    buttons_ = 0;
    dx_ = dy_ = 0;
    update_title();
}

void Gui::release_keys() {
    auto* m = session_.machine();
    for (unsigned sc = 0; sc < keys_down_.size(); ++sc)
        if (keys_down_[sc] && m) send_key(*m, static_cast<SDL_Scancode>(sc), false);
    keys_down_.reset();
}

// Ctrl+Alt combinations the window keeps for itself; true if consumed.
bool Gui::hotkey(const SDL_KeyboardEvent& key) {
    if (!(key.mod & SDL_KMOD_CTRL) || !(key.mod & SDL_KMOD_ALT)) return false;
    const bool down = key.down && !key.repeat;
    switch (key.scancode) {
        case SDL_SCANCODE_G:
            if (down) set_captured(false);
            return true;
        case SDL_SCANCODE_E:
            if (down && session_.on()) session_.machine()->request_next_disc();
            return true;
        case SDL_SCANCODE_F:
            if (down) {
                fullscreen_ = !fullscreen_;
                SDL_SetWindowFullscreen(window_, fullscreen_);
            }
            return true;
        case SDL_SCANCODE_P:
            if (down) press_power_button();
            return true;
        default: return false;
    }
}

void Gui::handle_event(const SDL_Event& ev) {
    for (auto& t : terminal_windows_)
        if (t->handle(ev)) return;
    if (SDL_Window* w = SDL_GetWindowFromEvent(&ev); w && w != window_) return;
    // ImGui takes a press and its release in separate frames, then acts.
    pending_frames_ = std::max(pending_frames_, 6);
    ImGuiIO& io = ImGui::GetIO();
    auto* machine = session_.machine();

    switch (ev.type) {
        case SDL_EVENT_QUIT:
        case SDL_EVENT_WINDOW_CLOSE_REQUESTED:
            if (session_.on()) open_quit_ = true;
            else quit_ = true;
            return;
        case SDL_EVENT_WINDOW_FOCUS_LOST:
            set_captured(false);
            release_keys();
            break;
        case SDL_EVENT_KEY_DOWN:
        case SDL_EVENT_KEY_UP: {
            if (hotkey(ev.key)) return;
            const unsigned sc = ev.key.scancode;
            const bool to_guest = machine && (captured_ || !io.WantCaptureKeyboard);
            if (ev.key.down && to_guest && !ev.key.repeat && sc < keys_down_.size()) {
                send_key(*machine, ev.key.scancode, true);
                keys_down_[sc] = true;
            } else if (!ev.key.down && sc < keys_down_.size() && keys_down_[sc]) {
                if (machine) send_key(*machine, ev.key.scancode, false);
                keys_down_[sc] = false;
            }
            if (captured_) return;
            break;
        }
        case SDL_EVENT_MOUSE_MOTION:
            if (captured_) {
                // Motion in guest pixels, whatever the window's scale.
                const float k = dest_.w > 0 ? static_cast<float>(tex_w_) / dest_.w : 1.0f;
                dx_ += ev.motion.xrel * k;
                dy_ += ev.motion.yrel * k;
                return;
            }
            break;
        case SDL_EVENT_MOUSE_BUTTON_DOWN:
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (captured_) {
                const unsigned bit = ev.button.button == SDL_BUTTON_LEFT    ? 1u
                                     : ev.button.button == SDL_BUTTON_RIGHT ? 2u
                                     : ev.button.button == SDL_BUTTON_MIDDLE ? 4u
                                                                             : 0u;
                if (bit && machine) {
                    buttons_ = ev.button.down ? (buttons_ | bit) : (buttons_ & ~bit);
                    machine->ioc().kbc().mouse_motion(0, 0, buttons_);
                }
                return;
            }
            if (ev.button.down && ev.button.button == SDL_BUTTON_LEFT && machine && picture_ && !io.WantCaptureMouse &&
                ev.button.x >= dest_.x && ev.button.x < dest_.x + dest_.w && ev.button.y >= dest_.y &&
                ev.button.y < dest_.y + dest_.h) {
                set_captured(true);
                return;
            }
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            if (captured_) return;
            break;
        default: break;
    }
    ImGui_ImplSDL3_ProcessEvent(&ev);
}

// --- Drawing -------------------------------------------------------------------

void Gui::update_picture() {
    auto* machine = session_.machine();
    ip22::Newport* newport = machine ? machine->newport() : nullptr;
    if (!newport) {
        picture_ = false;
        return;
    }
    if (!picture_stale_ && newport->display_version() == shown_version_) return;
    picture_stale_ = false;
    shown_version_ = newport->display_version();
    unsigned w = 0, h = 0;
    const bool had = picture_;
    picture_ = newport->render(pixels_, w, h);
    if (picture_ != had) update_title();
    if (!picture_) return;
    if (!texture_ || w != tex_w_ || h != tex_h_) {
        if (texture_) SDL_DestroyTexture(texture_);
        texture_ = SDL_CreateTexture(renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
                                     static_cast<int>(w), static_cast<int>(h));
        tex_w_ = w;
        tex_h_ = h;
    }
    SDL_UpdateTexture(texture_, nullptr, pixels_.data(), static_cast<int>(w * sizeof(u32)));
    pending_frames_ = std::max(pending_frames_, 1);
}

void Gui::update_meters(u64 now) {
    auto* m = session_.machine();
    if (!m) return;
    const auto a = m->activity();
    disk_light_.update(a.disk, now);
    cd_light_.update(a.cdrom, now);
    net_light_.update(a.net_tx + a.net_rx, now);
    const u64 instructions = m->cpu().instructions();
    if (meter_time_ns_ == 0) {
        meter_time_ns_ = now;
        last_activity_ = a;
        last_instructions_ = instructions;
        return;
    }
    const double dt = static_cast<double>(now - meter_time_ns_) * 1e-9;
    if (dt < 0.5) return;
    const double cycles = static_cast<double>(a.cycles - last_activity_.cycles);
    const double idle = static_cast<double>(a.idle_cycles - last_activity_.idle_cycles);
    cpu_load_ = cycles > 0 ? std::clamp(1.0 - idle / cycles, 0.0, 1.0) : 0.0;
    gfx_mpix_ = static_cast<double>(a.pixels - last_activity_.pixels) / dt / 1e6;
    speed_ = cycles / m->cycles_per_second() / dt;
    mips_ = static_cast<double>(instructions - last_instructions_) / dt / 1e6;
    meter_time_ns_ = now;
    last_activity_ = a;
    last_instructions_ = instructions;
}

void Gui::frame() {
    const u64 now = SDL_GetTicksNS();
    update_picture();
    if (session_.on()) {
        update_meters(now);
        if (now >= next_status_ns_) {  // lights go out and meters move: 10 frames a second
            next_status_ns_ = now + 100000000ull;
            pending_frames_ = std::max(pending_frames_, 1);
        }
    }
    if (status_until_ns_ && now >= status_until_ns_) {
        status_until_ns_ = 0;
        status_text_.clear();
        pending_frames_ = std::max(pending_frames_, 1);
    }
    if (pending_frames_ <= 0) return;
    --pending_frames_;

    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    ImGuiIO& io = ImGui::GetIO();
    float top = 0, bottom = io.DisplaySize.y;
    if (!fullscreen_) {
        draw_menu();
        top = ImGui::GetFrameHeight();
        if (show_status_) {
            bottom -= ImGui::GetFrameHeight();
            draw_status_bar();
        }
    }
    // The picture, as large as fits with its shape kept.
    const float area_w = io.DisplaySize.x, area_h = std::max(1.0f, bottom - top);
    if (picture_ && tex_w_ && tex_h_) {
        const float k = std::min(area_w / static_cast<float>(tex_w_), area_h / static_cast<float>(tex_h_));
        dest_.w = static_cast<float>(tex_w_) * k;
        dest_.h = static_cast<float>(tex_h_) * k;
        dest_.x = std::floor((area_w - dest_.w) / 2);
        dest_.y = top + std::floor((area_h - dest_.h) / 2);
    } else {
        dest_ = {0, top, area_w, area_h};
        const char* text = !session_.on() ? "Powered off - Machine > Power On"
                           : session_.settings().graphics
                               ? "No video signal (the console may be on View > Serial Port 1)"
                               : "No graphics board: the console is on View > Serial Port 1";
        const ImVec2 size = ImGui::CalcTextSize(text);
        ImGui::GetBackgroundDrawList()->AddText(
            ImVec2(std::floor((area_w - size.x) / 2), std::floor(top + (area_h - size.y) / 2)),
            IM_COL32(140, 140, 140, 255), text);
    }
    draw_settings();
    draw_popups();
    ImGui::Render();

    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    if (picture_ && texture_) {
        const ImVec2 fb = io.DisplayFramebufferScale;
        const SDL_FRect px{dest_.x * fb.x, dest_.y * fb.y, dest_.w * fb.x, dest_.h * fb.y};
        SDL_RenderTexture(renderer_, texture_, nullptr, &px);
    }
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer_);
    SDL_RenderPresent(renderer_);
}

void Gui::draw_menu() {
    if (!ImGui::BeginMainMenuBar()) return;
    auto* m = session_.machine();
    const std::string ctrl_alt = std::string("Ctrl+") + kAlt + "+";
    if (ImGui::BeginMenu("Machine")) {
        if (!m) {
            if (ImGui::MenuItem("Power On", (ctrl_alt + "P").c_str())) power_on();
        } else {
            if (ImGui::MenuItem("Power Button", (ctrl_alt + "P").c_str())) press_power_button();
            ImGui::SetItemTooltip("As the button on the front: the system shuts down and powers off");
            if (ImGui::MenuItem("Power Off Now")) power_off();
            ImGui::SetItemTooltip("Cuts the power at once, without shutting down (the NVRAM is kept)");
            if (ImGui::MenuItem("Reset")) m->reset();
            ImGui::SetItemTooltip("Restarts the machine from the boot PROM at once");
            ImGui::Separator();
            if (ImGui::MenuItem("Volume Up")) press_volume(ip22::Ioc2::PanelButton::VolumeUp);
            if (ImGui::MenuItem("Volume Down")) press_volume(ip22::Ioc2::PanelButton::VolumeDown);
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Settings...")) open_settings();
        if (ImGui::MenuItem("Open Settings File...")) choose(Pick::OpenSettings, settings_path_);
        if (ImGui::MenuItem("Save Settings As...")) choose(Pick::SaveSettings, settings_path_);
        ImGui::Separator();
        if (ImGui::MenuItem("Quit")) {
            if (m) open_quit_ = true;
            else quit_ = true;
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Media")) {
        ImGui::TextDisabled("CD-ROM (SCSI ID 4)");
        const bool drive = m && m->has_cdrom_drive();
        if (ImGui::MenuItem("Eject", nullptr, false, drive && m->disc_in_drive() >= 0)) eject();
        if (ImGui::MenuItem("Force Eject", nullptr, false, drive && m->disc_in_drive() >= 0)) {
            m->force_eject_disc();
            status("Disc ejected while the system had it locked in");
        }
        ImGui::SetItemTooltip("As a paper clip in the drive's emergency eject hole: also when the system has locked the disc in");
        if (ImGui::MenuItem("Insert Disc...", nullptr, false, drive)) choose(Pick::InsertDisc, {});
        if (ImGui::MenuItem("Next Disc in Set", (ctrl_alt + "E").c_str(), false, drive && !m->discs().empty()))
            m->request_next_disc();
        const std::vector<std::string>& discs = m ? m->discs() : session_.settings().cdroms;
        if (!discs.empty()) {
            ImGui::Separator();
            for (std::size_t i = 0; i < discs.size(); ++i) {
                const bool in = m && m->disc_in_drive() == static_cast<int>(i);
                ImGui::PushID(static_cast<int>(i));
                if (ImGui::MenuItem(file_name(discs[i]).c_str(), nullptr, in, drive) && !in) {
                    if (m->disc_in_drive() >= 0 && !m->eject_disc())
                        status("The disc is locked in by the system; eject it from IRIX first");
                    else
                        m->select_disc(i);
                }
                ImGui::SetItemTooltip("%s", discs[i].c_str());
                ImGui::PopID();
            }
        }
        ImGui::Separator();
        ImGui::TextDisabled("Hard disk (SCSI ID 1)");
        const std::string& disk = session_.settings().disk;
        ImGui::MenuItem(disk.empty() ? "(none)" : file_name(disk).c_str(), nullptr, false, false);
        if (ImGui::MenuItem("Change Disk...", nullptr, false, !m)) open_settings();
        if (m) ImGui::SetItemTooltip("Power off first");
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("View")) {
        for (unsigned port = 0; port < 2; ++port) {
            const bool open = terminal_windows_[port]->is_open();
            if (ImGui::MenuItem(port == 0 ? "Serial Port 1 (console)" : "Serial Port 2", nullptr, open)) {
                if (open) terminal_windows_[port]->close();
                else terminal_windows_[port]->open();
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem("Status Bar", nullptr, show_status_)) show_status_ = !show_status_;
        if (ImGui::MenuItem("Full Screen", (ctrl_alt + "F").c_str(), fullscreen_)) {
            fullscreen_ = !fullscreen_;
            SDL_SetWindowFullscreen(window_, fullscreen_);
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Help")) {
        if (ImGui::MenuItem("Keyboard and Mouse")) keys_open_ = true;
        if (ImGui::MenuItem("About SiliconGhost")) about_open_ = true;
        ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
}

void Gui::draw_status_bar() {
    const u64 now = SDL_GetTicksNS();
    ImGuiIO& io = ImGui::GetIO();
    const float h = ImGui::GetFrameHeight();
    ImGui::SetNextWindowPos(ImVec2(0, io.DisplaySize.y - h));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, h));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8 * scale_, ImGui::GetStyle().FramePadding.y));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0);
    ImGui::Begin("##status", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoBringToFrontOnFocus);
    auto* m = session_.machine();
    auto light = [&](const char* label, bool lit, ImU32 colour, const char* tip) {
        const float r = ImGui::GetTextLineHeight() * 0.32f;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const ImVec2 c(p.x + r, p.y + ImGui::GetTextLineHeight() * 0.5f);
        ImGui::GetWindowDrawList()->AddCircleFilled(c, r, lit ? colour : IM_COL32(60, 60, 60, 255));
        ImGui::Dummy(ImVec2(r * 2, ImGui::GetTextLineHeight()));
        ImGui::SameLine(0, 4 * scale_);
        ImGui::TextUnformatted(label);
        if (tip) ImGui::SetItemTooltip("%s", tip);
        ImGui::SameLine(0, 14 * scale_);
    };
    light("Power", m != nullptr, IM_COL32(60, 220, 60, 255), nullptr);
    light("Disk", m && disk_light_.lit(now), IM_COL32(255, 170, 0, 255), "Hard disk activity");
    std::string cd = "CD";
    if (m && m->has_cdrom_drive()) {
        const int d = m->disc_in_drive();
        cd += ": " + (d >= 0 ? file_name(m->discs()[static_cast<std::size_t>(d)]) : std::string("empty"));
        if (cd.size() > 34) cd = cd.substr(0, 31) + "...";
    }
    light(cd.c_str(), m && cd_light_.lit(now), IM_COL32(255, 170, 0, 255), "CD-ROM activity");
    light("Net", m && net_light_.lit(now), IM_COL32(60, 200, 255, 255), "Ethernet frames sent or received");
    if (m) {
        ImGui::Text("CPU %3.0f%%", cpu_load_ * 100);
        ImGui::SetItemTooltip("Time the CPU is not idle (from outside the system)");
        ImGui::SameLine(0, 14 * scale_);
        ImGui::Text("GFX %5.1f Mpix/s", gfx_mpix_);
        ImGui::SetItemTooltip("Pixels drawn by the Newport's REX3 per second");
        ImGui::SameLine(0, 14 * scale_);
        ImGui::Text("Speed %3.0f%%", speed_ * 100);
        ImGui::SetItemTooltip("Guest time against real time; %.0f million instructions a second", mips_);
        ImGui::SameLine(0, 14 * scale_);
    }
    if (!status_text_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "%s", status_text_.c_str());
        ImGui::SameLine();
    }
    const Settings& s = session_.settings();
    const CpuModel* model = find_cpu_model(s.cpu);
    char right[96];
    std::snprintf(right, sizeof right, "%s %.0f MHz, %u MiB", model ? model->label : s.cpu.c_str(), s.cpu_mhz, s.ram_mb);
    // Only the model name, before any detail in parentheses.
    std::string text = right;
    if (const auto paren = text.find(" ("); paren != std::string::npos) {
        const auto close = text.find(')', paren);
        text.erase(paren, close - paren + 1);
    }
    const float tw = ImGui::CalcTextSize(text.c_str()).x;
    const float x = ImGui::GetWindowWidth() - tw - 8 * scale_;
    if (x > ImGui::GetCursorPosX()) {
        ImGui::SetCursorPosX(x);
        ImGui::TextDisabled("%s", text.c_str());
    } else {
        ImGui::NewLine();
    }
    ImGui::End();
    ImGui::PopStyleVar(2);
}

// --- Settings ------------------------------------------------------------------

void Gui::open_settings() {
    edit_ = session_.settings();
    if (auto* m = session_.machine()) edit_.cdroms = m->discs();
    net_mode_ = edit_.net == "none" ? 2 : edit_.net.rfind("tap:", 0) == 0 ? 1 : 0;
    tap_name_ = net_mode_ == 1 ? edit_.net.substr(4) : std::string();
    // A connection this build or host lacks (a file from another machine) shows as none.
    if ((net_mode_ == 0 && !net::slirp_available()) || (net_mode_ == 1 && !net::tap_available())) net_mode_ = 2;
    mac_text_ = edit_.mac ? mac_text(*edit_.mac) : std::string();
    forward_texts_.clear();
    for (const auto& f : edit_.forwards) forward_texts_.push_back(forward_text(f));
    settings_error_.clear();
    disc_selected_ = -1;
    settings_open_ = true;
}

bool Gui::accept_settings() {
    Settings s = edit_;
    s.net = net_mode_ == 2 ? "none" : net_mode_ == 1 ? "tap:" + tap_name_ : "user";
    if (net_mode_ == 1 && tap_name_.empty()) {
        settings_error_ = "Network: enter the TAP interface's name";
        return false;
    }
    if (mac_text_.empty()) {
        s.mac.reset();
    } else {
        std::array<u8, 6> mac{};
        if (!parse_mac(mac_text_, mac)) {
            settings_error_ = "Network: the Ethernet address should look like 08:00:69:0c:5e:01";
            return false;
        }
        s.mac = mac;
    }
    s.forwards.clear();
    for (const auto& t : forward_texts_) {
        if (t.empty()) continue;
        net::PortForward f;
        if (!parse_forward(t, f)) {
            settings_error_ = "Network: port forward \"" + t + "\" should look like tcp:2222:22";
            return false;
        }
        s.forwards.push_back(f);
    }
    if (auto* m = session_.machine()) s.cdroms = m->discs();  // the drive's set is kept by the drive
    session_.set_settings(s);
    save_settings_to(settings_path_);
    settings_error_.clear();
    if (session_.on()) status("Settings saved; most take effect at the next power-on");
    return true;
}

void Gui::save_settings_to(const std::string& path) {
    Settings s = session_.settings();
    if (auto* m = session_.machine()) s.cdroms = m->discs();
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    std::string error;
    if (!save_settings(path, s, error)) message("The settings could not be saved:\n\n" + error);
}

void Gui::draw_settings() {
    if (!settings_open_) return;
    ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowSize(ImVec2(620 * scale_, 470 * scale_), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x / 2, io.DisplaySize.y / 2), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (!ImGui::Begin("Settings", &settings_open_, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return;
    }
    const bool running = session_.on();
    if (running) ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "The machine is on: changes take effect at the next power-on.");
    const float label_w = 150 * scale_;
    auto label = [&](const char* text) {
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(text);
        ImGui::SameLine(label_w);
    };
    auto path_row = [&](const char* text, const char* id, std::string& path, Pick browse, std::optional<Pick> create) {
        ImGui::PushID(id);
        label(text);
        const float buttons = (create ? 140.0f : 70.0f) * scale_;
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - buttons);
        ImGui::InputText("##path", &path);
        ImGui::SameLine();
        if (ImGui::Button("Browse...")) choose(browse, path);
        if (create) {
            ImGui::SameLine();
            if (ImGui::Button("New...")) choose(*create, path);
        }
        ImGui::PopID();
    };

    const float footer = ImGui::GetFrameHeightWithSpacing() * (settings_error_.empty() ? 1.5f : 2.5f);
    ImGui::BeginChild("##tabs", ImVec2(0, -footer));
    if (ImGui::BeginTabBar("##settings")) {
        if (ImGui::BeginTabItem("Machine")) {
            label("Processor");
            ImGui::SetNextItemWidth(-1);
            const CpuModel* current = find_cpu_model(edit_.cpu);
            if (ImGui::BeginCombo("##cpu", current ? current->label : edit_.cpu.c_str())) {
                for (const auto& model : cpu_models())
                    if (ImGui::Selectable(model.label, edit_.cpu == model.id)) edit_.cpu = model.id;
                ImGui::EndCombo();
            }
            label("Clock (MHz)");
            ImGui::SetNextItemWidth(120 * scale_);
            ImGui::InputDouble("##mhz", &edit_.cpu_mhz, 0, 0, "%.0f");
            edit_.cpu_mhz = std::clamp(edit_.cpu_mhz, 10.0, 1000.0);
            for (double mhz : kClockPresets) {
                ImGui::SameLine();
                char text[8];
                std::snprintf(text, sizeof text, "%.0f", mhz);
                if (ImGui::SmallButton(text)) edit_.cpu_mhz = mhz;
            }
            label("Memory");
            ImGui::SetNextItemWidth(120 * scale_);
            const std::string ram = std::to_string(edit_.ram_mb) + " MiB";
            if (ImGui::BeginCombo("##ram", ram.c_str())) {
                for (unsigned mb = 16; mb <= 256; mb += 16)
                    if (ImGui::Selectable((std::to_string(mb) + " MiB").c_str(), edit_.ram_mb == mb)) edit_.ram_mb = mb;
                ImGui::EndCombo();
            }
            label("Graphics");
            ImGui::SetNextItemWidth(-1);
            int gfx = edit_.graphics ? 0 : 1;
            ImGui::Combo("##gfx", &gfx, "Newport XL8 (8-bit)\0None (serial console only)\0");
            edit_.graphics = gfx == 0;
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Storage")) {
            path_row("Boot PROM", "prom", edit_.prom, Pick::Prom, std::nullopt);
            path_row("NVRAM", "nvram", edit_.nvram, Pick::Nvram, Pick::NewNvram);
            ImGui::SetCursorPosX(label_w);
            ImGui::TextDisabled("Saved whenever the system changes it; a new one starts blank");
            ImGui::Spacing();
            path_row("Hard disk (ID 1)", "disk", edit_.disk, Pick::Disk, Pick::NewDisk);
            ImGui::SetCursorPosX(label_w);
            ImGui::AlignTextToFramePadding();
            ImGui::TextDisabled("Size of a new disk:");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(100 * scale_);
            auto size_text = [](u64 mib) {
                return mib >= 1024 ? std::to_string(mib / 1024) + " GB" : std::to_string(mib) + " MB";
            };
            if (ImGui::BeginCombo("##newsize", size_text(kDiskSizesMiB[static_cast<std::size_t>(new_disk_size_)]).c_str())) {
                for (std::size_t i = 0; i < kDiskSizesMiB.size(); ++i)
                    if (ImGui::Selectable(size_text(kDiskSizesMiB[i]).c_str(), new_disk_size_ == static_cast<int>(i)))
                        new_disk_size_ = static_cast<int>(i);
                ImGui::EndCombo();
            }
            ImGui::Spacing();
            label("CD-ROM discs (ID 4)");
            ImGui::BeginGroup();
            ImGui::BeginDisabled(running);
            if (ImGui::BeginListBox("##discs", ImVec2(-1, 6 * ImGui::GetTextLineHeightWithSpacing()))) {
                for (std::size_t i = 0; i < edit_.cdroms.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    if (ImGui::Selectable(edit_.cdroms[i].c_str(), disc_selected_ == static_cast<int>(i)))
                        disc_selected_ = static_cast<int>(i);
                    ImGui::PopID();
                }
                ImGui::EndListBox();
            }
            if (ImGui::Button("Add...")) choose(Pick::AddDisc, edit_.cdroms.empty() ? std::string() : edit_.cdroms.back());
            const bool sel = disc_selected_ >= 0 && disc_selected_ < static_cast<int>(edit_.cdroms.size());
            const auto at = static_cast<std::size_t>(std::max(disc_selected_, 0));
            ImGui::SameLine();
            if (ImGui::Button("Remove") && sel) {
                edit_.cdroms.erase(edit_.cdroms.begin() + static_cast<std::ptrdiff_t>(at));
                disc_selected_ = -1;
            }
            ImGui::SameLine();
            if (ImGui::Button("Up") && sel && at > 0) {
                std::swap(edit_.cdroms[at], edit_.cdroms[at - 1]);
                --disc_selected_;
            }
            ImGui::SameLine();
            if (ImGui::Button("Down") && sel && at + 1 < edit_.cdroms.size()) {
                std::swap(edit_.cdroms[at], edit_.cdroms[at + 1]);
                ++disc_selected_;
            }
            ImGui::EndDisabled();
            ImGui::PushTextWrapPos(0);
            ImGui::TextDisabled(running ? "While running, change discs from the Media menu."
                                        : "The first disc is in the drive at power-on; ejecting it moves to the next.");
            ImGui::PopTextWrapPos();
            ImGui::EndGroup();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Network")) {
            label("Connection");
            ImGui::SetNextItemWidth(-1);
            // Only the connections this build and host can make.
            std::vector<std::pair<int, const char*>> modes;
            if (net::slirp_available()) modes.emplace_back(0, "User-mode NAT through the host");
            if (net::tap_available()) modes.emplace_back(1, "TAP interface");
            modes.emplace_back(2, "Not connected");
            const char* current = "Not connected";
            for (const auto& [mode, text] : modes)
                if (mode == net_mode_) current = text;
            if (ImGui::BeginCombo("##net", current)) {
                for (const auto& [mode, text] : modes)
                    if (ImGui::Selectable(text, mode == net_mode_)) net_mode_ = mode;
                ImGui::EndCombo();
            }
            if (net_mode_ == 0) {
                ImGui::SetCursorPosX(label_w);
                ImGui::TextDisabled("The guest is 10.0.2.15; the host and gateway 10.0.2.2, DNS 10.0.2.3");
            }
            if (net_mode_ == 1) {
                label("TAP interface");
                ImGui::SetNextItemWidth(200 * scale_);
                ImGui::InputText("##tap", &tap_name_);
            }
            label("Ethernet address");
            ImGui::SetNextItemWidth(200 * scale_);
            ImGui::InputTextWithHint("##mac", "kept in NVRAM", &mac_text_);
            if (net_mode_ == 0) {
                label("Port forwards");
                ImGui::BeginGroup();
                for (std::size_t i = 0; i < forward_texts_.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    ImGui::SetNextItemWidth(200 * scale_);
                    ImGui::InputTextWithHint("##fw", "tcp:HOSTPORT:GUESTPORT", &forward_texts_[i]);
                    ImGui::SameLine();
                    if (ImGui::Button("Remove")) {
                        forward_texts_.erase(forward_texts_.begin() + static_cast<std::ptrdiff_t>(i));
                        ImGui::PopID();
                        break;
                    }
                    ImGui::PopID();
                }
                if (ImGui::Button("Add")) forward_texts_.emplace_back("tcp:2222:22");
                ImGui::TextDisabled("A port on the host's loopback interface, passed to the guest");
                ImGui::EndGroup();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Performance")) {
            label("Timing");
            ImGui::SetNextItemWidth(-1);
            int timing = edit_.timing == Pacer::Mode::Realtime ? 0 : edit_.timing == Pacer::Mode::Cycle ? 1 : 2;
            ImGui::Combo("##timing", &timing,
                         "Real time: guest clocks follow the host's\0Cycle: one cycle per instruction, reproducible\0"
                         "Maximum: no waiting, guest time runs ahead\0");
            edit_.timing = timing == 0 ? Pacer::Mode::Realtime : timing == 1 ? Pacer::Mode::Cycle : Pacer::Mode::Max;
            if (timing == 0) {
                label("Speed limit");
                ImGui::SetNextItemWidth(200 * scale_);
                float limit = static_cast<float>(edit_.max_speed);
                ImGui::SliderFloat("##maxspeed", &limit, 1.0f, 4.0f, "%.1fx the real CPU");
                edit_.max_speed = limit;
                ImGui::SameLine();
                ImGui::TextDisabled("(IRIX 5.3 fails to boot beyond about 2.9)");
            }
            ImGui::Spacing();
            ImGui::Checkbox("Skip idle loops", &edit_.idle_skip);
            if (mips::R4000::jit_available())
                ImGui::Checkbox("Recompiler (translate guest code to host code)", &edit_.jit);
            ImGui::Checkbox("Block cache (decode guest code once)", &edit_.block_cache);
            ImGui::Spacing();
            label("Log level");
            ImGui::SetNextItemWidth(120 * scale_);
            static const char* const kLevels[] = {"error", "warn", "info", "debug", "trace"};
            if (ImGui::BeginCombo("##log", edit_.log.c_str())) {
                for (const char* l : kLevels)
                    if (ImGui::Selectable(l, edit_.log == l)) edit_.log = l;
                ImGui::EndCombo();
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Audio")) {
            ImGui::Checkbox("Play the machine's audio output", &edit_.audio);
            ImGui::Checkbox("Let the machine record from the host's audio input", &edit_.audio_in);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::EndChild();
    if (!settings_error_.empty()) ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", settings_error_.c_str());
    ImGui::Separator();
    if (ImGui::Button("OK", ImVec2(90 * scale_, 0)) && accept_settings()) settings_open_ = false;
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(90 * scale_, 0))) settings_open_ = false;
    if (!running) {
        ImGui::SameLine();
        if (ImGui::Button("OK and Power On", ImVec2(130 * scale_, 0)) && accept_settings()) {
            settings_open_ = false;
            power_on();
        }
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Saved to %s", file_name(settings_path_).c_str());
    ImGui::SetItemTooltip("%s", settings_path_.c_str());
    ImGui::End();
}

void Gui::draw_popups() {
    ImGuiIO& io = ImGui::GetIO();
    const ImVec2 centre(io.DisplaySize.x / 2, io.DisplaySize.y / 2);
    if (open_message_) {
        ImGui::OpenPopup("SiliconGhost##message");
        open_message_ = false;
    }
    if (open_quit_) {
        ImGui::OpenPopup("Quit SiliconGhost?");
        open_quit_ = false;
    }
    if (about_open_) {
        ImGui::OpenPopup("About SiliconGhost");
        about_open_ = false;
    }
    if (keys_open_) {
        ImGui::OpenPopup("Keyboard and Mouse");
        keys_open_ = false;
    }
    const auto flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings;
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("SiliconGhost##message", nullptr, flags)) {
        ImGui::PushTextWrapPos(480 * scale_);
        ImGui::TextUnformatted(message_.c_str());
        ImGui::PopTextWrapPos();
        if (ImGui::Button("OK", ImVec2(90 * scale_, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Quit SiliconGhost?", nullptr, flags)) {
        ImGui::TextUnformatted("The Indy is still on.");
        ImGui::TextUnformatted("Pressing its power button lets the system shut down first.");
        ImGui::Spacing();
        if (ImGui::Button("Shut Down and Quit")) {
            quit_after_power_off_ = true;
            press_power_button();
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Quit Now")) {
            quit_ = true;
            ImGui::CloseCurrentPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("About SiliconGhost", nullptr, flags)) {
        ImGui::TextUnformatted("SiliconGhost: an SGI Indy (IP22) emulator");
        ImGui::TextUnformatted("GNU General Public License, version 2 or later");
        ImGui::Spacing();
        ImGui::TextDisabled("Menus and dialogs: Dear ImGui (MIT licence)");
        ImGui::TextDisabled("SGI, Indy and IRIX are trademarks of their owners.");
        if (ImGui::Button("OK", ImVec2(90 * scale_, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Keyboard and Mouse", nullptr, flags)) {
        const std::string ca = std::string("Ctrl+") + kAlt + "+";
        ImGui::TextUnformatted("Click the screen to capture the mouse for the Indy.");
        ImGui::Spacing();
        if (ImGui::BeginTable("##keys", 2, ImGuiTableFlags_SizingFixedFit)) {
            const std::pair<std::string, const char*> rows[] = {
                {ca + "G", "Release the mouse"},
                {ca + "P", "Power button (power on when off)"},
                {ca + "E", "Next CD-ROM disc in the set"},
                {ca + "F", "Full screen on/off"},
            };
            for (const auto& [keys, what] : rows) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(keys.c_str());
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(what);
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        ImGui::TextDisabled("Keys are sent by position, as a PS/2 keyboard sends them.");
        ImGui::TextDisabled("In a serial terminal window, Ctrl+Shift+C/V copy and paste (Cmd+C/V on a Mac).");
        if (ImGui::Button("OK", ImVec2(90 * scale_, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

// --- Files -------------------------------------------------------------------

void Gui::choose(Pick what, const std::string& current) {
    auto* req = new PickRequest{this, what, folder_of(current)};
    const char* location = req->location.empty() ? nullptr : req->location.c_str();
    switch (what) {
        case Pick::Prom: SDL_ShowOpenFileDialog(on_chosen, req, window_, kPromFilters, 2, location, false); break;
        case Pick::Nvram: SDL_ShowOpenFileDialog(on_chosen, req, window_, kNvramFilters, 2, location, false); break;
        case Pick::NewNvram: SDL_ShowSaveFileDialog(on_chosen, req, window_, kNvramFilters, 2, location); break;
        case Pick::Disk: SDL_ShowOpenFileDialog(on_chosen, req, window_, kDiskFilters, 2, location, false); break;
        case Pick::NewDisk: SDL_ShowSaveFileDialog(on_chosen, req, window_, kDiskFilters, 2, location); break;
        case Pick::AddDisc: SDL_ShowOpenFileDialog(on_chosen, req, window_, kDiscFilters, 2, location, true); break;
        case Pick::InsertDisc: SDL_ShowOpenFileDialog(on_chosen, req, window_, kDiscFilters, 2, location, false); break;
        case Pick::OpenSettings: SDL_ShowOpenFileDialog(on_chosen, req, window_, kSettingsFilters, 2, location, false); break;
        case Pick::SaveSettings: SDL_ShowSaveFileDialog(on_chosen, req, window_, kSettingsFilters, 2, location); break;
    }
}

// May be called on another thread: the choice is queued for the main loop.
void Gui::on_chosen(void* userdata, const char* const* files, int) {
    std::unique_ptr<PickRequest> req(static_cast<PickRequest*>(userdata));
    if (!files) {
        SG_LOG(Warn, "display", "file chooser: %s", SDL_GetError());
        return;
    }
    std::lock_guard<std::mutex> lock(req->gui->picks_mutex_);
    for (; *files; ++files) req->gui->picks_.push_back({req->what, *files});
}

void Gui::apply_choice(const PickResult& r) {
    std::string error;
    switch (r.what) {
        case Pick::Prom: edit_.prom = r.path; break;
        case Pick::Nvram: edit_.nvram = r.path; break;
        case Pick::NewNvram:
            if (create_blank_image(r.path, kNvramBytes, error)) edit_.nvram = r.path;
            else message("No new NVRAM was made:\n\n" + error + "\n\nUse Browse... to choose an existing one.");
            break;
        case Pick::Disk: edit_.disk = r.path; break;
        case Pick::NewDisk:
            if (create_blank_image(r.path, kDiskSizesMiB[static_cast<std::size_t>(new_disk_size_)] << 20, error))
                edit_.disk = r.path;
            else message("No new disk was made:\n\n" + error + "\n\nUse Browse... to choose an existing one.");
            break;
        case Pick::AddDisc: edit_.cdroms.push_back(r.path); break;
        case Pick::InsertDisc:
            if (auto* m = session_.machine()) {
                if (m->disc_in_drive() >= 0 && !m->eject_disc())
                    status("The disc is locked in by the system; eject it from IRIX first");
                else if (!m->insert_disc(r.path))
                    message("The disc could not be opened:\n\n" + r.path);
            }
            break;
        case Pick::OpenSettings: {
            Settings s;
            if (!load_settings(r.path, s, error)) {
                message(error);
                break;
            }
            settings_path_ = r.path;
            session_.set_settings(s);
            if (settings_open_) open_settings();
            status(session_.on() ? "Settings loaded; they take effect at the next power-on" : "Settings loaded");
            break;
        }
        case Pick::SaveSettings:
            settings_path_ = r.path;
            save_settings_to(r.path);
            status("Settings saved to " + file_name(r.path));
            break;
    }
}

}  // namespace

int run_gui(Settings settings, const GuiOptions& options, volatile std::sig_atomic_t& stop,
            volatile std::sig_atomic_t& next_disc) {
    Gui gui(std::move(settings), options, stop, next_disc);
    return gui.run();
}

}  // namespace sg::app
