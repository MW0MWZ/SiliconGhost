// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
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
#include "app/menu_model.h"
#include "app/proms.h"
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
constexpr const char* kAppName = "Silicon Ghost";  // as macOS shows the application
#else
constexpr const char* kAlt = "Alt";
constexpr const char* kAppName = "SiliconGhost";
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

std::string default_settings_path() { return user_file("settings.ini"); }

// Where settings were kept before the SiliconGhost folder (SDL's preferences
// folder, worked out here so that looking does not create it).
std::string old_user_file(const std::string& name) {
    namespace fs = std::filesystem;
    fs::path dir;
#if defined(_WIN32)
    if (const char* appdata = std::getenv("APPDATA")) dir = appdata;
#elif defined(__APPLE__)
    if (const char* home = std::getenv("HOME")) dir = fs::path(home) / "Library" / "Application Support";
#else
    if (const char* data = std::getenv("XDG_DATA_HOME"); data && *data) dir = data;
    else if (const char* home = std::getenv("HOME")) dir = fs::path(home) / ".local" / "share";
#endif
    if (dir.empty()) return name;
    return (dir / "SiliconGhost" / "SiliconGhost" / name).string();
}

// What a file chooser was opened for.
enum class Pick { Prom, PromDir, Nvram, NewNvram, Disk, NewDisk, AddDisc, InsertDisc, OpenSettings, SaveSettings };

constexpr SDL_DialogFileFilter kPromFilters[] = {{"PROM images", "bin;rom;img"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kNvramFilters[] = {{"NVRAM images", "bin;nvram;nvr"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kDiskFilters[] = {{"Disk images", "img;raw;dsk;hd;hdd"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kDiscFilters[] = {{"CD-ROM images", "iso;img;bin;raw;cdr"}, {"All files", "*"}};
constexpr SDL_DialogFileFilter kSettingsFilters[] = {{"Settings", "ini;conf;cfg"}, {"All files", "*"}};

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
    void capture_at(float x, float y);
    bool auto_capture(float x, float y);
    void mouse_frame();
    void release_keys();
    void update_title();

    // Drawing
    void frame();
    void draw_menu();
    std::vector<MenuEntry> menus();
    void draw_entries(const std::vector<MenuEntry>& entries);
    void toggle_fullscreen();
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
    float scale_ = 1.0f;    // UI scale, in window coordinates
    float density_ = 1.0f;  // pixels per window coordinate (2 on Retina)
    SDL_Texture* texture_ = nullptr;
    unsigned tex_w_ = 0, tex_h_ = 0;
    std::vector<u32> pixels_;
    u64 shown_version_ = 0;
    bool picture_ = false, picture_stale_ = true;
    SDL_FRect dest_{};  // where the picture is drawn, in window coordinates
    bool fullscreen_ = false, show_status_ = true;
    bool native_menus_ = false;                   // macOS: the system menu bar has the menus
    std::vector<std::function<void()>> posted_;  // actions chosen there, run from the main loop
    int pending_frames_ = 3;

    bool captured_ = false;
    // Following the guest's hardware cursor (see mouse_frame)
    bool have_cursor_ = false, syncing_ = false;
    int last_cx_ = 0, last_cy_ = 0, hot_x_ = 0, hot_y_ = 0, sync_frames_ = 0;
    int push_[4] = {};  // pushed against the left, right, top and bottom edges
    float sync_x_ = 0, sync_y_ = 0;
    int sent_x_ = 0, sent_y_ = 0;          // the last steering step
    float gain_x_ = 1.0f, gain_y_ = 1.0f;  // the guest's pointer acceleration, as seen
    // Moving onto the screen may capture the mouse; not again after a release
    // until the pointer has been off the screen.
    bool capture_armed_ = true;
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
    double cpu_load_ = 0, gfx_mpix_ = 0, mips_ = 0;
    std::string status_text_, status_shown_;
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

    // The PROM the settings being edited would start with (or why there is
    // none), worked out again only when a setting it depends on changes.
    struct PromChoice {
        std::string key, text;
        bool ok = false;
    } prom_choice_;
    const PromChoice& prom_choice(const Settings& s);
};

const Gui::PromChoice& Gui::prom_choice(const Settings& s) {
    const std::string key = s.prom_dir + '\n' + s.prom + '\n' + s.machine + '\n' + s.cpu + (s.graphics ? "\ng" : "");
    if (key == prom_choice_.key) return prom_choice_;
    prom_choice_.key = key;
    PromImage p;
    std::string error;
    prom_choice_.ok = select_prom(s, p, error);
    if (prom_choice_.ok) {
        prom_choice_.text = "Boot PROM: " + std::filesystem::path(p.path).filename().string() + " (" +
                            (p.part.empty() ? std::string() : p.part + ", ") + "version " + p.version + ")";
    } else {
        prom_choice_.text = error;
    }
    return prom_choice_;
}

int Gui::run() {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SG_LOG(Error, "display", "SDL_Init failed: %s", SDL_GetError());
        return -1;
    }
    settings_path_ = options_.settings_file.empty() ? default_settings_path() : options_.settings_file;
    {
        // Settings saved before the SiliconGhost folder existed move into it.
        std::error_code ec;
        const std::string old = old_user_file("settings.ini");
        if (old != settings_path_ && !std::filesystem::exists(settings_path_, ec) && std::filesystem::exists(old, ec))
            std::filesystem::copy_file(old, settings_path_, ec);
    }
    if (!options_.power_on && options_.settings_file.empty()) {
        Settings saved;
        std::string error;
        if (std::filesystem::exists(settings_path_) && load_settings(settings_path_, saved, error))
            session_.set_settings(saved);
        else if (!std::filesystem::exists(settings_path_))
            session_.set_settings(first_run_settings());
    }

    const SDL_DisplayID display = SDL_GetPrimaryDisplay();
    scale_ = std::clamp(SDL_GetDisplayContentScale(display), 1.0f, 4.0f);
    // Room for a 1280x1024 picture with the bars, shrunk to fit the screen:
    // the status bar, and the menu bar except on macOS (whose menus are in
    // the system menu bar).
#if defined(__APPLE__)
    constexpr float kBars = 1;
#else
    constexpr float kBars = 2;
#endif
    const float bars = kBars * (13.0f + 6.0f) * scale_;
    float w = 1280, h = 1024 + bars;
    SDL_Rect usable;
    if (SDL_GetDisplayUsableBounds(display, &usable)) {
        const float fit = std::min({1.0f, usable.w * 0.95f / w, usable.h * 0.9f / h});
        w *= fit;
        h = (h - bars) * fit + bars;
    }
    window_ = SDL_CreateWindow(kAppName, static_cast<int>(w), static_cast<int>(h),
                               SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    renderer_ = window_ ? SDL_CreateRenderer(window_, nullptr) : nullptr;
    if (!renderer_) {
        SG_LOG(Error, "display", "cannot create window: %s", SDL_GetError());
        if (window_) SDL_DestroyWindow(window_);
        SDL_Quit();
        return -1;
    }
    // Sizes are in window coordinates, scaled for the desktop's UI scale; on
    // a high-density screen (Retina) text is rasterised at the pixel density
    // and drawn at the coordinate size, so it stays sharp.
    density_ = std::max(1.0f, SDL_GetWindowPixelDensity(window_));
    scale_ = std::clamp(SDL_GetWindowDisplayScale(window_) / density_, 1.0f, 4.0f);
    {
        // The bars at the window's own scale (the display's may differ): the
        // picture keeps its size.
        const float exact = kBars * (13.0f + 6.0f) * scale_;
        if (std::fabs(exact - bars) >= 1) SDL_SetWindowSize(window_, static_cast<int>(w), static_cast<int>(h - bars + exact));
    }
    ctx_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx_);
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGui::GetStyle().ScaleAllSizes(scale_);
    ImFontConfig font;
    font.SizePixels = 13.0f * scale_ * density_;
    io.Fonts->AddFontDefault(&font);
    io.FontGlobalScale = 1.0f / density_;
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
            std::string(kAppName) + " - " + kPortNames[port], terminals_[port],
            [this, ch](const std::string& s) {
                if (auto* m = session_.machine())
                    for (char c : s) m->serial_input(ch, static_cast<u8>(c));
            },
            scale_);
    }
    session_.set_serial_output([this](unsigned ch, u8 byte) {
        if (ch < 2) terminals_[ch == 1 ? 0 : 1].feed(byte);
    });
    // After everything the menus describe exists.
    native_menus_ = install_native_menus([this] { return menus(); },
                                         [this](std::function<void()> a) { posted_.push_back(std::move(a)); });

    if (options_.power_on) power_on();
    else if (!prom_choice(session_.settings()).ok) open_settings();  // no PROM for the machine yet
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
        if (!posted_.empty()) {  // chosen from the system menu bar
            auto actions = std::move(posted_);
            posted_.clear();
            for (auto& a : actions) a();
            pending_frames_ = std::max(pending_frames_, 3);
        }
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
        mouse_frame();
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
    std::string title = kAppName;
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
    std::fill(std::begin(push_), std::end(push_), 0);
    have_cursor_ = syncing_ = false;
    if (!on) capture_armed_ = false;
    update_title();
}

// Captures the mouse and brings the guest's pointer to (x, y) in the window.
void Gui::capture_at(float x, float y) {
    set_captured(true);
    syncing_ = dest_.w > 0 && dest_.h > 0;
    sync_frames_ = 0;
    sync_x_ = (x - dest_.x) * static_cast<float>(tex_w_) / dest_.w;
    sync_y_ = (y - dest_.y) * static_cast<float>(tex_h_) / dest_.h;
    sent_x_ = sent_y_ = 0;
    gain_x_ = gain_y_ = 1.0f;
}

// The pointer moved to (x, y) while the mouse was free: moving onto the screen
// captures it, where the setting allows and the guest shows its hardware
// cursor (which can then be steered to the host pointer, and released at the
// screen's edges again). Not while the window is in the background, a button
// is held (a drag from outside) or a menu or dialog is open.
bool Gui::auto_capture(float x, float y) {
    const bool on_screen =
        picture_ && x >= dest_.x && x < dest_.x + dest_.w && y >= dest_.y && y < dest_.y + dest_.h;
    if (!on_screen) {
        capture_armed_ = true;
        return false;
    }
    auto* m = session_.machine();
    ip22::GfxBoard* np = m ? m->graphics() : nullptr;
    int cx = 0, cy = 0;
    if (!capture_armed_ || !session_.settings().mouse_auto_capture || !np || !np->cursor_position(cx, cy) ||
        !(SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) || SDL_GetMouseState(nullptr, nullptr) ||
        ImGui::IsPopupOpen(nullptr, ImGuiPopupFlags_AnyPopupId))
        return false;
    capture_at(x, y);
    return true;
}

// Once a frame while the mouse is captured: the motion gathered goes to the
// PS/2 mouse. Where the system shows the Newport's hardware cursor (the X
// server does), the window also follows the guest's pointer: after a capture
// it steers the pointer to where the click was, and pushing against an edge
// of the screen where the pointer has stopped releases the mouse there, the
// host pointer carrying on from the same place.
void Gui::mouse_frame() {
    auto* m = session_.machine();
    if (!captured_ || !m) return;
    int x = static_cast<int>(dx_), y = static_cast<int>(dy_);
    dx_ -= static_cast<float>(x);
    dy_ -= static_cast<float>(y);
    ip22::GfxBoard* np = m->graphics();
    int cx = 0, cy = 0;
    const bool cursor = np && np->cursor_position(cx, cy) && tex_w_ && tex_h_;
    if (syncing_) {
        if (!cursor || ++sync_frames_ > 120) {
            syncing_ = false;
        } else {
            // Moves made meanwhile move the target. The guest accelerates
            // larger steps: how much is learnt from what the last step did,
            // and most of the rest of the way is sent each frame, so the
            // pointer gets there in a few frames without overshooting much.
            sync_x_ += static_cast<float>(x);
            sync_y_ += static_cast<float>(y);
            const int rest_x = static_cast<int>(std::lround(sync_x_)) - (cx + hot_x_);
            const int rest_y = static_cast<int>(std::lround(sync_y_)) - (cy + hot_y_);
            auto steer = [](int rest, int& sent, int moved, float& gain) {
                if (std::abs(sent) >= 4 && moved * sent > 0)
                    gain = std::clamp(static_cast<float>(moved) / static_cast<float>(sent), 1.0f, 8.0f);
                sent = std::abs(rest) <= 2 ? rest : static_cast<int>(std::lround(static_cast<float>(rest) * 0.75f / gain));
                sent = std::clamp(sent, -127, 127);
                return sent;
            };
            if (std::abs(rest_x) <= 1 && std::abs(rest_y) <= 1) {
                syncing_ = false;
                x = y = 0;
            } else {
                x = steer(rest_x, sent_x_, cx - last_cx_, gain_x_);
                y = steer(rest_y, sent_y_, cy - last_cy_, gain_y_);
            }
        }
    }
    if (x || y) m->ioc().kbc().mouse_motion(x, -y, buttons_);  // PS/2 counts up as positive
    if (!cursor || !have_cursor_ || syncing_) {
        have_cursor_ = cursor;
        last_cx_ = cx;
        last_cy_ = cy;
        return;
    }
    // Pushing against an edge, with the cursor there and not moving: the
    // glyph is 32 pixels, the pointer somewhere inside it.
    const int w = static_cast<int>(tex_w_), h = static_cast<int>(tex_h_);
    auto push = [](int& acc, int amount, bool stuck) { acc = stuck ? acc + amount : 0; };
    if (x < 0) push(push_[0], -x, cx >= last_cx_ && cx <= 0);
    if (x > 0) push(push_[1], x, cx <= last_cx_ && cx >= w - 33);
    if (y < 0) push(push_[2], -y, cy >= last_cy_ && cy <= 0);
    if (y > 0) push(push_[3], y, cy <= last_cy_ && cy >= h - 33);
    last_cx_ = cx;
    last_cy_ = cy;
    constexpr int kPushToLeave = 48;  // guest pixels pushed against the edge
    for (int edge = 0; edge < 4; ++edge) {
        if (push_[edge] < kPushToLeave) continue;
        // At an edge the pointer's position there is known: learn where it
        // sits in the glyph.
        if (edge == 0) hot_x_ = -cx;
        if (edge == 1) hot_x_ = (w - 1) - cx;
        if (edge == 2) hot_y_ = -cy;
        if (edge == 3) hot_y_ = (h - 1) - cy;
        const float px = static_cast<float>(std::clamp(cx + hot_x_, 0, w - 1)) + 0.5f;
        const float py = static_cast<float>(std::clamp(cy + hot_y_, 0, h - 1)) + 0.5f;
        float wx = dest_.x + px * dest_.w / static_cast<float>(w), wy = dest_.y + py * dest_.h / static_cast<float>(h);
        if (edge == 0) wx = dest_.x;
        if (edge == 1) wx = dest_.x + dest_.w - 1;
        if (edge == 2) wy = dest_.y;
        if (edge == 3) wy = dest_.y + dest_.h - 1;
        set_captured(false);
        SDL_WarpMouseInWindow(window_, wx, wy);
        return;
    }
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
            if (down) toggle_fullscreen();
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
            } else if (ev.key.down && ev.key.repeat && machine && sc < keys_down_.size() && keys_down_[sc]) {
                send_key(*machine, ev.key.scancode, true, true);  // typematic repeat
            } else if (!ev.key.down && sc < keys_down_.size() && keys_down_[sc]) {
                if (machine) send_key(*machine, ev.key.scancode, false);
                keys_down_[sc] = false;
            }
            if (captured_) return;
            break;
        }
        case SDL_EVENT_WINDOW_MOUSE_LEAVE:
            capture_armed_ = true;
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (!captured_ && machine && auto_capture(ev.motion.x, ev.motion.y)) return;
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
                capture_at(ev.button.x, ev.button.y);  // the guest's pointer to where the click was
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
    ip22::GfxBoard* newport = machine ? machine->graphics() : nullptr;
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
        // The status bar is looked at ten times a second, and redrawn only
        // when what it shows has changed: an idle machine costs no frames.
        if (now >= next_status_ns_) {
            next_status_ns_ = now + 100000000ull;
            auto* m = session_.machine();
            char sig[96];
            std::snprintf(sig, sizeof sig, "%d%d%d %d %3.0f %5.1f %4.1f %.0f %d", disk_light_.lit(now), cd_light_.lit(now),
                          net_light_.lit(now), m->disc_in_drive(), cpu_load_ * 100, gfx_mpix_,
                          session_.pacer()->cpu_speed(), std::floor(session_.pacer()->lag()), session_.turbo());
            if (status_shown_ != sig) {
                status_shown_ = sig;
                pending_frames_ = std::max(pending_frames_, 1);
            }
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
        if (!native_menus_) {
            draw_menu();
            top = ImGui::GetFrameHeight();
        }
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
        // Whole-number scaling (1280x1024 doubled on Retina) stays sharp.
        const float k = px.w / static_cast<float>(tex_w_);
        SDL_SetTextureScaleMode(texture_, std::fabs(k - std::round(k)) < 0.01f ? SDL_SCALEMODE_NEAREST : SDL_SCALEMODE_LINEAR);
        SDL_RenderTexture(renderer_, texture_, nullptr, &px);
    }
    // ImGui draws in window coordinates and clips in pixels: SDL's render
    // scale turns the one into the other (2 on a high-density screen).
    SDL_SetRenderScale(renderer_, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer_);
    SDL_SetRenderScale(renderer_, 1.0f, 1.0f);
    SDL_RenderPresent(renderer_);
}

// The menus. Actions look the machine up when they run: on macOS they run
// from the main loop after the menu closes, and may outlive the state the
// menu was built from.
std::vector<MenuEntry> Gui::menus() {
    using E = MenuEntry;
    auto* m = session_.machine();
    std::vector<E> machine;
    if (!m) {
        machine.push_back(E::item("Power On", [this] { power_on(); }).key('P'));
    } else {
        machine.push_back(E::item("Power Button", [this] { press_power_button(); })
                              .key('P')
                              .tip("As the button on the front: the system shuts down and powers off"));
        machine.push_back(E::item("Power Off Now", [this] { power_off(); })
                              .tip("Cuts the power at once, without shutting down (the NVRAM is kept)"));
        machine.push_back(E::item("Reset", [this] {
                              if (auto* x = session_.machine()) x->reset();
                          }).tip("Restarts the machine from the boot PROM at once"));
        machine.push_back(E::separator());
        machine.push_back(E::item("Volume Up", [this] { press_volume(ip22::Ioc2::PanelButton::VolumeUp); }));
        machine.push_back(E::item("Volume Down", [this] { press_volume(ip22::Ioc2::PanelButton::VolumeDown); }));
    }
    machine.push_back(E::separator());
    machine.push_back(E::item("Settings...", [this] { open_settings(); }).as(E::Role::Settings));
    machine.push_back(E::item("Open Settings File...", [this] { choose(Pick::OpenSettings, settings_path_); }));
    machine.push_back(E::item("Save Settings As...", [this] { choose(Pick::SaveSettings, settings_path_); }));
    machine.push_back(E::separator());
    machine.push_back(E::item("Quit", [this] {
                          if (session_.on()) open_quit_ = true;
                          else quit_ = true;
                      }).as(E::Role::Quit));

    std::vector<E> media;
    const bool drive = m && m->has_cdrom_drive();
    const bool loaded = drive && m->disc_in_drive() >= 0;
    media.push_back(E::heading("CD-ROM (SCSI ID 4)"));
    media.push_back(E::item("Eject", [this] { eject(); }, loaded));
    media.push_back(E::item("Force Eject", [this] {
                        if (auto* x = session_.machine()) {
                            x->force_eject_disc();
                            status("Disc ejected while the system had it locked in");
                        }
                    }, loaded).tip("As a paper clip in the drive's emergency eject hole: also when the system has locked the disc in"));
    media.push_back(E::item("Insert Disc...", [this] { choose(Pick::InsertDisc, {}); }, drive));
    media.push_back(E::item("Next Disc in Set", [this] {
                        if (auto* x = session_.machine()) x->request_next_disc();
                    }, drive && !m->discs().empty()).key('E'));
    const std::vector<std::string>& discs = m ? m->discs() : session_.settings().cdroms;
    if (!discs.empty()) media.push_back(E::separator());
    for (std::size_t i = 0; i < discs.size(); ++i) {
        const bool in = m && m->disc_in_drive() == static_cast<int>(i);
        media.push_back(E::item(file_name(discs[i]), [this, i] {
                            auto* x = session_.machine();
                            if (!x || x->disc_in_drive() == static_cast<int>(i)) return;
                            if (x->disc_in_drive() >= 0 && !x->eject_disc())
                                status("The disc is locked in by the system; eject it from IRIX first");
                            else
                                x->select_disc(i);
                        }, drive, in).tip(discs[i]));
    }
    media.push_back(E::separator());
    media.push_back(E::heading("Hard disk (SCSI ID 1)"));
    const std::string& disk = session_.settings().disk;
    media.push_back(E::item(disk.empty() ? "(none)" : file_name(disk), nullptr, false));
    media.push_back(E::item("Change Disk...", [this] { open_settings(); }, !m).tip(m ? "Power off first" : ""));

    std::vector<E> view;
    for (unsigned port = 0; port < 2; ++port) {
        auto* w = terminal_windows_[port].get();
        view.push_back(E::item(port == 0 ? "Serial Port 1 (console)" : "Serial Port 2", [this, port] {
                           auto* t = terminal_windows_[port].get();
                           if (!t) return;
                           if (t->is_open()) t->close();
                           else t->open();
                       }, w != nullptr, w && w->is_open()));
    }
    view.push_back(E::separator());
    view.push_back(E::item("Status Bar", [this] { show_status_ = !show_status_; }, true, show_status_));
    view.push_back(E::item("Full Screen", [this] { toggle_fullscreen(); }, true, fullscreen_).key('F'));
    view.push_back(E::item("Capture Mouse on Entry", [this] {
                       Settings s = session_.settings();
                       s.mouse_auto_capture = !s.mouse_auto_capture;
                       session_.set_settings(s);
                       save_settings_to(settings_path_);
                   }, true, session_.settings().mouse_auto_capture)
                       .tip("Capture the mouse when the pointer moves onto the screen (once the system shows its pointer)"));

    std::vector<E> help;
    help.push_back(E::item("Keyboard and Mouse", [this] { keys_open_ = true; }));
    help.push_back(E::item(std::string("About ") + kAppName, [this] { about_open_ = true; }).as(E::Role::About));

    return {E::submenu("Machine", std::move(machine)), E::submenu("Media", std::move(media)),
            E::submenu("View", std::move(view)), E::submenu("Help", std::move(help))};
}

void Gui::draw_entries(const std::vector<MenuEntry>& entries) {
    const std::string ctrl_alt = std::string("Ctrl+") + kAlt + "+";
    int id = 0;
    for (const MenuEntry& e : entries) {
        ImGui::PushID(id++);
        switch (e.kind) {
            case MenuEntry::Kind::Separator: ImGui::Separator(); break;
            case MenuEntry::Kind::Heading: ImGui::TextDisabled("%s", e.label.c_str()); break;
            case MenuEntry::Kind::Submenu:
                if (ImGui::BeginMenu(e.label.c_str(), e.enabled)) {
                    draw_entries(e.children);
                    ImGui::EndMenu();
                }
                break;
            case MenuEntry::Kind::Item: {
                const std::string keys = e.hotkey ? ctrl_alt + e.hotkey : std::string();
                if (ImGui::MenuItem(e.label.c_str(), keys.empty() ? nullptr : keys.c_str(), e.checked, e.enabled) && e.action)
                    e.action();
                if (!e.tooltip.empty()) ImGui::SetItemTooltip("%s", e.tooltip.c_str());
                break;
            }
        }
        ImGui::PopID();
    }
}

void Gui::draw_menu() {
    if (!ImGui::BeginMainMenuBar()) return;
    draw_entries(menus());
    ImGui::EndMainMenuBar();
}

void Gui::toggle_fullscreen() {
    fullscreen_ = !fullscreen_;
    SDL_SetWindowFullscreen(window_, fullscreen_);
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
        const Pacer* pacer = session_.pacer();
        const double cpu_speed = pacer ? pacer->cpu_speed() : 0;
        if (cpu_speed > 0) ImGui::Text("Speed %4.1fx", cpu_speed);
        else ImGui::Text("Speed  -  ");
        ImGui::SetItemTooltip("How many times as fast as a real %.0f MHz CPU it runs when busy, by the host's\n"
                              "measured speed. Now: %.0f million instructions a second.",
                              m->cycles_per_second() / 1e6, mips_);
        ImGui::SameLine(0, 14 * scale_);
        // Turbo: Maximum timing until pressed again (or the configured
        // timing is already Maximum or Free: shown lit, not pressable).
        {
            const bool available = session_.turbo_available();
            const bool on = available ? session_.turbo() : session_.base_timing() != Pacer::Mode::Cycle;
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(6 * scale_, 0));
            if (on) {
                ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(200, 60, 40, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(225, 80, 55, 255));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, IM_COL32(170, 45, 30, 255));
            }
            ImGui::BeginDisabled(!available);
            if (ImGui::SmallButton("Turbo")) {
                session_.set_turbo(!session_.turbo());
                status(session_.turbo() ? "Turbo on: maximum speed, fast disk and network"
                                        : "Turbo off: back to the set timing");
                pending_frames_ = std::max(pending_frames_, 2);
            }
            ImGui::EndDisabled();
            if (on) ImGui::PopStyleColor(3);
            ImGui::PopStyleVar();
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                ImGui::SetTooltip("%s", available ? "Run flat out (Maximum timing: no speed limit, fast disk and network)\n"
                                              "until pressed again; guest clocks still follow the host's"
                                            : "Timing is set to Maximum or Free in Settings: already flat out");
            ImGui::SameLine(0, 14 * scale_);
        }
        // Guest time behind the wall clock: the host cannot keep up.
        if (const double lag = pacer ? pacer->lag() : 0; lag >= 1.0) {
            ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.3f, 1.0f), "Behind %.0f s", lag);
            ImGui::SetItemTooltip("The host cannot run the machine fast enough to keep its clocks to real time");
            ImGui::SameLine(0, 14 * scale_);
        }
    }
    if (!status_text_.empty()) {
        ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "%s", status_text_.c_str());
        ImGui::SameLine();
    }
    const Settings& s = session_.settings();
    const CpuModel* model = find_cpu_model(s.cpu);
    char right[96];
    std::snprintf(right, sizeof right, "%s%s %.0f MHz, %u MiB", s.machine == "indigo2" ? "Indigo2, " : "",
                  model ? model->label : s.cpu.c_str(), s.cpu_mhz, s.ram_mb);
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
    make_consistent(edit_, false);  // a processor or memory size the platform cannot take is replaced
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
            // Each choice limits the next: the platform the processors, the
            // processor its clocks; the platform the memory sizes.
            const ip22::Board board = board_of(edit_);
            label("Hardware platform");
            ImGui::SetNextItemWidth(-1);
            const Platform* platform = find_platform(edit_.machine);
            if (ImGui::BeginCombo("##machine", platform ? platform->label : edit_.machine.c_str())) {
                for (const auto& p : platforms())
                    if (ImGui::Selectable(p.label, edit_.machine == p.id) && edit_.machine != p.id) {
                        edit_.machine = p.id;
                        make_consistent(edit_, false);
                    }
                ImGui::EndCombo();
            }
            label("Processor");
            ImGui::SetNextItemWidth(-1);
            const CpuModel* current = find_cpu_model(edit_.cpu);
            if (ImGui::BeginCombo("##cpu", current ? current->label : edit_.cpu.c_str())) {
                for (const auto& model : cpu_models())
                    if (model.fits(board) && ImGui::Selectable(model.label, edit_.cpu == model.id) &&
                        edit_.cpu != model.id) {
                        edit_.cpu = model.id;
                        make_consistent(edit_, true);
                    }
                ImGui::EndCombo();
            }
            current = find_cpu_model(edit_.cpu);
            label("Clock");
            ImGui::SetNextItemWidth(160 * scale_);
            char clock[32];
            std::snprintf(clock, sizeof clock, "%.0f MHz", edit_.cpu_mhz);
            if (ImGui::BeginCombo("##mhz", clock)) {
                if (current)
                    for (double mhz : current->mhz(board)) {
                        std::snprintf(clock, sizeof clock, "%.0f MHz", mhz);
                        if (ImGui::Selectable(clock, edit_.cpu_mhz == mhz)) edit_.cpu_mhz = mhz;
                    }
                ImGui::EndCombo();
            }
            label("Memory");
            ImGui::SetNextItemWidth(160 * scale_);
            const std::string ram = std::to_string(edit_.ram_mb) + " MiB";
            if (ImGui::BeginCombo("##ram", ram.c_str())) {
                for (unsigned mb : ram_sizes(board, current))
                    if (ImGui::Selectable((std::to_string(mb) + " MiB").c_str(), edit_.ram_mb == mb)) edit_.ram_mb = mb;
                ImGui::EndCombo();
            }
            label("Graphics");
            {
                // The boards the machine takes, then none.
                static const char* const kBoards[] = {"xl24", "solid-impact", "high-impact", "max-impact"};
                static const char* const kNames[] = {"Newport XL24 (24-bit)", "Solid IMPACT (not finished)",
                                                     "High IMPACT (not finished)", "Maximum IMPACT (not finished)"};
                const bool indigo2 = edit_.machine == "indigo2";
                const int boards = indigo2 ? 4 : 1;
                int gfx = boards;
                for (int i = 0; i < boards; ++i)
                    if (edit_.graphics && edit_.graphics_board == kBoards[i]) gfx = i;
                if (gfx == boards && edit_.graphics) gfx = 0;  // (an IMPACT setting on an Indy)
                ImGui::SetNextItemWidth(-1);
                if (ImGui::BeginCombo("##gfx", gfx < boards ? kNames[gfx] : "None (serial console only)")) {
                    for (int i = 0; i <= boards; ++i)
                        if (ImGui::Selectable(i < boards ? kNames[i] : "None (serial console only)", gfx == i)) gfx = i;
                    ImGui::EndCombo();
                }
                edit_.graphics = gfx < boards;
                if (edit_.graphics) edit_.graphics_board = kBoards[gfx];
            }
            if (edit_.graphics) {
                label("Console");
                ImGui::SetNextItemWidth(-1);
                int con = edit_.console == "graphics" ? 1 : edit_.console == "serial" ? 2 : 0;
                if (ImGui::Combo("##console", &con, "As the NVRAM has it\0Graphics screen and keyboard\0Serial port 1\0"))
                    edit_.console = con == 1 ? "graphics" : con == 2 ? "serial" : "";
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Where the PROM and the system show their console: the PROM's console\n"
                                      "variable, written to NVRAM at power-on. A new NVRAM takes it from the\n"
                                      "second power-on, once the PROM has set it up.");
            }
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::PushTextWrapPos(0.0f);
            {
                const PromChoice& pc = prom_choice(edit_);
                ImGui::Bullet();
                if (pc.ok) ImGui::TextDisabled("%s", pc.text.c_str());
                else ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", pc.text.c_str());
            }
            for (const std::string& note : configuration_notes(edit_)) {
                ImGui::Bullet();
                ImGui::TextDisabled("%s", note.c_str());
            }
            ImGui::PopTextWrapPos();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Storage")) {
            path_row("PROM folder", "promdir", edit_.prom_dir, Pick::PromDir, std::nullopt);
            ImGui::SetCursorPosX(label_w);
            ImGui::TextDisabled("Empty: %s", prom_folder(Settings{}).c_str());
            path_row("Specific PROM", "prom", edit_.prom, Pick::Prom, std::nullopt);
            ImGui::SetCursorPosX(label_w);
            ImGui::TextDisabled("Optional: leave empty to use the right one from the folder");
            {
                const PromChoice& pc = prom_choice(edit_);
                ImGui::SetCursorPosX(label_w);
                ImGui::PushTextWrapPos(0.0f);
                if (pc.ok) ImGui::TextUnformatted(pc.text.c_str());
                else ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f), "%s", pc.text.c_str());
                ImGui::PopTextWrapPos();
            }
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
                ImGui::TextDisabled("Guest 10.0.2.15, gateway (the host) 10.0.2.2, DNS 10.0.2.3");
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
                ImGui::Spacing();
                if (ImGui::CollapsingHeader("Setting up the guest")) {
                    const auto heading = [](const char* text) {
                        ImGui::Spacing();
                        ImGui::TextUnformatted(text);
                    };
                    const auto lines = [](const char* text) { ImGui::TextDisabled("%s", text); };
                    heading("IRIX, as root (lasts until the next restart):");
                    lines("  ifconfig ec0 10.0.2.15 netmask 255.255.255.0 up\n"
                          "  route add default 10.0.2.2 1\n"
                          "  echo nameserver 10.0.2.3 > /etc/resolv.conf");
                    heading("Keeping the address (IRIX):");
                    lines("  in /etc/hosts, under the name in /etc/sys_id");
                    heading("Keeping the route (IRIX 6.5):");
                    lines("  add to /etc/config/static-route.options:\n"
                          "  $ROUTE $QUIET add -net default 10.0.2.2");
                    heading("Keeping the route (IRIX 5.3):");
                    lines("  an executable /etc/init.d/network.local,\n"
                          "  linked as /etc/rc2.d/S31network:\n"
                          "    #!/bin/sh\n"
                          "    case \"$1\" in\n"
                          "    start) /usr/etc/route add default 10.0.2.2 1 ;;\n"
                          "    esac");
                    heading("NetBSD and OpenBSD:");
                    lines("  DHCP sets everything; or 10.0.2.2 in /etc/mygate");
                    heading("Checks, in order:");
                    lines("  netstat -rn         a default route to 10.0.2.2\n"
                          "  ping 10.0.2.2       the gateway\n"
                          "  ping 1.1.1.1        the way out\n"
                          "  ping www.sgi.com    the name server");
                    heading("If names fail:");
                    lines("  the host needs an IPv4 name server, or give the\n"
                          "  guest a public one (nameserver 1.1.1.1). On macOS 15\n"
                          "  and later, allow Silicon Ghost (or the terminal that\n"
                          "  started it) in System Settings > Privacy & Security >\n"
                          "  Local Network.");
                }
            }
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Performance")) {
            label("Timing");
            ImGui::SetNextItemWidth(-1);
            int timing = edit_.timing == Pacer::Mode::Realtime ? 0 : edit_.timing == Pacer::Mode::Cycle ? 1 : 2;
            if (ImGui::Combo("##timing", &timing,
                             "Real time: guest clocks follow the host's\0Cycle: one cycle per instruction, reproducible\0"
                             "Maximum: no speed limit, fast disk and network, guest clocks follow the host's\0"))
                edit_.timing = timing == 0 ? Pacer::Mode::Realtime : timing == 1 ? Pacer::Mode::Cycle : Pacer::Mode::Max;
            if (timing != 1) {
                label("Timed speed");
                ImGui::SetNextItemWidth(200 * scale_);
                float timed = static_cast<float>(edit_.timed_speed);
                ImGui::SliderFloat("##timedspeed", &timed, 1.0f, 4.0f, "%.1fx the real CPU");
                edit_.timed_speed = timed;
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("The CPU's speed while the system times it at boot. Its delays keep that\n"
                                      "length; everything else runs as fast as the host allows.");
                ImGui::SameLine();
                const double timed_mhz = edit_.cpu_mhz * edit_.timed_speed;
                ImGui::TextDisabled("%.0f MHz", timed_mhz);
                if (timing == 0) {
                    label("Speed limit");
                    bool cap = edit_.max_speed > 0;
                    if (ImGui::Checkbox("##cap", &cap)) edit_.max_speed = cap ? std::max(4.0, edit_.timed_speed) : 0;
                    ImGui::SameLine();
                    if (cap) {
                        ImGui::SetNextItemWidth(200 * scale_);
                        float limit = static_cast<float>(edit_.max_speed);
                        ImGui::SliderFloat("##maxspeed", &limit, 1.0f, 16.0f, "%.1fx the real CPU");
                        edit_.max_speed = limit;
                    } else {
                        ImGui::TextDisabled("none: as fast as the host allows");
                    }
                }
            }
            ImGui::Spacing();
            label("Quick SCSI scan");
            static const char* const kQuickScsiTip =
                "Absent SCSI devices answer within a millisecond instead of after the controller's\n"
                "programmed timeout, which takes the PROM's bus scan about two seconds";
            ImGui::Checkbox("Real time and cycle##quickscsi", &edit_.quick_scsi_scan);
            ImGui::SetItemTooltip("%s", kQuickScsiTip);
            ImGui::SameLine();
            ImGui::Checkbox("Maximum and turbo##quickscsimax", &edit_.quick_scsi_scan_max);
            ImGui::SetItemTooltip("%s", kQuickScsiTip);
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
        if (ImGui::BeginTabItem("Mouse")) {
            label("Capture");
            ImGui::SetNextItemWidth(-1);
            int capture = edit_.mouse_auto_capture ? 0 : 1;
            ImGui::Combo("##capture", &capture, "When the pointer moves onto the screen\0When the screen is clicked\0");
            edit_.mouse_auto_capture = capture == 0;
            ImGui::TextDisabled("Moving onto the screen captures the mouse once the system shows its pointer");
            ImGui::TextDisabled("(the IRIX desktop); a click always captures it. Ctrl+%s+G releases it.", kAlt);
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
        ImGui::OpenPopup((std::string(kAppName) + "##message").c_str());
        open_message_ = false;
    }
    if (open_quit_) {
        ImGui::OpenPopup("Quit?##quit");
        open_quit_ = false;
    }
    if (about_open_) {
        ImGui::OpenPopup("About##about");
        about_open_ = false;
    }
    if (keys_open_) {
        ImGui::OpenPopup("Keyboard and Mouse");
        keys_open_ = false;
    }
    const auto flags = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings;
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal((std::string(kAppName) + "##message").c_str(), nullptr, flags)) {
        ImGui::PushTextWrapPos(480 * scale_);
        ImGui::TextUnformatted(message_.c_str());
        ImGui::PopTextWrapPos();
        if (ImGui::Button("OK", ImVec2(90 * scale_, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Quit?##quit", nullptr, flags)) {
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
    if (ImGui::BeginPopupModal("About##about", nullptr, flags)) {
        ImGui::Text("%s: an SGI Indy (IP22) emulator", kAppName);
        ImGui::TextUnformatted("Copyright (C) 2026 Andy Taylor");
        ImGui::TextUnformatted("GNU General Public License, version 2 or later");
        ImGui::Spacing();
        ImGui::TextDisabled("Menus and dialogs: Dear ImGui (MIT licence)");
        if (!options_.log_file.empty()) ImGui::TextDisabled("Messages: %s", options_.log_file.c_str());
        ImGui::TextDisabled("SGI, Indy and IRIX are trademarks of their owners.");
        if (ImGui::Button("OK", ImVec2(90 * scale_, 0))) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(centre, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Keyboard and Mouse", nullptr, flags)) {
        const std::string ca = std::string("Ctrl+") + kAlt + "+";
        ImGui::TextUnformatted("Click the screen to capture the mouse for the Indy. Once the system shows its");
        ImGui::TextUnformatted("pointer (the desktop), moving onto the screen captures it (View > Capture Mouse");
        ImGui::TextUnformatted("on Entry) and pushing past an edge of the screen releases it.");
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
    // With nothing chosen yet, start in the matching folder of the user's
    // SiliconGhost folder.
    std::string start = folder_of(current);
    if (start.empty() && !user_data_folder().empty()) {
        const char* sub = what == Pick::Prom || what == Pick::PromDir ? "roms"
                          : what == Pick::Nvram || what == Pick::NewNvram ? "nvram"
                          : what == Pick::OpenSettings || what == Pick::SaveSettings ? ""
                                                                         : "diskImages";
        const auto dir = std::filesystem::path(user_data_folder()) / sub;
        std::error_code ec;
        if (std::filesystem::is_directory(dir, ec)) start = dir.string() + "/";
    }
    auto* req = new PickRequest{this, what, start};
    const char* location = req->location.empty() ? nullptr : req->location.c_str();
    switch (what) {
        case Pick::Prom: SDL_ShowOpenFileDialog(on_chosen, req, window_, kPromFilters, 2, location, false); break;
        case Pick::PromDir: SDL_ShowOpenFolderDialog(on_chosen, req, window_, location, false); break;
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
        case Pick::PromDir: edit_.prom_dir = r.path; break;
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

std::string user_file(const std::string& name) {
    const std::string base = user_data_folder();
    if (base.empty()) return old_user_file(name);
    make_user_data_folders();
    return (std::filesystem::path(base) / name).string();
}

#if !defined(__APPLE__)
bool install_native_menus(std::function<std::vector<MenuEntry>()>, std::function<void(std::function<void()>)>) {
    return false;
}
#endif

int run_gui(Settings settings, const GuiOptions& options, volatile std::sig_atomic_t& stop,
            volatile std::sig_atomic_t& next_disc) {
    Gui gui(std::move(settings), options, stop, next_disc);
    return gui.run();
}

}  // namespace sg::app
