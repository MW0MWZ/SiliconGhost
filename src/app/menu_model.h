// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// The window's menus, described once: drawn in the window on Linux and
// Windows, and as the system menu bar on macOS.
#pragma once

#include <functional>
#include <string>
#include <vector>

namespace sg::app {

struct MenuEntry {
    enum class Kind { Item, Separator, Heading, Submenu };
    // Items macOS keeps in its application menu instead.
    enum class Role { None, About, Settings, Quit };

    Kind kind = Kind::Item;
    std::string label;
    char hotkey = 0;  // Ctrl+Alt (Ctrl+Option) + this letter, upper case
    bool checked = false, enabled = true;
    std::string tooltip;
    Role role = Role::None;
    std::function<void()> action;  // looks up what it acts on when run
    std::vector<MenuEntry> children;

    static MenuEntry item(std::string label, std::function<void()> action, bool enabled = true, bool checked = false) {
        MenuEntry e;
        e.label = std::move(label);
        e.action = std::move(action);
        e.enabled = enabled;
        e.checked = checked;
        return e;
    }
    static MenuEntry separator() {
        MenuEntry e;
        e.kind = Kind::Separator;
        return e;
    }
    static MenuEntry heading(std::string label) {
        MenuEntry e;
        e.kind = Kind::Heading;
        e.label = std::move(label);
        return e;
    }
    static MenuEntry submenu(std::string label, std::vector<MenuEntry> children) {
        MenuEntry e;
        e.kind = Kind::Submenu;
        e.label = std::move(label);
        e.children = std::move(children);
        return e;
    }
    MenuEntry& key(char k) {
        hotkey = k;
        return *this;
    }
    MenuEntry& tip(std::string t) {
        tooltip = std::move(t);
        return *this;
    }
    MenuEntry& as(Role r) {
        role = r;
        return *this;
    }
};

// macOS: replaces the menu bar with the application menu (About, Settings,
// Hide, Quit), `menus()` as built each time one opens, and the Window menu.
// Chosen items are handed to `post` to run from the main loop. False on
// other systems (the window draws the menus itself).
bool install_native_menus(std::function<std::vector<MenuEntry>()> menus,
                          std::function<void(std::function<void()>)> post);

}  // namespace sg::app
