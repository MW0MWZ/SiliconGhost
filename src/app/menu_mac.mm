// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright (C) 2026 Andy Taylor
// The macOS menu bar: the application menu, the window's menus (rebuilt
// from the shared description each time one opens) and the Window menu.
#import <Cocoa/Cocoa.h>

#include <cctype>
#include <string>
#include <utility>

#include "app/menu_model.h"

using sg::app::MenuEntry;

// One menu item's action, kept by the item.
@interface SGMenuAction : NSObject {
@public
    std::function<void()> run;
}
@end

@implementation SGMenuAction
@end

@interface SGMenuController : NSObject <NSMenuDelegate> {
@public
    std::function<std::vector<MenuEntry>()> menus;
    std::function<void(std::function<void()>)> post;
}
- (void)fill:(NSMenu*)menu from:(const std::vector<MenuEntry>&)entries;
@end

@implementation SGMenuController

- (void)chosen:(NSMenuItem*)item {
    SGMenuAction* action = item.representedObject;
    if (action && action->run) post(action->run);
}

// The application menu's About, Settings and Quit run the entry with that
// role, looked up when chosen.
- (void)runRole:(MenuEntry::Role)role {
    std::function<void()> found;
    std::function<void(const std::vector<MenuEntry>&)> search = [&](const std::vector<MenuEntry>& entries) {
        for (const MenuEntry& e : entries) {
            if (e.role == role && e.action) found = e.action;
            search(e.children);
        }
    };
    search(menus());
    if (found) post(found);
}
- (void)about:(id)sender {
    (void)sender;
    [self runRole:MenuEntry::Role::About];
}
- (void)settings:(id)sender {
    (void)sender;
    [self runRole:MenuEntry::Role::Settings];
}
- (void)quit:(id)sender {
    (void)sender;
    [self runRole:MenuEntry::Role::Quit];
}

- (void)fill:(NSMenu*)menu from:(const std::vector<MenuEntry>&)entries {
    [menu removeAllItems];
    bool after_separator = true;  // no separator first, nor two in a row
    for (const MenuEntry& e : entries) {
        if (e.role != MenuEntry::Role::None) continue;  // in the application menu instead
        NSString* title = [NSString stringWithUTF8String:e.label.c_str()];
        switch (e.kind) {
            case MenuEntry::Kind::Separator:
                if (!after_separator) [menu addItem:[NSMenuItem separatorItem]];
                after_separator = true;
                continue;
            case MenuEntry::Kind::Heading: {
                NSMenuItem* heading = [[NSMenuItem alloc] initWithTitle:title action:NULL keyEquivalent:@""];
                heading.enabled = NO;
                [menu addItem:heading];
                break;
            }
            case MenuEntry::Kind::Submenu: {
                NSMenuItem* holder = [[NSMenuItem alloc] initWithTitle:title action:NULL keyEquivalent:@""];
                NSMenu* sub = [[NSMenu alloc] initWithTitle:title];
                sub.autoenablesItems = NO;
                [self fill:sub from:e.children];
                holder.submenu = sub;
                [menu addItem:holder];
                break;
            }
            case MenuEntry::Kind::Item: {
                NSString* key = e.hotkey ? [NSString stringWithFormat:@"%c", std::tolower(static_cast<unsigned char>(e.hotkey))]
                                         : @"";
                NSMenuItem* item = [[NSMenuItem alloc] initWithTitle:title action:@selector(chosen:) keyEquivalent:key];
                if (e.hotkey) item.keyEquivalentModifierMask = NSEventModifierFlagControl | NSEventModifierFlagOption;
                item.target = self;
                SGMenuAction* action = [[SGMenuAction alloc] init];
                action->run = e.action;
                item.representedObject = action;
                item.enabled = e.enabled && e.action;
                item.state = e.checked ? NSControlStateValueOn : NSControlStateValueOff;
                if (!e.tooltip.empty()) item.toolTip = [NSString stringWithUTF8String:e.tooltip.c_str()];
                [menu addItem:item];
                break;
            }
        }
        after_separator = false;
    }
    while (menu.numberOfItems > 0 && [menu itemAtIndex:menu.numberOfItems - 1].isSeparatorItem)
        [menu removeItemAtIndex:menu.numberOfItems - 1];
}

// Each top menu is rebuilt from the current description as it opens.
- (void)menuNeedsUpdate:(NSMenu*)menu {
    const std::string title = menu.title.UTF8String;
    for (const MenuEntry& e : menus())
        if (e.kind == MenuEntry::Kind::Submenu && e.label == title) [self fill:menu from:e.children];
}

@end

namespace sg::app {

namespace {
SGMenuController* g_controller = nil;  // the menus' target, kept for the program's life
}  // namespace

bool install_native_menus(std::function<std::vector<MenuEntry>()> menus,
                          std::function<void(std::function<void()>)> post) {
    g_controller = [[SGMenuController alloc] init];
    g_controller->menus = std::move(menus);
    g_controller->post = std::move(post);
    NSString* name = @"Silicon Ghost";
    NSMenu* bar = [[NSMenu alloc] init];

    // Application menu
    NSMenu* app = [[NSMenu alloc] initWithTitle:name];
    [app addItemWithTitle:[@"About " stringByAppendingString:name] action:@selector(about:) keyEquivalent:@""].target =
        g_controller;
    [app addItem:[NSMenuItem separatorItem]];
    [app addItemWithTitle:@"Settings…" action:@selector(settings:) keyEquivalent:@","].target = g_controller;
    [app addItem:[NSMenuItem separatorItem]];
    NSMenuItem* services = [[NSMenuItem alloc] initWithTitle:@"Services" action:NULL keyEquivalent:@""];
    NSMenu* services_menu = [[NSMenu alloc] initWithTitle:@"Services"];
    services.submenu = services_menu;
    [app addItem:services];
    NSApp.servicesMenu = services_menu;
    [app addItem:[NSMenuItem separatorItem]];
    [app addItemWithTitle:[@"Hide " stringByAppendingString:name] action:@selector(hide:) keyEquivalent:@"h"];
    NSMenuItem* others = [app addItemWithTitle:@"Hide Others" action:@selector(hideOtherApplications:) keyEquivalent:@"h"];
    others.keyEquivalentModifierMask = NSEventModifierFlagOption | NSEventModifierFlagCommand;
    [app addItemWithTitle:@"Show All" action:@selector(unhideAllApplications:) keyEquivalent:@""];
    [app addItem:[NSMenuItem separatorItem]];
    [app addItemWithTitle:[@"Quit " stringByAppendingString:name] action:@selector(quit:) keyEquivalent:@"q"].target =
        g_controller;
    NSMenuItem* app_item = [[NSMenuItem alloc] init];
    app_item.submenu = app;
    [bar addItem:app_item];

    // The window's menus, with the Window menu before Help.
    NSMenu* help = nil;
    for (const MenuEntry& e : g_controller->menus()) {
        if (e.kind != MenuEntry::Kind::Submenu) continue;
        NSString* title = [NSString stringWithUTF8String:e.label.c_str()];
        if (e.label == "Help") {
            NSMenu* window = [[NSMenu alloc] initWithTitle:@"Window"];
            [window addItemWithTitle:@"Minimize" action:@selector(performMiniaturize:) keyEquivalent:@"m"];
            [window addItemWithTitle:@"Zoom" action:@selector(performZoom:) keyEquivalent:@""];
            [window addItem:[NSMenuItem separatorItem]];
            [window addItemWithTitle:@"Bring All to Front" action:@selector(arrangeInFront:) keyEquivalent:@""];
            NSMenuItem* window_item = [[NSMenuItem alloc] initWithTitle:@"Window" action:NULL keyEquivalent:@""];
            window_item.submenu = window;
            [bar addItem:window_item];
            NSApp.windowsMenu = window;
        }
        NSMenu* menu = [[NSMenu alloc] initWithTitle:title];
        menu.autoenablesItems = NO;
        menu.delegate = g_controller;
        [g_controller fill:menu from:e.children];  // key equivalents work before it is first opened
        NSMenuItem* holder = [[NSMenuItem alloc] initWithTitle:title action:NULL keyEquivalent:@""];
        holder.submenu = menu;
        [bar addItem:holder];
        if (e.label == "Help") help = menu;
    }
    if (help) NSApp.helpMenu = help;
    NSApp.mainMenu = bar;
    return true;
}

}  // namespace sg::app
