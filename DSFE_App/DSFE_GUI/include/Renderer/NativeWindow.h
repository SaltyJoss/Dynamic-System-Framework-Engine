// DSFE_GUI Renderer/NativeWindow.h
#ifndef DSFE_GUI_NATIVE_WINDOW_H
#define DSFE_GUI_NATIVE_WINDOW_H

#pragma once

namespace renderer {
    // Platform-agnostic native window handle carrier.
    // Windows: handle = HWND, connection unused.
    // Linux/XCB: handle = xcb_window_t (as integer), connection = xcb_connection_t*.
    struct NativeWindow {
        void* handle = nullptr;
        void* connection = nullptr;
    };
}

#endif // DSFE_GUI_NATIVE_WINDOW_H