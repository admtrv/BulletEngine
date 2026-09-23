/*
 * Launcher.h
 */

#pragma once

#include "project/Settings.h"

#include "interface/elements/Widgets.h"

#include <string>

namespace BulletEngine {
namespace interface {

// what stands before editor, until project is open
class Launcher {
public:
    void draw();

private:
    void drawBar();
    void drawRecent();
    void drawCard(const std::string& path);
    void drawCreate();

    bool open(const std::string& path);     // remembers it when it works

    BulletRender::interface::BrowserState m_browser{.title = "Open Project"};
    bool m_creating = false;                // form is up over list

    // what form is filled with
    char m_name[128] = "NewGame";
    project::Mode m_mode = project::Mode::Mode3D;
    BulletRender::interface::AssetFieldState m_folderSlot{.browser = {.title = "Choose Folder"}};

    std::string m_pendingDelete;            // card asked to go, list is busy being drawn
    std::string m_error;
};

} // namespace interface
} // namespace BulletEngine
