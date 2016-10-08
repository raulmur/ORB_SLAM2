/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_DISPLAY_INTERNAL_H
#define PANGOLIN_DISPLAY_INTERNAL_H

#include <pangolin/platform.h>
#include <pangolin/display/window.h>

#include <pangolin/display/view.h>
#include <pangolin/display/user_app.h>
#include <pangolin/compat/function.h>
#include <pangolin/compat/memory.h>

#include <map>
#include <queue>

#ifdef BUILD_PANGOLIN_VIDEO
#  include <pangolin/video/video_output.h>
#endif // BUILD_PANGOLIN_VIDEO


namespace pangolin
{

// Forward Declarations
#ifdef HAVE_PYTHON
class ConsoleView;
#endif // HAVE_PYTHON
class GlFont;

typedef std::map<const std::string,View*> ViewMap;
typedef std::map<int,boostd::function<void(void)> > KeyhookMap;

struct PANGOLIN_EXPORT PangolinGl : public WindowInterface
{
    PangolinGl();
    ~PangolinGl();
    
    // Base container for displays
    View base;
    
    // Named views which are managed by pangolin (i.e. created / deleted by pangolin)
    ViewMap named_managed_views;

    // Optional user app
    UserApp* user_app;
    
    // Global keypress hooks
    KeyhookMap keypress_hooks;
    
    // Manage fullscreen (ToggleFullscreen is quite new)
    bool is_double_buffered;
    bool is_fullscreen;
    GLint windowed_size[2];
    
    // State relating to interactivity
    bool quit;
    int had_input;
    int has_resized;
    int mouse_state;
    View* activeDisplay;
    
    std::queue<std::pair<std::string,Viewport> > screen_capture;
    
#ifdef BUILD_PANGOLIN_VIDEO
    View* record_view;
    VideoOutput recorder;
#endif

#ifdef HAVE_PYTHON
    ConsoleView* console_view;
#endif

    boostd::shared_ptr<GlFont> font;

    virtual void ToggleFullscreen() PANGOLIN_OVERRIDE {
        pango_print_warn("ToggleFullscreen: Not available with non-pangolin window.\n");
    }

    virtual void ProcessEvents() PANGOLIN_OVERRIDE {
        pango_print_warn("ProcessEvents: Not available with non-pangolin window.\n");
    }

    virtual void SwapBuffers() PANGOLIN_OVERRIDE {
        pango_print_warn("SwapBuffers: Not available with non-pangolin window.\n");
    }

    virtual void MakeCurrent() PANGOLIN_OVERRIDE {
        pango_print_warn("MakeCurrent: Not available with non-pangolin window.\n");
    }

    virtual void Move(int /*x*/, int /*y*/) PANGOLIN_OVERRIDE {
        pango_print_warn("Move: Not available with non-pangolin window.\n");
    }

    virtual void Resize(unsigned int /*w*/, unsigned int /*h*/) PANGOLIN_OVERRIDE {
        pango_print_warn("Resize: Not available with non-pangolin window.\n");
    }


};

PangolinGl* GetCurrentContext();
void AddNewContext(const std::string& name, boostd::shared_ptr<PangolinGl> newcontext);
void DeleteContext(const std::string& name);
PangolinGl *FindContext(const std::string& name);

}

#endif // PANGOLIN_DISPLAY_INTERNAL_H

