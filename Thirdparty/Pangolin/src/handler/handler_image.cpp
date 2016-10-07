#include <pangolin/handler/handler_image.h>
#include <pangolin/gl/glfont.h>

namespace pangolin
{

ImageViewHandler::ImageViewHandler()
    : linked_view_handler(0),
      use_nn(false)
{
    SetDimensions(1, 1);
}

ImageViewHandler::ImageViewHandler(size_t w, size_t h)
    : linked_view_handler(0),
      use_nn(false)
{
    SetDimensions(w,h);
}

void ImageViewHandler::SetDimensions(size_t w, size_t h)
{
    rview_default = pangolin::XYRangef(-0.5f, w-0.5f, -0.5f, h-0.5f),
    rview_max = pangolin::XYRangef(-0.5f, w-0.5f, -0.5f, h-0.5f),
    rview = rview_default;
    target = rview;
}

void ImageViewHandler::UpdateView()
{
    // TODO: Base this on current framerate.
    const float animate_factor = 1.0f / 5.0f;

    if( linked_view_handler ) {
        // Synchronise rview and target with linked plotter
        rview = linked_view_handler->rview;
        target = linked_view_handler->target;
        selection = linked_view_handler->selection;
    }else{
        // Clamp target to image dimensions.
        AdjustScale();
        AdjustTranslation();

        // Animate view window toward target
        pangolin::XYRangef d = target - rview;
        rview += d * animate_factor;
    }
}

void ImageViewHandler::glSetViewOrtho()
{
    const pangolin::XYRangef& xy = GetViewToRender();

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(xy.x.min, xy.x.max, xy.y.max, xy.y.min, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void ImageViewHandler::glRenderTexture(pangolin::GlTexture& tex)
{
    glRenderTexture(tex.tid, tex.width, tex.height);
}

void ImageViewHandler::glRenderTexture(GLuint tex, GLint width, GLint height)
{
    const pangolin::XYRangef& xy = GetViewToRender();
    const float w = (float)width;
    const float h = (float)height;

    // discrete coords, (-0.5, -0.5) - (w-0.5, h-0.5)
    const GLfloat l = xy.x.min;
    const GLfloat r = xy.x.max;
    const GLfloat b = xy.y.max;
    const GLfloat t = xy.y.min;

    // continuous coords, (0,0) - (1,1)
    const GLfloat ln = (l + 0.5f) / w;
    const GLfloat rn = (r + 0.5f) / w;
    const GLfloat bn = (b + 0.5f) / h;
    const GLfloat tn = (t + 0.5f) / h;

    const GLfloat sq_vert[]  = { l,t,  r,t,  r,b,  l,b };
    const GLfloat sq_tex[]  = { ln,tn,  rn,tn,  rn,bn,  ln,bn };

    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, UseNN() ? GL_NEAREST : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, UseNN() ? GL_NEAREST : GL_LINEAR);

    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, 0, sq_tex);
    glVertexPointer(2, GL_FLOAT, 0, sq_vert);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void ImageViewHandler::glRenderOverlay()
{
    const pangolin::XYRangef& selxy = GetSelection();
    const GLfloat sq_select[] = {
        selxy.x.min, selxy.y.min,
        selxy.x.max, selxy.y.min,
        selxy.x.max, selxy.y.max,
        selxy.x.min, selxy.y.max
    };
    glColor4f(1.0,0.0,0.0,1.0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 0, sq_select);
    glDrawArrays(GL_LINE_LOOP, 0, 4);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glColor4f(1.0,1.0,1.0,1.0);

    if( std::abs(selxy.Area()) > 0) {
        // Render text
        pangolin::Viewport v;
        glGetIntegerv( GL_VIEWPORT, &v.l );
        float xpix, ypix;
        ImageToScreen(v, selxy.x.max, selxy.y.max, xpix, ypix);

        // Save previous value
        GLboolean gl_blend_enabled;
        glGetBooleanv(GL_BLEND, &gl_blend_enabled);

        // Ensure that blending is enabled for rendering text.
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::GlFont::I().Text(
            "%.2f x %.2f",
            selxy.x.Size(), selxy.y.Size()
        ).DrawWindow(xpix,ypix);

        pangolin::GlFont::I().Text(
            "(%.1f,%.1f)->(%.1f,%.1f)",
            selxy.x.min, selxy.y.min,
            selxy.x.max, selxy.y.max
        ).DrawWindow(xpix, ypix - 1.0f * pangolin::GlFont::I().Height());

        // Restore previous value
        if(!gl_blend_enabled) glDisable(GL_BLEND);
    }
}

void ImageViewHandler::ScreenToImage(Viewport& v, float xpix, float ypix, float& ximg, float& yimg)
{
    ximg = rview.x.min + rview.x.Size() * (xpix - v.l) / (float)v.w;
    yimg = rview.y.min + rview.y.Size() * ( 1.0f - (ypix - v.b) / (float)v.h);
}

void ImageViewHandler::ImageToScreen(Viewport& v, float ximg, float yimg, float& xpix, float& ypix)
{
    xpix = (ximg -rview.x.min) * (float)v.w / rview.x.Size() + v.l;
    ypix = v.b - (float)v.h * ((yimg - rview.y.min) / rview.y.Size() - 1.0f);
}

bool ImageViewHandler::UseNN() const
{
    return use_nn;
}

bool& ImageViewHandler::UseNN()
{
    return use_nn;
}

pangolin::XYRangef& ImageViewHandler::GetViewToRender()
{
    return rview;
}

float ImageViewHandler::GetViewScale()
{
    return rview_max.x.Size() / rview.x.Size();
}

pangolin::XYRangef& ImageViewHandler::GetView()
{
    return target;
}

pangolin::XYRangef& ImageViewHandler::GetDefaultView()
{
    return rview_default;
}

pangolin::XYRangef& ImageViewHandler::GetSelection()
{
    return selection;
}

void ImageViewHandler::GetHover(float& x, float& y)
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    x = tv.hover_img[0];
    y = tv.hover_img[1];
}

void ImageViewHandler::SetView(const pangolin::XYRangef& range)
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    tv.rview = range;
    tv.target = range;
}

void ImageViewHandler::SetViewSmooth(const pangolin::XYRangef& range)
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    tv.target = range;
}

void ImageViewHandler::ScrollView(float x, float y)
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    ScrollViewSmooth(x,y);
    tv.rview.x += x;
    tv.rview.y += y;
}

void ImageViewHandler::ScrollViewSmooth(float x, float y)
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    tv.target.x += x;
    tv.target.y += y;
}

void ImageViewHandler::ScaleView(float x, float y, float cx, float cy)
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    ScaleViewSmooth(x,y,cx,cy);
    tv.rview.x.Scale(x,cx);
    tv.rview.y.Scale(y,cy);
}

void ImageViewHandler::ScaleViewSmooth(float x, float y, float cx, float cy)
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    tv.target.x.Scale(x,cx);
    tv.target.y.Scale(y,cy);
}

void ImageViewHandler::ResetView()
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    tv.target = tv.rview_default;
}

///////////////////////////////////////////////////////
/// pangolin::Handler
///////////////////////////////////////////////////////

void ImageViewHandler::Keyboard(View&, unsigned char key, int /*x*/, int /*y*/, bool pressed)
{
    XYRangef& sel = linked_view_handler ? linked_view_handler->selection : selection;
    const float mvfactor = 1.0f / 10.0f;
    const float c[2] = { rview.x.Mid(), rview.y.Mid() };


    if(pressed) {
        if(key == '\r') {
            if( sel.Area() != 0.0f && std::isfinite(sel.Area()) ) {
                // Set view to equal selection
                SetViewSmooth(sel);

                // Reset selection
                sel.x.max = sel.x.min;
                sel.y.max = sel.y.min;
            }
        }else if(key == 'n') {
            use_nn = !use_nn;
        }else if(key == 'l') {
            if(to_link) {
                linked_view_handler = to_link;
                to_link = 0;
            }else{
                to_link = this;
            }
        }else if(key == PANGO_SPECIAL + PANGO_KEY_LEFT) {
            const float w = target.x.Size();
            const float dx = mvfactor*w;
            ScrollViewSmooth(-dx, 0);
        }else if(key == PANGO_SPECIAL + PANGO_KEY_RIGHT) {
            const float w = target.x.Size();
            const float dx = mvfactor*w;
            ScrollViewSmooth(+dx, 0);
        }else if(key == PANGO_SPECIAL + PANGO_KEY_DOWN) {
            const float h = target.y.Size();
            const float dy = mvfactor*h;
            ScrollViewSmooth(0, -dy);
        }else if(key == PANGO_SPECIAL + PANGO_KEY_UP) {
            const float h = target.y.Size();
            const float dy = mvfactor*h;
            ScrollViewSmooth(0, +dy);
        }else if(key == '=') {
            ScaleViewSmooth(0.5, 0.5, c[0], c[1]);
        }else if(key == '-') {
            ScaleViewSmooth(2.0, 2.0, c[0], c[1]);
        }else if(key == '#') {
            ResetView();
        }else if(key == 1) {
            // ctrl-a: select all.
            sel = rview;
        }else{
            pango_print_debug("Unhandled ImageViewHandler::Keyboard. Key: %u\n", (unsigned int)key);
        }
    }
}

void ImageViewHandler::Mouse(View& view, pangolin::MouseButton button, int x, int y, bool pressed, int button_state)
{
    XYRangef& sel = linked_view_handler ? linked_view_handler->selection : selection;
    ScreenToImage(view.v, (float)x, (float)y, hover_img[0], hover_img[1]);

    const float scinc = 1.05f;
    const float scdec = 1.0f/scinc;

    if(button_state & KeyModifierCtrl) {
        const float mvfactor = 1.0f/20.0f;

        if(button == MouseWheelUp) {
            ScrollViewSmooth(0.0f, +mvfactor*rview.y.Size() );
        }else if(button == MouseWheelDown) {
            ScrollViewSmooth(0.0f, -mvfactor*rview.y.Size() );
        }else if(button == MouseWheelLeft) {
            ScrollViewSmooth(+mvfactor*rview.x.Size(), 0.0f );
        }else if(button == MouseWheelRight) {
            ScrollViewSmooth(-mvfactor*rview.x.Size(), 0.0f );
        }
    }else{
        if(button == MouseButtonLeft) {
            // Update selected range
            if(pressed) {
                sel.x.min = hover_img[0];
                sel.y.min = hover_img[1];
            }
            sel.x.max = hover_img[0];
            sel.y.max = hover_img[1];
        }else if(button == MouseWheelUp) {
            ScaleViewSmooth(scdec, scdec, hover_img[0], hover_img[1]);
        }else if(button == MouseWheelDown) {
            ScaleViewSmooth(scinc, scinc, hover_img[0], hover_img[1]);
        }
    }

    FixSelection(sel);
    last_mouse_pos[0] = x;
    last_mouse_pos[1] = y;
}

void ImageViewHandler::MouseMotion(View& view, int x, int y, int button_state)
{
    XYRangef& sel = linked_view_handler ? linked_view_handler->selection : selection;
    const int d[2] = {x-last_mouse_pos[0], y-last_mouse_pos[1]};

    // Update hover status (after potential resizing)
    ScreenToImage(view.v, (float)x, (float)y, hover_img[0], hover_img[1]);

    if( button_state == MouseButtonLeft )
    {
        // Update selected range
        sel.x.max = hover_img[0];
        sel.y.max = hover_img[1];
    }else if(button_state == MouseButtonRight )
    {
        Special(view, InputSpecialScroll, (float)x, (float)y, (float)d[0], (float)d[1], 0.0f, 0.0f, button_state);
    }

    last_mouse_pos[0] = x;
    last_mouse_pos[1] = y;

}

void ImageViewHandler::PassiveMouseMotion(View&, int /*x*/, int /*y*/, int /*button_state*/)
{
}

void ImageViewHandler::Special(View& view, pangolin::InputSpecial inType, float x, float y, float p1, float p2, float /*p3*/, float /*p4*/, int /*button_state*/)
{
    ScreenToImage(view.v, x, y, hover_img[0], hover_img[1]);

    if(inType == InputSpecialScroll) {
        const float d[2] = {p1,p2};
        const float is[2] = {rview.x.Size(),rview.y.Size() };
        const float df[2] = {is[0]*d[0]/(float)view.v.w, is[1]*d[1]/(float)view.v.h};
        ScrollView(-df[0], df[1]);
    } else if(inType == InputSpecialZoom) {
        float scale = 1.0f - p1;
        ScaleView(scale, scale, hover_img[0], hover_img[1]);
    }

    // Update hover status (after potential resizing)
    ScreenToImage( view.v, x, y, hover_img[0], hover_img[1]);
}

void ImageViewHandler::FixSelection(XYRangef& sel)
{
    // Make sure selection matches sign of current viewport
    if( (sel.x.min<sel.x.max) != (rview.x.min<rview.x.max) ) {
        std::swap(sel.x.min, sel.x.max);
    }
    if( (sel.y.min<sel.y.max) != (rview.y.min<rview.y.max) ) {
        std::swap(sel.y.min, sel.y.max);
    }
}

void ImageViewHandler::AdjustScale()
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    if(tv.target.x.AbsSize() > tv.rview_max.x.AbsSize())
        tv.target.x.Scale(tv.rview_max.x.AbsSize() / tv.target.x.AbsSize(), tv.target.x.Mid());
    if(tv.target.y.AbsSize() > tv.rview_max.y.AbsSize())
        tv.target.y.Scale(tv.rview_max.y.AbsSize() / tv.target.y.AbsSize(), tv.target.y.Mid());
}

void ImageViewHandler::AdjustTranslation()
{
    ImageViewHandler& tv = linked_view_handler ? *linked_view_handler : *this;
    if( tv.target.x.max > tv.rview_max.x.max) tv.target.x -= tv.target.x.max - tv.rview_max.x.max;
    if( tv.target.x.min < tv.rview_max.x.min) tv.target.x -= tv.target.x.min - tv.rview_max.x.min;

    if( tv.target.y.max > tv.rview_max.y.max) tv.target.y -= tv.target.y.max - tv.rview_max.y.max;
    if( tv.target.y.min < tv.rview_max.y.min) tv.target.y -= tv.target.y.min - tv.rview_max.y.min;
}

float ImageViewHandler::animate_factor = 1.0f / 2.0f;
ImageViewHandler* ImageViewHandler::to_link = 0;

}
