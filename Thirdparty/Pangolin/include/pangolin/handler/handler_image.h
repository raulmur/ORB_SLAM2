/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
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

#ifndef PANGOLIN_HANDLER_IMAGE_H
#define PANGOLIN_HANDLER_IMAGE_H

#include <pangolin/image/image_utils.h>
#include <pangolin/display/viewport.h>
#include <pangolin/display/view.h>
#include <pangolin/handler/handler.h>
#include <pangolin/plot/range.h>
#include <pangolin/gl/gl.h>

namespace pangolin
{

class ImageViewHandler : public Handler
{
public:
    // Default constructor: User must call SetDimensions() once image dimensions are known.
    // Default range is [0,1] in x and y.
    ImageViewHandler();

    // View ranges store extremes of image (boundary of pixels)
    // in 'discrete' coords, where 0,0 is center of top-left pixel.
    ImageViewHandler(size_t w, size_t h);

    void SetDimensions(size_t w, size_t h);

    void UpdateView();

    void glSetViewOrtho();

    void glRenderTexture(pangolin::GlTexture& tex);
    void glRenderTexture(GLuint tex, GLint width, GLint height);

    void glRenderOverlay();

    void ScreenToImage(Viewport& v, float xpix, float ypix, float& ximg, float& yimg);

    void ImageToScreen(Viewport& v, float ximg, float yimg, float& xpix, float& ypix);

    bool UseNN() const;

    bool& UseNN();

    pangolin::XYRangef& GetViewToRender();

    float GetViewScale();

    pangolin::XYRangef& GetView();

    pangolin::XYRangef& GetDefaultView();

    pangolin::XYRangef& GetSelection();

    void GetHover(float& x, float& y);

    void SetView(const pangolin::XYRangef& range);

    void SetViewSmooth(const pangolin::XYRangef& range);

    void ScrollView(float x, float y);

    void ScrollViewSmooth(float x, float y);

    void ScaleView(float x, float y, float cx, float cy);

    void ScaleViewSmooth(float x, float y, float cx, float cy);

    void ResetView();

    ///////////////////////////////////////////////////////
    /// pangolin::Handler
    ///////////////////////////////////////////////////////

    void Keyboard(View&, unsigned char key, int /*x*/, int /*y*/, bool pressed) PANGOLIN_OVERRIDE;

    void Mouse(View& view, pangolin::MouseButton button, int x, int y, bool pressed, int button_state) PANGOLIN_OVERRIDE;

    void MouseMotion(View& view, int x, int y, int button_state) PANGOLIN_OVERRIDE;

    void PassiveMouseMotion(View&, int /*x*/, int /*y*/, int /*button_state*/) PANGOLIN_OVERRIDE;

    void Special(View& view, pangolin::InputSpecial inType, float x, float y, float p1, float p2, float /*p3*/, float /*p4*/, int /*button_state*/) PANGOLIN_OVERRIDE;

protected:
    void FixSelection(XYRangef& sel);

    void AdjustScale();

    void AdjustTranslation();

    static ImageViewHandler* to_link;
    static float animate_factor;

    ImageViewHandler* linked_view_handler;

    pangolin::XYRangef rview_default;
    pangolin::XYRangef rview_max;
    pangolin::XYRangef rview;
    pangolin::XYRangef target;
    pangolin::XYRangef selection;

    float hover_img[2];
    int last_mouse_pos[2];

    bool use_nn;
};

}
#endif // PANGOLIN_HANDLER_IMAGE_H
