#include <pangolin/pangolin.h>
#include <pangolin/hud/oculus_hud.h>
#include <pangolin/compat/bind.h>

// This Oculus sample is experimental - the OculusHud API is subject to change.
int main(int argc, char ** argv) {
    pangolin::CreateWindowAndBind("Main",640,480);
    glEnable(GL_DEPTH_TEST);

    pangolin::OculusHud oculus;
    pangolin::OpenGlRenderState s_cam = oculus.DefaultRenderState();
    s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(-1.5,1.5,-1.5, 0,0,0, pangolin::AxisY));
    oculus.SetHandler(new pangolin::Handler3DFramebuffer(oculus.Framebuffer(), s_cam));

    // Create Pangolin panel (displayed in both views) with sample input variables.
    oculus.CommonView().AddDisplay(
        pangolin::CreatePanel("ui")
            .SetBounds(0.0, 0.6, pangolin::Attach::Pix(150), pangolin::Attach::Pix(-150))
    );
    pangolin::RegisterKeyPressCallback(' ', boostd::bind(&pangolin::View::ToggleShow, boostd::ref(oculus.CommonView()) ) );
    pangolin::Var<bool> a_button("ui.A Button",false,false);
    pangolin::Var<double> a_double("ui.A Double",3,0,5);
    pangolin::Var<int> an_int("ui.An Int",2,0,5);

    while( !pangolin::ShouldQuit() )
    {
        // Update modelview matrix with head transform
        s_cam.GetModelViewMatrix() = oculus.HeadTransformDelta() * s_cam.GetModelViewMatrix();

        oculus.Framebuffer().Bind();
        glClearColor(1,1,1,0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        for(unsigned int view = 0; view < oculus.NumEyes(); ++view)
        {
            oculus[view].Activate();
            s_cam.ApplyNView(view);
            pangolin::glDrawColouredCube();
        }
        oculus.Framebuffer().Unbind();

        pangolin::FinishFrame();
    }

    return 0;
}
