#ifndef PANGOLIN_OCULUS_HUD_H
#define PANGOLIN_OCULUS_HUD_H

#include <pangolin/compat/memory.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/glsl.h>
#include <pangolin/handler/handler_glbuffer.h>

#include <OVR.h>

namespace pangolin
{

class OculusHud;

struct HandlerOculus : public pangolin::Handler
{
    HandlerOculus(OculusHud& oculus, pangolin::Handler* h = 0);

    void Keyboard(View&, unsigned char key, int x, int y, bool pressed);
    void Mouse(View&, MouseButton button, int x, int y, bool pressed, int button_state);
    void MouseMotion(View&, int x, int y, int button_state);
    void PassiveMouseMotion(View&, int x, int y, int button_state);
    void Special(View&, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state);

    void SetHandler(pangolin::Handler* h);

protected:
    OculusHud& oculus;
    pangolin::Handler* handler;
};


class OculusHud : public View
{
public:
    OculusHud();

    void Render();
    void RenderFramebuffer();

    void SetHandler(Handler *handler);

    void SetParams(float focalLength, float lensXOffset, float eye_y, float eye_z);

    OVR::HMDInfo& HmdInfo();

    unsigned int NumEyes() const;

    View& CommonView();

    OpenGlMatrix HeadTransform();
    OpenGlMatrix HeadTransformDelta();
    pangolin::GlFramebuffer& Framebuffer();
    pangolin::OpenGlRenderState& DefaultRenderState();

    void UnwarpPoint(unsigned int view, const float in[2], float out[2]);

protected:
    // Oculus SDK Shader for applying lens and chromatic distortion.
    static const char* PostProcessFullFragShaderSrc;

    void InitialiseOculus();
    void InitialiseFramebuffer();
    void InitialiseShader();

    pangolin::GlTexture colourbuffer;
    pangolin::GlRenderBuffer depthbuffer;
    pangolin::GlFramebuffer framebuffer;
    pangolin::GlSlProgram occ;
    bool post_unwarp;

    OVR::Ptr<OVR::DeviceManager> pManager;
    OVR::Ptr<OVR::HMDDevice> pHMD;
    OVR::Ptr<OVR::SensorDevice> pSensor;
    boostd::shared_ptr<OVR::SensorFusion> pFusionResult;
    OVR::HMDInfo HMD;
    OVR::Util::Render::StereoConfig stereo;

    View eyeview[2];
    View common;
    pangolin::OpenGlRenderState default_cam;
    HandlerOculus handler;

    // Center to Oculus transform
    OpenGlMatrix T_oc;
};

}

#endif // PANGOLIN_OCULUS_HUD_H
