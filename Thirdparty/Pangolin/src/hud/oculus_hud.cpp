#include <pangolin/pangolin.h>
#include <pangolin/hud/oculus_hud.h>
#include <pangolin/display/display_internal.h>


namespace pangolin
{

std::ostream& operator<<(std::ostream& s, const Viewport& v)
{
    s << v.l << "," << v.b << ": " << v.w << "x" << v.h;
    return s;
}

OculusHud::OculusHud()
    : post_unwarp(true), handler(*this)
{
    InitialiseOculus();
    InitialiseFramebuffer();
    InitialiseShader();

    // Make this the default fullscreen view
    DisplayBase().AddDisplay(*this);
    SetBounds(Attach::Pix(0),Attach::Pix(HMD.VResolution),Attach::Pix(0),Attach::Pix(HMD.HResolution));
    handler.SetHandler(&pangolin::StaticHandler);
    View::SetHandler(&handler);

    // Initialise per-eye views / parameters
    for(int i=0; i<2; ++i) {
        int hw = HMD.HResolution/2;
        eyeview[i].SetBounds(Attach::Pix(0),Attach::Pix(HMD.VResolution),Attach::Pix(i*hw),Attach::Pix((1+i)*hw));
        this->AddDisplay(eyeview[i]);
    }

    // Setup default projection parameters
    const OVR::Util::Render::DistortionConfig& Distortion = stereo.GetDistortionConfig();
    const float ppos = HMD.HResolution * Distortion.XCenterOffset / 4.0f;
    SetParams(204.4, ppos, 0.177, -0.083);

    common.SetHandler(&pangolin::StaticHandler);
    this->AddDisplay(common);

    T_oc = IdentityMatrix();

    pangolin::SetFullscreen();
}

void OculusHud::SetHandler(Handler *h)
{
    for(int i=0; i<2; ++i) {
        eyeview[i].SetHandler(h);
    }
}

void OculusHud::SetParams(float focalLength, float lensXOffset, float eye_y, float eye_z )
{
    const float eye_dist = HMD.InterpupillaryDistance / 2.0;

    for(int i=0; i<2; ++i) {
//        OVR::Util::Render::StereoEyeParams eye =
//                stereo.GetEyeRenderParams( OVR::Util::Render::StereoEye(1+i) );
//        default_cam.GetProjectionMatrix(i) = eye.Projection;
//        default_cam.GetViewOffset(i) = eye.ViewAdjust;

        default_cam.GetProjectionMatrix(i) =
                pangolin::ProjectionMatrix(
                    HMD.HResolution/2.0, HMD.VResolution,
                    focalLength, focalLength,
                    HMD.HResolution/4.0 -(i*2-1)*lensXOffset, HMD.VResolution/2.0,
                    0.2,100
                    );

        default_cam.GetViewOffset(i) =
                pangolin::OpenGlMatrix::Translate( -(i*2-1)*eye_dist, -eye_y, -eye_z);;
    }
}

OVR::HMDInfo& OculusHud::HmdInfo()
{
    return HMD;
}

unsigned int OculusHud::NumEyes() const
{
    return 2;
}

View& OculusHud::CommonView()
{
    return common;
}

OpenGlMatrix OculusHud::HeadTransform()
{
    const OVR::Quatf q = pFusionResult->GetOrientation();
    const OVR::Matrix4f T_oc = OVR::Matrix4f(q).Inverted();
    return OpenGlMatrix(T_oc);
}

OpenGlMatrix OculusHud::HeadTransformDelta()
{
    OpenGlMatrix T_2c = HeadTransform();
    OpenGlMatrix T_21 = T_2c * T_oc.Inverse();

    // Update
    T_oc = T_2c;

    return T_21;
}


pangolin::GlFramebuffer& OculusHud::Framebuffer()
{
    return framebuffer;
}

pangolin::OpenGlRenderState& OculusHud::DefaultRenderState()
{
    return default_cam;
}

void OculusHud::UnwarpPoint(unsigned int view, const float in[2], float out[2])
{
    const OVR::Util::Render::DistortionConfig& Distortion = stereo.GetDistortionConfig();

    const int   sx = 1-view*2;
    const float cx = 0.25 + view*0.5;
    const float as = stereo.GetAspect();

    float LensCenter[] = { cx + sx*Distortion.XCenterOffset / 4.0f, 0.5f };
    float Scale[] = { 1.0f / (4.0f*Distortion.Scale), as / (2.0f*Distortion.Scale) };
    float ScaleIn[] = {4.0f, 2.0f / as};

    float theta[] = {
        ScaleIn[0] * (in[0] / v.w - LensCenter[0]),
        ScaleIn[1] * (in[1] / v.h - LensCenter[1])
    };

    const float r = std::sqrt(theta[0]*theta[0] + theta[1]*theta[1]);
    const float nr = const_cast<OVR::Util::Render::DistortionConfig&>(Distortion).DistortionFn(r);

    float theta1[] = {
        nr * theta[0] / r,
        nr * theta[1] / r
    };

    out[0] = v.w * (LensCenter[0] + Scale[0] * theta1[0]);
    out[1] = v.h * (LensCenter[1] + Scale[1] * theta1[1]);
}

void OculusHud::RenderFramebuffer()
{    
    Activate();

    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glColor3f(1.0,1.0,1.0);

    if(post_unwarp) {
        const float as = stereo.GetAspect();
        const OVR::Util::Render::DistortionConfig& Distortion = stereo.GetDistortionConfig();

        // Render framebuffer views to screen via distortion shader
        for(int i=0; i<2; ++i)
        {
            const int   sx = 1-i*2;
            const float cx = 0.25 + i*0.5;
            occ.Bind();
            occ.SetUniform("LensCenter", cx + sx*Distortion.XCenterOffset / 4.0f, 0.5f);
            occ.SetUniform("ScreenCenter", cx, 0.5f);
            occ.SetUniform("ScaleIn", 4.0f, 2.0f / as);
            occ.SetUniform("Scale",   1.0f / (4.0f*Distortion.Scale), as / (2.0f*Distortion.Scale) );
            occ.SetUniform("HmdWarpParam", HMD.DistortionK[0], HMD.DistortionK[1], HMD.DistortionK[2], HMD.DistortionK[3] );
            occ.SetUniform("ChromAbParam", HMD.ChromaAbCorrection[0], HMD.ChromaAbCorrection[1], HMD.ChromaAbCorrection[2], HMD.ChromaAbCorrection[3]);

            pangolin::Viewport& vp = eyeview[i].v;
            vp.Activate();
            colourbuffer.RenderToViewport(vp, false);

            occ.Unbind();
        }
    }else{
        this->Activate();
        colourbuffer.RenderToViewport(false);
    }
}

void OculusHud::Render()
{
    if(show) {
        // If render function defined, use it to render left / right images
        // to framebuffer
        if(this->extern_draw_function) {
            framebuffer.Bind();
            glClearColor(1,1,1,0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            for(int i=0; i<2; ++i) {
                eyeview[i].Activate();
                default_cam.ApplyNView(i);
                this->extern_draw_function(*this);
            }
            framebuffer.Unbind();
        }

        if(common.IsShown() && common.NumChildren()) {
            framebuffer.Bind();
            RenderChildren();
            for(int i=0; i<2; ++i) {
                Viewport v = eyeview[i].v;
                v.l += 160 -i*120;
                v.w -= 160;
                v.h -= 220;
                common.Resize(v);
                common.Render();
            }
            framebuffer.Unbind();
        }

        RenderFramebuffer();
    }
}

const char* OculusHud::PostProcessFullFragShaderSrc =
    "uniform vec2 LensCenter;\n"
    "uniform vec2 ScreenCenter;\n"
    "uniform vec2 Scale;\n"
    "uniform vec2 ScaleIn;\n"
    "uniform vec4 HmdWarpParam;\n"
    "uniform vec4 ChromAbParam;\n"
    "uniform sampler2D Texture0;\n"
    "\n"
    // Scales input texture coordinates for distortion.
    // ScaleIn maps texture coordinates to Scales to ([-1, 1]), although top/bottom will be
    // larger due to aspect ratio.
    "void main()\n"
    "{\n"
    "   vec2  theta = (gl_TexCoord[0].xy - LensCenter) * ScaleIn;\n" // Scales to [-1, 1]
    "   float rSq= theta.x * theta.x + theta.y * theta.y;\n"
    "   vec2  theta1 = theta * (HmdWarpParam.x + rSq*(HmdWarpParam.y + rSq*(HmdWarpParam.z + rSq*HmdWarpParam.w) ) );\n"
    "   \n"
    "   // Detect whether blue texture coordinates are out of range since these will scaled out the furthest.\n"
    "   vec2 thetaBlue = theta1 * (ChromAbParam.z + ChromAbParam.w * rSq);\n"
    "   vec2 tcBlue = LensCenter + Scale * thetaBlue;\n"
    "   if (!all(equal(clamp(tcBlue, ScreenCenter-vec2(0.25,0.5), ScreenCenter+vec2(0.25,0.5)), tcBlue)))\n"
    "   {\n"
    "       gl_FragColor = vec4(0);\n"
    "       return;\n"
    "   }\n"
    "   \n"
    "   // Now do blue texture lookup.\n"
    "   float blue = texture2D(Texture0, tcBlue).b;\n"
    "   \n"
    "   // Do green lookup (no scaling).\n"
    "   vec2  tcGreen = LensCenter + Scale * theta1;\n"
    "   vec4  center = texture2D(Texture0, tcGreen);\n"
    "   \n"
    "   // Do red scale and lookup.\n"
    "   vec2  thetaRed = theta1 * (ChromAbParam.x + ChromAbParam.y * rSq);\n"
    "   vec2  tcRed = LensCenter + Scale * thetaRed;\n"
    "   float red = texture2D(Texture0, tcRed).r;\n"
    "   \n"
    "   gl_FragColor = vec4(red, center.g, blue, 1);\n"
    "}\n";

void OculusHud::InitialiseOculus()
{
    OVR::System::Init();
    pManager = *OVR::DeviceManager::Create();
    if(!pManager) {
        throw std::runtime_error("Unable to create Oculus device manager\n");
    }

    pHMD = *pManager->EnumerateDevices<OVR::HMDDevice>().CreateDevice();
    if (pHMD) {
        pHMD->GetDeviceInfo(&HMD);
        pSensor = *pHMD->GetSensor();
        if (pSensor) {
            pFusionResult = boostd::shared_ptr<OVR::SensorFusion>(new OVR::SensorFusion());
            pFusionResult->AttachToSensor(pSensor);
        }else{
            pango_print_error("Unable to get sensor\n");
        }
    }else{
        pango_print_error("Unable to create device\n");
        // Set defaults instead
        HMD.HResolution            = 1280;
        HMD.VResolution            = 800;
        HMD.HScreenSize            = 0.14976f;
        HMD.VScreenSize            = HMD.HScreenSize / (1280.0/800.0);
        HMD.InterpupillaryDistance = 0.064f;
        HMD.LensSeparationDistance = 0.0635f;
        HMD.EyeToScreenDistance    = 0.041f;
        HMD.DistortionK[0]         = 1.0f;
        HMD.DistortionK[1]         = 0.22f;
        HMD.DistortionK[2]         = 0.24f;
        HMD.DistortionK[3]         = 0;
    }

    stereo.SetStereoMode(OVR::Util::Render::Stereo_LeftRight_Multipass);
    stereo.SetFullViewport(OVR::Util::Render::Viewport(0,0, HMD.HResolution, HMD.VResolution) );
    stereo.SetHMDInfo(HMD);

    if (HMD.HScreenSize > 0.140f) {
        stereo.SetDistortionFitPointVP(-1.0f, 0.0f);
    }else{
        stereo.SetDistortionFitPointVP(0.0f, 1.0f);
    }
}

void OculusHud::InitialiseFramebuffer()
{
    colourbuffer.Reinitialise(HMD.HResolution, HMD.VResolution, GL_RGBA8);
    depthbuffer.Reinitialise(HMD.HResolution, HMD.VResolution, GL_DEPTH_COMPONENT24);
    framebuffer.AttachDepth(depthbuffer);
    framebuffer.AttachColour(colourbuffer);
}

void OculusHud::InitialiseShader()
{
    occ.AddShader(pangolin::GlSlFragmentShader, PostProcessFullFragShaderSrc);
    occ.Link();
}

HandlerOculus::HandlerOculus(OculusHud& oculus, pangolin::Handler* handler)
    : oculus(oculus), handler(handler)
{

}

// Pointer to context defined in display.cpp
extern __thread PangolinGl* context;

void HandlerOculus::Keyboard(View& d, unsigned char key, int x, int y, bool pressed)
{
    if(handler) {
        const float in[] = { (float)x, (float)y};
        float out[2];
        oculus.UnwarpPoint(x < d.v.w/2 ?0:1, in, out );
        handler->Keyboard(d, key, out[0], out[1], pressed);
        context->activeDisplay = &d;
    }
}

void HandlerOculus::Mouse(View& d, MouseButton button, int x, int y, bool pressed, int button_state)
{
    if(handler) {
        const float in[] = { (float)x, (float)y};
        float out[2];
        oculus.UnwarpPoint(x < d.v.w/2 ?0:1, in, out );
        handler->Mouse(d, button, out[0], out[1], pressed, button_state);
        context->activeDisplay = &d;
    }
}

void HandlerOculus::MouseMotion(View& d, int x, int y, int button_state)
{
    if(handler) {
        const float in[] = { (float)x, (float)y};
        float out[2];
        oculus.UnwarpPoint(x < d.v.w/2 ?0:1, in, out );
        handler->MouseMotion(d, out[0], out[1], button_state);
        context->activeDisplay = &d;
    }
}

void HandlerOculus::PassiveMouseMotion(View& d, int x, int y, int button_state)
{
    if(handler) {
        const float in[] = { (float)x, (float)y};
        float out[2];
        oculus.UnwarpPoint(x < d.v.w/2 ?0:1, in, out );
        handler->PassiveMouseMotion(d, out[0], out[1], button_state);
        context->activeDisplay = &d;
    }
}

void HandlerOculus::Special(View& d, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state)
{
    if(handler) {
        const float in[] = { (float)x, (float)y};
        float out[2];
        oculus.UnwarpPoint(x < d.v.w/2 ?0:1, in, out );
        handler->Special(d, inType, out[0], out[1], p1, p2, p3, p4, button_state);
        context->activeDisplay = &d;
    }
}

void HandlerOculus::SetHandler(pangolin::Handler* h)
{
    handler = h;
}


}
