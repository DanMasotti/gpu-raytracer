#include "view.h"

#include <unistd.h>
#include "viewformat.h"
#include <QApplication>
#include <QKeyEvent>
#include <iostream>

#include "lib/ResourceLoader.h"
#include "lib/Sphere.h"
#include "gl/textures/Texture2D.h"

using namespace CS123::GL;

View::View(QWidget *parent) : QGLWidget(ViewFormat(), parent),
    m_FBO1(nullptr),
    m_time(),
    m_timer(),
    m_captureMouse(false),
    m_height(std::min(1000, height())),
    m_width(std::min(1000, width())),
    m_angleX(-0.5f),
    m_angleY(0.5f),
    m_zoom(15.f),
    m_leftSpeed(0.1f),
    m_centerSpeed(0.02f),
    m_rightSpeed(0.01),
    m_sleepTime(1),
    m_depth(2)
{

    // View needs all mouse move events, not just mouse drag events
    setMouseTracking(true);

    // Hide the cursor
    if (m_captureMouse) {
        QApplication::setOverrideCursor(Qt::BlankCursor);
    }

    // View needs keyboard focus
    setFocusPolicy(Qt::StrongFocus);

    // The update loop is implemented using a timer
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(tick()));
}

View::~View()
{
}

void View::initializeGL() {
    // All OpenGL initialization *MUST* be done during or after this
    // method. Before this method is called, there is no active OpenGL
    // context and all OpenGL calls have no effect.

    //initialize glew
    glewExperimental = GL_TRUE;
    GLenum err = glewInit();
    if (GLEW_OK != err) {
        /* Problem: glewInit failed, something is seriously wrong. */
        std::cerr << "Something is very wrong, glew initialization failed." << std::endl;
    }
    std::cout << "Using GLEW " <<  glewGetString( GLEW_VERSION ) << std::endl;

    // Start a timer that will try to get 60 frames per second (the actual
    // frame rate depends on the operating system and other running programs)
    m_time.start();
    m_timer.start(1000 / 60);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    // Full screen quad
    std::string quadVertexSource = ResourceLoader::loadResourceFileToString(":/shaders/quad.vert");
    std::string rayTracerFragmentSource = ResourceLoader::loadResourceFileToString(":/shaders/rayTracer.frag");
    m_rayTracerProgram = std::make_unique<Shader>(quadVertexSource, rayTracerFragmentSource);

    std::vector<GLfloat> quadData{
        -1.f, 1.f, 0.0,
         0.f, 0.f,
        -1.f, -1.f, 0.0,
         0.f, 1.f,
         1.f, 1.f, 0.0,
         1.f, 0.f,
         1.f, -1.f, 0.0,
         1.f, 1.f
    };

    m_quad = std::make_unique<OpenGLShape>();
    m_quad->setVertexData(&quadData[0], quadData.size(), VBO::GEOMETRY_LAYOUT::LAYOUT_TRIANGLE_STRIP, 4);
    m_quad->setAttribute(ShaderAttrib::POSITION, 3, 0, VBOAttribMarker::DATA_TYPE::FLOAT, false);
    m_quad->setAttribute(ShaderAttrib::TEXCOORD0, 2, 3*sizeof(GLfloat), VBOAttribMarker::DATA_TYPE::FLOAT, false);
    m_quad->buildVAO();

    m_FBO1 = std::make_unique<FBO>(1,
                                  FBO::DEPTH_STENCIL_ATTACHMENT::DEPTH_ONLY,
                                  std::min(1000, m_width),
                                  std::min(1000, m_height),
                                  TextureParameters::WRAP_METHOD::CLAMP_TO_EDGE,
                                  TextureParameters::FILTER_METHOD::LINEAR,
                                  GL_FLOAT
                                  );
}

void View::paintGL() {
    m_FBO1->bind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, m_width, m_height);
    m_rayTracerProgram->bind();

    glm::mat4 M_film2World = glm::inverse(m_scale*m_view);
    m_rayTracerProgram->setUniform("M_film2World", M_film2World);

    m_rayTracerProgram->setUniform("time", static_cast<float>(m_time.msec()/1000.f));
    m_rayTracerProgram->setUniform("dimensions", glm::vec2(m_width, m_height));
    m_rayTracerProgram->setUniform("depth", m_depth);
    m_rayTracerProgram->setUniform("rightSpeed", m_rightSpeed);
    m_rayTracerProgram->setUniform("leftSpeed", m_leftSpeed);
    m_rayTracerProgram->setUniform("centerSpeed", m_centerSpeed);

    m_FBO1->getColorAttachment(0).bind();
    m_quad->draw();
    m_FBO1->unbind();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, m_width, m_height);

    m_quad->draw();
}


void View::rebuildMatrices() {
    m_view = glm::translate(glm::vec3(0.f, 0.f, -m_zoom)) *
             glm::rotate(m_angleY, glm::vec3(1,0,0)) *
             glm::rotate(m_angleX, glm::vec3(0,1,0));

    m_projection = (glm::perspective(0.8f, static_cast<float>(m_width/m_height), 0.1f, 100.f));
    m_scale = glm::transpose(glm::mat4({
                           m_width, 0.f, 0.f, 0.f,
                             0.f, m_height, 0.f, 0.f,
                            0.f, 0.f, 100.f, 0.f,
                            0.f, 0.f, 0.f, 1.f
                        }));
    update();
}


void View::resizeGL(int w, int h) {
    float ratio = static_cast<QGuiApplication *>(QCoreApplication::instance())->devicePixelRatio();
    w = static_cast<int>(w / ratio);
    h = static_cast<int>(h / ratio);

    m_width = std::min(w, 1000);
    m_height = std::min(h, 1000);
    glViewport(0, 0, m_width, m_height);

    m_FBO1 = std::make_unique<FBO>(1,
                                  FBO::DEPTH_STENCIL_ATTACHMENT::DEPTH_ONLY,
                                  std::min(1000, m_width),
                                  std::min(1000, m_height),
                                  TextureParameters::WRAP_METHOD::CLAMP_TO_EDGE,
                                  TextureParameters::FILTER_METHOD::LINEAR,
                                  GL_FLOAT
                                  );
    rebuildMatrices();
}

void View::mousePressEvent(QMouseEvent *event) {

}

void View::mouseMoveEvent(QMouseEvent *event) {
    // This starter code implements mouse capture, which gives the change in
    // mouse position since the last mouse movement. The mouse needs to be
    // recentered after every movement because it might otherwise run into
    // the edge of the screen, which would stop the user from moving further
    // in that direction. Note that it is important to check that deltaX and
    // deltaY are not zero before recentering the mouse, otherwise there will
    // be an infinite loop of mouse move events.
    if(m_captureMouse) {
        int deltaX = event->x() - width() / 2;
        int deltaY = event->y() - height() / 2;
        if (!deltaX && !deltaY) return;
        QCursor::setPos(mapToGlobal(QPoint(width() / 2, height() / 2)));

        // TODO: Handle mouse movements here

    }
}

void View::mouseReleaseEvent(QMouseEvent *event) {

}

void View::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_Escape) QApplication::quit();

    // TODO: Handle keyboard presses here
}

void View::keyReleaseEvent(QKeyEvent *event) {

}

// TODO: I don't know why time isn't being sent to fragment
void View::tick() {
    // Get the number of seconds since the last tick (variable update rate)
    float seconds = m_time.restart() * 0.001f;

    // TODO: Implement the demo update here

    // Flag this view for repainting (Qt will call paintGL() soon after)
    Sleep(m_sleepTime);
    update();
}

