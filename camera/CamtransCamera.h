#ifndef CAMTRANSCAMERA_H
#define CAMTRANSCAMERA_H

#include "Camera.h"


/**
 * @class CamtransCamera
 *.2
 * The perspective camera to be implemented in the Camtrans lab.
 */
class CamtransCamera : public Camera {
public:
    // Initialize your camera.
    CamtransCamera();

    // Sets the aspect ratio of this camera. Automatically called by the GUI when the window is
    // resized.
    virtual void setAspectRatio(float aspectRatio);

    // Returns the projection matrix given the current camera settings.
    virtual glm::mat4x4 getProjectionMatrix() const;

    // Returns the view matrix given the current camera settings.
    virtual glm::mat4x4 getViewMatrix() const;

    // Returns the matrix that scales down the perspective view volume into the canonical
    // perspective view volume, given the current camera settings.
    virtual glm::mat4x4 getScaleMatrix() const;

    // Returns the matrix the unhinges the perspective view volume, given the current camera
    // settings.
    virtual glm::mat4x4 getPerspectiveMatrix() const;

    // Returns the current position of the camera.
    glm::vec4 getPosition() const;

    // Returns the current 'look' vector for this camera.
    glm::vec4 getLook() const;

    // Returns the current 'up' vector for this camera (the 'V' vector).
    glm::vec4 getUp() const;

    // Returns the currently set aspect ratio.
    float getAspectRatio() const;

    // Returns the currently set height angle.
    float getHeightAngle() const;

    // Move this camera to a new eye position, and orient the camera's axes given look and up
    // vectors.
    void orientLook(const glm::vec4 &eye, const glm::vec4 &look, const glm::vec4 &up);

    // Sets the height angle of this camera.
    void setHeightAngle(float h);

    // Translates the camera along a given vector.
    void translate(const glm::vec4 &v);

    // Rotates the camera about the U axis by a specified number of degrees.
    void rotateU(float degrees);

    // Rotates the camera about the V axis by a specified number of degrees.
    void rotateV(float degrees);

    // Rotates the camera about the W axis by a specified number of degrees.
    void rotateW(float degrees);

    // Sets the near and far clip planes for this camera.
    void setClip(float nearPlane, float farPlane);


    glm::vec4 getU();
    glm::vec4 getV();
    glm::vec4 getW();


private:
    float m_aspectRatio;
    float m_near, m_far;
    float m_thetaH, m_thetaW;
    glm::vec4 m_eye, m_up;
    glm::mat4 m_translationMatrix, m_perspectiveTransformation;
    glm::mat4 m_scaleMatrix, m_rotationMatrix;

    glm::vec4 m_w, m_v, m_u;

    void updateProjectionMatrix();
    void updatePerspectiveMatrix();
    void updateScaleMatrix();
    void updateViewMatrix();
    void updateRotationMatrix();
    void updateTranslationMatrix();

};

#endif // CAMTRANSCAMERA_H
