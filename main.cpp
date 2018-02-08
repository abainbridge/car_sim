// Project headers
#include "vector2.h"

// Deadfrog lib headers
#include "df_time.h"
#include "df_font_aa.h"
#include "df_window.h"

// Platform headers
#include <windows.h>

// Standard headers
#include <algorithm>


using std::min;

DfFontAa *g_font = NULL;
double g_advanceTime = 0.0;
double g_frameStartTime = 0.0;
double g_renderScale = 18.0;


void DrawVector(Vector2 start, Vector2 direction)
{
    Vector2 end = start + direction;
    Vector2 ortho = direction.GetPerpendicular().Normalize() * 0.3;
    double directionLen = direction.Len();
    direction.SetLen(directionLen - 0.3);
    Vector2 nearEnd = start + direction;
    Vector2 leftNearEnd = (nearEnd - ortho) * g_renderScale;
    Vector2 rightNearEnd = (nearEnd + ortho) * g_renderScale;
    start *= g_renderScale;
    end *= g_renderScale;
    DrawLine(g_window->bmp, start.x, start.y, end.x, end.y, g_colourWhite);
    DrawLine(g_window->bmp, leftNearEnd.x, leftNearEnd.y, end.x, end.y, g_colourWhite);
    DrawLine(g_window->bmp, rightNearEnd.x, rightNearEnd.y, end.x, end.y, g_colourWhite);
}


const double halfFrontTrack = 0.75;
const double halfRearTrack = halfFrontTrack;
const double halfWheelbase = 1.165;
const double len = 3.995;
const double halfLen = len / 2.0;
const double halfFrontWidth = 0.82;
const double halfRearWidth = 0.9;
const double halfWheelSize = 0.35;
const double mass = 1095.0;
const double momentInertia = (mass * len * len) / 12.0; // Formula is for a rod.
const double coefFriction = 0.7;
const double physicsTimestep = 0.002;
const double gravity = 9.81;
const double maxSlipAngleRadians = 0.07;


struct Wheel
{
    Vector2 m_prevPos;
    Vector2 m_pos;
    Vector2 m_front;
    Vector2 m_force;

    void CalcLateralForce() {
        Vector2 moveSinceLastUpdate = m_pos - m_prevPos;
        moveSinceLastUpdate.Normalize();

        double slipAngle = moveSinceLastUpdate.AngleBetween(m_front);
        if (slipAngle > maxSlipAngleRadians)
            slipAngle = maxSlipAngleRadians;
        if (slipAngle < -maxSlipAngleRadians)
            slipAngle = -maxSlipAngleRadians;
        Vector2 ortho = m_front.GetPerpendicular();

        double fractionOfMaxLateralForce = slipAngle / maxSlipAngleRadians;
        double weightOnWheel = mass * gravity / 4.0;
        double forceMagnitude = fractionOfMaxLateralForce * weightOnWheel * coefFriction;
        m_force = ortho * forceMagnitude;
    }
};


struct Skidmarks
{
    static const int MAX_ITEMS = 2000;
    Vector2 m_positions[MAX_ITEMS];
    int m_head;

    Skidmarks() {
        for (int i = 0; i < MAX_ITEMS; i++) {
            m_positions[i].Set(-1e6, -1e6);
        }
        m_head = 0;
    }

    void Add(Vector2 const &pos) {
        m_positions[m_head] = pos;
        m_head++;
        if (m_head >= MAX_ITEMS) {
            m_head = 0;
        }
    }

    void Render() {
        for (int i = 0; i < MAX_ITEMS; i++) {
            if (m_positions[i].x > 0.0) {
                PutPix(g_window->bmp, m_positions[i].x * g_renderScale, m_positions[i].y * g_renderScale, g_colourWhite);
            }
        }
    }
};


struct Car
{
    Vector2 m_pos;
    Vector2 m_front;
    Vector2 m_vel;
    double m_angVel;
    double m_steeringAngle;
    double m_advanceTimeRemainder;

    Wheel m_wheels[4]; // Order is: front left, front right, rear right, rear left.
    Skidmarks m_skidmarks;

    Car() {
        m_pos.Set(15.0, 15.0);
        m_front.Set(1.0, 0.0);
        m_vel = m_front * 1.0;
        m_angVel = 0.0;
        m_steeringAngle = 0.0;
        m_advanceTimeRemainder = 0.0;

        UpdateWheels();
        for (int i = 0; i < 4; i++) {
            m_wheels[i].m_prevPos = m_wheels[i].m_pos;
        }
    }

    void UpdateWheels() {
        for (int i = 0; i < 4; i++) {
            m_wheels[i].m_prevPos = m_wheels[i].m_pos;
        }

        const Vector2 right(m_front.y, -m_front.x);
        m_wheels[0].m_pos = m_pos + m_front * halfWheelbase - right * halfFrontTrack;
        m_wheels[1].m_pos = m_wheels[0].m_pos + right * halfFrontTrack * 2.0;
        m_wheels[2].m_pos = m_pos - m_front * halfWheelbase + right * halfRearTrack;
        m_wheels[3].m_pos = m_wheels[2].m_pos - right * halfRearTrack * 2.0;

        for (int i = 0; i < 4; i++) {
            m_wheels[i].m_front = m_front;
        }

        m_wheels[0].m_front.Rotate(m_steeringAngle);
        m_wheels[1].m_front.Rotate(m_steeringAngle);
    }

    void AdvanceStep() {
        // Calculate forces, acceleration and angular acceleration.
        Vector2 accel;
        double angularAccel = 0.0;

        for (int i = 0; i < 4; i++) {
            m_wheels[i].CalcLateralForce();
            accel += m_wheels[i].m_force;
            
            Vector2 carCentreToWheel = m_wheels[i].m_pos - m_pos;
            double projectedForce = carCentreToWheel.AngleBetween(m_wheels[i].m_force) * m_wheels[i].m_force.Len();
            double torque = projectedForce * carCentreToWheel.Len();
            angularAccel -= torque;
        }
        accel *= 1.0 / mass;
        angularAccel /= momentInertia;

        // Update velocity and angular velocity.
        m_vel += accel * physicsTimestep;
        m_angVel += angularAccel * physicsTimestep;

        // Update position and orientation.
        m_pos += m_vel * physicsTimestep;
        m_front.Rotate(m_angVel * physicsTimestep);
        m_front.Normalize();

        UpdateWheels();
    }

    void Advance()
    {
        const double MAX_STEERING_LOCK = 0.7;
        m_steeringAngle += g_input.mouseVelX * 0.003;
        if (m_steeringAngle > MAX_STEERING_LOCK) m_steeringAngle = MAX_STEERING_LOCK;
        if (m_steeringAngle < -MAX_STEERING_LOCK) m_steeringAngle = -MAX_STEERING_LOCK;

        if (g_input.rmb)
            m_vel += m_front * (g_advanceTime * 3.0);
        if (g_input.lmb)
            m_vel -= m_front * (g_advanceTime * 6.0);

        double timeToAdvance = g_advanceTime + m_advanceTimeRemainder;
        while (timeToAdvance > 0.0) {
            AdvanceStep();
            timeToAdvance -= physicsTimestep;
        }
        m_advanceTimeRemainder = timeToAdvance;

        for (int i = 0; i < 4; i++) {
            m_skidmarks.Add(m_wheels[i].m_pos);
        }
    }

    void Render() {
        const Vector2 right(m_front.y, -m_front.x);

        m_skidmarks.Render();

        
        //
        // Draw body
        
        Vector2 a = m_pos + m_front * halfLen - right * halfFrontWidth;
        Vector2 b = a + right * halfFrontWidth * 2.0;
        Vector2 c = m_pos - m_front * halfLen + right * halfRearWidth;
        Vector2 d = c - right * halfRearWidth * 2.0;
        a *= g_renderScale;
        b *= g_renderScale;
        c *= g_renderScale;
        d *= g_renderScale;

        DrawLine(g_window->bmp, a.x, a.y, b.x, b.y, g_colourWhite);
        DrawLine(g_window->bmp, b.x, b.y, c.x, c.y, g_colourWhite);
        DrawLine(g_window->bmp, c.x, c.y, d.x, d.y, g_colourWhite);
        DrawLine(g_window->bmp, d.x, d.y, a.x, a.y, g_colourWhite);


        //
        // Draw Wheels

        for (int i = 0; i < 4; i++) {
            Vector2 a = m_wheels[i].m_pos + m_wheels[i].m_front * halfWheelSize;
            Vector2 b = a - m_wheels[i].m_front * halfWheelSize * 2.0;
            a *= g_renderScale;
            b *= g_renderScale;
            DrawLine(g_window->bmp, a.x, a.y, b.x, b.y, g_colourWhite);

            DrawVector(m_wheels[i].m_pos, m_wheels[i].m_force * 0.005);
        }

        double mph = m_vel.Len() * 3600.0 / 1609.3;
        DrawTextLeftAa(g_font, g_colourWhite, g_window->bmp, 10, 10, 20, "MPH: %.1f", mph);
    }
};


int WINAPI WinMain(HINSTANCE _hInstance, HINSTANCE /*_hPrevInstance*/, 
                   LPSTR cmdLine, int /*_iCmdShow*/)
{
    // Setup the window
    int width, height;
    GetDesktopRes(&width, &height);
    CreateWin(width - 200, height - 100, WT_WINDOWED, "Car sim");
    ClearBitmap(g_window->bmp, g_colourWhite);

    g_font = CreateFontAa("Arial", 5);

    Car car;
    ClearBitmap(g_window->bmp, g_colourBlack);
    InputManagerAdvance();

    // Continue to display the window until the user presses escape or clicks the close icon
    while (!g_window->windowClosed)
    {
        InputManagerAdvance();

        double now = DfGetTime();
        g_advanceTime = now - g_frameStartTime;
        g_advanceTime = min(g_advanceTime, 0.1);
        g_frameStartTime = now;

        // Advance Physics
        car.Advance();

        // Render
        ClearBitmap(g_window->bmp, g_colourBlack);

        car.Render();

        UpdateWin();
        DfSleepMillisec(50);
    }

    return 0;
}
