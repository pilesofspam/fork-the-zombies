//
// Copyright (c) 2008-2021 the Urho3D project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once

#include "zhelpers.hpp"
#include "Sample.h"

zmq::context_t context(1);
zmq::socket_t subscriber (context, ZMQ_SUB);

namespace Urho3D
{

class Node;
class Scene;

}

/// Skeletal animation example.
/// This sample demonstrates:
///     - Populating a 3D scene with skeletally animated AnimatedModel components;
///     - Moving the animated models and advancing their animation using a custom component
///     - Enabling a cascaded shadow map on a directional light, which allows high-quality shadows
///       over a large area (typically used in outdoor scenes for shadows cast by sunlight)
///     - Displaying renderer debug geometry
class SkeletalAnimation : public Sample
{
    URHO3D_OBJECT(SkeletalAnimation, Sample);

public:
    /// Construct.
    explicit SkeletalAnimation(Context* context);

    /// Setup after engine initialization and before running the main loop.
    void Start() override;

protected:
    SoundSource* musicSource_;
    Sprite* checkerSprite_;
    /// Return XML patch instructions for screen joystick layout for a specific sample app, if any.
    String GetScreenJoystickPatchString() const override { return
        "<patch>"
        "    <remove sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/attribute[@name='Is Visible']\" />"
        "    <replace sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]/element[./attribute[@name='Name' and @value='Label']]/attribute[@name='Text']/@value\">Debug</replace>"
        "    <add sel=\"/element/element[./attribute[@name='Name' and @value='Button1']]\">"
        "        <element type=\"Text\">"
        "            <attribute name=\"Name\" value=\"KeyBinding\" />"
        "            <attribute name=\"Text\" value=\"SPACE\" />"
        "        </element>"
        "    </add>"
        "</patch>";
    }

private:
    /// Construct the scene content.
    void CreateScene();
    /// receive important zmq stuff
    void SetupZMQ();
    /// Let's get some damn zombies out there!
    void StartZombies();
    /// Construct an instruction text to the UI.
    void CreateInstructions();
    /// Set up a viewport for displaying the scene.
    void SetupViewport();
    /// Subscribe to application-wide logic update and post-render update events.
    void SubscribeToEvents();
    /// Read input and moves the camera.
    void MoveCamera(float timeStep);
    /// Handle the logic update event.
    void SpawnObject(IntVector2 pos);
    /// Utility function to raycast to the cursor position. Return true if hit
    bool Raycast(float maxDistance, Vector3& hitPos, Drawable*& hitDrawable);
    /// Handle the logic update event.
    void HandleUpdate(StringHash eventType, VariantMap& eventData);
    /// Handle the post-render update event.
    void HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData);
    /// What to do when the fork hits something
    void HandleNodeCollision(StringHash eventType, VariantMap& eventData);
    /// Spawn a new zombie
    void SpawnZombie(Vector3 whereAmI);
    /// health bar
    void PlotScore(int howMuch);
    void PlotLife(int lifeLeft);
    /// text for game end
    void EndText();
    /// Clear the zombies for a new game
    void ClearZombies();
    /// check for a new ZMQ message coming in
    int CheckMessages();
    /// zombies are hitting me, make noise!
    void PlayHitSound();
    /// checkerboard for calibration
    void CreateChecker();
    /// now you see me
    void CheckerVisible(bool enable);
    /// Flag for drawing debug geometry.
    bool drawDebug_;
};
