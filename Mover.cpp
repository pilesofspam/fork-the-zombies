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

#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationState.h>
#include <Urho3D/Graphics/Animation.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Audio/Audio.h>
#include <Urho3D/Audio/AudioEvents.h>
#include <Urho3D/Audio/Sound.h>
#include <Urho3D/Audio/SoundSource.h>


#include "Mover.h"

#include <Urho3D/DebugNew.h>

Mover::Mover(Context* context) :
    LogicComponent(context),
    moveSpeed_(0.0f),
    rotationSpeed_(0.0f)
{
    // Only the scene update event is needed: unsubscribe from the rest for optimization
    SetUpdateEventMask(USE_UPDATE);
}

void Mover::SetParameters(float moveSpeed, float rotationSpeed, const BoundingBox& bounds)
{
    moveSpeed_ = moveSpeed;
    rotationSpeed_ = rotationSpeed;
    bounds_ = bounds;
}

void Mover::Update(float timeStep)
{
    auto* cache = GetSubsystem<ResourceCache>();
    node_->Translate(Vector3::DOWN * moveSpeed_ * timeStep);

    // If in risk of going outside the plane, rotate the model right
    Vector3 pos = node_->GetPosition();
    //if (pos.x_ < bounds_.min_.x_ || pos.x_ > bounds_.max_.x_ || pos.z_ < bounds_.min_.z_ || pos.z_ > bounds_.max_.z_)
    //    node_->Roll(rotationSpeed_ * timeStep);
    if (pos.z_ < bounds_.min_.z_ && moveSpeed_ > 0.0f)
    {
        moveSpeed_ = 0.0f;
        auto* modelObject = node_->GetComponent<AnimatedModel>();
        modelObject->RemoveAllAnimationStates();
        Animation* attackAnimation = cache->GetResource<Animation>("Models/zombie/zombie_attack1.ani");
        if (Random(1.0f) < 0.5f)
        {
            attackAnimation = cache->GetResource<Animation>("Models/zombie/zombie_attack2.ani");
        }

        AnimationState* attack = modelObject->AddAnimationState(attackAnimation);
        if (attack)
        {
            // Enable full blending weight and looping
            attack->SetWeight(1.0f);
            attack->SetLooped(true);
            attack->SetTime(0.0f);
        }
    }

    // Get the model's first (only) animation state and advance its time. Note the convenience accessor to other components
    // in the same scene node
    auto* model = node_->GetComponent<AnimatedModel>(true);
    if (model->GetNumAnimationStates())
    {

        AnimationState* state = model->GetAnimationStates()[0];
        state->AddTime(timeStep);

    }
}
