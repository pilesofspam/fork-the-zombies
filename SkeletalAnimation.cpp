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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Audio/Audio.h>
#include <Urho3D/Audio/AudioEvents.h>
#include <Urho3D/Audio/Sound.h>
#include <Urho3D/Audio/SoundSource.h>
#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/Animation.h>
#include <Urho3D/Graphics/AnimationState.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/Physics/CollisionShape.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Physics/RigidBody.h>
#include <Urho3D/Physics/PhysicsEvents.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include "Mover.h"
#include "SkeletalAnimation.h"
#include <iostream>
#include <string>
#include "Urho3D/IO/Log.h"
#include <Urho3D/DebugNew.h>

#define INIT        0
#define START       1
#define RUN         2
#define END         3
#define GAMEOVER    5

#define HITMESSAGE  1
#define CALMESSAGE  2
#define CLEARCAL    3

#define SPAWNCHANGE 50
#define SPAWNMIN    300
#define SPAWNSTART  1500
#define checkHit    1000
#define HITDAMAGE   5

URHO3D_DEFINE_APPLICATION_MAIN(SkeletalAnimation)

// yes I put a stupid global in place, sue me
int gameState=INIT;
int score=0;
float messageX, messageY;
int playerHealth=100;
unsigned gameTime=0;
unsigned startTime=0;
unsigned hitTime=0;
int zombieSpawnRate=SPAWNSTART; //in ms
std::vector<zmq::pollitem_t> p;

SkeletalAnimation::SkeletalAnimation(Context* context) :
    Sample(context),
    musicSource_(nullptr),
    drawDebug_(false)
{
    // Register an object factory for our custom Mover component so that we can create them to scene nodes
    context->RegisterFactory<Mover>();
}

void SkeletalAnimation::Start()
{
    Sample::Setup();
    engineParameters_[EP_SOUND] = true;
    SetupZMQ();

    gameTime = Time::GetSystemTime();
    SetRandomSeed(gameTime);
    // Execute base class startup
    Sample::Start();

    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateInstructions();

    // Setup the viewport for displaying the scene
    SetupViewport();

    //create checkerboard for cal
    CreateChecker();

    // Hook up to the frame update and render post-update events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);
    SetLogoVisible(false);

}

void SkeletalAnimation::SetupZMQ()
{

    subscriber.connect("tcp://192.168.1.61:5109");
    subscriber.setsockopt( ZMQ_SUBSCRIBE, "", 0);


}

void SkeletalAnimation::CreateScene()
{
    auto* cache = GetSubsystem<ResourceCache>();

    if (!scene_)
        scene_ = new Scene(context_);
    else
        scene_->Clear();

    // Create music sound source
    musicSource_ = scene_->CreateComponent<SoundSource>();
    // Set the sound type to music so that master volume control works correctly
    musicSource_->SetSoundType(SOUND_MUSIC);
    // Create octree, use default volume (-1000, -1000, -1000) to (1000, 1000, 1000)
    // Also create a DebugRenderer component so that we can draw debug geometry
    scene_->CreateComponent<Octree>();
    scene_->CreateComponent<PhysicsWorld>();
    scene_->CreateComponent<DebugRenderer>();

    // Create scene node & StaticModel component for showing a static plane
    Node* planeNode = scene_->CreateChild("Plane");
    planeNode->SetScale(Vector3(50.0f, 1.0f, 50.0f));
    auto* planeObject = planeNode->CreateComponent<StaticModel>();
    planeObject->SetModel(cache->GetResource<Model>("Models/Plane.mdl"));
    planeObject->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));


    planeNode->CreateComponent<RigidBody>();
    auto* shape = planeNode->CreateComponent<CollisionShape>();
    // Set a box shape of size 1 x 1 x 1 for collision. The shape will be scaled with the scene node scale, so the
    // rendering and physics representation sizes should match (the box model is also 1 x 1 x 1.)
    shape->SetBox(Vector3::ONE);

    // Create a Zone component for ambient lighting & fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    auto* zone = zoneNode->CreateComponent<Zone>();
    zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone->SetAmbientColor(Color(0.5f, 0.5f, 0.5f));
    zone->SetFogColor(Color(0.4f, 0.5f, 0.8f));
    zone->SetFogStart(100.0f);
    zone->SetFogEnd(300.0f);

    // Create a directional light to the world. Enable cascaded shadows on it
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.6f, -1.0f, 0.8f));
    auto* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true);
    light->SetColor(Color(0.5f, 0.5f, 0.5f));
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    // Set cascade splits at 10, 50 and 200 world units, fade shadows out at 80% of maximum shadow distance
    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));
    Node* skyNode = scene_->CreateChild("Sky");
    skyNode->SetScale(500.0f); // The scale actually does not matter
    auto* skybox = skyNode->CreateComponent<Skybox>();
    skybox->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    skybox->SetMaterial(cache->GetResource<Material>("Materials/Skybox.xml"));


    // Create the camera. Limit far clip distance to match the fog
    cameraNode_ = scene_->CreateChild("Camera");
    auto* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(300.0f);

    // Set an initial position for the camera scene node above the plane
    cameraNode_->SetPosition(Vector3(0.0f, 7.0f, -40.0f));
}

void SkeletalAnimation::StartZombies()
{
    // Create animated models
    startTime = Time::GetSystemTime();
    const unsigned NUM_MODELS = 3;
    const float MODEL_MOVE_SPEED = 2.7f;
    const float MODEL_ROTATE_SPEED = 100.0f;
    const BoundingBox bounds(Vector3(-20.0f, 0.0f, -25.0f), Vector3(20.0f, 0.0f, 50.0f));

    for (unsigned i = 0; i < NUM_MODELS; ++i)
    {
        SpawnZombie(Vector3(Random(40.0f) - 20.0f, 0.0f, Random(15.0f)+25.0f));
    }
}

void SkeletalAnimation::CreateInstructions()
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();
    auto* style = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");

    Sound* sound=cache->GetResource<Sound>("Music/background.ogg");
    sound->SetLooped(true);  // sound can be set to be repeated
    // you can use an existing or a new node to append the sound to
    musicSource_->Play(sound);
    GetSubsystem<Audio>()->SetMasterGain(SOUND_MUSIC, 1.0f);


    SharedPtr<Cursor> cursor(new Cursor(context_));
    cursor->SetStyleAuto(style);
    ui->SetCursor(cursor);
    // Construct new Text object, set string to display and font to use
    auto* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetText(
        "FORK\n"
        "the zombies!\n"
        "Fire to begin"
    );
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Creepster-Regular.ttf"), 90);
    // The text has multiple rows. Center them in relation to each other
    instructionText->SetTextAlignment(HA_CENTER);
    instructionText->SetColor(Color(0.0f, 0.0f, 0.0f));

    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, 0);

}

void SkeletalAnimation::SetupViewport()
{
    auto* renderer = GetSubsystem<Renderer>();

    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void SkeletalAnimation::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(SkeletalAnimation, HandleUpdate));

    // Subscribe HandlePostRenderUpdate() function for processing the post-render update event, sent after Renderer subsystem is
    // done with defining the draw calls for the viewports (but before actually executing them.) We will request debug geometry
    // rendering during that event
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(SkeletalAnimation, HandlePostRenderUpdate));

}

void SkeletalAnimation::MoveCamera(float timeStep)
{
    auto* ui = GetSubsystem<UI>();
    auto* input = GetSubsystem<Input>();
    auto* cache = GetSubsystem<ResourceCache>();
    ui->GetCursor()->SetVisible(!input->GetMouseButtonDown(MOUSEB_RIGHT));

    // Do not move if the UI has a focused element (the console)
    if (ui->GetFocusElement())
        return;

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    //cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);

    // "Shoot" a physics object with left mousebutton
    if (input->GetMouseButtonPress(MOUSEB_LEFT))
    {
        Sound* sound;
        if (gameState==INIT)
        {

            musicSource_->Stop();  //kill the startup music
            sound=cache->GetResource<Sound>("Sounds/zombies/zombiewalk.wav");
            sound->SetLooped(true);  // sound can be set to be repeated
            // you can use an existing or a new node to append the sound to
            musicSource_->Play(sound);
            GetSubsystem<Audio>()->SetMasterGain(SOUND_MUSIC, 1.0f);

            StartZombies();
            gameState=RUN;
            ui->GetRoot()->RemoveAllChildren();
            auto* lifeText = ui->GetRoot()->CreateChild<Text>("health");
            auto* scoreText = ui->GetRoot()->CreateChild<Text>("score");
            lifeText->SetText("Health:100");
            lifeText->SetFont(cache->GetResource<Font>("Fonts/Creepster-Regular.ttf"), 90);
            // The text has multiple rows. Center them in relation to each other
            lifeText->SetTextAlignment(HA_CENTER);
            lifeText->SetColor(Color(0.0f, 1.0f, 0.0f));

            // Position the text relative to the screen center
            lifeText->SetHorizontalAlignment(HA_CENTER);
            lifeText->SetVerticalAlignment(VA_BOTTOM);
            lifeText->SetPosition(0, 0);

            scoreText->SetText("Score:0");
            scoreText->SetFont(cache->GetResource<Font>("Fonts/Creepster-Regular.ttf"), 90);
            scoreText->SetTextAlignment(HA_LEFT);
            scoreText->SetColor(Color(0.0f, 0.0f, 1.0f));
            scoreText->SetHorizontalAlignment(HA_LEFT);
            scoreText->SetVerticalAlignment(VA_TOP);
            scoreText->SetPosition(0, 0);
        }
        else if (gameState == RUN)
        {
            IntVector2 debugXY = ui->GetCursorPosition();
            SpawnObject(debugXY);

        }
    }
    // Toggle debug geometry with space
    if (input->GetKeyPress(KEY_SPACE))
        drawDebug_ = !drawDebug_;
}

void SkeletalAnimation::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;
    auto* cache = GetSubsystem<ResourceCache>();
    auto* graphics = GetSubsystem<Graphics>();

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();


    if (gameState == INIT)
    {
        int shouldICalibrate=CheckMessages();
        if (shouldICalibrate == CALMESSAGE)
            CheckerVisible(true);
        else if (shouldICalibrate == CLEARCAL)
            CheckerVisible(false);
    }


    if (gameState == RUN)
    {
        if (CheckMessages() == HITMESSAGE)
        {
            IntVector2 forkShot;
            forkShot.x_ = messageX * graphics->GetWidth();
            forkShot.y_ = messageY * graphics->GetHeight();
            SpawnObject(forkShot);
        }
        if ((gameTime + zombieSpawnRate) < Time::GetSystemTime())
        {
            gameTime = Time::GetSystemTime();
            SpawnZombie(Vector3(Random(40.0f) - 20.0f, 0.0f, Random(15.0f)+25.0f));
            if (zombieSpawnRate > SPAWNMIN)
                zombieSpawnRate=zombieSpawnRate - SPAWNCHANGE; //speed it up each time
        }

        if ((hitTime + checkHit) < Time::GetSystemTime())
        {
            hitTime = Time::GetSystemTime();
            const auto& children = scene_->GetChildren();
            for( unsigned int j = 0; j < children.Size(); ++j ) {
                Node* cnode = children[j];
                if (cnode->GetName() =="zomb")
                {

                    if (cnode->GetPosition().z_ <= -25.0f )
                    {
                        playerHealth = playerHealth - HITDAMAGE;  //remove 1 health for every zombie close
                        PlayHitSound();
                    }
                }
            }
        }
        PlotLife(playerHealth);
        if (playerHealth <= 0)
        {
            gameState=END;
            gameTime = Time::GetSystemTime();
            EndText();

        }

    }
    if  (gameState == END)
    {
        gameState = GAMEOVER;
        zombieSpawnRate=SPAWNSTART;
        musicSource_->Stop();  //kill the startup music
        Sound* sound=cache->GetResource<Sound>("Sounds/zombies/howl.wav");
        sound->SetLooped(false);  // sound can be set to be repeated
        // you can use an existing or a new node to append the sound to
        musicSource_->Play(sound);
        GetSubsystem<Audio>()->SetMasterGain(SOUND_MUSIC, 1.0f);
    }

    if (gameState == GAMEOVER)
    {
        if ((gameTime + 5000) < Time::GetSystemTime())
        {
            auto* ui = GetSubsystem<UI>();
            static_cast<Text*>(ui->GetRoot()->GetChild("health", true))->Remove();
            score=0;
            gameState=INIT;
            musicSource_->Stop();
            playerHealth=100;
            scene_->Clear();
            ClearZombies();  // really, clear everything, forks included
            CreateScene();
            CreateInstructions();

            // Setup the viewport for displaying the scene
            SetupViewport();

            // Hook up to the frame update and render post-update events
            SubscribeToEvents();

            // Set the mouse mode to use in the sample
            Sample::InitMouseMode(MM_RELATIVE);
            SetLogoVisible(false);
        }
    }
    // Move the camera, scale movement with time step
    MoveCamera(timeStep);
}

void SkeletalAnimation::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    // If draw debug mode is enableCheckerVisible(bool enable)d, draw viewport debug geometry, which will show eg. drawable bounding boxes and skeleton
    // bones. Note that debug geometry has to be separately requested each frame. Disable depth test so that we can see the
    // bones properly
    if (drawDebug_)
        GetSubsystem<Renderer>()->DrawDebugGeometry(false);
}

void SkeletalAnimation::SpawnObject(IntVector2 pos)
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* input = GetSubsystem<Input>();
    auto mousePos=input->GetMousePosition();
    auto* graphics = GetSubsystem<Graphics>();
    auto* camera = cameraNode_->GetComponent<Camera>();
    auto* ui = GetSubsystem<UI>();
    auto* sound = cache->GetResource<Sound>("Sounds/zombies/boing.wav");

    if (sound)
    {

        auto* soundSource = scene_->CreateComponent<SoundSource>();
        // Component will automatically remove itself when the sound finished playing
        soundSource->SetAutoRemoveMode(REMOVE_COMPONENT);
        soundSource->Play(sound);
        // In case we also play music, set the sound volume below maximum so that we don't clip the output
        soundSource->SetGain(0.7f);
    }

    std::cout << "X:" << pos.x_;
    std::cout << "  Y:" << pos.y_ << "\n";
    Ray cameraRay = camera->GetScreenRay((float)pos.x_ / graphics->GetWidth(), (float)pos.y_ / graphics->GetHeight());
    std::cout << "DIrection: X:" << cameraRay.direction_.x_ << "  Y:" << cameraRay.direction_.y_ << "  Z:" << cameraRay.direction_.z_ << "\n";
    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild("fork");
    boxNode->SetPosition(cameraNode_->GetPosition());
    //boxNode->SetRotation(Quaternion(pitch_, yaw_, 0.0f));
    boxNode->SetRotation(Quaternion(Vector3(0.0f, 0.0f, 1.0f), cameraRay.direction_));
    boxNode->SetScale(0.0050f);
    auto* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/fork.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/silver.xml"));
    boxObject->SetCastShadows(true);

    // Create physics components, use a smaller mass also
    auto* body = boxNode->CreateComponent<RigidBody>();
    body->SetMass(0.25f);
    body->SetFriction(500.0f);
    auto* shape = boxNode->CreateComponent<CollisionShape>();
    //shape->SetCapsule(1.0f,10.0f);
    //shape->SetTriangleMesh(cache->GetResource<Model>("Models/fork.mdl"));
    shape->SetBox(Vector3(30.0f,30.0f,315.0f));
    shape->SetPosition(Vector3(0.0f,35.0f,0.0f));

    const float OBJECT_VELOCITY = 50.0f;

    // Set initial velocity for the RigidBody based on camera forward vector. Add also a slight up component
    // to overcome gravity better
    //body->SetLinearVelocity(cameraNode_->GetRotation() * Vector3(0.0f, 0.25f, 1.0f) * OBJECT_VELOCITY);
    body->SetLinearVelocity(cameraRay.direction_ * Vector3(1.0f, 1.25f, 1.0f) * OBJECT_VELOCITY);
    SubscribeToEvent(boxNode, E_NODECOLLISION, URHO3D_HANDLER(SkeletalAnimation, HandleNodeCollision));
    body->ApplyTorqueImpulse(Vector3(0.0f,Random(-0.1f,0.1f),0.0f));
}

bool SkeletalAnimation::Raycast(float maxDistance, Vector3& hitPos, Drawable*& hitDrawable)
{
    hitDrawable = nullptr;

    auto* ui = GetSubsystem<UI>();
    IntVector2 pos = ui->GetCursorPosition();
    // Check the cursor is visible and there is no UI element in front of the cursor
    //if (!ui->GetCursor()->IsVisible() || ui->GetElementAt(pos, true))
    //    return false;

    pos = ui->ConvertUIToSystem(pos);

    auto* graphics = GetSubsystem<Graphics>();
    auto* camera = cameraNode_->GetComponent<Camera>();
    Ray cameraRay = camera->GetScreenRay((float)pos.x_ / graphics->GetWidth(), (float)pos.y_ / graphics->GetHeight());
    // Pick only geometry objects, not eg. zones or lights, only get the first (closest) hit
    PODVector<RayQueryResult> results;
    RayOctreeQuery query(results, cameraRay, RAY_TRIANGLE, maxDistance, DRAWABLE_GEOMETRY);
    scene_->GetComponent<Octree>()->RaycastSingle(query);
    if (results.Size())
    {
        RayQueryResult& result = results[0];
        hitPos = result.position_;
        hitDrawable = result.drawable_;
        return true;
    }

    return false;
}

void SkeletalAnimation::HandleNodeCollision(StringHash eventType, VariantMap& eventData)
{
    using namespace NodeCollision;
    auto* cache = GetSubsystem<ResourceCache>();
    auto* otherBody = static_cast<RigidBody*>(eventData[P_OTHERBODY].GetPtr());
    RigidBody* flyingBody = static_cast<RigidBody*>(eventData[NodeCollision::P_BODY].GetPtr());
    URHO3D_LOGINFO( otherBody->GetNode()->GetName());
    flyingBody->GetNode()->Remove();
    if (otherBody->GetNode()->GetName() == "zomb")
    {
        Sound* sound = cache->GetResource<Sound>("Sounds/zombies/hit.wav");
        auto* soundSource = scene_->CreateComponent<SoundSource>();
        //add one to score
        score++;
        PlotScore(score);
        //hit sound
        if (sound)
        {
            // Create a SoundSource component for playing the sound. The SoundSource component plays
            // non-positional audio, so its 3D position in the scene does not matter. For positional sounds the
            // SoundSource3D component would be used instead

            // Component will automatically remove itself when the sound finished playing
            soundSource->SetAutoRemoveMode(REMOVE_COMPONENT);
            soundSource->Play(sound);
            // In case we also play music, set the sound volume below maximum so that we don't clip the output
            soundSource->SetGain(0.75f);
        }

        // die sound
        String zombieSound = "Sounds/zombies/" + String(Random(1,4)) + ".wav";
        Sound* deathSound = cache->GetResource<Sound>(zombieSound);
        auto* newSource = scene_->CreateComponent<SoundSource>();
        if (deathSound)
        {
            // Create a SoundSource component for playing the sound. The SoundSource component plays

            // Component will automatically remove itself when the sound finished playing
            newSource->SetAutoRemoveMode(REMOVE_COMPONENT);
            newSource->Play(deathSound);
            // In case wwe also play music, set the sound volume below maximum so that we don't clip the output
            newSource->SetGain(0.75f);
        }

        auto* mover = otherBody->GetNode()->GetComponent<Mover>();
        const BoundingBox bounds(Vector3(-20.0f, 0.0f, -25.0f), Vector3(20.0f, 0.0f, 20.0f));
        mover->SetParameters(0, 0, bounds);
        Vector3 zomPlacement = otherBody->GetNode()->GetPosition();
        if (zomPlacement.z_ <= bounds.min_.z_)
        {
            zomPlacement.z_ = bounds.min_.z_ + 0.1;
            otherBody->GetNode()->SetPosition(zomPlacement);
        }
        otherBody->GetNode()->SetRotation(Quaternion(90.0f, Random(180.0f) - 100.0f, 0.0f));
        auto* modelObject = otherBody->GetNode()->GetComponent<AnimatedModel>();
        modelObject->RemoveAllAnimationStates();
        auto* deathAnimation = cache->GetResource<Animation>("Models/zombie/zombie_death.ani");
        AnimationState* die = modelObject->AddAnimationState(deathAnimation);
        if (die)
        {
            // Enable full blending weight and looping
            die->SetWeight(1.0f);
            die->SetLooped(false);
            die->SetTime(0.0f);
        }
        auto* zombody = otherBody->GetNode()->GetComponent<RigidBody>();
        // get it outta the way so we can't hit it again
        zombody->ReleaseBody();
    }
    else{
        auto* sound = cache->GetResource<Sound>("Sounds/zombies/miss.wav");

        if (sound)
        {
            // Create a SoundSource component for playing the sound. The SoundSource component plays
            // non-positional audio, so its 3D position in the scene does not matter. For positional sounds the
            // SoundSource3D component would be used instead
            auto* soundSource = scene_->CreateComponent<SoundSource>();
            // Component will automatically remove itself when the sound finished playing
            soundSource->SetAutoRemoveMode(REMOVE_COMPONENT);
            soundSource->Play(sound);
            // In case we also play music, set the sound volume below maximum so that we don't clip the output
            soundSource->SetGain(0.75f);
        }
    }

}

void SkeletalAnimation::SpawnZombie(Vector3 whereAmI)
{
    auto* cache = GetSubsystem<ResourceCache>();
    const float MODEL_MOVE_SPEED = 2.7f;
    const float MODEL_ROTATE_SPEED = 100.0f;
    float zombieAngle = (whereAmI.x_ * 0.4f);  //this way you don't have attacking zombies off screen
    Node* modelNode = scene_->CreateChild("zomb");
    modelNode->SetPosition(whereAmI);
    modelNode->SetRotation(Quaternion(90.0f, zombieAngle, 0.0f));
    const BoundingBox bounds(Vector3(-20.0f, 0.0f, -25.0f), Vector3(20.0f, 0.0f, 50.0f));

    auto* modelObject = modelNode->CreateComponent<AnimatedModel>();
    modelObject->SetModel(cache->GetResource<Model>("Models/zombie/zombie_walk.mdl"));
    auto* walkAnimation=cache->GetResource<Animation>("Models/zombie/zombie_walk2.ani");;
    switch (Random(1,6))
    {
        case 1:
            modelObject->SetMaterial(cache->GetResource<Material>("Models/zombie/Materials/zombie1.xml"));

        break;
        case 2:

            walkAnimation = cache->GetResource<Animation>("Models/zombie/zombie_walk.ani");
            modelObject->SetMaterial(cache->GetResource<Material>("Models/zombie/Materials/zombie2.xml"));
        break;
        case 3:
            modelObject->SetMaterial(cache->GetResource<Material>("Models/zombie/Materials/zombie3.xml"));

        break;
        case 4:
            modelObject->SetMaterial(cache->GetResource<Material>("Models/zombie/Materials/zombie4.xml"));
            //walkAnimation = cache->GetResource<Animation>("Models/zombie/zombie_attack2.ani");
        break;
        default:

            modelObject->SetMaterial(cache->GetResource<Material>("Models/zombie/Materials/zombie5.xml"));
            //walkAnimation = cache->GetResource<Animation>("Models/zombie/zombie_death.ani");

    }
    modelObject->SetCastShadows(true);
    modelNode->SetScale(Vector3(0.1f, 0.1f, 0.1f));
    auto* zombody = modelNode->CreateComponent<RigidBody>();
    // The Trigger mode makes the rigid body only detect collisions, but impart no forces on the
    // colliding objects
    zombody->SetTrigger(true);
    auto* zomshape = modelNode->CreateComponent<CollisionShape>();
    // Create the capsule shape with an offset so that it is correctly aligned with the model, which
    // has its origin at the feet
    zomshape->SetBox(Vector3::ONE * 100);


    // Create an AnimationState for a walk animation. Its time position will need to be manually updated to advance the
    // animation, The alternative would be to use an AnimationController component which updates the animation automatically,
    // but we need to update the model's position manually in any case


    AnimationState* walk = modelObject->AddAnimationState(walkAnimation);

    // The state would fail to create (return null) if the animation was not found
    if (walk)
    {
        // Enable full blending weight and looping
        walk->SetWeight(1.0f);
        walk->SetLooped(true);
        walk->SetTime(Random(walkAnimation->GetLength()));
    }

    // Create our custom Mover component that will move & animate the model during each frame's update
    auto* mover = modelNode->CreateComponent<Mover>();
    mover->SetParameters(MODEL_MOVE_SPEED, MODEL_ROTATE_SPEED, bounds);

}

void SkeletalAnimation::PlotLife(int lifeLeft)
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();
    auto* style = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");
    Text* lifeText = static_cast<Text*>(ui->GetRoot()->GetChild("health", true));
    String healthBar="Health:" + String(lifeLeft);
    lifeText->SetText(healthBar);
    if (lifeLeft < 30)
    {
        lifeText->SetColor(Color(0.0f, 0.0f, 0.0f));
    }
    else
    {
        lifeText->SetColor(Color(0.0f, 1.0f, 0.0f));
    }
}

void SkeletalAnimation::PlotScore(int howMuch)
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();
    auto* style = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");
    Text* scoreText = static_cast<Text*>(ui->GetRoot()->GetChild("score", true));
    String scoreBar="score:" + String(howMuch);
    scoreText->SetText(scoreBar);
    scoreText->SetColor(Color(0.0f, 0.0f, 1.0f));

}

void SkeletalAnimation::EndText()
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();
    auto* style = cache->GetResource<XMLFile>("UI/DefaultStyle.xml");
    Text* lifeText = static_cast<Text*>(ui->GetRoot()->GetChild("health", true));
    lifeText->SetFont(cache->GetResource<Font>("Fonts/Creepster-Regular.ttf"), 140);
    lifeText->SetText("GAME OVER");
    lifeText->SetColor(Color(0.0f, 0.0f, 0.0f));
    lifeText->SetVerticalAlignment(VA_CENTER);
}

void SkeletalAnimation::ClearZombies()
{
    const auto& children = scene_->GetChildren();
    for( unsigned int j = 0; j < children.Size(); ++j ) {
        Node* cnode = children[j];
        if ((cnode->GetName() =="zomb") || (cnode->GetName() == "fork"))
        {
            cnode->RemoveAllChildren();
            cnode->RemoveAllComponents();
            cnode->Remove();

        }
    }
}

int SkeletalAnimation::CheckMessages()
{
    std::string stuff;
    zmq_msg_t message;
    zmq_msg_init(&message);
    int RC = zmq_msg_recv(&message, subscriber, ZMQ_DONTWAIT);
    if (RC != -1)
    {
        stuff = std::string((char*) zmq_msg_data(&message), zmq_msg_size(&message));
        //ok now check for vald spot
        size_t foundX = stuff.find("X:");
        size_t foundY = stuff.find("Y:");
        size_t calmessage = stuff.find("CAL");
        size_t clearmessage = stuff.find("CLEAR");
        if ((foundX != std::string::npos) && (foundY != std::string::npos))
        {
            std::string Xstring = stuff.substr(foundX+2,(foundY-foundX)-2);
            std::string Ystring = stuff.substr(foundY+2,stuff.length());
            messageX = stof(Xstring);
            messageY = stof(Ystring);
            //std::cout << "X:" <<  messageX;
            //std::cout << "  Y:" << messageY << "\n";
            return HITMESSAGE;  //this is a HIT message
        }
        else if (calmessage != std::string::npos)
        {
                return CALMESSAGE; //calibrate message
        }
        else if (clearmessage != std::string::npos)
        {
                return CLEARCAL; //calibrate message
        }
        return 0;
    }
    else
    {
        return 0;
    }
}

void SkeletalAnimation::PlayHitSound()
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* sound = cache->GetResource<Sound>("Sounds/zombies/cartoonpunch.wav");
    if (sound)
    {

        auto* soundSource = scene_->CreateComponent<SoundSource>();
        // Component will automatically remove itself when the sound finished playing
        soundSource->SetAutoRemoveMode(REMOVE_COMPONENT);
        soundSource->Play(sound);
        // In case we also play music, set the sound volume below maximum so that we don't clip the output
        soundSource->SetGain(1.0f);
    }
}

void SkeletalAnimation::CreateChecker()
{
    auto* graphics = GetSubsystem<Graphics>();
    // Get logo texture
    ResourceCache* cache = GetSubsystem<ResourceCache>();
    Texture2D* checkerTexture = cache->GetResource<Texture2D>("Textures/checker16x10_1280.png");
    if (!checkerTexture)
        return;
    // Create logo sprite and add to the UI layout
    UI* ui = GetSubsystem<UI>();
    checkerSprite_ = ui->GetRoot()->CreateChild<Sprite>();
    // Set logo sprite texture
    checkerSprite_->SetTexture(checkerTexture);
    int textureWidth = graphics->GetWidth();
    int textureHeight = graphics->GetHeight();
    // Set logo sprite scale
    checkerSprite_->SetScale(1.0f);
    // Set logo sprite size
    checkerSprite_->SetSize(textureWidth, textureHeight);
    // Set logo sprite hot spot
    checkerSprite_->SetHotSpot(textureWidth, textureHeight);
    // Set logo sprite alignment
    checkerSprite_->SetAlignment(HA_RIGHT, VA_BOTTOM);
    // Make logo fully opaque to hide the scene underneath
    checkerSprite_->SetOpacity(1.0f);
    // Set a high priority so we cover all
    checkerSprite_->SetPriority(100);
    checkerSprite_->SetVisible(false);
}

void SkeletalAnimation::CheckerVisible(bool enable)
{
    checkerSprite_->SetVisible(enable);
}

