#pragma once

#include <box2d/box2d.h>
#include <SimulationConfig.h>

class Simulation;
static constexpr float MTP = 30.0f;
static constexpr float PTM = 1.0f / MTP;

struct PhysicsSimulation {
public:
    std::vector<b2BodyId> bodyList = std::vector<b2BodyId>();
    Simulation *sim = nullptr;

    // // Конструктор по умолчанию
    // PhysicsSimulation()
    //     : sim(nullptr), world(b2Vec2(0.0f, -10.0f)) {
    //     InitializeWorld();
    // }

    // Конструктор, принимающий ссылку на Simulation
    PhysicsSimulation(Simulation *) {

        InitializeWorld();
    }

    std::vector<Vec2<int> > *MakeHull(int x, int y);

    void Update(float timeStep);

    b2WorldId &GetWorldId();

    const std::vector<b2BodyId> *GetBodyList() const;

    void CreateBody(Vec2<int> pos);

    void CreateBodyWithHull(Vec2<int> pos, std::vector<Vec2<int> > points);

private:
    b2WorldId worldId = b2_nullWorldId;

    // Инициализация мира
    void InitializeWorld() {
        b2WorldDef worldDef = b2DefaultWorldDef();
        worldDef.workerCount = 16;
        worldDef.userTaskContext = this;
        worldDef.enableSleep = true;
        worldId = b2CreateWorld( &worldDef );
        // Параметры зоны
        float offset = 4.0f;
        float left = (0.0f + offset) * PTM;
        float right = (XRES - 1 - offset) * PTM;
        float bottom = (0.0f + offset) * PTM;
        float top = (YRES - 1 - offset) * PTM;

        // Создание тела для границ



        // Нижняя граница
        {
            b2BodyDef groundBodyDef = b2DefaultBodyDef();
            b2BodyId groundBody = b2CreateBody(worldId, &groundBodyDef);
            b2Segment segment = {{left, bottom}, {right, bottom}};
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            b2CreateSegmentShape(groundBody, &shapeDef, &segment);
            bodyList.push_back(groundBody);
        }
        {
            b2BodyDef groundBodyDef = b2DefaultBodyDef();
            b2BodyId groundBody = b2CreateBody(worldId, &groundBodyDef);
            b2Segment segment = {{left, top},{right, top}};
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            b2CreateSegmentShape(groundBody, &shapeDef, &segment);
            bodyList.push_back(groundBody);
        }
        {
            b2BodyDef groundBodyDef = b2DefaultBodyDef();
            b2BodyId groundBody = b2CreateBody(worldId, &groundBodyDef);
            b2Segment segment = {{left, bottom}, {left, top}};
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            b2CreateSegmentShape(groundBody, &shapeDef, &segment);
            bodyList.push_back(groundBody);
        }
        {
            b2BodyDef groundBodyDef = b2DefaultBodyDef();
            b2BodyId groundBody = b2CreateBody(worldId, &groundBodyDef);
            b2Segment segment = {{right, bottom}, {right, top}};
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            b2CreateSegmentShape(groundBody, &shapeDef, &segment);
            bodyList.push_back(groundBody);
        }
    }
};

// Преобразование из Box2D пространства в пиксели
inline Vec2<int> fromB2Space(b2Vec2 pos) {
    return {static_cast<int>(pos.x * MTP), YRES - static_cast<int>(pos.y * MTP) - 1};
}

// Преобразование из пикселей в Box2D пространство
inline b2Vec2 toB2Space(Vec2<int> pos) {
    return {static_cast<float>(pos.X) * PTM, (YRES - static_cast<float>(pos.Y) + 1) * PTM};
}
