#pragma once

#include <box2d/box2d.h>
#include <SimulationConfig.h>

class Simulation;
static constexpr float MTP = 30.0f;
static constexpr float PTM = 1.0f / MTP;

struct PhysicsSimulation {
public:
    Simulation *sim = nullptr;

    // // Конструктор по умолчанию
    // PhysicsSimulation()
    //     : sim(nullptr), world(b2Vec2(0.0f, -10.0f)) {
    //     InitializeWorld();
    // }

    // Конструктор, принимающий ссылку на Simulation
    PhysicsSimulation(Simulation*) // TODO: fix broken constructor.
        :  world(b2Vec2(0.0f, -10.0f)) {
        InitializeWorld();
    }

    void Update(float timeStep);

    b2World &GetWorld();

    b2Body *GetBodyList() const;

    void CreateBody(Vec2<int> pos);

private:
    b2World world;

    // Инициализация мира
    void InitializeWorld() {
        // Параметры зоны
        float offset = 4.0f;
        float left = (0.0f + offset) * PTM;
        float right = (XRES - 1 - offset) * PTM;
        float bottom = (0.0f + offset) * PTM;
        float top = (YRES - 1 - offset) * PTM;

        // Создание тела для границ
        b2BodyDef groundBodyDef;
        b2Body *groundBody = world.CreateBody(&groundBodyDef);

        // Создание границ в виде EdgeShape
        b2EdgeShape edge;

        // Нижняя граница
        edge.SetTwoSided(b2Vec2(left, bottom), b2Vec2(right, bottom));
        groundBody->CreateFixture(&edge, 0.0f);

        // Верхняя граница
        edge.SetTwoSided(b2Vec2(left, top), b2Vec2(right, top));
        groundBody->CreateFixture(&edge, 0.0f);

        // Левая граница
        edge.SetTwoSided(b2Vec2(left, bottom), b2Vec2(left, top));
        groundBody->CreateFixture(&edge, 0.0f);

        // Правая граница
        edge.SetTwoSided(b2Vec2(right, bottom), b2Vec2(right, top));
        groundBody->CreateFixture(&edge, 0.0f);

        // создаём динамический ящик (примерно)
        for (int32 i = 0; i < 10; ++i) {
            // Логика создания ящиков (если требуется)
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