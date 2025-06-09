#include <box2d/box2d.h>
#include "PhysicsSimulation.h"

#include <common/Vec2.h>
#include <json/config.h>

#include "Simulation.h"
#include "common/tpt-compat.h"
#include "ElementClasses.h"


struct GluedParticle {
    int partIndex;
    Vec2<int> offset;
};

struct Glue {
    std::vector<GluedParticle> *usedParts;
};

void PhysicsSimulation::Update(float timeStep) {
    world.Step(timeStep, 6, 2);

    for (auto body = world.GetBodyList(); body; body = body->GetNext()) {
        auto b2pos = body->GetPosition();
        auto pos = fromB2Space({b2pos.x, b2pos.y});
        b2BodyUserData userData = body->GetUserData();
        if (userData.pointer == 0) {
            continue;
        }

        auto angle = -body->GetAngle();

        Glue *data = reinterpret_cast<Glue *>(userData.pointer);
        for (auto it = data->usedParts->begin(); it != data->usedParts->end();) {
            auto glueParticle = *it;
            Particle particle = sim->parts[glueParticle.partIndex];
            if (particle.type == 0 || particle.type != PT_WOOD) {
                it = data->usedParts->erase(it); // Remove particles with type 0
            } else {
                // auto newX = pos.X + glueParticle.offset.X;
                // auto newY = pos.Y + glueParticle.offset.Y;
                float ox = glueParticle.offset.X;
                float oy = glueParticle.offset.Y;

                float rx = ox * cos(angle) - oy * sin(angle);
                float ry = ox * sin(angle) + oy * cos(angle);

                auto newX = pos.X + static_cast<int>(rx);
                auto newY = pos.Y + static_cast<int>(ry);

                sim->move(glueParticle.partIndex, particle.x,
                          particle.y, newX, newY);
                ++it;
            }
        }
    }
}

b2World &PhysicsSimulation::GetWorld() {
    return world;
}

b2Body *PhysicsSimulation::GetBodyList() const {
    return const_cast<b2Body *>(world.GetBodyList());
}

void PhysicsSimulation::CreateBody(Vec2<int> pos) {
    b2BodyDef boxDef;

    boxDef.type = b2_dynamicBody;
    auto pos2 = toB2Space(pos);
    boxDef.position.Set(pos2.x, pos2.y);


    b2Body *box = world.CreateBody(&boxDef);


    b2PolygonShape boxShape;
    boxShape.SetAsBox(3.0f * PTM, 30.0f * PTM);


    b2FixtureDef fixtureDef;
    fixtureDef.shape = &boxShape;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;
    box->CreateFixture(&fixtureDef);

    // make tpt particles

    Glue *glue = new Glue();
    glue->usedParts = new std::vector<GluedParticle>();

    b2AABB aabb;
    boxShape.ComputeAABB(&aabb, box->GetTransform(), 0);

    const auto lowerBound = fromB2Space(aabb.lowerBound);
    const auto upperBound = fromB2Space(aabb.upperBound);
    // printf("minx:%d maxx:%d miny:%d maxy:%d", lowerBound.X, upperBound.X, lowerBound.Y, upperBound.Y);
    // printf("diffx:%d diffy:%d\n", upperBound.X - lowerBound.X, lowerBound.Y - upperBound.Y);
    // for (int x = lowerBound.X + 1; x <= upperBound.X; ++x) {
    //     for (int y = upperBound.Y + 2; y <= lowerBound.Y + 1; ++y) {
    for (int x = lowerBound.X; x <= upperBound.X; x++) {
        for (int y = upperBound.Y; y <= lowerBound.Y; y++) {
            auto partIndex = sim->create_part(-3, x, y, PT_WOOD);
            auto gluedParticle = GluedParticle{partIndex, {x - pos.X, (y - pos.Y)}};
            glue->usedParts->push_back(gluedParticle);
        }
    }
    // float minX = fromB2Space() aabb.lowerBound.x;
    // float maxX = aabb.upperBound.x;
    // float minY = aabb.lowerBound.y;
    // float maxY = aabb.upperBound.y;

    box->GetUserData().pointer = reinterpret_cast<uintptr_t>(glue);
};
