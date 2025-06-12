#include <box2d/box2d.h>
#include "PhysicsSimulation.h"

#include <common/RasterGeometry.h>
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
    Vec2<int> lowerLocalBound;
    Vec2<int> upperLocalBound;
};


Vec2<int> rotate(int x, int y, float angle) {
    float rx = x * cos(angle) - y * sin(angle);
    float ry = x * sin(angle) + y * cos(angle);
    return {static_cast<int>(rx), static_cast<int>(ry)};
}

double float_mod_ring(float x, float n) {
    return std::fmod(std::fmod(x, n) + n, n);
}

std::vector<GluedParticle> rotateSprite(std::vector<GluedParticle> points, float angle) {
    auto s = sin(angle);
    auto c = cos(angle);

    auto result = std::vector<GluedParticle>();

    // angle = angle - M_PI / 4;
    // angle = float_mod_ring(angle, 2 * M_PI);
    // auto quadAngle = float_mod_ring(angle,  M_PI/2);
    // quadAngle = quadAngle - M_PI / 4;
    // printf("qa %f\n", quadAngle);
    //
    // printf("%f\n", angle);
    // float c = cos(quadAngle);
    // if (angle<M_PI/2) { // >
    //     printf(">", angle);
    //
    //     for (auto point: points) {
    //         result.push_back({point.partIndex, {point.offset.Y, -point.offset.X }});
    //     }
    // }
    // else if (angle>=M_PI/2 && angle<M_PI) { //v
    //     printf("v", angle);
    //     for (auto point: points) {
    //         result.push_back({point.partIndex, {-point.offset.X, -point.offset.Y }});
    //     }
    // }
    // else if (angle>=M_PI && angle<3*M_PI/2) { // <
    //     printf("<", angle);
    //     for (auto point: points) {
    //         result.push_back({point.partIndex, {-point.offset.Y, point.offset.X }});
    //     }
    // }
    // else if (angle>=3*M_PI/2) { // ^
    //     printf("^", angle);
    //     for (auto point: points) {
    //         result.push_back({point.partIndex, {int(point.offset.X * c), point.offset.Y }});
    //     }
    // }
    auto tmp = std::vector<GluedParticle>();
    for (auto point: points) {
        float x = point.offset.X;
        float y = point.offset.Y;
        int rx = ceil( x * c - y * s);
        int ry = ceil(x * s + y * c);

        result.push_back({point.partIndex, {rx, ry}});
    }

    return result;
}

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
        int minX = 9999;
        int maxX = -9999;
        int minY = 9999;
        int maxY = -9999;
        for (auto it = data->usedParts->begin(); it != data->usedParts->end();) {
            auto glueParticle = *it;
            Particle particle = sim->parts[glueParticle.partIndex];
            if (particle.type == 0 || particle.type != PT_WOOD) {
                it = data->usedParts->erase(it); // Remove particles with type 0
            } else {
                if (glueParticle.offset.X > maxX) {
                    maxX = glueParticle.offset.X;
                }
                if (glueParticle.offset.X < minX) {
                    minX = glueParticle.offset.X;
                }
                if (glueParticle.offset.Y > maxY) {
                    maxY = glueParticle.offset.Y;
                }
                if (glueParticle.offset.Y < minY) {
                    minY = glueParticle.offset.Y;
                }
                ++it;
            }
        }
        // auto xOffsets = new int[maxY - minY + 1];
        // auto yOffsets = new int[maxX - minX + 1];
        //
        // for (int x = minX; x <= maxX; x++) {
        //
        //     RasterizeLine<false>(rotate(0, minY, angle), rotate(0, maxY, angle),
        //                          [xOffsets, yOffsets, minY, minX](const Vec2<int> pos) {
        //                              xOffsets[pos.Y - minY] = pos.X;
        //                              yOffsets[pos.X - minX] = pos.Y;
        //                          });
        // }
        // auto yOffsets = new int[data->upperLocalBound.Y - data->lowerLocalBound.Y + 1];
        // auto xOffsets = new int[data->upperLocalBound.Y - data->lowerLocalBound.Y + 1];
        // auto yOffsets = new int[data->upperLocalBound.X - data->lowerLocalBound.X + 1];
        // RasterizeLine<false>(rotate(0, data->lowerLocalBound.Y, angle), rotate(0, data->upperLocalBound.Y, angle),
        //                      [xOffsets, yOffsets, data](const Vec2<int> pos) {
        //                          xOffsets[pos.Y-data->lowerLocalBound.Y] = pos.X;
        //                          yOffsets[pos.X-data->lowerLocalBound.X] = pos.Y;
        //                      });

        // if (body->IsAwake())
        for (auto glueParticle: rotateSprite(*data->usedParts, angle)) {
            Particle particle = sim->parts[glueParticle.partIndex];

            auto newX = pos.X + glueParticle.offset.X;
            auto newY = pos.Y + glueParticle.offset.Y;

            sim->move(glueParticle.partIndex, particle.x,
                      particle.y, newX, newY);
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
    boxShape.SetAsBox(30.0f * PTM, 30.0f * PTM);


    b2FixtureDef fixtureDef;
    fixtureDef.shape = &boxShape;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;
    box->CreateFixture(&fixtureDef);

    // make tpt particles

    Glue *glue = new Glue{
        new std::vector<GluedParticle>(),
        {0, 0},
        {0, 0},
    };

    b2AABB aabb;
    boxShape.ComputeAABB(&aabb, box->GetTransform(), 0);

    const auto lowerBound = fromB2Space(aabb.lowerBound);
    glue->lowerLocalBound = {lowerBound.X - pos.X, lowerBound.Y - pos.Y};
    const auto upperBound = fromB2Space(aabb.upperBound);
    glue->upperLocalBound = {lowerBound.X - pos.X, lowerBound.Y - pos.Y};
    for (int x = lowerBound.X; x <= upperBound.X; x++) {
        for (int y = upperBound.Y; y <= lowerBound.Y; y++) {
            auto partIndex = sim->create_part(-3, x, y, PT_WOOD);
            auto gluedParticle = GluedParticle{
                partIndex,
                {x - pos.X, y - pos.Y}
            };
            glue->usedParts->push_back(gluedParticle);
        }
    }

    box->GetUserData().pointer = reinterpret_cast<uintptr_t>(glue);
};
