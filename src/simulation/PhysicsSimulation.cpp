#include <box2d/box2d.h>
#include "PhysicsSimulation.h"

#include <iostream>
#include <common/RasterGeometry.h>
#include <common/Vec2.h>
#include <json/config.h>

#include "Simulation.h"
#include "common/tpt-compat.h"
#include "ElementClasses.h"


static int sides[] = {
    0, 1,
    1, 0,
    0, -1,
    -1, 0
};

inline Vec2<int> getSide(const int index) {
    return {sides[index * 2], sides[index * 2 + 1]};
}

// b2PolygonShape MakePolygonShape(const std::vector<Vec2<int>>& pts) {
//     if (pts.size() < 3) throw std::runtime_error("Нужно ≥3 точки");
//
//     std::vector<b2Vec2> v;
//     v.reserve(pts.size());
//     for (auto &p : pts) {
//         v.emplace_back(static_cast<float>(p.X), static_cast<float>(p.Y));
//     }
//
//     b2Hull hull = b2ComputeHull(v.data(), static_cast<int>(v.size()));
//     if (hull.count == 0) throw std::runtime_error("Не удалось построить hull");
//
//     b2PolygonShape shape;
//     shape.Set(hull);  // устанавливает форму из hull
//     return shape;
// }
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

std::vector<GluedParticle> rotateSprite(std::vector<GluedParticle> points, b2Rot rot) {
    auto s = -rot.s;
    auto c = rot.c;

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
        int rx = ceil(x * c - y * s);
        int ry = ceil(x * s + y * c);

        result.push_back({point.partIndex, {rx, ry}});
    }

    return result;
}


std::vector<Vec2<int> > *PhysicsSimulation::MakeHull(int x, int y) {
    auto points = new std::vector<Vec2<int> >();

    char *bitmap = (char *) malloc(XRES * YRES); //Bitmap for checking
    try {
        CoordStack &cs = sim->getCoordStackSingleton();
        cs.clear();
        cs.push(x, y);

        do {
            cs.pop(x, y);
            auto isPoint = false;
            for (int i = 0; i < 4; i++) {
                const auto side = getSide(i);
                const auto newX = x + side.X;
                const auto newY = y + side.Y;
                if (newX < 0 || newX >= XRES || newY < 0 || newY >= YRES) {
                    isPoint = true;
                    continue;
                }
                if (bitmap[newX + newY * XRES] == 0) {
                    auto part = sim->pmap[newX][newY];
                    if (part == 0)
                        isPoint = true;
                    else {
                        cs.push(newX, newY);
                        bitmap[newX + newY * XRES] = 1;
                    }
                }
            }
            if (isPoint) points->push_back({x, y});
        } while (cs.getSize() > 0);
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        free(bitmap);
        return points;
    }
    free(bitmap);
    return points;
}

void PhysicsSimulation::Update(float timeStep) {
    b2World_Step(worldId, 1.0f / 60.0f, 6);

    for (auto body: bodyList) {
        auto b2pos = b2Body_GetPosition(body);
        auto pos = fromB2Space({b2pos.x, b2pos.y});
        auto userData = b2Body_GetUserData(body);
        if (userData == nullptr) {
            continue;
        }

        auto rot = b2Body_GetRotation(body);


        const auto data = static_cast<Glue *>(userData);
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
        for (auto glueParticle: rotateSprite(*data->usedParts, rot)) {
            Particle particle = sim->parts[glueParticle.partIndex];

            auto newX = pos.X + glueParticle.offset.X;
            auto newY = pos.Y + glueParticle.offset.Y;

            sim->move(glueParticle.partIndex, particle.x,
                      particle.y, newX, newY);
        }
    }
}

b2WorldId &PhysicsSimulation::GetWorldId() {
    return worldId;
}

const std::vector<b2BodyId> *PhysicsSimulation::GetBodyList() const {
    return &bodyList;
}

void PhysicsSimulation::CreateBody(Vec2<int> pos) {
    b2BodyDef boxDef = b2DefaultBodyDef();

    boxDef.type = b2_dynamicBody;
    boxDef.position = toB2Space(pos);


    b2BodyId bodyId = b2CreateBody(worldId, &boxDef);
    // b2Body *box = world.CreateBody(&boxDef);


    b2Polygon boxShape = b2MakeBox(30.0f * PTM, 30.0f * PTM);
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 2.0f;

    b2CreatePolygonShape(bodyId, &shapeDef, &boxShape);

    // make tpt particles

    Glue *glue = new Glue{
        new std::vector<GluedParticle>(),
        {0, 0},
        {0, 0},
    };

    const b2AABB aabb = b2ComputePolygonAABB(&boxShape, b2Body_GetTransform(bodyId));

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
    b2Body_SetUserData(bodyId, glue);
    bodyList.push_back(bodyId);
};


void PhysicsSimulation::CreateBodyWithHull(Vec2<int> pos, std::vector<Vec2<int> > points) {
    // b2BodyDef boxDef;
    //
    // boxDef.type = b2_dynamicBody;
    // auto pos2 = toB2Space(pos);
    // boxDef.position.Set(pos2.x, pos2.y);
    //
    //
    // b2Body *box = world.CreateBody(&boxDef);
    //
    //
    // // b2PolygonShape boxShape;
    // // boxShape.SetAsBox(30.0f * PTM, 30.0f * PTM);
    // b2Vec2 points[] = {{-1.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}};
    // auto hull = b2Hull(points, 3);
    // float radius = 0.1f;
    // b2Polygon roundedTriangle = b2MakePolygon(&hull, radius);
    //
    //
    // b2FixtureDef fixtureDef;
    // fixtureDef.shape = &boxShape;
    // fixtureDef.density = 1.0f;
    // fixtureDef.friction = 0.3f;
    // box->CreateFixture(&fixtureDef);
    //
    // // make tpt particles
    //
    // Glue *glue = new Glue{
    //     new std::vector<GluedParticle>(),
    //     {0, 0},
    //     {0, 0},
    // };
    // box->GetUserData().pointer = reinterpret_cast<uintptr_t>(glue);
};
