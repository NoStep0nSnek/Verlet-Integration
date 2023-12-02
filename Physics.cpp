#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <SFML/Graphics.hpp>
#include <stdio.h>
#include <stdlib.h>


inline float distanceBetweenVertices(sf::Vector2f pos1, sf::Vector2f pos2) {
    float& x1 = pos1.x;
    float& x2 = pos2.x;
    float& y1 = pos1.y;
    float& y2 = pos2.y;
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

namespace physics {
    #define MAX_VERTICES 8
    #define MAX_EDGES 4
    #define MAX_BODY_VERTICES 8
    #define MAX_BODY_EDGES 8
    #define MAX_BODIES 2
    #define GRAVITY .5 // 9.8 M/S

    struct PhysicsBody;
    struct Edge;

    std::vector<PhysicsBody> bodies;

    struct Vertex {
        sf::Vector2f Position;
        sf::Vector2f OldPosition;
        sf::Vector2f Acceleration;
        sf::Vector2f Velocity;
    };

    struct Edge {
        Vertex* v1 = NULL;
        Vertex* v2 = NULL;

        float Stiffness = 1;
        float OriginalLength; // The length of the edge when it was created
        PhysicsBody* Parent = NULL; // The physics body that it belongs to
    };

    struct PhysicsBody {
        bool anchored = false; // If it's anchored then it will not simulate physics
        float mass = 1000; // In kg

        sf::Vector2f Center;
        std::vector<Vertex> vertices;
        std::vector<Edge> edges;
        std::vector<Edge> constrainingEdges; // edges made to stop the shape from deforming
        void ProjectToAxis(sf::Vector2f& Axis, float& Min, float& Max);
    };

    struct {
        float Depth = 0;
        sf::Vector2f Normal;

        Edge *E;
        Vertex *V;
    } CollisionInfo;

    int SGN(float x) {
        return (x > 0) - (x < 0);
    }

    // calculates the weighted center a polygon.
    // Needs to be adjusted to go off of surface area instead.
    inline sf::Vector2f calculateCenter(std::vector<sf::Vector2f> Vertexs) {
        sf::Vector2f center;
        float avgX = 0;
        float avgY = 0;
        for (int i = 0; i < Vertexs.size(); i++) {
            avgX += Vertexs[i].x;
            avgY += Vertexs[i].y;
        }
        avgX /= Vertexs.size();
        avgY /= Vertexs.size();
        center.x = avgX;
        center.y = avgY;
        return center;
    }


    void MakeShape(std::vector<sf::Vector2f> Points, bool anchored_ = false) {

        //using namespace std;

        PhysicsBody body_;
        bodies.push_back(body_);
        PhysicsBody &body = bodies[bodies.size() - 1];

        body.anchored = anchored_;
        // vertices
        for (int i = 0; i < Points.size(); i++) {
            Vertex V;
            V.Position = Points[i];
            body.vertices.push_back(V);
        }

        // edges
        // the initial edge
        Edge E;
        std::cout << body.vertices.size() << "V\n";
        E.v1 = &body.vertices[0];
        E.v2 = &body.vertices[1];
        E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
        E.Parent = &body;
        body.edges.push_back(E);

        for (short int i = 1; i < Points.size()-1; i++) {
            E.v1 = &body.vertices[i];
            E.v2 = &body.vertices[i+1];
            E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
            E.Parent = &body;
            body.edges.push_back(E);
        }

        // the last edge
        E.v1 = &body.vertices[Points.size() - 1];
        E.v2 = &body.vertices[0];
        E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
        E.Parent = &body;
        body.edges.push_back(E);

        // constraining edge
        E.v1 = &body.vertices[3];
        E.v2 = &body.vertices[1];
        E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
        E.Parent = &body;
        //body.constrainingEdges.push_back(E);

        // constraining edge #2
        E.v1 = &body.vertices[0];
        E.v2 = &body.vertices[2];
        E.OriginalLength = distanceBetweenVertices(E.v1->Position, E.v2->Position);
        E.Parent = &body;
        body.constrainingEdges.push_back(E);
    }

    // Returns the dot products of two vectors
    double dot(const sf::Vector2f& A, const sf::Vector2f& B) {
        return A.x * B.x + A.y * B.y;
    }

    // normalizes two vectors
    // see: https://stackoverflow.com/questions/10095524/normalize-a-vector
    void NormalizeVector(sf::Vector2f& vec) {
        float sum = vec.x + vec.y;
        vec.x / sum;
        vec.y / sum;
    }
    // Linear transform to find the orthogonal vector of the edge
    sf::Vector2f calculate_normalised_projection_axis(const sf::Vector2f& current_Vertex, const sf::Vector2f& next_Vertex) {
        const double axis_x = -(next_Vertex.y - current_Vertex.y);
        const double axis_y = next_Vertex.x - current_Vertex.x;
        const double magnitude = hypot(axis_x, axis_y);

        sf::Vector2f axis_normalised;
        axis_normalised.x = axis_x / magnitude;
        axis_normalised.y = axis_y / magnitude;

        return axis_normalised;
    }

    // Project the vertices of each polygon onto a axis
    void compute_projections(const std::vector<Vertex>& bounds_a, const std::vector<Vertex>& bounds_b, const sf::Vector2f& axis_normalised, std::vector<double>& projections_a, std::vector<double>& projections_b) {
        projections_a.clear();
        projections_b.clear();

        for (size_t i = 0; i < bounds_a.size(); i++) {
            const double projection_a = dot(axis_normalised, bounds_a[i].Position);


            const double projection_b = dot(axis_normalised, bounds_b[i].Position);

            projections_a.push_back(projection_a);
            projections_b.push_back(projection_b);
        }
    }

    // Check if the projections of two polygons overlap
    bool is_overlapping(const std::vector<double>& projections_a, const std::vector<double>& projections_b) {
        const double max_projection_a = *std::max_element(projections_a.begin(), projections_a.end());
        const double min_projection_a = *std::min_element(projections_a.begin(), projections_a.end());
        const double max_projection_b = *std::max_element(projections_b.begin(), projections_b.end());
        const double min_projection_b = *std::min_element(projections_b.begin(), projections_b.end());

        // True if projection overlaps but does not necessarily mean the polygons are intersecting yet
        return !(max_projection_a < min_projection_b or max_projection_b < min_projection_a);
    }

    int dotProduct(sf::Vector2f vect_A, sf::Vector2f vect_B)
    {

        int product = 0;

        // Loop for calculate dot product
        //for (int i = 0; i < 2/*It is two dimensional*/; i++)

            //product = product + vect_A[i] * vect_B[i];
        product = product + vect_A.x * vect_B.x;
        product = product + vect_A.y * vect_B.y;
        return product;
    }

    void ProjectToAxis(sf::Vector2f Axis, float& Min, float& Max, std::vector<sf::Vector2f> Vertices) {
        float DotP = dotProduct(Axis, Vertices[0]);

        //Set the minimum and maximum values to the projection of the first vertex
        Min = Max = DotP;

        for (int i = 1; i < Vertices.size(); i++) {
            //Project the rest of the vertices onto the axis and extend
            //the interval to the left/right if necessary
            DotP = dotProduct(Axis, Vertices[i]);

            Min = std::min(DotP, Min);
            Max = std::max(DotP, Max);
        }
    }

    float IntervalDistance(float MinA, float MaxA, float MinB, float MaxB) {
        if (MinA < MinB)
            return MinB - MaxA;
        else
            return MinA - MaxB;
    }

    // Check if two convex polygons intersect
    bool separating_axis_intersect(PhysicsBody& b1,PhysicsBody& b2) {
        std::vector<sf::Vector2f> bounds_a;
        std::vector<sf::Vector2f> bounds_b;
        for (short int i = 0; i < b1.vertices.size(); i++) {
            bounds_a.push_back(b1.vertices[i].Position);
        }
        for (short int i = 0; i < b2.vertices.size(); i++) {
            bounds_b.push_back(b2.vertices[i].Position);
        }
        std::vector<double> projections_a;
        std::vector<double> projections_b;
        projections_a.reserve(bounds_a.size());
        projections_b.reserve(bounds_b.size());

        /*for (size_t i = 0; i < bounds_a.size(); i++) {
            const sf::Vector2f current_Vertex = bounds_a[i];
            const sf::Vector2f next_Vertex = bounds_a[(i + 1) % bounds_a.size()];
            const sf::Vector2f axis_normalised = calculate_normalised_projection_axis(current_Vertex, next_Vertex);
            compute_projections(bounds_a, bounds_b, axis_normalised, projections_a, projections_b);

            float minA, minB, maxA, maxB; //Project both bodies onto the perpendicular axis
            ProjectToAxis(axis_normalised, minA, maxA, bounds_a);
            ProjectToAxis(axis_normalised, minB, maxB, bounds_b);

            if (!is_overlapping(projections_a, projections_b)) return false;



            float Distance = IntervalDistance(minA, maxA, minB, maxB);
            if (abs(Distance) < minDistance) {
                minDistance = abs(Distance);
            }


            CollisionInfo.Normal = axis_normalised;
            //std::cout << "AXIS: " << axis_normalised.x << "," << axis_normalised.y << "\n";
            CollisionInfo.E.v1 = new Vertex;
            CollisionInfo.E.v2 = new Vertex;
            CollisionInfo.E.v1->Position = current_Vertex;
            CollisionInfo.E.v2->Position = next_Vertex;
        }

        for (size_t i = 0; i < bounds_b.size(); i++) {
            const sf::Vector2f current_Vertex = bounds_b[i];
            const sf::Vector2f next_Vertex = bounds_b[(i + 1) % bounds_b.size()];
            const sf::Vector2f axis_normalised = calculate_normalised_projection_axis(current_Vertex, next_Vertex);
            compute_projections(bounds_a, bounds_b, axis_normalised, projections_a, projections_b);

            if (!is_overlapping(projections_a, projections_b)) return false;

            float minA, minB, maxA, maxB; //Project both bodies onto the perpendicular axis
            ProjectToAxis(axis_normalised, minA, maxA, bounds_a);
            ProjectToAxis(axis_normalised, minB, maxB, bounds_b);

            float Distance = IntervalDistance(minA, maxA, minB, maxB);
            if (abs(Distance) < minDistance) {
                minDistance = abs(Distance);
            }

            CollisionInfo.Normal = axis_normalised;
            CollisionInfo.E = E; //Store the edge, as it is the collision edge
        }*/

        float MinDistance = 10000.0f;

        for (int i = 0; i < b1.edges.size() + b2.edges.size(); i++) {
            Edge *E;

            if (i < b1.edges.size())
                E = &b1.edges[i];
            else
                E = &b2.edges[i - b1.edges.size()];

            const sf::Vector2f axis_normalised = calculate_normalised_projection_axis(E->v1->Position, E->v2->Position);
            compute_projections(b1.vertices, b2.vertices, axis_normalised, projections_a, projections_b);

            if (!is_overlapping(projections_a, projections_b)) {
                return false;
            }

            float minA, minB, maxA, maxB; //Project both bodies onto the perpendicular axis
            ProjectToAxis(axis_normalised, minA, maxA, bounds_a);
            ProjectToAxis(axis_normalised, minB, maxB, bounds_b);

            float Distance = IntervalDistance(minA, maxA, minB, maxB);
            MinDistance = abs(Distance);

            CollisionInfo.Normal = axis_normalised;
            CollisionInfo.E = E; //Store the edge, as it is the collision edge
        }

        //This is needed to make sure that the collision normal is Vertexing at B1
        //int Sign = SGN(dotProduct(CollisionInfo.Normal,(B1->Center - B2->Center)));
        sf::Vector2f BA = calculateCenter(bounds_a);
        sf::Vector2f BB = calculateCenter(bounds_b);
        int Sign = SGN(dotProduct(CollisionInfo.Normal, (BA - BB)));

        //Remember that the line equation is N*( R - R0 ). We choose B2->Center
        //as R0; the normal N is given by the collision normal

        if (Sign != 1)
            CollisionInfo.Normal = -CollisionInfo.Normal; //Revert the collision normal if it Vertexs away from B1

        CollisionInfo.Depth = MinDistance;

        float SmallestD = 10000.0f; //Initialize the smallest distance to a high value

        for (int I = 0; I < bounds_a.size(); I++) {
            //Measure the distance of the vertex from the line using the line equation
            //float Distance = CollisionInfo.Normal * (B1->Vertices[I]->Position - B2->Center);
            float Distance = dotProduct(CollisionInfo.Normal, (bounds_a[I] - BB));

            //If the measured distance is smaller than the smallest distance reported
            //so far, set the smallest distance and the collision vertex
            if (Distance < SmallestD) {
                SmallestD = Distance;
                //CollisionInfo.V = new Vertex;
                CollisionInfo.V = &b1.vertices[I];
            }
        }
        return true;
    }

    void collisionResponse() {
        sf::Vector2f CollisionVector = CollisionInfo.Normal * CollisionInfo.Depth;
        Vertex *V1 = CollisionInfo.E->v1;
        Vertex *V2 = CollisionInfo.E->v2;

        sf::Vector2f P1 = CollisionInfo.E->v1->Position;
        sf::Vector2f P2 = CollisionInfo.E->v2->Position;

        float T;
        if (abs(P1.x - P2.x) > abs(P1.y - P2.y))
            T = (CollisionInfo.V->Position.x - CollisionVector.x - P1.x) / (P2.x - P1.x);
        else
            T = (CollisionInfo.V->Position.y - CollisionVector.y - P1.y) / (P2.y - P1.y);

        float Lambda = 1.0f / (T * T + (1 - T) * (1 - T));

        for (int i = 0; i < bodies[0].edges.size(); i++) {
            bodies[0].edges[i].v1->Position -= CollisionVector * (1 - T) * 0.5f * Lambda;
            bodies[0].edges[i].v2->Position -= CollisionVector * T * 0.5f * Lambda;
        }
        V1->Position -= CollisionVector * (1 - T) * 0.5f * Lambda;
        V2->Position -= CollisionVector * T * 0.5f * Lambda;

        //CollisionInfo.V->Position += CollisionVector * 0.05f;



        // simulates friction
        // FrictionVector = ?*m*g;
        // g = gravitational constant (9.8 on earth)
        // .01 is friction coefficient
        float Friction = 0 * 2 * GRAVITY;
        sf::Vector2f FrictionVector(V1->Velocity.x * Friction, V1->Velocity.y * Friction);
        //V1->Position += FrictionVector;
        FrictionVector.x = V2->Velocity.x * Friction;
        FrictionVector.y = V2->Velocity.y * Friction;
        //V2->Position += FrictionVector;
    }

    void updateEdges(bool inverse = false) {
        for (int b = 0; b < bodies.size(); b++) {
            PhysicsBody body = bodies[b];
            int i = 0;
            for (int a = 0; a < body.edges.size() + body.constrainingEdges.size(); a++) {
                if (inverse) {
                    i = body.edges.size() + body.constrainingEdges.size() - 1 - a;
                } else {
                    i = a;
                }
                /*Edge* E;
                if (i < body.edges.size()) {
                    //std::cout << i << "\n";
                    E = &bodies[b].edges[i];
                }
                else {
                    E = &bodies[b].constrainingEdges[i - body.edges.size()];
                }
                //Calculate the vector mentioned above
                sf::Vector2f V1V2 = E->v2->Position - E->v1->Position;

                //Calculate the current distance
                float V1V2Length = distanceBetweenVertices(E->v1->Position, E->v2->Position);

                //Calculate the difference from the original length
                float Diff = V1V2Length - E->OriginalLength;

                NormalizeVector(V1V2);

                //Push both vertices apart by half of the difference respectively
                //so the distance between them equals the original length
                E->v1->Position += V1V2 * Diff * 0.5f;
                E->v2->Position -= V1V2 * Diff * 0.5f;
                */
                Edge *E;
                if (i < body.edges.size()) {
                    //std::cout << i << "\n";
                    E = &bodies[b].edges[i];
                } else {
                    E = &bodies[b].constrainingEdges[i-body.edges.size()];
                }

                // Calculate the vector mentioned above
                /*sf::Vector2f V1V2 = E->v2->Position - E->v1->Position;

                float V1V2_Length = distanceBetweenVertexs(E->v1->Position, E->v2->Position);
                float diff = E->OriginalLength - V1V2_Length; // Difference between original length and current length
                std::cout << "\nLEN: " << diff << "\n";
                //V1V2.Normalize(); // Convert to normal?
                //float V1V2f = Normalize(V1V2); // Convert to normal?
                float dx = E->v1->Position.x - E->v1->Position.x;
                float dy = E->v1->Position.y - E->v1->Position.y;
                float offsetx = dx * diff * 0.5;
                float offsety = dy * diff * 0.5;

                float totalX = 0;*/
                // Push both vertices apart by half the distance respectively
                // So the distances equals the original length
                //E->v1->Position.x -= offsetx;
                //E->v1->Position.y -= offsety;
                //E->v2->Position.x += offsetx;
                //E->v2->Position.y += offsety;

                float dx = E->v2->Position.x - E->v1->Position.x;
                float dy = E->v2->Position.y - E->v1->Position.y;
                float dist = sqrt(dx * dx + dy * dy);
                float diff = (E->OriginalLength - dist) / dist * 1;

                // gets offset of the Vertexs
                float offsetx = dx * diff * 0.5;
                float offsety = dy * diff * 0.5;

                // calculate "mass"
                //float m1 = 1 + 1;
                //float m2 = 1 / m1;
                //m1 = 1 / m1;
                float m1 = 1;
                float m2 = 1;

                //std::cout << "X:" << body.edges[0].v1.Position.x << "\n";
                E->v1->Position.x -= offsetx * m1;
                E->v1->Position.y -= offsety * m1;


                ///////////////////////////////////////////////////////////////////////////////////////////////////
                

                E->v2->Position.x += offsetx * m2;
                E->v2->Position.y += offsety * m2;

            }
        }
    }

    /*
    IterateCollisions is a method that does multiple things. It iterates over all bodies, calls the respective UpdateEdges
    method, recalculates the body center and then does the collision detection (and the collision response, if necessary).
    Of course, it doesn't just do this once, but repeats those steps a few times.
    The more repetitions are made, the more realistic the physics will look.
    The reason was explained above (if you've forgotten, better read it again.)
    */

    void IterateCollisions() {

        // recenters shape. Does not currently account for volume. Center is also recalculated elsewhere in program. Need to fix this.
        for (short int i = 0; i < bodies.size(); i++) {
            PhysicsBody &body = bodies[i];
            // recalculates body center
            float totalX = 0;
            float totalY = 0;
            for (short int j = 0; j < body.vertices.size(); j++) {
                totalX += body.vertices[j].Position.x;
                totalY += body.vertices[j].Position.y;
            }
            totalX /= body.vertices.size();
            totalY /= body.vertices.size();
            body.Center = sf::Vector2f(totalX, totalY);
        }

        for (int i = 0; i < bodies.size(); i++) {
            PhysicsBody& B1 = bodies[i];
            //printPosition(B1->vertices[1]->Position.x, B1->vertices[1]->Position.y);
            for (int j = 0; j < bodies.size(); j++) {
                PhysicsBody& B2 = bodies[j];
                //printPosition(B2->edges[1]->v1->Position.x, B2->vertices[1]->Position.y);
                if (i != j && !bodies[j].anchored) {
                    if (separating_axis_intersect(B1, B2)) {
                        collisionResponse();
                    }
                }
            }
        }
    }

    void UpdateForces() {
        // gravitational force
        for (int i = 0; i < bodies.size(); i++) {
            PhysicsBody &body = bodies[i];
            if (!body.anchored) {
                float gravForce = GRAVITY;
                float gravAccel = gravForce / body.mass;
                // verts
                for (int i = 0; i < body.vertices.size(); i++) {
                    body.vertices[i].Acceleration.y += gravAccel;
                    //body.vertices[i]->Position.y += body.vertices[i]->Acceleration.y;
                    if (body.vertices[i].Acceleration.y > .03) {
                        body.vertices[i].Acceleration.y = .03;
                    }
                }
                
                for (int i = 0; i < body.edges.size(); i++) {
                    //std::cout << i << "ii";
                    body.edges[i].v1->Position.y += body.edges[i].v1->Acceleration.y;
                    body.edges[i].v2->Position.y += body.edges[i].v2->Acceleration.y;
                }
            }
        }
    }



    void update() {

        // call update edges multiple times to maintain shape
        // note that when an edge updates it may interfere with the length of another

        // constrains Vertices
        for (int i = 0; i < physics::bodies.size(); i++) {

            PhysicsBody& body = physics::bodies[i];
            if (!body.anchored) {
                // verts
                for (int j = 0; j < body.vertices.size(); j++) {
                    body.vertices[j].OldPosition = body.vertices[j].Position;
                    //body.vertices[i]->Position.y += body.vertices[i]->Acceleration.y;
                    if (body.vertices[j].Position.y > 150) {
                        body.vertices[j].Position.y = 150;
                    }
                    if (body.vertices[j].Position.x < 0) {
                        body.vertices[j].Position.x = 0;
                    }
                }
            }
        }


        // Calculates velocity
        for (int i = 0; i < physics::bodies.size(); i++) {

            PhysicsBody& body = physics::bodies[i];
            if (!body.anchored) {
                // verts
                for (int j = 0; j < body.vertices.size(); j++) {
                    body.vertices[j].Velocity = body.vertices[j].OldPosition - body.vertices[j].Position;
                }
            }
        }

        UpdateForces();

        updateEdges(true);
        //updateEdges(true);

        IterateCollisions();


        //updateEdges();

        //UpdateForces();

        //IterateCollisions();


        //updateEdges();
    }
}

void renderVertices(sf::RenderWindow& window) {
    using namespace std;
    window.clear();
    for (int i = 0; i < physics::bodies.size(); i++) {
        //cout << i << "i\n";
        physics::PhysicsBody &body = physics::bodies[i];
        for (int j = 0; j < body.vertices.size(); j++) {
            physics::Vertex V = body.vertices[j];
            sf::CircleShape circle(5);
            circle.setPosition(V.Position);
            circle.setOrigin(circle.getGlobalBounds().width / 2, circle.getGlobalBounds().height / 2);
            circle.setFillColor(sf::Color::Red);
            window.draw(circle);
        }

        for (int j = 0; j < body.edges.size(); j++) {
            physics::Vertex V1 = *body.edges[j].v1;
            sf::CircleShape circle(5);
            circle.setPosition(V1.Position);
            circle.setOrigin(circle.getGlobalBounds().width / 2, circle.getGlobalBounds().height / 2);
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            physics::Vertex V2 = *body.edges[j].v2;
            circle.setPosition(V2.Position);
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            sf::RectangleShape rect(sf::Vector2f(2, distanceBetweenVertices(V1.Position, V2.Position)));
            rect.setOrigin(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2);
            float avgX = (V1.Position.x + V2.Position.x) / 2;
            float avgY = (V1.Position.y + V2.Position.y) / 2;
            float rotation = atan2(V1.Position.y - V2.Position.y, V1.Position.x - V2.Position.x);
            // converts radians to degrees
            rotation = rotation * 180 / 3.14159;
            rect.setPosition(avgX, avgY);
            rect.setRotation(rotation + 90);
            rect.setFillColor(sf::Color::Green);
            window.draw(rect);

        }

        for (int j = 0; j < body.constrainingEdges.size(); j++) {
            physics::Vertex V1 = *body.constrainingEdges[j].v1;
            sf::CircleShape circle(5);
            circle.setPosition(V1.Position);
            circle.setOrigin(circle.getGlobalBounds().width / 2, circle.getGlobalBounds().height / 2);
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            physics::Vertex V2 = *body.constrainingEdges[j].v2;
            circle.setPosition(V2.Position);
            circle.setFillColor(sf::Color::Green);
            window.draw(circle);

            sf::RectangleShape rect(sf::Vector2f(2, distanceBetweenVertices(V1.Position, V2.Position)));
            rect.setOrigin(rect.getGlobalBounds().width / 2, rect.getGlobalBounds().height / 2);
            float avgX = (V1.Position.x + V2.Position.x) / 2;
            float avgY = (V1.Position.y + V2.Position.y) / 2;
            float rotation = atan2(V1.Position.y - V2.Position.y, V1.Position.x - V2.Position.x);
            // converts radians to degrees
            rotation = rotation * 180 / 3.14159;
            rect.setPosition(avgX, avgY);
            rect.setRotation(rotation + 90);
            rect.setFillColor(sf::Color::Green);
            window.draw(rect);

        }
    }
    window.display();
}

int main() {

    using namespace std;
    
    physics::MakeShape({sf::Vector2f(50,0),sf::Vector2f(50,50), sf::Vector2f(100,50), sf::Vector2f(100,0)});
    physics::MakeShape({ sf::Vector2f(30, 80), sf::Vector2f(30, 130), sf::Vector2f(80, 130), sf::Vector2f(80, 80)}, true); // uncommenting this will produce a bug

    sf::RenderWindow window(sf::VideoMode(400, 400), "Physics Test");

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        physics::update();
        renderVertices(window);
    }
    return 0;
}
