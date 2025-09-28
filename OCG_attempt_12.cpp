#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

class OGC3D {
private:
    GLFWwindow* window;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> prev_vertices;
    std::vector<glm::vec3> velocities;
    std::vector<glm::uvec3> triangles;
    std::vector<float> displacement_bounds;
    float contact_radius = 0.08f;
    float dt = 0.01f;
    const int grid_x = 8, grid_y = 8, grid_z = 2; // 2x8x8 volume
    bool mouse_down = false;
    glm::vec3 mouse_ray_origin, mouse_ray_dir;
    int dragged_vertex = -1;
    float gamma_p = 0.3f;
    
    glm::vec3 camera_pos = glm::vec3(0.0f, 3.0f, 8.0f);
    glm::vec3 camera_front = glm::vec3(0.0f, -0.3f, -1.0f);
    glm::vec3 camera_up = glm::vec3(0.0f, 1.0f, 0.0f);
    
public:
    bool initialize() {
        if (!glfwInit()) return false;
        
        window = glfwCreateWindow(800, 600, "OGC 3D - Thick Volume + Floor", NULL, NULL);
        if (!window) {
            glfwTerminate();
            return false;
        }
        
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        glfwSetWindowUserPointer(window, this);
        glfwSetMouseButtonCallback(window, mouse_button_callback);
        glfwSetCursorPosCallback(window, cursor_position_callback);
        
        glEnable(GL_DEPTH_TEST);
        
        // Create 2x8x8 volumetric cloth (thick block)
        float spacing = 0.1f;
        for (int z = 0; z < grid_z; ++z) {
            for (int y = 0; y < grid_y; ++y) {
                for (int x = 0; x < grid_x; ++x) {
                    float pos_x = (x - grid_x/2.0f) * spacing;
                    float pos_y = 3.0f + (y - grid_y/2.0f) * spacing; // Start above floor
                    float pos_z = (z - grid_z/2.0f) * spacing;
                    vertices.push_back(glm::vec3(pos_x, pos_y, pos_z));
                    velocities.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
                }
            }
        }
        
        prev_vertices = vertices;
        displacement_bounds.resize(vertices.size(), contact_radius);
        
        // Create triangles for each face of the volume
        createVolumeTriangles();
        
        return true;
    }
    
    void run() {
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            updatePhysics();
            renderScene();
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }
    
    void shutdown() {
        glfwDestroyWindow(window);
        glfwTerminate();
    }
    
private:
    void createVolumeTriangles() {
        // Create triangles for front and back faces
        for (int y = 0; y < grid_y - 1; ++y) {
            for (int x = 0; x < grid_x - 1; ++x) {
                // Front face (z=0)
                int v0 = y * grid_x + x;
                int v1 = y * grid_x + x + 1;
                int v2 = (y + 1) * grid_x + x;
                int v3 = (y + 1) * grid_x + x + 1;
                triangles.push_back(glm::uvec3(v0, v1, v2));
                triangles.push_back(glm::uvec3(v1, v3, v2));
                
                // Back face (z=1)
                int b0 = v0 + grid_x * grid_y;
                int b1 = v1 + grid_x * grid_y;
                int b2 = v2 + grid_x * grid_y;
                int b3 = v3 + grid_x * grid_y;
                triangles.push_back(glm::uvec3(b0, b2, b1));
                triangles.push_back(glm::uvec3(b1, b2, b3));
            }
        }
        
        // Create triangles for side faces
        for (int z = 0; z < grid_z - 1; ++z) {
            for (int y = 0; y < grid_y - 1; ++y) {
                // Left face (x=0)
                int v0 = z * grid_x * grid_y + y * grid_x;
                int v1 = z * grid_x * grid_y + (y + 1) * grid_x;
                int v2 = (z + 1) * grid_x * grid_y + y * grid_x;
                int v3 = (z + 1) * grid_x * grid_y + (y + 1) * grid_x;
                triangles.push_back(glm::uvec3(v0, v1, v2));
                triangles.push_back(glm::uvec3(v1, v3, v2));
                
                // Right face (x=7)
                int r0 = v0 + grid_x - 1;
                int r1 = v1 + grid_x - 1;
                int r2 = v2 + grid_x - 1;
                int r3 = v3 + grid_x - 1;
                triangles.push_back(glm::uvec3(r0, r2, r1));
                triangles.push_back(glm::uvec3(r1, r2, r3));
            }
        }
        
        // Create triangles for top and bottom faces
        for (int z = 0; z < grid_z - 1; ++z) {
            for (int x = 0; x < grid_x - 1; ++x) {
                // Top face (y=7)
                int v0 = z * grid_x * grid_y + (grid_y - 1) * grid_x + x;
                int v1 = z * grid_x * grid_y + (grid_y - 1) * grid_x + x + 1;
                int v2 = (z + 1) * grid_x * grid_y + (grid_y - 1) * grid_x + x;
                int v3 = (z + 1) * grid_x * grid_y + (grid_y - 1) * grid_x + x + 1;
                triangles.push_back(glm::uvec3(v0, v1, v2));
                triangles.push_back(glm::uvec3(v1, v3, v2));
                
                // Bottom face (y=0)
                int b0 = z * grid_x * grid_y + x;
                int b1 = z * grid_x * grid_y + x + 1;
                int b2 = (z + 1) * grid_x * grid_y + x;
                int b3 = (z + 1) * grid_x * grid_y + x + 1;
                triangles.push_back(glm::uvec3(b0, b2, b1));
                triangles.push_back(glm::uvec3(b1, b2, b3));
            }
        }
    }
    
    void updatePhysics() {
        // Apply gravity
        glm::vec3 gravity = glm::vec3(0.0f, -9.8f, 0.0f);
        for (size_t i = 0; i < vertices.size(); ++i) {
            velocities[i] += gravity * dt;
        }
        
        // OGC penetration prevention
        for (int iteration = 0; iteration < 2; ++iteration) {
            computeDisplacementBounds3D();
            
            for (size_t v_idx = 0; v_idx < vertices.size(); ++v_idx) {
                glm::vec3 ogc_force = computeOGCForce3D(v_idx);
                velocities[v_idx] += ogc_force * dt;
            }
            
            applyDisplacementConstraints();
        }
        
        // Integrate
        for (size_t i = 0; i < vertices.size(); ++i) {
            vertices[i] += velocities[i] * dt;
        }
        
        // Damping
        for (size_t i = 0; i < velocities.size(); ++i) {
            velocities[i] *= 0.98f;
        }
        
        // Floor collision
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (vertices[i].y < 0.0f) {
                vertices[i].y = 0.0f;
                velocities[i].y = 0.0f;
            }
        }
        
        prev_vertices = vertices;
    }
    
    void computeDisplacementBounds3D() {
        for (size_t v_idx = 0; v_idx < vertices.size(); ++v_idx) {
            float d_min_v = std::numeric_limits<float>::max();
            
            // Check distance to all triangles
            for (size_t t_idx = 0; t_idx < triangles.size(); ++t_idx) {
                if (isTriangleAdjacentToVertex(t_idx, v_idx)) continue;
                
                float distance = pointToTriangleDistance3D(
                    vertices[v_idx],
                    vertices[triangles[t_idx][0]],
                    vertices[triangles[t_idx][1]], 
                    vertices[triangles[t_idx][2]]
                );
                d_min_v = std::min(d_min_v, distance);
            }
            
            // Check distance to other vertices
            for (size_t other_v = 0; other_v < vertices.size(); ++other_v) {
                if (other_v == v_idx) continue;
                float dist = glm::length(vertices[v_idx] - vertices[other_v]);
                d_min_v = std::min(d_min_v, dist);
            }
            
            displacement_bounds[v_idx] = gamma_p * d_min_v;
            displacement_bounds[v_idx] = std::max(0.001f, 
                std::min(displacement_bounds[v_idx], contact_radius));
        }
    }
    
    glm::vec3 computeOGCForce3D(int vertex_idx) {
        glm::vec3 total_force(0.0f);
        glm::vec3 vertex_pos = vertices[vertex_idx];
        
        // Triangle face repulsion
        for (size_t t_idx = 0; t_idx < triangles.size(); ++t_idx) {
            if (isTriangleAdjacentToVertex(t_idx, vertex_idx)) continue;
            
            float distance = pointToTriangleDistance3D(
                vertex_pos,
                vertices[triangles[t_idx][0]],
                vertices[triangles[t_idx][1]], 
                vertices[triangles[t_idx][2]]
            );
            
            if (distance < contact_radius) {
                glm::vec3 closest = getClosestPointOnTriangle3D(
                    vertex_pos,
                    vertices[triangles[t_idx][0]],
                    vertices[triangles[t_idx][1]],
                    vertices[triangles[t_idx][2]]
                );
                
                glm::vec3 away_dir = glm::normalize(vertex_pos - closest);
                float strength = (contact_radius - distance) * 30.0f;
                total_force += away_dir * strength;
            }
        }
        
        // Vertex-vertex repulsion
        for (size_t other_v = 0; other_v < vertices.size(); ++other_v) {
            if (other_v == vertex_idx) continue;
            
            float distance = glm::length(vertex_pos - vertices[other_v]);
            if (distance < contact_radius) {
                glm::vec3 away_dir = glm::normalize(vertex_pos - vertices[other_v]);
                float strength = (contact_radius - distance) * 25.0f;
                total_force += away_dir * strength;
            }
        }
        
        return total_force;
    }
    
    void applyDisplacementConstraints() {
        for (size_t v_idx = 0; v_idx < vertices.size(); ++v_idx) {
            glm::vec3 proposed_displacement = vertices[v_idx] - prev_vertices[v_idx];
            float displacement_length = glm::length(proposed_displacement);
            
            if (displacement_length > displacement_bounds[v_idx]) {
                float scale = displacement_bounds[v_idx] / displacement_length;
                glm::vec3 constrained_displacement = proposed_displacement * scale;
                vertices[v_idx] = prev_vertices[v_idx] + constrained_displacement;
                velocities[v_idx] = constrained_displacement / dt;
            }
        }
    }
    
    bool isTriangleAdjacentToVertex(size_t tri_idx, int vertex_idx) {
        const auto& tri = triangles[tri_idx];
        return (tri[0] == vertex_idx || tri[1] == vertex_idx || tri[2] == vertex_idx);
    }
    
    glm::vec3 getClosestPointOnTriangle3D(const glm::vec3& point, const glm::vec3& a, 
                                         const glm::vec3& b, const glm::vec3& c) {
        glm::vec3 ab = b - a;
        glm::vec3 ac = c - a;
        glm::vec3 ap = point - a;
        
        float d1 = glm::dot(ab, ap);
        float d2 = glm::dot(ac, ap);
        
        if (d1 <= 0.0f && d2 <= 0.0f) return a;
        
        glm::vec3 bp = point - b;
        float d3 = glm::dot(ab, bp);
        float d4 = glm::dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) return b;
        
        glm::vec3 cp = point - c;
        float d5 = glm::dot(ab, cp);
        float d6 = glm::dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) return c;
        
        // Return centroid for simplicity
        return (a + b + c) / 3.0f;
    }
    
    float pointToTriangleDistance3D(const glm::vec3& point, const glm::vec3& a, 
                                   const glm::vec3& b, const glm::vec3& c) {
        glm::vec3 closest = getClosestPointOnTriangle3D(point, a, b, c);
        return glm::length(point - closest);
    }
    
    void renderScene() {
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f/600.0f, 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(camera_pos, glm::vec3(0.0f, 1.0f, 0.0f), camera_up);
        
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(projection));
        
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(glm::value_ptr(view));
        
        // Draw floor
        glColor3f(0.4f, 0.4f, 0.4f);
        glBegin(GL_QUADS);
        glVertex3f(-5.0f, 0.0f, -5.0f);
        glVertex3f(5.0f, 0.0f, -5.0f);
        glVertex3f(5.0f, 0.0f, 5.0f);
        glVertex3f(-5.0f, 0.0f, 5.0f);
        glEnd();
        
        // Draw volumetric cloth
        glColor4f(0.3f, 0.5f, 0.8f, 0.8f);
        glBegin(GL_TRIANGLES);
        for (const auto& tri : triangles) {
            glVertex3f(vertices[tri[0]].x, vertices[tri[0]].y, vertices[tri[0]].z);
            glVertex3f(vertices[tri[1]].x, vertices[tri[1]].y, vertices[tri[1]].z);
            glVertex3f(vertices[tri[2]].x, vertices[tri[2]].y, vertices[tri[2]].z);
        }
        glEnd();
        
        // Draw displacement bounds
        glColor4f(0.0f, 1.0f, 1.0f, 0.15f);
        for (size_t i = 0; i < vertices.size(); ++i) {
            drawSphere(vertices[i], displacement_bounds[i]);
        }
        
        // Draw vertices
        glColor3f(1.0f, 0.7f, 0.2f);
        glPointSize(5.0f);
        glBegin(GL_POINTS);
        for (const auto& vertex : vertices) {
            glVertex3f(vertex.x, vertex.y, vertex.z);
        }
        glEnd();
    }
    
    void drawSphere(const glm::vec3& center, float radius) {
        int slices = 6, stacks = 4;
        for (int i = 0; i < stacks; ++i) {
            float phi1 = i * 3.14159f / stacks;
            float phi2 = (i + 1) * 3.14159f / stacks;
            
            glBegin(GL_QUAD_STRIP);
            for (int j = 0; j <= slices; ++j) {
                float theta = j * 2.0f * 3.14159f / slices;
                
                glm::vec3 p1 = center + radius * glm::vec3(
                    sin(phi1) * cos(theta), cos(phi1), sin(phi1) * sin(theta));
                glm::vec3 p2 = center + radius * glm::vec3(
                    sin(phi2) * cos(theta), cos(phi2), sin(phi2) * sin(theta));
                
                glVertex3f(p1.x, p1.y, p1.z);
                glVertex3f(p2.x, p2.y, p2.z);
            }
            glEnd();
        }
    }
    
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
        OGC3D* sim = static_cast<OGC3D*>(glfwGetWindowUserPointer(window));
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            sim->mouse_down = (action == GLFW_PRESS);
            if (!sim->mouse_down) {
                sim->dragged_vertex = -1;
            }
        }
    }
    
    static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
        OGC3D* sim = static_cast<OGC3D*>(glfwGetWindowUserPointer(window));
        // Mouse interaction removed for simplicity
    }
};

int main() {
    OGC3D sim;
    if (!sim.initialize()) return -1;
    
    std::cout << "OGC 3D - 2x8x8 Volume + Floor + Gravity\n";
    std::cout << "Thick cloth block falling onto floor\n";
    sim.run();
    sim.shutdown();
    return 0;
}
