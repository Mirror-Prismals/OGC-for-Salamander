#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

class OGC2DSimulation {
private:
    GLFWwindow* window;
    std::vector<glm::vec2> vertices;
    std::vector<glm::vec2> velocities;
    std::vector<glm::uvec2> edges;
    std::vector<glm::vec2> edge_normals;
    std::vector<float> displacement_bounds; // OGC key feature
    float contact_radius = 0.08f;
    float dt = 0.008f;
    glm::vec2 gravity = glm::vec2(0.0f, -30.0f);
    const int grid_size = 12;
    
public:
    bool initialize() {
        if (!glfwInit()) return false;
        
        window = glfwCreateWindow(800, 600, "OGC 2D - Proper Implementation", NULL, NULL);
        if (!window) {
            glfwTerminate();
            return false;
        }
        
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        
        // Initialize cloth grid
        float spacing = 0.06f;
        for (int y = 0; y < grid_size; ++y) {
            for (int x = 0; x < grid_size; ++x) {
                float pos_x = (x - grid_size/2.0f) * spacing;
                float pos_y = 0.9f - y * spacing * 0.7f;
                vertices.push_back(glm::vec2(pos_x, pos_y));
                velocities.push_back(glm::vec2(0.0f, 0.0f));
            }
        }
        
        // Create edges
        for (int y = 0; y < grid_size; ++y) {
            for (int x = 0; x < grid_size - 1; ++x) {
                edges.push_back(glm::uvec2(y*grid_size + x, y*grid_size + x + 1));
            }
        }
        for (int x = 0; x < grid_size; ++x) {
            for (int y = 0; y < grid_size - 1; ++y) {
                edges.push_back(glm::uvec2(y*grid_size + x, (y+1)*grid_size + x));
            }
        }
        
        // Compute edge normals
        for (const auto& edge : edges) {
            glm::vec2 edge_vec = vertices[edge[1]] - vertices[edge[0]];
            glm::vec2 normal = glm::normalize(glm::vec2(-edge_vec.y, edge_vec.x));
            edge_normals.push_back(normal);
        }
        
        // Fix top row
        for (int x = 0; x < grid_size; ++x) {
            velocities[x] = glm::vec2(0.0f, 0.0f);
        }
        
        displacement_bounds.resize(vertices.size(), contact_radius);
        
        return true;
    }
    
    void run() {
        while (!glfwWindowShouldClose(window)) {
            glClear(GL_COLOR_BUFFER_BIT);
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
    void updatePhysics() {
        // Apply gravity with smaller steps for stability
        for (int step = 0; step < 3; ++step) {
            for (size_t i = grid_size; i < vertices.size(); ++i) {
                velocities[i] += gravity * (dt / 3.0f);
            }
            
            // OGC displacement bounds computation
            computeDisplacementBounds();
            
            // Apply displacement constraints
            for (size_t i = grid_size; i < vertices.size(); ++i) {
                glm::vec2 proposed_move = velocities[i] * (dt / 3.0f);
                float move_length = glm::length(proposed_move);
                
                if (move_length > displacement_bounds[i]) {
                    proposed_move = proposed_move * (displacement_bounds[i] / move_length);
                }
                
                vertices[i] += proposed_move;
            }
            
            // OGC contact forces
            for (size_t v_idx = grid_size; v_idx < vertices.size(); ++v_idx) {
                glm::vec2 contact_force = computeOGCForce(v_idx, vertices[v_idx]);
                velocities[v_idx] += contact_force * (dt / 3.0f);
            }
            
            // Strong damping for settling
            for (size_t i = grid_size; i < velocities.size(); ++i) {
                velocities[i] *= 0.92f;
            }
            
            // Boundary constraints
            for (size_t i = 0; i < vertices.size(); ++i) {
                vertices[i].x = std::clamp(vertices[i].x, -1.0f, 1.0f);
                if (vertices[i].y < -0.9f) {
                    vertices[i].y = -0.9f;
                    velocities[i].y = 0.0f;
                }
            }
        }
    }
    
    void computeDisplacementBounds() {
        // OGC algorithm: compute maximum allowed displacement per vertex
        for (size_t v_idx = 0; v_idx < vertices.size(); ++v_idx) {
            float min_distance = contact_radius;
            
            for (size_t e_idx = 0; e_idx < edges.size(); ++e_idx) {
                if (isEdgeAdjacentToVertex(e_idx, v_idx)) continue;
                
                float distance = pointToLineSegmentDistance(
                    vertices[v_idx], 
                    vertices[edges[e_idx][0]], 
                    vertices[edges[e_idx][1]]
                );
                
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
            
            // OGC displacement bound formula
            displacement_bounds[v_idx] = std::max(0.0f, min_distance - 0.02f);
        }
    }
    
    glm::vec2 computeOGCForce(int vertex_idx, const glm::vec2& vertex_pos) {
        glm::vec2 total_force(0.0f);
        
        for (size_t e_idx = 0; e_idx < edges.size(); ++e_idx) {
            if (isEdgeAdjacentToVertex(e_idx, vertex_idx)) continue;
            
            glm::vec2 closest_point = getClosestPointOnLineSegment(
                vertex_pos, vertices[edges[e_idx][0]], vertices[edges[e_idx][1]]
            );
            float distance = glm::length(vertex_pos - closest_point);
            
            if (distance < contact_radius) {
                glm::vec2 edge_normal = edge_normals[e_idx];
                glm::vec2 to_vertex = vertex_pos - closest_point;
                
                if (glm::dot(edge_normal, to_vertex) < 0) {
                    edge_normal = -edge_normal;
                }
                
                float force_magnitude = (1.0f - distance/contact_radius) * 25.0f;
                total_force += force_magnitude * edge_normal;
            }
        }
        
        return total_force;
    }
    
    bool isEdgeAdjacentToVertex(size_t edge_idx, int vertex_idx) {
        const auto& edge = edges[edge_idx];
        return (edge[0] == vertex_idx || edge[1] == vertex_idx);
    }
    
    float pointToLineSegmentDistance(const glm::vec2& point, const glm::vec2& a, const glm::vec2& b) {
        glm::vec2 ab = b - a;
        glm::vec2 ap = point - a;
        float ab_length_sq = glm::dot(ab, ab);
        if (ab_length_sq == 0) return glm::length(ap);
        
        float t = glm::dot(ap, ab) / ab_length_sq;
        t = std::clamp(t, 0.0f, 1.0f);
        return glm::length(point - (a + t * ab));
    }
    
    glm::vec2 getClosestPointOnLineSegment(const glm::vec2& point, const glm::vec2& a, const glm::vec2& b) {
        glm::vec2 ab = b - a;
        glm::vec2 ap = point - a;
        float ab_length_sq = glm::dot(ab, ab);
        if (ab_length_sq == 0) return a;
        
        float t = glm::dot(ap, ab) / ab_length_sq;
        t = std::clamp(t, 0.0f, 1.0f);
        return a + t * ab;
    }
    
    void renderScene() {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        // Draw floor
        glColor3f(0.4f, 0.4f, 0.4f);
        glBegin(GL_QUADS);
        glVertex2f(-1.0f, -0.9f);
        glVertex2f(1.0f, -0.9f);
        glVertex2f(1.0f, -0.88f);
        glVertex2f(-1.0f, -0.88f);
        glEnd();
        
        // Draw OGC displacement bounds (key feature)
        glColor4f(1.0f, 0.0f, 0.0f, 0.15f);
        for (size_t i = grid_size; i < vertices.size(); ++i) {
            drawCircle(vertices[i].x, vertices[i].y, displacement_bounds[i]);
        }
        
        // Draw contact radii
        glColor4f(0.0f, 1.0f, 0.0f, 0.1f);
        for (size_t i = grid_size; i < vertices.size(); ++i) {
            drawCircle(vertices[i].x, vertices[i].y, contact_radius);
        }
        
        // Draw cloth edges
        glColor3f(0.9f, 0.9f, 1.0f);
        glLineWidth(2.0f);
        glBegin(GL_LINES);
        for (const auto& edge : edges) {
            glVertex2f(vertices[edge[0]].x, vertices[edge[0]].y);
            glVertex2f(vertices[edge[1]].x, vertices[edge[1]].y);
        }
        glEnd();
        
        // Draw vertices
        glColor3f(1.0f, 0.5f, 0.2f);
        glPointSize(6.0f);
        glBegin(GL_POINTS);
        for (const auto& vertex : vertices) {
            glVertex2f(vertex.x, vertex.y);
        }
        glEnd();
    }
    
    void drawCircle(float x, float y, float radius) {
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(x, y);
        for (int i = 0; i <= 24; ++i) {
            float angle = i * 2.0f * 3.14159f / 24.0f;
            glVertex2f(x + cos(angle) * radius, y + sin(angle) * radius);
        }
        glEnd();
    }
};

int main() {
    OGC2DSimulation sim;
    if (!sim.initialize()) return -1;
    sim.run();
    sim.shutdown();
    return 0;
}
