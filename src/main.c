#include "dsm.h"

unsigned int actions[36] = {
    SPEED_UP, SPEED_UP, SPEED_UP, SPEED_UP, SPEED_UP, SPEED_UP,
    SPEED_DOWN, SPEED_DOWN, SPEED_DOWN, SPEED_DOWN, SPEED_DOWN, SPEED_DOWN,
    LEFT, LEFT, LEFT, LEFT, LEFT, LEFT,
    RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT,
    BLADE_UP, BLADE_UP, BLADE_UP, BLADE_UP, BLADE_UP, BLADE_UP, 
    BLADE_DOWN, BLADE_DOWN, BLADE_DOWN, BLADE_DOWN, BLADE_DOWN, BLADE_DOWN
};


int main() {
    int width = 800;
    int height = 800;
    int num_agents = 1;
    int horizon = 1024;
    float agent_speed = 1;
    int vision = 5;
    bool discretize = true;

    int render_cell_size = 1;
    int seed = 42;

    Env* env = alloc_room_env();
    reset_room(env);
 
    Renderer* renderer = init_renderer(render_cell_size, width, height);

    int t = 0;
    while (!WindowShouldClose()) {
        // User can take control of the first agent
        env->action = PASS;
        if (IsKeyDown(KEY_UP)    || IsKeyDown(KEY_W)) env->action = SPEED_UP;
        if (IsKeyDown(KEY_DOWN)  || IsKeyDown(KEY_S)) env->action = SPEED_DOWN;
        if (IsKeyDown(KEY_LEFT)  || IsKeyDown(KEY_A)) env->action = LEFT;
        if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) env->action = RIGHT;
        if (IsKeyDown(KEY_Q)) env->action = BLADE_UP;
        if (IsKeyDown(KEY_E)) env->action = BLADE_DOWN;

        bool done = step(env);
        if (done) {
            printf("Done\n");
            reset_room(env);
        }
        render_global(renderer, env);

    }
    close_renderer(renderer);
    free_allocated_grid(env);
    return 0;
}
