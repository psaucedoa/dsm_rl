#include "fluid.h"

int main() {


 
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
