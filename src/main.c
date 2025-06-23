#include "dsm.h"

unsigned int actions[36] = {
    SPEED_UP, SPEED_UP, SPEED_UP, SPEED_UP, SPEED_UP, SPEED_UP,
    SPEED_DOWN, SPEED_DOWN, SPEED_DOWN, SPEED_DOWN, SPEED_DOWN, SPEED_DOWN,
    LEFT, LEFT, LEFT, LEFT, LEFT, LEFT,
    RIGHT, RIGHT, RIGHT, RIGHT, RIGHT, RIGHT,
    BLADE_UP, BLADE_UP, BLADE_UP, BLADE_UP, BLADE_UP, BLADE_UP, 
    BLADE_DOWN, BLADE_DOWN, BLADE_DOWN, BLADE_DOWN, BLADE_DOWN, BLADE_DOWN
};

void test_multiple_envs() {
    Env** envs = (Env**)calloc(10, sizeof(Env*));
    for (int i = 0; i < 10; i++) {
        envs[i] = alloc_room_env();
        reset_room(envs[i]);
    }

    for (int i = 0; i < 41; i++) {
        for (int j = 0; j < 10; j++) {
            envs[j]->actions[0] = actions[i];
            step(envs[j]);
        }
    }
    for (int i = 0; i < 10; i++) {
        free_allocated_grid(envs[i]);
    }
    free(envs);
    printf("Done\n");
}

int main() {
    int width = 150;
    int height = 150;
    int num_agents = 1;
    int horizon = 128;
    float agent_speed = 1;
    int vision = 5;
    bool discretize = true;

    int render_cell_size = 4;
    int seed = 42;

    //test_multiple_envs();
    //exit(0);

    Env* env = alloc_room_env();
    reset_room(env);
    /*
    Env* env = allocate_grid(width, height, num_agents, horizon,
        vision, agent_speed, discretize);
    env->agents[0].spawn_y = 16;
    env->agents[0].spawn_x = 16;
    env->agents[0].color = AGENT_2;
    Env* env = alloc_locked_room_env();
    reset_locked_room(env);
    */
 
    Renderer* renderer = init_renderer(render_cell_size, width, height);

    int t = 0;
    while (!WindowShouldClose()) {
        // User can take control of the first agent
        env->actions[0] = PASS;
        if (IsKeyDown(KEY_UP)    || IsKeyDown(KEY_W)) env->actions[0] = SPEED_UP;
        if (IsKeyDown(KEY_DOWN)  || IsKeyDown(KEY_S)) env->actions[0] = SPEED_DOWN;
        if (IsKeyDown(KEY_LEFT)  || IsKeyDown(KEY_A)) env->actions[0] = LEFT;
        if (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D)) env->actions[0] = RIGHT;
        if (IsKeyDown(KEY_Q)) env->actions[0] = BLADE_UP;
        if (IsKeyDown(KEY_E)) env->actions[0] = BLADE_DOWN;

        //for (int i = 0; i < num_agents; i++) {
        //    env->actions[i] = rand() % 4;
        //}
        //env->actions[0] = actions[t];
        bool done = step(env);
        if (done) {
            printf("Done\n");
            reset_room(env);
        }
        render_global(renderer, env);

        /*
        t++;
        if (t == 41) {
            exit(0);
        }
        */
    }
    close_renderer(renderer);
    free_allocated_grid(env);
    return 0;
}
