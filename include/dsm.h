#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "raylib.h"
#include "rlgl.h"

#define PASS 0
#define SPEED_UP 1
#define SPEED_DOWN 2
#define LEFT 3
#define RIGHT 4
#define BLADE_UP 5
#define BLADE_DOWN 6

#define EMPTY 0
#define WALL 1
#define LAVA 2
#define GOAL 3
#define REWARD 4
#define OBJECT 5
#define AGENT_1 6
#define AGENT_2 7
#define AGENT_3 8
#define AGENT_4 9
#define AGENT_5 10
#define AGENT_6 11
#define AGENT_7 12
#define AGENT_8 13

#define NUM_AGENTS 8

#define TIMESTEP 0.1

// ---------------------------------------------------------------

int rand_color() {
    return AGENT_1 + rand()%(AGENT_4 - AGENT_1 + 1);
}

int is_agent(int tile) {
    return tile >= AGENT_1 && tile <= AGENT_8;
}

typedef struct Agent Agent;
struct Agent
{
    float y;
    float x;
    float vel;
    float theta;  // gonna keep map aligned ot initial position, but following blade frame
    float theta_dot;
    float blade_pos;

    float blade_width;
    float blade_thick;

    float spawn_y;
    float spawn_x;

    int color;
};

typedef struct Env Env;
struct Env
{
    // add in the 'fluid' properties here as well...
    // float eq_reynolds;
    // float eq_viscocity;
    // float eq_temperature;
    // etc

    int width;
    int height;

    int num_agents;  // potential for multi-agent stuff
    int horizon;
    
    int vision;  // FoW type-deal, mirroring the actual gridmap / camera
    
    // float speed;
    // bool discretize;
    
    int obs_size;

    int tick;
    float episode_return;

    unsigned char* grid;
    unsigned char* height_map;

    Agent* agents;
    unsigned char* observations;
    unsigned int* actions;
    float* rewards;
    float* dones;
};

/**
 * Initialize grid values
 */
Env* init_grid(
    unsigned char* observations, unsigned int* actions, float* rewards, float* dones,
    int width, int height, int num_agents, int horizon,
    int vision, float speed, bool discretize) 
{
    Env* env = (Env*)calloc(1, sizeof(Env));

    env->width = width;
    env->height = height;
    env->num_agents = num_agents;
    env->horizon = horizon;
    env->vision = vision;
    // env->speed = speed;
    // env->discretize = discretize;  // TODO: label
    env->obs_size = 2*vision + 1;

    env->grid = (unsigned char*)calloc(width*height, sizeof(unsigned char));
    env->height_map = (unsigned char*)calloc(width*height, sizeof(unsigned char));

    env->agents = (Agent*)calloc(num_agents, sizeof(Agent));
    env->observations = observations;
    env->actions = actions;
    env->rewards = rewards;
    env->dones = dones;  // TODO: label
    return env;
}

/**
 * Allocate memory for env
 */
Env* allocate_grid(
    int width, int height, int num_agents, int horizon,
    int vision, float speed, bool discretize)
{
    int obs_size = 2*vision + 1;
    unsigned char* observations = (unsigned char*)calloc(num_agents*obs_size*obs_size, sizeof(unsigned char));
    unsigned int* actions = (unsigned int*)calloc(num_agents, sizeof(unsigned int));
    float* rewards = (float*)calloc(num_agents, sizeof(float));
    float* dones = (float*)calloc(num_agents, sizeof(float));
  
    return init_grid(observations, actions, rewards, dones,
        width, height, num_agents, horizon, vision, speed, discretize);
}

/**
 * Free env
 */
void free_env(Env* env)
{
    free(env->grid);
    free(env->height_map);
    free(env->agents);
    free(env);
}

/**
 * Free all allocated memory
 */
void free_allocated_grid(Env* env)
{
    free(env->observations);
    free(env->actions);
    free(env->rewards);
    free(env->dones);
    free_env(env);
}

/**
 * TODO: label
 */
int grid_offset(Env* env, int y, int x)
{
    return y*env->width + x;
}

/**
 * get observations
 */
void compute_observations(Env* env) {
    for (int agent_idx = 0; agent_idx < env->num_agents; agent_idx++)
    {
        Agent* agent = &env->agents[agent_idx];
        float y = agent->y;
        float x = agent->x;
        int r = y;  // cast to int
        int c = x;

        int obs_offset = agent_idx*env->obs_size*env->obs_size;
        for (int dr = -env->vision; dr <= env->vision; dr++)
        {
            for (int dc = -env->vision; dc <= env->vision; dc++)
            {
                int rr = r + dr;
                int cc = c + dc;
                int adr = grid_offset(env, rr, cc);
                env->observations[obs_offset] = env->grid[adr];
                obs_offset++;
            }
        }
    }
}

/**
 * Reset env
 */
void reset(Env* env, int seed)
{
    env->tick = 0;
    env->episode_return = 0;

    // Add borders
    // int left = env->speed * env->vision;
    // int right = env->width - env->speed*env->vision - 1;
    // int bottom = env->height - env->speed*env->vision - 1;

    int left = env->vision;
    int right = env->width - env->vision - 1;
    int bottom = env->height - env->vision - 1;

    // // TODO: What
    // for (int r = 0; r < left; r++)
    // {
    //     for (int c = 0; c < env->width; c++)
    //     {
    //         int adr = grid_offset(env, r, c);
    //         env->grid[adr] = WALL;
    //     }
    // }

    // // TODO: What
    // for (int r = 0; r < env->height; r++)
    // {
    //     for (int c = 0; c < left; c++)
    //     {
    //         int adr = grid_offset(env, r, c);
    //         env->grid[adr] = WALL;
    //     }
    // }

    // // TODO: What
    // for (int c = right; c < env->width; c++)
    // {
    //     for (int r = 0; r < env->height; r++)
    //     {
    //         env->grid[grid_offset(env, r, c)] = WALL;
    //     }
    // }

    // // TODO: What
    // for (int r = bottom; r < env->height; r++)
    // {
    //     for (int c = 0; c < env->width; c++)
    //     {
    //         env->grid[grid_offset(env, r, c)] = WALL;
    //     }
    // }

    for (int r = 0; r < env->height; r++)
    {
        for (int c = 0; c < env->width; c++)
        {
            int adr = grid_offset(env, r, c);
            env->height_map[adr] = 128 * sinf(c);
        }
    }

    // Agent spawning
    for (int i = 0; i < env->num_agents; i++)
    {
        Agent* agent = &env->agents[i];
        int adr = grid_offset(env, agent->spawn_y, agent->spawn_x);
        assert(env->grid[adr] == EMPTY);
        assert(is_agent(agent->color));
        agent->y = agent->spawn_y;
        agent->x = agent->spawn_x;
        agent->blade_width = 20;
        agent->blade_thick = 2;
        env->grid[adr] = agent->color;
        agent->theta = 0;
    }
    compute_observations(env);
}

/**
 * Plane fitting thing
 */
void calculate_neighborhood_height()
{
    // this should just calculate the best-fit plane. 
    // In this case, can't we just do average height?
    // Anyway, used to set relative blade height
}

/**
 * Iterate!
 */
bool step(Env* env) {
    // TODO: Handle discrete vs continuous
    /*
    if self.discretize:
        actions_discrete = np_actions
    else:
        actions_continuous = np_actions
    */
    bool done = false;
    for (int agent_idx = 0; agent_idx < env->num_agents; agent_idx++)
    {
        // Discrete case only
        int atn = env->actions[agent_idx];
        float vel = 0;

        if (atn == PASS)
        {
            continue;
        }
        
        else if (atn == SPEED_UP)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->vel += 1;

            if (agent->vel > 10)
            {
                agent->vel = 10;
            }
        }
        
        else if (atn == SPEED_DOWN)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->vel -= 1;
        
            if (agent->vel < -10)
            {
                agent->vel = -10;
            }
        }
        
        else if (atn == LEFT)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->theta_dot += 0.1;
            if (agent->theta_dot > 1)
            {
                agent->theta_dot = 1;
            }
        }

        else if (atn == RIGHT)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->theta_dot -= 0.1;

            if (agent->theta_dot < -1)
            {
                agent->theta_dot = -1;
            }
        }

        else if (atn == BLADE_UP)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->blade_pos += 1;

            if (agent->blade_pos > 10)
            {
                agent->blade_pos = 10;
            }
        }

        else if (atn == BLADE_DOWN)
        {
            Agent* agent = &env->agents[agent_idx];
            agent->blade_pos -= 1;

            if (agent->blade_pos < 0)
            {
                agent->blade_pos = 0;
            }

        }

        else
        {
            printf("Invalid action: %i\n", atn);
            exit(1);
        }


        Agent* agent = &env->agents[agent_idx];
        agent->theta += agent->theta_dot * TIMESTEP;
        if (agent->theta > 2*PI)
        {
            agent->theta -= 2*PI;
        }
        if (agent->theta < 0)
        {
            agent->theta += 2*PI;
        }

        float y = agent->y;
        float x = agent->x;

        // we have the speed and heading (theta)
        // 0 is straight up, 90 is left...
        // also, this might be costly?

        float dest_y = TIMESTEP * agent->vel * cosf(agent->theta) + y;
        float dest_x = TIMESTEP * agent->vel * sinf(agent->theta) + x;

        agent->y = dest_y;
        agent->x = dest_x;

        int adr = grid_offset(env, y, x);
        int dest_adr = grid_offset(env, dest_y, dest_x);
        int dest_tile = env->grid[dest_adr];

        // REWARDS!
        if (dest_tile == REWARD || dest_tile == GOAL)
        {
            env->grid[dest_adr] = EMPTY;
            env->rewards[agent_idx] = 1.0;
            env->episode_return += 1.0;
            dest_tile = EMPTY;
            done = true;
        }

        if (dest_tile == EMPTY)
        {
            env->grid[adr] = EMPTY;
            env->grid[dest_adr] = agent->color;
        }
    }
    compute_observations(env);

    env->tick += 1;
    if (env->tick >= env->horizon) {
        done = true;
    }

    return done;
}

// Raylib client
Color COLORS[] = {
    (Color){6, 24, 24, 255},
    (Color){0, 0, 255, 255},
    (Color){0, 128, 255, 255},
    (Color){128, 128, 128, 255},
    (Color){255, 0, 0, 255},
    (Color){255, 255, 255, 255},
    (Color){255, 85, 85, 255},
    (Color){170, 170, 170, 255},
    (Color){0, 255, 255, 255},
    (Color){255, 255, 0, 255},
};

Rectangle UV_COORDS[7] = {
    (Rectangle){0, 0, 0, 0},
    (Rectangle){512, 0, 128, 128},
    (Rectangle){0, 0, 0, 0},
    (Rectangle){0, 0, 128, 128},
    (Rectangle){128, 0, 128, 128},
    (Rectangle){256, 0, 128, 128},
    (Rectangle){384, 0, 128, 128},
};

typedef struct {
    int cell_size;
    int width;
    int height;
    Texture2D gato;
} Renderer;

Renderer* init_renderer(int cell_size, int width, int height) {
    Renderer* renderer = (Renderer*)calloc(1, sizeof(Renderer));
    renderer->cell_size = cell_size;
    renderer->width = width;
    renderer->height = height;

    InitWindow(width*cell_size, height*cell_size, "PufferLib Ray Grid");
    SetTargetFPS(10);

    return renderer;
}

void close_renderer(Renderer* renderer) {
    CloseWindow();
    free(renderer);
}

void render_global(Renderer* renderer, Env* env) {
    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }

    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});

    int ts = renderer->cell_size;
    for (int r = 0; r < env->height; r++)
    {
        for (int c = 0; c < env->width; c++)
        {
            int adr = grid_offset(env, r, c);
            int tile = env->grid[adr];
            DrawRectangle(c*ts, r*ts, ts, ts, (Color){128, 128, env->height_map[adr], 255});
        }
    }

    Agent* agent = &env->agents[0];
    float deg = agent->theta * 180 / PI;

    Rectangle blade;
    blade.x = agent->x; // - agent->blade_width / 2;
    blade.y = agent->y; // - agent->blade_thick / 2;
    blade.height = agent->blade_thick;
    blade.width = agent->blade_width;

    // draw blade
    // DrawRectangle(agent->x, agent->y, agent->blade_width, agent->blade_thick, (Color){255, 255, 255, 255});
    DrawRectanglePro(
        blade,
        (Vector2){10, -20},
        -1*deg,
        YELLOW
    );

    DrawCircle(
        agent->x,
        agent->y,
        5,
        RED
    );

    DrawText(TextFormat("A-VEL: %02.02f deg/s", agent->theta_dot), 20, 20, 10, WHITE);
    DrawText(TextFormat("THETA: %02.02f deg", deg), 20, 40, 10, WHITE);
    DrawText(TextFormat("SPEED: %02.02f px/s", agent->vel), 20, 60, 10, WHITE);
    DrawText(TextFormat("X POS: %02.02f px", agent->x), 20, 80, 10, WHITE);
    DrawText(TextFormat("Y POS: %02.02f px", agent->y), 20, 100, 10, WHITE);
    DrawText(TextFormat("BLADE: %02.02f px", agent->blade_pos), 20, 120, 10, WHITE);
    EndDrawing();

}

Env* alloc_room_env()
{
    int width = 101;
    int height = 101;
    int num_agents = 1;
    int horizon = 512;
    float agent_speed = 1;
    int vision = 3;
    bool discretize = true;

    Env* env = allocate_grid(width+2*vision, height+2*vision, num_agents, horizon,
            vision, agent_speed, discretize);

    env->agents[0].spawn_y = 150;
    env->agents[0].spawn_x = 150;
    env->agents[0].color = AGENT_1;
    return env;
}


void reset_room(Env* env) {
    for (int r = 0; r < env->height; r++) {
        for (int c = 0; c < env->width; c++) {
            int adr = grid_offset(env, r, c);
            env->grid[adr] = EMPTY;
        }
    }
    reset(env, 0);

    // int vision = 3;
    // int adr = grid_offset(env, 7+vision, 9+vision);
    // //env->grid[adr] = GOAL;

    // adr = grid_offset(env, 16+vision, 17+vision);
    // env->grid[adr] = GOAL;
}
